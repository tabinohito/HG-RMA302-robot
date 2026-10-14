"""Adapter for the user's cppmain.cpp UART motor-bench firmware, not MCSDK MCP.

Only the main control loop sends ping. A wedged host therefore cannot keep a
stale speed command alive through a separate heartbeat thread.
"""

import math
import time

from .config import Config, MotorSettings
from .control import Output
from .serial_devices import open_ports


class EscError(RuntimeError):
    pass


class EscFault(EscError):
    """Keep the fault frame before stop clears the MCU's diagnostic state."""

    def __init__(self, path, status, previous_status, phase):
        self.path = path
        self.status = dict(status)
        self.previous_status = dict(previous_status or {})
        self.phase = phase
        super().__init__(f"{path}: ESC fault: {status['fault']} "
                         f"(previous={self.previous_status.get('state', '?')}, "
                         f"req_mrpm={status.get('req_mrpm', '?')}, est_mrpm={status.get('est_mrpm', '?')})")

    @property
    def recoverable(self):
        return (self.status['fault'] == 'observer_lock' and self.status['state'] == 'fault'
                and self.status['cal'] == 'ok' and self.status['MOE'] == '0'
                and int(self.status['CCER'], 0) == 0)


def parse_status(line: str) -> dict[str, str] | None:
    if not line.startswith("state="):
        return None
    fields = dict(item.split("=", 1) for item in line.split() if "=" in item)
    required = {"state", "fault", "cal", "MOE", "CCER", "bus_mV", "R_mOhm", "L_uH", "n"}
    if not required <= fields.keys():
        raise EscError("Incomplete ESC status: " + line)
    return fields


def check_stopped(status: dict[str, str]):
    if not (status["state"] == "idle" and status["fault"] == "none" and
            status["cal"] == "ok" and status["MOE"] == "0" and int(status["CCER"], 0) == 0):
        raise EscError("ESC is not calibrated and PWM-off: " + str(status))


class SerialPort:
    def __init__(self, path: str, baudrate=115200):
        import serial

        self.path = path
        self.serial = serial.Serial(path, baudrate=baudrate, timeout=0, write_timeout=.02,
                                    exclusive=True, xonxoff=False, rtscts=False, dsrdtr=False)
        self.buffer = bytearray()

    def send(self, command: str):
        payload = (command + "\r\n").encode("ascii")
        if self.serial.out_waiting > 128:
            raise EscError(f"{self.path}: UART transmit backlog")
        if self.serial.write(payload) != len(payload):
            raise EscError(f"{self.path}: incomplete UART write")

    def read_lines(self) -> list[str]:
        count = self.serial.in_waiting
        if count > 8192:
            raise EscError(f"{self.path}: UART receive backlog")
        if count:
            self.buffer.extend(self.serial.read(count))
        lines = []
        while b"\n" in self.buffer:
            raw, _, rest = self.buffer.partition(b"\n")
            self.buffer = bytearray(rest)
            lines.append(raw.decode("ascii", errors="replace").strip())
        if len(self.buffer) > 2048:
            raise EscError(f"{self.path}: overlong UART line")
        return [line for line in lines if line]

    def stop(self):
        # Ctrl-C is handled immediately by the firmware, even mid-command.
        self.serial.reset_output_buffer()
        payload = b"\x03\r\nstop\r\n"
        if self.serial.write(payload) != len(payload):
            raise EscError(f"{self.path}: incomplete stop write")

    def synchronize(self):
        """Drain old VCP data to a quiet boundary, only before arming.

        ST-Link can retain a partial status line across closes/resets. Dropping
        complete lines alone leaves that prefix attached to the first reply.
        """
        self.stop()
        self.serial.reset_input_buffer()
        self.buffer.clear()
        started = last_data = time.monotonic()
        while time.monotonic() - started < 2:
            count = self.serial.in_waiting
            if count:
                self.serial.read(min(count, 8192))
                last_data = time.monotonic()
            now = time.monotonic()
            if now - started >= .3 and now - last_data >= .15:
                return
            time.sleep(.005)
        raise EscError(f"{self.path}: no quiet UART boundary while stopped")

    def close(self):
        self.serial.close()


def exchange(port, command: str, predicate, timeout=.6) -> list[str]:
    """Used only while stopped, before entering the real-time control loop."""
    port.read_lines()  # Discard earlier unsolicited status / old acknowledgments.
    port.send(command)
    deadline = time.monotonic() + timeout
    received = []
    while time.monotonic() < deadline:
        lines = port.read_lines()
        for line in lines:
            if line.startswith("ERR"):
                raise EscError(f"{port.path}: {line}")
            received.append(line)
        if predicate(received):
            return received
        time.sleep(.005)
    raise EscError(f"{port.path}: no valid response to {command!r}")


def query_status(port) -> dict[str, str]:
    lines = exchange(port, "status", lambda lines: any(line.startswith("state=") for line in lines))
    return next(parse_status(line) for line in reversed(lines) if line.startswith("state="))


def prepare(port, motor: MotorSettings) -> dict[str, str]:
    # VCP history and line-coding recovery must finish before the first query.
    port.synchronize()
    lines = exchange(port, "cartcap", lambda lines: any(line.startswith("cart_protocol=") for line in lines))
    cap = dict(item.split("=", 1) for item in next(line for line in lines if line.startswith("cart_protocol=")).split())
    if any(cap.get(key) != value for key, value in
           {"cart_protocol": "4", "signed_rpm": "1", "continuous_speed": "1", "controlled_reverse": "1", "controlled_zero": "1",
            "reverse_dwell_ms": "2000", "watchdog_ms": "500", "min_mrpm": "8000",
            "max_mrpm": "25000", "startup_mA": "600", "slew_mrpm_s": "20000",
            "max_slew_mrpm_s": "20000", "startup_target_mrpm": "8000", "live_target": "1"}.items()):
        raise EscError(f"{port.path}: incompatible firmware; install the protocol 4 controlled-reversal firmware")
    # Recalibrate with PWM off to account for current-sense offset drift after warm-up.
    exchange(port, "zero", lambda lines: "calibration=started" in lines)
    deadline = time.monotonic() + 1
    while True:
        status = query_status(port)
        if status["cal"] != "pending" or time.monotonic() >= deadline:
            break
        time.sleep(.05)
    check_stopped(status)
    if "speed_mode" not in status or status.get("cart_protocol") != "4":
        raise EscError(f"{port.path}: old firmware; signed speed mode is missing")
    if not int(cap["bus_min_mV"]) <= int(status["bus_mV"]) <= int(cap["bus_max_mV"]):
        raise EscError(f"{port.path}: bus outside firmware limits {cap['bus_min_mV']}..{cap['bus_max_mV']} mV")
    if int(cap.get("max_speed_iq_mA", "0")) != 2000 or int(cap.get("phase_trip_mA", "0")) != 4000:
        raise EscError(f"{port.path}: unexpected firmware current range")
    if motor.resistance_ohm is not None:
        # params has no ACK. The following cartcap reply proves the MCU has
        # processed params and that older periodic status lines have passed.
        # Send both in one exchange: draining after params could discard its ERR.
        exchange(port, f"params {motor.resistance_ohm:.9g} {motor.inductance_h:.9g}\r\ncartcap",
                 lambda lines: any(line.startswith("cart_protocol=") for line in lines))
        status = query_status(port)
        check_stopped(status)
        expected_r, expected_l = motor.resistance_ohm * 1000, motor.inductance_h * 1e6
        actual_r, actual_l = int(status["R_mOhm"]), int(status["L_uH"])
        if abs(actual_r - expected_r) > 1 or abs(actual_l - expected_l) > 1:
            raise EscError(f"{port.path}: motor parameters were not applied: "
                           f"expected R={expected_r:g} mOhm L={expected_l:g} uH; "
                           f"actual R={actual_r} mOhm L={actual_l} uH")
    if int(status["R_mOhm"]) <= 0 or int(status["L_uH"]) <= 0:
        raise EscError(f"{port.path}: R/L unset; configure measured values for this motor")
    exchange(port, "hfi off", lambda lines: any(line.startswith("hfi=") for line in lines))
    return status


class Wheel:
    """Nonblocking arm/start/speed/stop state machine, one outstanding rpm ACK."""

    def __init__(self, port, config: Config):
        self.port = port
        self.config = config
        self.phase = "stopped"
        self.deadline = 0.0
        self.last_status_at = 0.0
        self.last_ping_at = -math.inf
        self.last_rpm_at = -math.inf
        self.pending_rpm: int | None = None
        self.last_sent_rpm: int | None = None
        self.rpm_deadline = 0.0
        self.active_seen = False
        self.arm_ack = False
        self.last_count: int | None = None
        self.last_status: dict[str, str] | None = None
        self.direction = 1
        self.clearing_observer = False

    def receive(self, now: float):
        for line in self.port.read_lines():
            if line.startswith("ERR") or line == "drive=rejected":
                raise EscError(f"{self.port.path}: {line}")
            status = parse_status(line)
            if status is not None:
                count = int(status["n"])
                if self.last_count is not None and count < self.last_count:
                    raise EscError(f"{self.port.path}: ESC rebooted")
                if self.last_count is None or count > self.last_count:
                    self.last_status_at = now
                previous = self.last_status
                self.last_count, self.last_status = count, status
                if status["fault"] != "none":
                    fault = EscFault(self.port.path, status, previous, self.phase)
                    # A periodic fault frame may already be in flight before
                    # stop. It must not spend another retry or hide other faults.
                    if self.phase == "stopping" and self.clearing_observer and fault.recoverable:
                        continue
                    raise fault
                if self.phase == "stopping" and status["state"] == "idle" and status["MOE"] == "0":
                    check_stopped(status)
                    self.phase = "stopped"
                    self.clearing_observer = False
                elif self.phase == "running":
                    if status.get("speed_mode") == "1" and (status["MOE"] == "1" or
                            (status["state"] == "ready" and status["MOE"] == "0" and int(status["CCER"], 0) == 0)):
                        if int(status.get("iq_max_mA", "-1")) != round(self.config.drive.max_current_a * 1000):
                            raise EscError(f"{self.port.path}: wrong speed current limit")
                        if int(status.get("slew_mrpm_s", "-1")) != round(self.config.drive.acceleration_rpm_s * 1000):
                            raise EscError(f"{self.port.path}: wrong speed acceleration")
                        if status["state"] not in ("align", "ramp", "blend", "closed", "decelerate", "settle", "ready"):
                            raise EscError(f"{self.port.path}: unexpected drive state")
                        if status.get("uart_direction") not in ("1", "-1"):
                            raise EscError(f"{self.port.path}: wrong motor direction")
                        self.active_seen = True
                    elif self.active_seen:
                        raise EscError(f"{self.port.path}: ESC stopped unexpectedly")
            elif line.startswith("armed for ") and self.phase == "arming":
                self.arm_ack = True
            elif line == "drive=started" and self.phase == "starting":
                self.phase = "running"
                self.deadline = now + .4
                self.last_status_at = now
            elif line.startswith("OK wheel_target_mrpm=") and self.pending_rpm is not None:
                if int(line.split("=", 1)[1]) != self.pending_rpm:
                    raise EscError(f"{self.port.path}: wrong speed acknowledgment")
                self.pending_rpm = None

    def update(self, target_rpm: float, now: float, *, enabled=True):
        cfg = self.config.drive
        if not math.isfinite(target_rpm) or (target_rpm != 0 and not
                cfg.min_speed_rpm <= abs(target_rpm) <= cfg.max_speed_rpm):
            raise EscError("Speed outside supported range")
        # Three decimals matches firmware milli-RPM reporting.
        millirpm = round(target_rpm * 1000)
        if not enabled:
            if self.phase not in ("stopped", "stopping"):
                self.stop(now)
            self._check_deadlines(now)
            return
        self._check_deadlines(now)
        if self.phase == "stopping":
            return
        direction = 1 if millirpm > 0 else -1
        if self.phase == "stopped":
            if millirpm == 0:
                return
            self.direction = direction
            self.port.send("arm")
            self.phase = "arming"
            self.deadline = now + .4
            self.arm_ack = False
        elif self.phase == "arming" and self.arm_ack:
            if millirpm == 0:
                self.stop(now)
                return
            self.direction = direction
            cfg = self.config
            self.port.send(f"runrpm {millirpm / 1000:.3f} {cfg.drive.max_current_a:g} {cfg.drive.acceleration_rpm_s:g}")
            self.last_sent_rpm = millirpm
            self.phase = "starting"
            self.deadline = now + .4
            self.active_seen = False
        elif (self.phase == "running" and self.pending_rpm is None and now - self.last_rpm_at >= .02
              and millirpm != self.last_sent_rpm):
            # Protocol 4 accepts zero and opposite signs; the ESC brakes first.
            # One ACK in flight; intermediate stick positions are never queued.
            self.port.send(f"rpm {millirpm / 1000:.3f}")
            self.pending_rpm = millirpm
            self.last_sent_rpm = millirpm
            self.last_rpm_at = now
            self.rpm_deadline = now + .25
        if now - self.last_ping_at >= .1:
            self.port.send("ping")
            self.last_ping_at = now

    def _check_deadlines(self, now: float):
        if self.phase in ("arming", "starting", "stopping") and now > self.deadline:
            raise EscError(f"{self.port.path}: {self.phase} timeout")
        if self.phase == "running":
            if not self.active_seen and now > self.deadline:
                raise EscError(f"{self.port.path}: PWM-on/speed mode was not confirmed")
            if now - self.last_status_at > .35:
                raise EscError(f"{self.port.path}: ESC telemetry timeout")
            if self.pending_rpm is not None and now > self.rpm_deadline:
                raise EscError(f"{self.port.path}: speed acknowledgment timeout")

    def stop(self, now: float, *, clearing_observer=False):
        self.port.stop()
        self.port.send("status")
        self.phase = "stopping"
        self.deadline = now + .6
        self.pending_rpm = None
        self.last_sent_rpm = None
        self.arm_ack = False
        self.active_seen = False
        self.clearing_observer = clearing_observer


class EscPair:
    def __init__(self, config: Config, port_factory=SerialPort):
        self.config = config
        self.ports = []
        self.wheels = []
        self._initialize_recovery()
        try:
            self.ports = open_ports(config, port_factory)
            # Stop BOTH before doing any handshake which might take time.
            for port in self.ports:
                port.stop()
            for port, motor in zip(self.ports, (config.left_motor, config.right_motor)):
                prepare(port, motor)
                self.wheels.append(Wheel(port, config))
        except BaseException:
            self.close()
            raise

    def _initialize_recovery(self):
        self.retry_after = 0.0
        self.retry_count = 0
        self.retry_blocked = False
        self.healthy_since = None
        self.recovery_poll_at = 0.0
        self.notices = []
        self.fault_events = []

    def recovery_display(self, now):
        if self.retry_blocked:
            return "RECOVERY_LIMIT", "再始動失敗が続いたため待機。LB解放・中立から再操作"
        if now < self.retry_after:
            return "RECOVERING", (f"observer_lock 復帰待ち {self.retry_after-now:.1f}秒 "
                                  f"試行{self.retry_count}/{self.config.drive.observer_retry_limit}（LB解放で取消）")
        return None

    def _record_fault(self, fault, now):
        side = next((side for side, port in zip(("left", "right"), self.ports)
                     if port.path == fault.path), None)
        self.fault_events.append({"monotonic_s": now, "port": fault.path, "side": side,
                                  "usb_serial": getattr(self.config, f"{side}_serial", None),
                                  "phase": fault.phase, "previous_status": fault.previous_status,
                                  "status": fault.status, "retry_count_before": self.retry_count})

    def _begin_recovery(self, faults, now):
        # Stop both wheels before clearing/rearming either. A failed inner wheel
        # must not leave the other wheel driving an unintended turn.
        for wheel in self.wheels:
            wheel.stop(now, clearing_observer=True)
        self.retry_after = now + self.config.drive.observer_retry_delay_s
        self.recovery_poll_at = now + .25
        self.healthy_since = None
        if self.retry_count >= self.config.drive.observer_retry_limit:
            self.retry_blocked = True
            self.notices.append("RECOVERY_LIMIT: observer_lock が続いたため再操作待ち。LB解放・中立から再操作してください。")
        else:
            self.retry_count += 1
            paths = ", ".join(fault.path for fault in faults)
            self.notices.append(f"RECOVERING: {paths}: observer_lock。両輪停止後、最新の指令で自動再始動 "
                                f"({self.retry_count}/{self.config.drive.observer_retry_limit})")

    def _check_recovery_ready(self, now):
        for wheel, motor in zip(self.wheels, (self.config.left_motor, self.config.right_motor)):
            status = wheel.last_status
            if wheel.phase != "stopped" or status is None or now-wheel.last_status_at > .35:
                raise EscError(f"{wheel.port.path}: recovery needs fresh PWM-off status")
            check_stopped(status)
            if status.get("cart_protocol") != "4" or not 18000 <= int(status['bus_mV']) <= 22000:
                raise EscError(f"{wheel.port.path}: recovery firmware/supply check failed")
            r, l = int(status['R_mOhm']), int(status['L_uH'])
            if r <= 0 or l <= 0 or (motor.resistance_ohm is not None and
                    (abs(r-motor.resistance_ohm*1000) > 1 or abs(l-motor.inductance_h*1e6) > 1)):
                raise EscError(f"{wheel.port.path}: motor parameters changed during recovery")

    def update(self, output: Output, now: float):
        try:
            # Inspect both receivers before sending a new speed to either wheel.
            faults = []
            for wheel in self.wheels:
                try:
                    wheel.receive(now)
                except EscFault as fault:
                    self._record_fault(fault, now)
                    faults.append(fault)
            if faults:
                fatal = next((fault for fault in faults if not fault.recoverable), None)
                if fatal is not None or self.config.drive.observer_retry_limit == 0:
                    raise fatal or faults[0]
                self._begin_recovery(faults, now)
            if not output.enabled:
                # Cancellation is immediate. Keep the cooldown even if a new LB
                # hold arrives, but never retain a movement command for later.
                if self.retry_count or self.retry_blocked:
                    self.notices.append("RECOVERY_CANCELLED: 再始動を取り消しました。")
                self.retry_count = 0
                self.retry_blocked = False
                self.healthy_since = None
            waiting = now < self.retry_after
            if waiting and now >= self.recovery_poll_at:
                for port in self.ports:
                    port.send("status")
                self.recovery_poll_at = now + .25
            if self.retry_after and not waiting and not output.enabled:
                self.retry_after = 0
            if output.enabled and not waiting and not self.retry_blocked and self.retry_after:
                self._check_recovery_ready(now)
                self.retry_after = 0
                self.notices.append("RECOVERY_RESTART: 最新のスティック指令で再始動します。")
            enabled = output.enabled and not waiting and not self.retry_blocked
            targets = (output.left_rpm, output.right_rpm) if enabled else (0.0, 0.0)
            for wheel, target in zip(self.wheels, targets):
                wheel.update(target, now, enabled=enabled)
            if enabled and all(w.phase == "running" and (w.last_status or {}).get("state") == "closed"
                               for w in self.wheels):
                if self.healthy_since is None:
                    self.healthy_since = now
                elif now-self.healthy_since >= 10:
                    self.retry_count = 0
            else:
                self.healthy_since = None
        except BaseException:
            self.stop()
            raise

    def stop(self) -> list[str]:
        errors = []
        for port in self.ports:
            try:
                port.stop()
            except Exception as exc:
                errors.append(f"{port.path}: stop failed: {exc}")
        return errors

    def close(self) -> list[str]:
        errors = self.stop()
        for port in self.ports:
            try:
                port.close()
            except Exception as exc:
                errors.append(f"{port.path}: close failed: {exc}")
        self.ports = []
        return errors
