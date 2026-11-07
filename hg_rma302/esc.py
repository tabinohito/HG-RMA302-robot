"""Adapter for the user's cppmain.cpp UART motor-bench firmware, not MCSDK MCP.

Only the main control loop sends ping. A wedged host therefore cannot keep a
stale speed command alive through a separate heartbeat thread.
"""

import math
import secrets
import time
from dataclasses import dataclass

from .config import Config, MotorSettings
from .control import Output
from .wheel_sync import WheelSync
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
        reason_key = {'invalid_sample': 'sample_reason', 'observer_lock': 'observer_reason'}.get(status['fault'])
        detail = f", {reason_key}={status[reason_key]}" if reason_key in status else ""
        if status['fault'] == 'ntc':
            detail += f", ntc_raw={status.get('ntc', '?')}, limit={status.get('ntc_trip_raw', '?')}"
        super().__init__(f"{path}: ESC fault: {status['fault']} "
                         f"(previous={self.previous_status.get('state', '?')}, "
                         f"req_mrpm={status.get('req_mrpm', '?')}, est_mrpm={status.get('est_mrpm', '?')}{detail})")

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
        self.received_bytes = 0
        self.service = None  # Optional main-thread input/power service during preflight only.

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
            data = self.serial.read(count)
            self.received_bytes += len(data)
            self.buffer.extend(data)
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
            if self.service:
                self.service()
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
    received_before = getattr(port, 'received_bytes', None)
    port.send(command)
    deadline = time.monotonic() + timeout
    received = []
    while time.monotonic() < deadline:
        if getattr(port, 'service', None):
            port.service()
        lines = port.read_lines()
        for line in lines:
            if line.startswith("ERR"):
                raise EscError(f"{port.path}: {line}")
            received.append(line)
        if predicate(received):
            return received
        time.sleep(.005)
    byte_count = (port.received_bytes - received_before
                  if received_before is not None else 'unknown')
    partial = bytes(getattr(port, 'buffer', b''))
    # Preserve the difference between silence, a truncated line and a reply
    # from an incompatible device. These details also reach --log on failure
    # before EscPair has finished construction.
    detail = (f"received_bytes={byte_count}, complete_lines={len(received)}, "
              f"partial_bytes={len(partial)}, timeout_s={timeout:g}")
    if received:
        detail += f", last_line={received[-1][:160]!r}"
    if partial:
        detail += f", partial={partial[:160]!r}"
    raise EscError(f"{port.path}: no valid response to {command!r} ({detail})")


def query_status(port) -> dict[str, str]:
    lines = exchange(port, "status", lambda lines: any(line.startswith("state=") for line in lines))
    return next(parse_status(line) for line in reversed(lines) if line.startswith("state="))


@dataclass(frozen=True)
class PreparedEsc:
    status: dict[str, str]
    bus_limits_mv: tuple[int, int]
    config_id: int | None = None
    settings: dict[str, int] | None = None


def check_bus_voltage(path, status, limits_mv, *, context="startup"):
    low, high = limits_mv
    measured = int(status["bus_mV"])
    if not low <= measured <= high:
        cause = "undervoltage" if measured < low else "overvoltage"
        raise EscError(f"{path}: {context} bus outside firmware limits {low}..{high} mV; "
                       f"measured={measured} mV ({measured / 1000:.3f} V, {cause})")


def apply_settings(port, settings: dict[str, int]) -> int:
    """Stage all fields with PWM off, commit atomically, then verify readback."""
    token = secrets.randbelow(0xffffffff) + 1
    exchange(port, f'cfgbegin {token}', lambda lines: f'config_begin={token}' in lines)
    for key, value in settings.items():
        exchange(port, f'cfgset {key} {value}',
                 lambda lines, key=key, value=value: f'config_set={key} value={value}' in lines)
    exchange(port, f'cfgapply {token}', lambda lines: f'config_applied={token}' in lines)
    lines = exchange(port, 'cartcfg', lambda lines: any(s.startswith('config_version=') for s in lines))
    result = dict(item.split('=',1) for item in next(s for s in reversed(lines) if s.startswith('config_version=')).split())
    expected = {'config_version':2, 'config_id':token, 'pending':0, **settings}
    if result != {key:str(value) for key,value in expected.items()}:
        mismatch = {k: {'expected':str(v), 'actual':result.get(k)} for k,v in expected.items() if result.get(k)!=str(v)}
        raise EscError(f'{port.path}: ESC configuration readback mismatch: {mismatch or result}')
    return token


def prepare(port, motor: MotorSettings, settings: dict[str, int], *, allow_standby=False) -> PreparedEsc:
    # VCP history and line-coding recovery must finish before the first query.
    port.synchronize()
    lines = exchange(port, "cartcap", lambda lines: any(line.startswith("cart_protocol=") for line in lines))
    cap = dict(item.split("=", 1) for item in next(line for line in lines if line.startswith("cart_protocol=")).split())
    if any(cap.get(key) != value for key, value in
           {"cart_protocol": "4", "signed_rpm": "1", "continuous_speed": "1", "controlled_reverse": "1", "controlled_zero": "1",
            "reverse_dwell_ms": "2000", "watchdog_ms": "500", "live_target": "1"}.items()):
        raise EscError(f"{port.path}: incompatible firmware; install the protocol 4 controlled-reversal firmware")
    if cap.get('config_protocol') != '2':
        raise EscError(f'{port.path}: runtime configuration unsupported; install the acceleration-FF JSON settings firmware (config_protocol=2)')
    supported_max = int(cap.get('supported_max_mrpm', '25000'))
    if settings['max_mrpm'] > supported_max:
        raise EscError(f"{port.path}: firmware supports up to {supported_max/1000:g} RPM; "
                       "install the 100 RPM tank-control firmware")
    config_id = apply_settings(port, settings)
    bus_limits = settings['bus_min_mv'], settings['bus_max_mv']
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
    # Powered-cart mode may configure an idle ESC supplied only through USB.
    # Firmware voltage limits remain unchanged; arming is gated on both real
    # bus voltages in power_drive. Overvoltage is always an error.
    if not (allow_standby and 0 <= int(status['bus_mV']) < bus_limits[0]):
        check_bus_voltage(port.path, status, bus_limits)
    if not 50 <= int(status.get('ntc','0')) <= settings['ntc_trip_raw']:
        raise EscError(f"{port.path}: NTC outside configured limits 50..{settings['ntc_trip_raw']}; measured={status.get('ntc','missing')} (raw ADC)")
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
    if status.get('config_id') != str(config_id):
        raise EscError(f'{port.path}: ESC configuration lost during startup')
    return PreparedEsc(status, bus_limits, config_id, settings)


class Wheel:
    """Nonblocking arm/start/speed/stop state machine, one outstanding rpm ACK."""

    def __init__(self, port, config: Config, *, bus_limits_mv=(18000, 22000), config_id=None):
        self.port = port
        self.config = config
        self.bus_limits_mv = bus_limits_mv
        self.config_id = config_id
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
        self.awaiting_fault_since = None

    def receive(self, now: float):
        clearing_on_entry = self.phase == 'stopping' and self.clearing_observer
        observed_fault = None
        rejected_observer = False
        fatal_reply = None
        for line in self.port.read_lines():
            if line.startswith("ERR") or line == "drive=rejected":
                if (line.startswith('ERR rpm requires active runrpm; ')
                        and 'fault=observer_lock' in line.split()):
                    rejected_observer = True
                else:
                    fatal_reply = line
                continue
            status = parse_status(line)
            if status is not None:
                if self.config_id is not None and status.get('config_id') != str(self.config_id):
                    raise EscError(f'{self.port.path}: ESC configuration changed or reset; restart drive to reapply JSON')
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
                    # Drain the whole already-read batch: a later hard fault
                    # must take precedence over an earlier observer fault.
                    if observed_fault is None or not fault.recoverable:
                        observed_fault = fault
                    continue
                if observed_fault is not None:
                    continue
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
        if observed_fault is not None and not observed_fault.recoverable:
            self.awaiting_fault_since = None
            raise observed_fault
        if fatal_reply is not None:
            raise EscError(f"{self.port.path}: {fatal_reply}")
        if observed_fault is not None:
            self.awaiting_fault_since = None
            raise observed_fault
        if rejected_observer and not clearing_on_entry:
            if self.awaiting_fault_since is None:
                # The ERR can arrive before the full fault frame, including
                # across USB reads. Request proof of the PWM-off fault; never
                # construct a fictitious status from the short error reply.
                self.awaiting_fault_since = now
                self.pending_rpm = None
                self.port.send('status')
        if self.awaiting_fault_since is not None and now-self.awaiting_fault_since > .25:
            raise EscError(f'{self.port.path}: rejected speed command without confirmed fault status')

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
        if self.awaiting_fault_since is not None:
            return  # EscPair has stopped the peer while the fault frame arrives.
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
        elif (self.phase == "running" and self.pending_rpm is None and now - self.last_rpm_at >= .01
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
        self.awaiting_fault_since = None


class EscPair:
    def __init__(self, config: Config, port_factory=SerialPort, *, service=None, allow_standby=False):
        self.config = config
        self.allow_standby = allow_standby
        self.yaw_feedback = None
        self.ports = []
        self.wheels = []
        self._initialize_recovery()
        self.speed_sync = WheelSync(config.drive)
        try:
            def factory(path, baudrate):
                port = port_factory(path, baudrate)
                if service is not None:
                    port.service = service
                return port
            self.ports = open_ports(config, factory, service=service)
            # Stop BOTH before doing any handshake which might take time.
            for port in self.ports:
                port.stop()
            for side, port, motor, settings in zip(('left', 'right'), self.ports,
                    (config.left_motor, config.right_motor),
                    (config.left_esc or config.esc, config.right_esc or config.esc)):
                try:
                    prepared = prepare(port, motor, settings.wire(config.drive), allow_standby=allow_standby)
                except EscError as exc:
                    serial_id = getattr(config, side + '_serial') or '(path configured)'
                    raise EscError(f'{side} ESC serial={serial_id}: {exc}') from exc
                wheel = Wheel(port, config, bus_limits_mv=prepared.bus_limits_mv, config_id=prepared.config_id)
                if allow_standby:
                    wheel.last_status = prepared.status
                    wheel.last_count = int(prepared.status['n'])
                    wheel.last_status_at = time.monotonic()
                self.wheels.append(wheel)
            self.standby_poll_at = 0.0
            for port in self.ports:
                port.service = None
        except BaseException:
            self.close()
            raise

    def poll_standby(self, now):
        """Refresh idle bus voltages without arming or lowering any ESC limit."""
        for wheel in self.wheels:
            wheel.receive(now)
        if now >= self.standby_poll_at:
            for port in self.ports:
                port.send('status')
            self.standby_poll_at = now + .2

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
            return "RECOVERY_LIMIT", "再始動失敗が続いたため待機。両スティック中立・十字キー解放から再操作"
        if now < self.retry_after:
            return "RECOVERING", (f"observer_lock 復帰待ち {self.retry_after-now:.1f}秒 "
                                  f"試行{self.retry_count}/{self.config.drive.observer_retry_limit}（両スティック中立・十字キー解放で取消）")
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
            self.notices.append("RECOVERY_LIMIT: observer_lock が続いたため再操作待ち。両スティック中立・十字キー解放から再操作してください。")
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
            if status.get("cart_protocol") != "4":
                raise EscError(f"{wheel.port.path}: recovery firmware check failed")
            check_bus_voltage(wheel.port.path, status, wheel.bus_limits_mv, context="recovery supply")
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
            if getattr(self, 'allow_standby', False) and not all(
                    w.last_status is not None and now-w.last_status_at <= .5 and
                    w.bus_limits_mv[0] <= int(w.last_status['bus_mV']) <= w.bus_limits_mv[1]
                    for w in self.wheels):
                # Defense in depth: accepting USB-only preflight must never
                # make arm/runrpm possible until both drive supplies are valid.
                output = Output(reason='WAIT_POWER')
            if any(w.awaiting_fault_since is not None for w in self.wheels):
                # Stop the healthy peer immediately. Leave the rejecting ESC's
                # diagnostic state intact until its full fault report arrives.
                # No new speed/arm/ping is sent to that ESC while waiting.
                for wheel in self.wheels:
                    if (not output.enabled or not (output.left_rpm or output.right_rpm)
                            or (wheel.awaiting_fault_since is None and wheel.phase not in ('stopped','stopping'))):
                        wheel.stop(now)
                self.speed_sync.reset()
                self.speed_sync.reason = 'FAULT_STATUS_WAIT'
                return
            motion_requested = output.enabled and bool(output.left_rpm or output.right_rpm)
            if not motion_requested:
                # Neutral cancels retries too; never retain an old movement.
                # Preserve cooldown if another command arrives immediately.
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
            if self.retry_after and not waiting and not motion_requested:
                self.retry_after = 0
            if motion_requested and not waiting and not self.retry_blocked and self.retry_after:
                self._check_recovery_ready(now)
                self.retry_after = 0
                self.notices.append("RECOVERY_RESTART: 最新のスティック指令で再始動します。")
            enabled = output.enabled and not waiting and not self.retry_blocked
            if enabled and any(not math.isfinite(value) or (value != 0 and not
                    self.config.drive.min_speed_rpm <= abs(value) <= self.config.drive.max_speed_rpm)
                    for value in (output.left_rpm, output.right_rpm)):
                raise EscError("Speed outside supported range")
            targets = self.speed_sync.update(output, self.wheels, now, enabled=enabled,
                                             yaw_control=getattr(self,'yaw_feedback',None) is not None)
            if getattr(self, "yaw_feedback", None) is not None:
                targets = self.yaw_feedback.correct(targets, output, self.wheels, now)
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
