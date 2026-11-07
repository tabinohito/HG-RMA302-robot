import argparse
import json
import signal
import sys
import time
import traceback
from dataclasses import asdict
from datetime import datetime, timezone

from .config import load_config
from .control import DriveController, InputState, wheel_speed
from .diagnostics import DriveLog, FAULT_LOG, write_fault_events
from .esc import EscPair, SerialPort
from .gamepad import Gamepad, list_gamepads
from .serial_devices import open_ports, serial_device_report


def report_esc_events(pair, logger=None):
    for notice in pair.notices:
        if logger:
            logger.record("notice", message=notice)
        print(notice, flush=True)
    pair.notices.clear()
    if pair.fault_events:
        if logger:
            for event in pair.fault_events:
                logger.record("esc_fault", event=event)
        try:
            write_fault_events(pair.fault_events)
            print(f"ESC fault details: {FAULT_LOG}", flush=True)
        except OSError as exc:
            print(f"ESC fault log could not be saved: {exc}; details={pair.fault_events!r}", file=sys.stderr)
        pair.fault_events.clear()


def run(config, *, live=False, seconds=None, log_path=None):
    # Open the log before hardware, and drain it only after the ESCs are stopped.
    logger = DriveLog(log_path) if log_path is not None else None
    result = "finished"
    try:
        if logger:
            print(f"LOG: {logger.path}", flush=True)
            logger.record("session_start", schema_version=1,
                          started_at=datetime.now(timezone.utc).isoformat(),
                          live=live, seconds=seconds, config=asdict(config))
        if live and config.power.enabled:
            from .power_drive import run_powered
            run_powered(config, seconds=seconds, logger=logger)
        else:
            _run(config, live=live, seconds=seconds, logger=logger)
    except BaseException as exc:
        result = "error"
        if logger:
            logger.record("error", error_type=type(exc).__name__, message=str(exc),
                          traceback=traceback.format_exc())
        raise
    finally:
        if logger:
            logger.record("session_end", result=result)
            warning = logger.close()
            if warning:
                print(warning, file=sys.stderr)
            else:
                print(f"LOG保存完了: {logger.path}", flush=True)


def _run(config, *, live=False, seconds=None, logger=None):
    pair = None
    gamepad = None
    output = None
    imu = yaw = None
    stopped = False
    stop_reason = "finished"

    def stop_signal(signum, frame):
        nonlocal stopped, stop_reason
        stopped = True
        stop_reason = signal.Signals(signum).name

    previous_handlers = {sig: signal.signal(sig, stop_signal) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        gamepad = Gamepad(config.gamepad, event_timeout_s=config.drive.input_timeout_s)
        controller = DriveController(config.drive, require_input_updates=gamepad.requires_event_updates)
        device = gamepad.device
        print(f"Gamepad: {device.name}  {device.path}  USB={device.info.vendor:04x}:{device.info.product:04x}", flush=True)
        if logger:
            logger.record("gamepad", name=device.name, path=device.path,
                          usb_id=f"{device.info.vendor:04x}:{device.info.product:04x}",
                          requires_event_updates=gamepad.requires_event_updates)
        if live and config.imu.enabled:
            from .imu import ImuLink, YawFeedback
            imu=ImuLink(config.imu);yaw=YawFeedback(imu,config.drive)
        if live:
            pair = EscPair(config)
            if yaw: pair.yaw_feedback=yaw
            if logger:
                logger.record("esc_connected", devices={
                    side: {"serial": getattr(config, side + "_serial"), "path": port.path}
                    for side, port in zip(("left", "right"), pair.ports)})
            for side, port in zip(("left", "right"), pair.ports):
                print(f"{side} ESC: serial={getattr(config, side + '_serial') or '(path)'} → {port.path}", flush=True)
                settings = getattr(config, side + '_esc') or config.esc
                print(f"  JSON設定を確認済み: 電圧 {settings.bus_min_v:g}〜{settings.bus_max_v:g} V / "
                      f"相電流保護 {settings.phase_trip_a:g} A / NTC上限 {settings.ntc_trip_raw}（ADC生値）", flush=True)
                print(f"  始動電流 {min(settings.startup_current_a, config.drive.max_current_a):g} A / "
                      f"加速FF 最大{settings.acceleration_ff_a:g} A（PIとの合計は各輪{config.drive.max_current_a:g} A以内）", flush=True)
        if live:
            print(f"LIVE: 目標速度 {config.drive.min_speed_rpm:g}〜{config.drive.max_speed_rpm:g} RPM / 電流上限 各輪{config.drive.max_current_a:g} A / 速度指令の変化 {config.drive.acceleration_rpm_s:g} RPM/s", flush=True)
        else:
            print("DRY RUN: 表示のみ。モーターは動きません。実機操作には --live を付けてください。", flush=True)
        print("Xboxロゴ=タンク/ジョイスティック切替。左縦=前後・左横=曲がり・右横=旋回。タンクは左右縦=各輪。開始・切替後は中立。", flush=True)
        print(f"スティック: 不感帯±{config.drive.deadzone:g}の外を "
              f"{config.drive.min_speed_rpm:g}〜{config.drive.stick_full_scale_rpm:g} RPMへ線形換算。"
              "中立=0 RPM。始動中は最低速度で推定確立を待ちます。", flush=True)
        print(f"十字キー優先: 上下=前後 {wheel_speed(config.drive.dpad_drive_ratio, config.drive):g} RPM / "
              f"左右=その場旋回 各輪{wheel_speed(config.drive.dpad_turn_ratio, config.drive):g} RPM。斜め押しは停止。", flush=True)
        print("中立=制御減速、B=即時PWM停止・停止保持。両スティック中立・十字キー解放でStart=解除。Ctrl-C=終了。", flush=True)
        print(f"十字キーは両輪の始動完了を待って加速し、速度差を補正します。"
              f"指令平滑化 {config.drive.dpad_smoothing_s:g} 秒。", flush=True)
        print(f"スティック加速の平滑化 {config.drive.stick_smoothing_s:g} 秒。減速・中立・反転は平滑化で待たせません。", flush=True)
        if gamepad.requires_event_updates:
            print("無線ゲームパッド: 起動時は中立でStartを押して離して入力を更新してください。", flush=True)
            print(f"入力が{config.drive.input_timeout_s:g}秒変化しないと停止します。復帰は両スティック中立・十字キー解放から。", flush=True)
        else:
            print("有線ゲームパッド: スティック・十字キーを保持して操作できます。USB切断でPWM停止。", flush=True)
        if live and config.drive.observer_retry_limit:
            print(f"observer_lock は両輪停止後に自動復帰（最大{config.drive.observer_retry_limit}回）。"
                  "両スティック中立・十字キー解放、またはBで取消。", flush=True)
        started = time.monotonic()
        last_log_sample = -float("inf")
        last_print = -float("inf")
        previous_reason = None
        while not stopped:
            now = time.monotonic()
            if seconds is not None and now - started >= seconds:
                stop_reason = "duration"
                break
            # Evaluate every complete frame so brief release/B edges are not lost.
            if imu: imu.poll(now)
            frames = gamepad.read()
            now = time.monotonic()
            output = None
            for sample in frames:
                output = controller.update(sample, now)
                if pair and (not output.enabled or not (output.left_rpm or output.right_rpm)):
                    pair.update(output, now)
                if logger:
                    logger.snapshot(sample, output, pair, now, kind="input_frame",imu=yaw)
            output = controller.update(gamepad.state, now)
            if imu and imu.reason(now):
                output=controller.inhibit(imu.reason(now))
                if imu.reason(now) not in ("IMU_WAIT","IMU_CALIBRATING"):
                    raise RuntimeError(imu.reason(now))
            if pair:
                pair.update(output, now)
            if logger:
                # Control/ACK polling runs at 200 Hz; periodic disk snapshots
                # stay at 50 Hz. Input frames and fault events are never throttled.
                if now-last_log_sample >= .02:
                    logger.snapshot(gamepad.state, output, pair, now,imu=yaw)
                    last_log_sample = now
                warning = logger.warning()
                if warning:
                    print(warning, file=sys.stderr)
            if pair:
                report_esc_events(pair, logger)
            recovery = pair.recovery_display(now) if pair and output.enabled else None
            sync = getattr(pair, 'speed_sync', None)
            reason = recovery[0] if recovery else (sync.reason if sync and sync.reason else output.reason)
            if reason != previous_reason or now - last_print >= .25:
                status = ""
                if pair:
                    status = " ESC=" + ",".join(
                        f"{wheel.phase}/{(wheel.last_status or {}).get('state', '?')}"
                        f"({int((wheel.last_status or {}).get('est_mrpm', 0)) / 1000:+.2f}RPM)" for wheel in pair.wheels)
                sample = gamepad.state
                inputs = (f"stick L={sample.left_stick:+.2f} R={sample.right_stick:+.2f} "
                          f"Dpad X={sample.dpad_x:+d} Y={sample.dpad_y:+d}")
                wait = ""
                if sync:
                    wait = f" RPM_send L={sync.targets[0]:+.2f} R={sync.targets[1]:+.2f}"
                if controller.direction_wait_until is not None:
                    wait = f" 逆転待機 残り{max(0, controller.direction_wait_until - now):.1f}秒（両スティック中立・十字キー解放で取消）"
                if recovery:
                    wait = " " + recovery[1]
                print(f"{controller.mode:8} {reason:24} {inputs} RPM_cmd L={output.left_rpm:+.2f} R={output.right_rpm:+.2f}{status}{wait}", flush=True)
                last_print, previous_reason = now, reason
            time.sleep(max(0, .005 - (time.monotonic() - now)))
    except BaseException:
        stop_reason = "error"
        if logger:
            logger.snapshot(gamepad.state if gamepad else None, output, pair,
                            time.monotonic(), kind="error_snapshot")
        raise
    finally:
        try:
            if pair:
                errors = pair.close()
                for error in errors:
                    print(error, file=sys.stderr)
                if logger:
                    # close sends stop, but does not wait for a PWM-off ACK.
                    logger.record("esc_cleanup", stop_errors=errors)
                report_esc_events(pair, logger)
        finally:
            try:
                if imu: imu.close()
                if gamepad:
                    gamepad.close()
            finally:
                for sig, handler in previous_handlers.items():
                    signal.signal(sig, handler)
                if logger:
                    logger.record("drive_exit", reason=stop_reason)


def demo(config):
    """Deterministic tank-control examples; no devices opened."""
    for name, controls in (
        ("forward", dict(left_stick=1, right_stick=1)),
        ("reverse", dict(left_stick=-1, right_stick=-1)),
        ("left-wheel", dict(left_stick=1)),
        ("right-wheel", dict(right_stick=1)),
        ("pivot-right", dict(left_stick=1, right_stick=-1)),
        ("dpad-up", dict(dpad_y=1)), ("dpad-down", dict(dpad_y=-1)),
        ("dpad-left", dict(dpad_x=-1)), ("dpad-right", dict(dpad_x=1)),
    ):
        controller = DriveController(config.drive)
        controller.update(InputState(last_input_at=0), 0)
        output = controller.update(InputState(**controls, last_input_at=.02), .02)
        print(f"{name}: L={output.left_rpm:+.2f} RPM R={output.right_rpm:+.2f} RPM")
        stopped = controller.update(InputState(estop=True, last_input_at=.04), .04)
        assert not stopped.enabled


def inspect_ports(config, seconds):
    """Receive unsolicited telemetry only; send no UART commands, including stop."""
    ports = []
    try:
        ports = open_ports(config, SerialPort)
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            for port in ports:
                for line in port.read_lines():
                    print(f"{port.path}: {line}", flush=True)
            time.sleep(.01)
    finally:
        for port in ports:
            port.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description="HG-RMA302 / F310・F710・Xbox 360 compatible / two ST-Link UART ESCs")
    parser.add_argument("command", choices=("devices", "gamepad", "demo", "inspect", "check", "power", "imu", "drive"))
    parser.add_argument("--config", help="configuration JSON (default: config.example.json; gamepad can run without it)")
    parser.add_argument("--device", help="gamepad event device (gamepad only; default: auto-detect)")
    parser.add_argument("--live", action="store_true", help="send drive commands (drive only)")
    parser.add_argument("--log", nargs="?", const="", default=None, metavar="PATH",
                        help="save drive diagnostics as JSONL; omit PATH for a timestamped file (drive only)")
    parser.add_argument("--seconds", type=float, help="exit after this duration")
    args = parser.parse_args(argv)
    if args.live and args.command != "drive":
        parser.error("--live is only valid for drive")
    if args.device and args.command != "gamepad":
        parser.error("--device is only valid for gamepad")
    if args.log is not None and args.command != "drive":
        parser.error("--log is only valid for drive")
    if args.seconds is not None and not 0 < args.seconds <= 3600:
        parser.error("--seconds must be in (0, 3600]")
    try:
        if args.command == "gamepad":
            from .input_monitor import monitor
            path = args.device or (load_config(args.config).gamepad if args.config else None)
            monitor(path, seconds=args.seconds)
            return 0
        if args.command == "devices":
            config = load_config(args.config) if args.config else None
            print(json.dumps({"serial": serial_device_report(config),
                              "gamepads": list_gamepads()}, ensure_ascii=False, indent=2))
            return 0
        config = load_config(args.config or "config.example.json")
        if args.command == "demo":
            demo(config)
        elif args.command == "inspect":
            inspect_ports(config, args.seconds or 3)
        elif args.command == "imu":
            from .imu import monitor_imu
            monitor_imu(config.imu,seconds=args.seconds)
        elif args.command == "power":
            from .power import monitor_power
            monitor_power(config.power, seconds=args.seconds)
        elif args.command == "check":
            pair = EscPair(config, allow_standby=config.power.enabled)
            try:
                print("Both ESCs: JSON settings applied and verified; calibration, PWM-off, NTC and R/L OK. No arm/run sent.")
                if config.power.enabled:
                    for side,wheel in zip(('left','right'),pair.wheels):
                        print(f'{side}: bus={wheel.last_status["bus_mV"]} mV (USB-only standby accepted; drive requires voltage within configured limits)')
                else:
                    print('Both ESC supplies within configured limits.')
            finally:
                for error in pair.close():
                    print(error, file=sys.stderr)
        else:
            run(config, live=args.live, seconds=args.seconds, log_path=args.log)
    except KeyboardInterrupt:
        return 130
    except (OSError, RuntimeError, ValueError, TypeError, KeyError, ImportError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0
