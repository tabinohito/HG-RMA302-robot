import argparse
import json
import signal
import sys
import time

from .config import load_config
from .control import DriveController, InputState
from .diagnostics import FAULT_LOG, write_fault_events
from .esc import EscPair, SerialPort
from .gamepad import Gamepad, list_gamepads
from .serial_devices import open_ports, serial_device_report


def report_esc_events(pair):
    for notice in pair.notices:
        print(notice, flush=True)
    pair.notices.clear()
    if pair.fault_events:
        try:
            write_fault_events(pair.fault_events)
            print(f"ESC fault details: {FAULT_LOG}", flush=True)
        except OSError as exc:
            print(f"ESC fault log could not be saved: {exc}; details={pair.fault_events!r}", file=sys.stderr)
        pair.fault_events.clear()


def run(config, *, live=False, seconds=None):
    pair = None
    gamepad = None
    stopped = False

    def stop_signal(signum, frame):
        nonlocal stopped
        stopped = True

    previous_handlers = {sig: signal.signal(sig, stop_signal) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        gamepad = Gamepad(config.gamepad, event_timeout_s=config.drive.input_timeout_s)
        controller = DriveController(config.drive, require_input_updates=gamepad.requires_event_updates)
        device = gamepad.device
        print(f"Gamepad: {device.name}  {device.path}  USB={device.info.vendor:04x}:{device.info.product:04x}", flush=True)
        if live:
            pair = EscPair(config)
            for side, port in zip(("left", "right"), pair.ports):
                print(f"{side} ESC: serial={getattr(config, side + '_serial') or '(path)'} → {port.path}", flush=True)
        if live:
            print(f"LIVE: 目標速度 {config.drive.min_speed_rpm:g}〜{config.drive.max_speed_rpm:g} RPM / 電流上限 各輪{config.drive.max_current_a:g} A / 速度指令の変化 {config.drive.acceleration_rpm_s:g} RPM/s", flush=True)
        else:
            print("DRY RUN: 表示のみ。モーターは動きません。実機操作には --live を付けてください。", flush=True)
        print("開始時はスティック中立・LB解放でREADYを確認→LBを保持して操作。B=停止保持、Start=解除、Ctrl-C=終了。", flush=True)
        if gamepad.requires_event_updates:
            print("無線ゲームパッド: 起動時は中立でLBを一度押して離して入力を更新してください。", flush=True)
            print(f"入力が{config.drive.input_timeout_s:g}秒変化しないと停止します。復帰は中立・LB解放から。", flush=True)
        else:
            print("有線ゲームパッド: スティックを固定して操作できます。LB解放・USB切断でPWM停止。", flush=True)
        print("LB保持のまま方向切替できます。逆転時は各ESCが減速してから最新の方向へ再始動。LBを離した後の逆転再開には停止待機があります。", flush=True)
        if live and config.drive.observer_retry_limit:
            print(f"observer_lock は両輪停止後に自動復帰（最大{config.drive.observer_retry_limit}回）。LB解放・Bで取消。", flush=True)
        started = time.monotonic()
        last_print = -float("inf")
        previous_reason = None
        while not stopped:
            now = time.monotonic()
            if seconds is not None and now - started >= seconds:
                break
            # Evaluate every complete frame so brief release/B edges are not lost.
            frames = gamepad.read()
            now = time.monotonic()
            output = None
            for sample in frames:
                output = controller.update(sample, now)
                if pair and not output.enabled:
                    pair.update(output, now)
            output = controller.update(gamepad.state, now)
            if pair:
                pair.update(output, now)
                report_esc_events(pair)
            recovery = pair.recovery_display(now) if pair and output.enabled else None
            reason = recovery[0] if recovery else output.reason
            if reason != previous_reason or now - last_print >= .25:
                status = ""
                if pair:
                    status = " ESC=" + ",".join(
                        f"{wheel.phase}/{(wheel.last_status or {}).get('state', '?')}"
                        f"({int((wheel.last_status or {}).get('est_mrpm', 0)) / 1000:+.2f}RPM)" for wheel in pair.wheels)
                sample = gamepad.state
                inputs = f"stick X={sample.steering:+.2f} forward={sample.throttle:+.2f} LB={int(sample.deadman)}"
                wait = ""
                if controller.direction_wait_until is not None:
                    wait = f" 逆転待機 残り{max(0, controller.direction_wait_until - now):.1f}秒（LB解放で取消）"
                if recovery:
                    wait = " " + recovery[1]
                print(f"{reason:24} {inputs} RPM_cmd L={output.left_rpm:+.2f} R={output.right_rpm:+.2f}{status}{wait}", flush=True)
                last_print, previous_reason = now, reason
            time.sleep(max(0, .02 - (time.monotonic() - now)))
    finally:
        if pair:
            for error in pair.close():
                print(error, file=sys.stderr)
            report_esc_events(pair)
        if gamepad:
            gamepad.close()
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)


def demo(config):
    """Deterministic, no devices opened; exercise forward, reverse and pivot."""
    for name, throttle, steering in (("forward", 1, 0), ("reverse", -1, 0), ("pivot-right", 0, 1)):
        controller = DriveController(config.drive)
        controller.update(InputState(last_input_at=0), 0)
        controller.update(InputState(deadman=True, last_input_at=.02), .02)
        for n in range(2, 52):
            now = n * .02
            output = controller.update(InputState(throttle, steering, True, last_input_at=now), now)
        print(f"{name}: L={output.left_rpm:+.2f} RPM R={output.right_rpm:+.2f} RPM")
        output = controller.update(InputState(throttle, steering, False, last_input_at=1.04), 1.04)
        print(f"release-LB: L={output.left_rpm:+.2f} RPM R={output.right_rpm:+.2f} RPM")


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
    parser.add_argument("command", choices=("devices", "gamepad", "demo", "inspect", "check", "drive"))
    parser.add_argument("--config", help="configuration JSON (default: config.example.json; gamepad can run without it)")
    parser.add_argument("--device", help="gamepad event device (gamepad only; default: auto-detect)")
    parser.add_argument("--live", action="store_true", help="send drive commands (drive only)")
    parser.add_argument("--seconds", type=float, help="exit after this duration")
    args = parser.parse_args(argv)
    if args.live and args.command != "drive":
        parser.error("--live is only valid for drive")
    if args.device and args.command != "gamepad":
        parser.error("--device is only valid for gamepad")
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
        elif args.command == "check":
            pair = EscPair(config)
            try:
                print("Both ESCs: protocol, calibration, PWM-off, supply and R/L OK. No arm/run sent.")
            finally:
                for error in pair.close():
                    print(error, file=sys.stderr)
        else:
            run(config, live=args.live, seconds=args.seconds)
    except KeyboardInterrupt:
        return 130
    except (OSError, RuntimeError, ValueError, TypeError, KeyError, ImportError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0
