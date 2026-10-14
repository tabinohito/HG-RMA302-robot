"""Read-only gamepad diagnostics, independent of the drive/ESC state machines."""

from collections import deque
import errno
import sys
import time

from .gamepad import _evdev, list_gamepads, validate_gamepad


BUTTONS = {
    304: "A", 305: "B", 307: "X", 308: "Y", 310: "LB", 311: "RB",
    314: "BACK", 315: "START", 316: "GUIDE", 317: "L3", 318: "R3",
}
AXES = {
    0: "左スティック X", 1: "左スティック Y", 2: "LT",
    3: "右スティック X", 4: "右スティック Y", 5: "RT",
    16: "十字キー X", 17: "十字キー Y",
}


def _code_name(evdev, event_type, code):
    name = evdev.ecodes.bytype.get(event_type, {}).get(code, str(code))
    return "/".join(name) if isinstance(name, (list, tuple)) else str(name)


def monitor(path=None, *, seconds=None):
    evdev = _evdev()
    if path is None:
        devices = list_gamepads()
        if not devices:
            raise RuntimeError("対応ゲームパッドが見つかりません。USB接続と、F310/F710のX/DスイッチのX位置を確認してください。")
        if len(devices) != 1:
            raise RuntimeError("対応ゲームパッドが複数あります。--device /dev/input/eventN で選択してください。")
        path = devices[0]["path"]
    device = evdev.InputDevice(path)
    events = frames = dropped = 0
    last_event_at = None
    recent = deque(maxlen=3)
    terminal = sys.stdout.isatty()
    try:
        caps = validate_gamepad(device)
        try:
            device.grab()
        except OSError as exc:
            if exc.errno == errno.EBUSY:
                raise RuntimeError("ゲームパッドは別のプログラムが使用中です。hg-cart drive等をCtrl-Cで終了してから実行してください。") from exc
            raise
        key_codes = sorted(caps.get(evdev.ecodes.EV_KEY, []))
        axis_codes = sorted(caps.get(evdev.ecodes.EV_ABS, []))
        started = time.monotonic()
        last_render = -float("inf")
        last_render_events = -1
        if terminal:
            print("\033[2J\033[H", end="", flush=True)
        while seconds is None or time.monotonic() - started < seconds:
            try:
                for event in device.read():
                    if event.type == evdev.ecodes.EV_SYN:
                        if event.code == evdev.ecodes.SYN_REPORT:
                            frames += 1
                        elif event.code == evdev.ecodes.SYN_DROPPED:
                            dropped += 1
                        continue
                    if event.type not in (evdev.ecodes.EV_KEY, evdev.ecodes.EV_ABS):
                        continue
                    events += 1
                    last_event_at = time.monotonic()
                    if event.type == evdev.ecodes.EV_KEY:
                        label = BUTTONS.get(event.code, _code_name(evdev, event.type, event.code))
                        value = {0: "離した", 1: "押した", 2: "リピート"}.get(event.value, str(event.value))
                    else:
                        label = AXES.get(event.code, _code_name(evdev, event.type, event.code))
                        value = str(event.value)
                    recent.append(f"{last_event_at - started:7.2f}s  {label}: {value}")
            except BlockingIOError:
                pass

            now = time.monotonic()
            # ioctl returns the kernel's cached state, including held buttons.
            # It is useful for display but is NOT proof of new input reports.
            if now - last_render >= .1 and (terminal or events != last_render_events or now - last_render >= 1):
                keys = set(device.active_keys())
                axes = {code: device.absinfo(code) for code in axis_codes}
                age = "未受信" if last_event_at is None else f"{now - last_event_at:.2f}秒前"
                lines = [
                    "Gamepad 入力テスト / モーター出力なし / Ctrl-Cで終了",
                    f"認識: {device.name}  {device.path}  USB={device.info.vendor:04x}:{device.info.product:04x}",
                    f"受信イベント数: {events}  フレーム数: {frames}  最終入力: {age}  取りこぼし: {dropped}",
                    "ボタン: 1=押下 / 0=解放（L3/R3=スティック押し込み）",
                ]
                labels = [f"{BUTTONS.get(code, _code_name(evdev, evdev.ecodes.EV_KEY, code))}={int(code in keys)}" for code in key_codes]
                lines.extend("  " + "  ".join(labels[i:i+6]) for i in range(0, len(labels), 6))
                if 16 in axes and 17 in axes:
                    x, y = axes[16].value, axes[17].value
                    lines.append(f"  十字キー: 上={int(y < 0)} 下={int(y > 0)} 左={int(x < 0)} 右={int(x > 0)}")
                lines.append("軸: 生値 / 正規化値（スティックは左・上が負、右・下が正）")
                for code, info in axes.items():
                    normalized = (info.value - info.min) / max(1, info.max - info.min)
                    if code not in (2, 5):
                        normalized = normalized * 2 - 1
                    label = AXES.get(code, _code_name(evdev, evdev.ecodes.EV_ABS, code))
                    lines.append(f"  {label}: {info.value:6d} / {normalized:+.3f}  [{info.min}..{info.max}]")
                lines.extend(["最近受信した入力:", *(recent or ["  まだ入力イベントがありません。"]),
                              "状態表示はOSの保持値です。操作信号の受信はイベント数で確認してください。",
                              "LBの保持は不要です。操作して受信イベント数が増えるか確認してください。"])
                text = "\n".join(lines)
                print(("\033[H" + text + "\033[J") if terminal else text + "\n", flush=True)
                last_render, last_render_events = now, events
            time.sleep(.02)
    finally:
        device.close()
        print(f"入力テスト終了: 受信イベント数={events}, フレーム数={frames}", flush=True)
