"""Linux evdev XInput reader; never treats cached state as a heartbeat."""

from dataclasses import replace
import fcntl
import struct
import time

from .control import InputState

# Linux input-event-codes.h, stable evdev ABI.
EV_SYN, EV_KEY, EV_ABS = 0, 1, 3
SYN_REPORT, SYN_DROPPED = 0, 3
ABS_X, ABS_Y = 0, 1
BTN_B, BTN_LB, BTN_START = 305, 310, 315
SUPPORTED_GAMEPADS = {
    (0x046D, 0xC21D): "Logitech F310 (XInput)",
    (0x046D, 0xC21F): "Logitech F710 (XInput)",
    # The user's replacement controller reports this ID on the USB bus.
    (0x045E, 0x028E): "Xbox 360 compatible (XInput)",
}
WIRED_GAMEPAD_IDS = frozenset({(0x046D, 0xC21D), (0x045E, 0x028E)})
EVIOCSCLOCKID = 0x400445A0  # _IOW('E', 0xa0, int), linux/input.h


def normalize(value: int, minimum: int, maximum: int) -> float:
    if maximum <= minimum:
        raise ValueError("Invalid joystick axis range")
    return max(-1.0, min(1.0, 2 * (value - minimum) / (maximum - minimum) - 1))


class EventDecoder:
    """Publish whole SYN_REPORT frames, preserving button edges within a batch."""

    def __init__(self, axes: dict[int, tuple[int, int, int]], keys=()):
        self.axes = axes
        self.state = InputState(
            steering=normalize(*axes[ABS_X]),
            throttle=-normalize(*axes[ABS_Y]),
            deadman=BTN_LB in keys, estop=BTN_B in keys, reset=BTN_START in keys,
        )
        self.changed = False
        self.changed_at = self.state.last_input_at

    def feed(self, event_type: int, code: int, value: int, timestamp: float):
        if event_type == EV_SYN and code == SYN_DROPPED:
            # Terminate the run instead of risking a lost release/stop edge.
            raise OSError("Gamepad SYN_DROPPED: input queue overflow; restart with LB released")
        field = None
        if event_type == EV_ABS and code in (ABS_X, ABS_Y):
            _, low, high = self.axes[code]
            value = normalize(value, low, high)
            field = "steering" if code == ABS_X else "throttle"
            if code == ABS_Y:
                value = -value
        elif event_type == EV_KEY and value in (0, 1):
            field = {BTN_LB: "deadman", BTN_B: "estop", BTN_START: "reset"}.get(code)
            value = bool(value)
        if field is not None:
            self.state = replace(self.state, **{field: value})
            self.changed = True
            self.changed_at = timestamp
        if event_type == EV_SYN and code == SYN_REPORT and self.changed:
            self.state = replace(self.state, last_input_at=self.changed_at)
            self.changed = False
            return self.state
        return None


def _evdev():
    try:
        import evdev
    except ImportError as exc:
        raise RuntimeError("Install hardware dependencies: pip install -e .") from exc
    return evdev


def list_gamepads() -> list[dict]:
    evdev = _evdev()
    result = []
    for path in sorted(evdev.list_devices()):
        device = evdev.InputDevice(path)
        try:
            model = SUPPORTED_GAMEPADS.get((device.info.vendor, device.info.product))
            if model is not None:
                result.append({"path": path, "name": device.name, "phys": device.phys,
                               "model": model, "usb_id": f"{device.info.vendor:04x}:{device.info.product:04x}"})
        finally:
            device.close()
    return result


def validate_gamepad(device):
    info = device.info
    if (info.vendor, info.product) not in SUPPORTED_GAMEPADS:
        raise ValueError(f"Unsupported gamepad USB {info.vendor:04x}:{info.product:04x}; "
                         "use F310/F710 in X mode or the supported Xbox 360-compatible controller")
    caps = device.capabilities(absinfo=False)
    if not {ABS_X, ABS_Y} <= set(caps.get(EV_ABS, [])) or not {
        BTN_B, BTN_LB, BTN_START
    } <= set(caps.get(EV_KEY, [])):
        raise ValueError("Gamepad axis/button mapping is unsupported")
    return caps


class Gamepad:
    def __init__(self, path: str | None = None, *, event_timeout_s: float = .5):
        self.event_timeout_s = event_timeout_s
        evdev = _evdev()
        if path is None:
            devices = list_gamepads()
            if len(devices) != 1:
                raise RuntimeError(f"Expected one supported XInput gamepad, found {len(devices)}; "
                                   "set gamepad path; F310/F710 switch must be in X mode")
            path = devices[0]["path"]
        self.device = evdev.InputDevice(path)
        try:
            validate_gamepad(self.device)
            self.requires_event_updates = (self.device.info.vendor, self.device.info.product) not in WIRED_GAMEPAD_IDS
            self.last_device_check_at = -float("inf")
            fcntl.ioctl(self.device.fd, EVIOCSCLOCKID, struct.pack("i", time.CLOCK_MONOTONIC))
            self.device.grab()
            axes = {}
            for code in (ABS_X, ABS_Y):
                info = self.device.absinfo(code)
                axes[code] = (info.value, info.min, info.max)
            self.decoder = EventDecoder(axes, self.device.active_keys())
        except BaseException:
            self.device.close()
            raise

    @property
    def state(self) -> InputState:
        return replace(self.decoder.state, last_device_check_at=self.last_device_check_at)

    def read(self) -> list[InputState]:
        frames = []
        try:
            # Bounded work per tick; old kernel timestamps cannot refresh a command.
            for event in self.device.read():
                frame = self.decoder.feed(event.type, event.code, event.value, event.timestamp())
                if frame is not None:
                    frames.append(frame)
        except BlockingIOError:
            pass
        if not self.requires_event_updates:
            # A wired pad need not emit events while the controls are held.
            # EVIOCGKEY fails on a disconnected evdev device. Keep this poll's
            # timestamp separate from the timestamps of actual input events.
            self.device.active_keys()
            self.last_device_check_at = time.monotonic()
            if any(not 0 <= self.last_device_check_at - frame.last_input_at < self.event_timeout_s for frame in frames):
                raise OSError("Gamepad input queue is stale; restart with LB released")
            frames = [replace(frame, last_device_check_at=self.last_device_check_at) for frame in frames]
        return frames

    def close(self):
        self.device.close()  # Also releases EVIOCGRAB.
