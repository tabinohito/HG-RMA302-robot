import json
import math
from dataclasses import dataclass, fields
from pathlib import Path

from .control import DriveSettings


@dataclass(frozen=True)
class MotorSettings:
    resistance_ohm: float | None = None
    inductance_h: float | None = None

    def __post_init__(self):
        if (self.resistance_ohm is None) != (self.inductance_h is None):
            raise ValueError("Specify both resistance_ohm and inductance_h, or neither")
        if self.resistance_ohm is not None:
            for value, low, high in ((self.resistance_ohm, .001, 100),
                                     (self.inductance_h, 1e-6, .1)):
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
                    raise ValueError("Invalid measured motor R/L")


@dataclass(frozen=True)
class Config:
    drive: DriveSettings
    gamepad: str | None = None
    left_port: str = ""
    right_port: str = ""
    baudrate: int = 115200
    left_motor: MotorSettings = MotorSettings()
    right_motor: MotorSettings = MotorSettings()
    left_serial: str | None = None
    right_serial: str | None = None
    connect_timeout_s: float = 5.0

    def __post_init__(self):
        for key in ("left_serial", "right_serial"):
            value = getattr(self, key)
            if value is not None and (not isinstance(value, str) or not value or
                                      value != value.strip() or not value.isascii()):
                raise ValueError(f"{key} must be a complete USB serial number or null")
        if self.left_serial is not None and self.left_serial == self.right_serial:
            raise ValueError("Left and right ESC must use different USB serial numbers")
        if any(not isinstance(port, str) for port in (self.left_port, self.right_port)):
            raise ValueError("Serial ports must be strings")
        value = self.connect_timeout_s
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 <= value <= 30:
            raise ValueError("connect_timeout_s must be in [0, 30]")


def load_config(path: str) -> Config:
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("Configuration must be a JSON object")
    allowed = {f.name for f in fields(Config)}
    if raw.keys() - allowed:
        raise ValueError(f"Unknown configuration keys: {sorted(raw.keys() - allowed)}")
    drive = raw.pop("drive", {})
    if not isinstance(drive, dict):
        raise ValueError("drive must be a JSON object")
    for key in ("left_motor", "right_motor"):
        motor = raw.pop(key, {})
        if not isinstance(motor, dict):
            raise ValueError(f"{key} must be a JSON object")
        raw[key] = MotorSettings(**motor)
    config = Config(drive=DriveSettings(**drive), **raw)
    if config.gamepad is not None and not isinstance(config.gamepad, str):
        raise ValueError("gamepad must be a device path or null")
    if type(config.baudrate) is not int or config.baudrate != 115200:
        raise ValueError("The supplied firmware uses baudrate 115200")
    return config
