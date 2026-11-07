"""Runtime host settings for the optional RP2040 power/panel controller."""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class PowerSettings:
    enabled: bool = False
    serial: str | None = None
    port: str = ''
    telemetry_timeout_s: float = .5
    usb_min_v: float = 3.5
    usb_max_v: float = 6.5

    def __post_init__(self):
        if type(self.enabled) is not bool:
            raise ValueError('power.enabled must be boolean')
        if self.serial is not None and (not isinstance(self.serial,str) or not self.serial or
                self.serial != self.serial.strip() or not self.serial.isascii()):
            raise ValueError('power.serial must be a complete USB serial number')
        if not isinstance(self.port,str):
            raise ValueError('power.port must be a device path')
        for name in ('telemetry_timeout_s','usb_min_v','usb_max_v'):
            v=getattr(self,name)
            if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v):
                raise ValueError(f'power.{name} must be finite')
        if not .2 <= self.telemetry_timeout_s <= .6:
            raise ValueError('power.telemetry_timeout_s must be in [0.2, 0.6]')
        if not 0 < self.usb_min_v < self.usb_max_v < 7:
            raise ValueError('Require 0 < power.usb_min_v < power.usb_max_v < 7')
        if self.enabled and not (self.serial or self.port):
            raise ValueError('Set power.serial (recommended) or power.port')


def supply_state(millivolts: int, low: int, high: int, settings: PowerSettings) -> str:
    if millivolts > high:
        return 'OVERVOLTAGE'
    if low <= millivolts <= high:
        return 'READY'
    if settings.usb_min_v*1000 <= millivolts <= settings.usb_max_v*1000:
        return 'USB_ONLY'
    return 'WAIT_POWER'
