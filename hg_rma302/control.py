"""Hardware-independent mixer and operator interlocks. Wheel speeds are RPM."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class DriveSettings:
    max_current_a: float = 2.0
    min_speed_rpm: float = 8.0
    max_speed_rpm: float = 15.0
    deadzone: float = 0.12
    input_timeout_s: float = 0.5
    loop_timeout_s: float = 0.15
    left_sign: int = 1
    right_sign: int = 1
    acceleration_rpm_s: float = 20.0
    steering_gain: float = 1.5
    observer_retry_limit: int = 3
    observer_retry_delay_s: float = 2.5

    def __post_init__(self):
        for name in ("max_current_a", "min_speed_rpm", "max_speed_rpm", "acceleration_rpm_s", "steering_gain", "deadzone",
                     "input_timeout_s", "loop_timeout_s", "observer_retry_delay_s"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
        if not .1 <= self.max_current_a <= 2:
            raise ValueError("max_current_a must be in [0.1, 2]")
        if not 8 <= self.min_speed_rpm <= self.max_speed_rpm <= 25:
            raise ValueError("Require 8 <= min_speed_rpm <= max_speed_rpm <= 25")
        if not 1 <= self.acceleration_rpm_s <= 20:
            raise ValueError("acceleration_rpm_s must be in [1, 20]")
        if not .5 <= self.steering_gain <= 3:
            raise ValueError("steering_gain must be in [0.5, 3]")
        if type(self.observer_retry_limit) is not int or not 0 <= self.observer_retry_limit <= 5:
            raise ValueError("observer_retry_limit must be an integer in [0, 5]")
        if not 2.5 <= self.observer_retry_delay_s <= 10:
            raise ValueError("observer_retry_delay_s must be in [2.5, 10]")
        if not 0.01 <= self.deadzone < 0.5:
            raise ValueError("deadzone must be in [0.01, 0.5)")
        if not 0.1 <= self.input_timeout_s <= 1:
            raise ValueError("input_timeout_s must be in [0.1, 1.0]")
        if not 0.04 <= self.loop_timeout_s <= 0.2:
            raise ValueError("loop_timeout_s must be in [0.04, 0.2]")
        for sign in (self.left_sign, self.right_sign):
            if type(sign) is not int or sign not in (-1, 1):
                raise ValueError("Motor direction signs must be -1 or 1")


@dataclass(frozen=True)
class InputState:
    throttle: float = 0.0  # Up / forward is positive.
    steering: float = 0.0  # Right / clockwise is positive.
    deadman: bool = False
    estop: bool = False
    reset: bool = False
    last_input_at: float = -math.inf  # Monotonic event time, not polling time.
    connected: bool = True
    last_device_check_at: float = -math.inf  # Successful wired-device ioctl; not an input event.


@dataclass(frozen=True)
class Output:
    left_rpm: float = 0.0
    right_rpm: float = 0.0
    enabled: bool = False
    reason: str = "DISARMED"


def deadzone(value: float, width: float) -> float:
    value = max(-1.0, min(1.0, value))
    if abs(value) <= width:
        return 0.0
    return math.copysign((abs(value) - width) / (1 - width), value)


def mix(throttle: float, steering: float) -> tuple[float, float]:
    """Signed differential drive: up=forward, right=clockwise, X alone=pivot."""
    throttle = max(-1.0, min(1.0, throttle))
    steering = max(-1.0, min(1.0, steering))
    left, right = throttle + steering, throttle - steering
    scale = max(1.0, abs(left), abs(right))
    return left / scale, right / scale


class DriveController:
    def __init__(self, settings: DriveSettings, *, require_input_updates: bool = True):
        self.settings = settings
        self.require_input_updates = require_input_updates
        self.armed = False
        self.motion_this_hold = False
        self.ready = False
        self.estop_latched = False
        self.previous_deadman = False
        self.previous_reset = False
        self.previous_time: float | None = None
        self.output = Output()
        self.last_signs = [0, 0]
        self.last_motion_at = -math.inf
        self.direction_wait_until: float | None = None

    def _stop(self, reason: str) -> Output:
        if self.motion_this_hold and self.previous_time is not None:
            # A cold restart dwell begins at PWM-off, including when the stick
            # had already been neutral during this LB hold.
            self.last_motion_at = self.previous_time
        self.armed = False
        self.motion_this_hold = False
        self.ready = False
        self.direction_wait_until = None
        self.output = Output(reason=reason)
        return self.output

    def _wait_for_direction(self) -> Output:
        # Keep the operator's live LB authorization, but explicitly stop BOTH
        # wheels. Faults and LB release still use _stop() and revoke it.
        self.output = Output(reason="DIRECTION_CHANGE_WAIT")
        return self.output

    def update(self, sample: InputState, now: float) -> Output:
        cfg = self.settings
        dt = 0.0 if self.previous_time is None else now - self.previous_time
        self.previous_time = now
        pressed = sample.deadman and not self.previous_deadman
        reset_pressed = sample.reset and not self.previous_reset
        self.previous_deadman = sample.deadman
        self.previous_reset = sample.reset
        # Latch stop even when another fault or stale input is present.
        if sample.estop:
            self.estop_latched = True
        if not sample.connected:
            return self._stop("DISCONNECTED")
        if not all(math.isfinite(x) for x in (sample.throttle, sample.steering, now)):
            return self._stop("INVALID_INPUT")
        checked_at = sample.last_input_at if self.require_input_updates else sample.last_device_check_at
        age = now - checked_at
        if not math.isfinite(age) or not 0 <= age < cfg.input_timeout_s:
            return self._stop("INPUT_TIMEOUT" if self.require_input_updates else "GAMEPAD_POLL_TIMEOUT")
        if not 0 <= dt <= cfg.loop_timeout_s:
            return self._stop("LOOP_TIMEOUT")
        neutral = abs(sample.throttle) <= cfg.deadzone and abs(sample.steering) <= cfg.deadzone
        if self.estop_latched:
            if reset_pressed and not sample.estop and not sample.deadman and neutral:
                self.estop_latched = False
            else:
                return self._stop("STOP_LATCHED")
        if not sample.deadman:
            result = self._stop("READY" if neutral else "CENTER_STICK")
            self.ready = neutral
            return result
        if pressed:
            self.armed = self.ready and neutral
            self.ready = False
        if not self.armed:
            return self._stop("RELEASE_LB_AND_CENTER")
        if self.direction_wait_until is not None:
            if now < self.direction_wait_until:
                return self._wait_for_direction()
            self.direction_wait_until = None
        left, right = mix(deadzone(sample.throttle, cfg.deadzone),
                          cfg.steering_gain * deadzone(sample.steering, cfg.deadzone))
        # Apply the minimum-start speed to the pair's scale, not each wheel:
        # lifting the inner wheel to 8 RPM destroyed the requested turn ratio.
        peak = max(abs(left), abs(right))
        scale = (cfg.min_speed_rpm + peak * (cfg.max_speed_rpm-cfg.min_speed_rpm)) / peak if peak else 0
        def speed(value):
            rpm = value * scale
            return rpm if abs(rpm) >= cfg.min_speed_rpm else 0.0
        targets = (speed(left) * cfg.left_sign, speed(right) * cfg.right_sign)
        signs = [0 if value == 0 else (1 if value > 0 else -1) for value in targets]
        if not self.motion_this_hold and any(old and new and old != new for old, new in zip(self.last_signs, signs)):
            # A fresh LB authorization follows an uncontrolled PWM-off stop.
            # In the same hold, protocol 4 handles deceleration/reversal. Start the
            # full host wait at this stop request, allowing 0.5 s for UART.
            # This is not a measured standstill. Use the latest stick target
            # after waiting, never a stored movement from before the wait.
            if now - self.last_motion_at < 2.5:
                self.direction_wait_until = now + 2.5
                return self._wait_for_direction()
        self.output = Output(*targets, enabled=True, reason="DRIVING")
        for index, value in enumerate(targets):
            if value:
                self.motion_this_hold = True
                self.last_signs[index] = 1 if value > 0 else -1
                self.last_motion_at = now
        return self.output
