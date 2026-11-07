"""Independent wheel commands and operator interlocks. Wheel speeds are RPM."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class DriveSettings:
    max_current_a: float = 2.0
    min_speed_rpm: float = 8.0
    max_speed_rpm: float = 15.0
    stick_max_speed_rpm: float | None = None
    deadzone: float = 0.12
    input_timeout_s: float = 0.5
    loop_timeout_s: float = 0.15
    left_sign: int = 1
    right_sign: int = 1
    acceleration_rpm_s: float = 20.0
    steering_gain: float = 1.5  # Retained for old JSON; joystick steering uses joystick_turn_ratio.
    control_mode: str = "tank"
    steering_deadzone: float = 0.2
    joystick_turn_ratio: float = 0.5
    dpad_drive_ratio: float = 0.5
    dpad_turn_ratio: float = 0.25
    dpad_smoothing_s: float = 0.15
    stick_smoothing_s: float = 0.15
    observer_retry_limit: int = 3
    observer_retry_delay_s: float = 2.5

    def __post_init__(self):
        if self.control_mode not in ("tank", "joystick"):
            raise ValueError("control_mode must be tank or joystick")
        for name in ("max_current_a", "min_speed_rpm", "max_speed_rpm", "acceleration_rpm_s", "steering_gain", "steering_deadzone", "joystick_turn_ratio", "dpad_drive_ratio", "dpad_turn_ratio", "dpad_smoothing_s", "stick_smoothing_s", "deadzone",
                     "input_timeout_s", "loop_timeout_s", "observer_retry_delay_s"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
        if not .1 <= self.max_current_a <= 2:
            raise ValueError("max_current_a must be in [0.1, 2]")
        if not 8 <= self.min_speed_rpm <= self.max_speed_rpm <= 100:
            raise ValueError("Require 8 <= min_speed_rpm <= max_speed_rpm <= 100")
        value = self.stick_max_speed_rpm
        if value is not None and (isinstance(value, bool) or not isinstance(value, (int, float))
                                  or not math.isfinite(value)
                                  or not self.min_speed_rpm <= value <= self.max_speed_rpm):
            raise ValueError("stick_max_speed_rpm must be null or in [min_speed_rpm, max_speed_rpm]")
        if not 1 <= self.acceleration_rpm_s <= 30:
            raise ValueError("acceleration_rpm_s must be in [1, 30]")
        if not .5 <= self.steering_gain <= 3:
            raise ValueError("steering_gain must be in [0.5, 3]")
        if not .01 <= self.steering_deadzone < .8:
            raise ValueError("steering_deadzone must be in [0.01, 0.8)")
        for name in ("dpad_drive_ratio", "dpad_turn_ratio", "joystick_turn_ratio"):
            if not 0 < getattr(self, name) <= 1:
                raise ValueError(f"{name} must be in (0, 1]")
        for name in ("dpad_smoothing_s", "stick_smoothing_s"):
            if not 0 <= getattr(self, name) <= 1:
                raise ValueError(f"{name} must be in [0, 1]")
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

    @property
    def stick_full_scale_rpm(self) -> float:
        return self.max_speed_rpm if self.stick_max_speed_rpm is None else self.stick_max_speed_rpm


@dataclass(frozen=True)
class InputState:
    left_stick: float = 0.0  # Vertical axes: up / forward is positive.
    right_stick: float = 0.0
    dpad_x: int = 0  # Left=-1, right=+1.
    dpad_y: int = 0  # Down=-1, up=+1.
    estop: bool = False
    reset: bool = False
    last_input_at: float = -math.inf  # Monotonic event time, not polling time.
    connected: bool = True
    last_device_check_at: float = -math.inf  # Successful wired-device ioctl; not an input event.
    left_x: float = 0.0
    right_x: float = 0.0
    mode_toggle: bool = False  # Xbox/Guide button, rising edge only.


@dataclass(frozen=True)
class Output:
    left_rpm: float = 0.0
    right_rpm: float = 0.0
    enabled: bool = False
    reason: str = "DISARMED"
    synchronize: bool = False  # D-pad commands move both wheels together.
    control_mode: str = "tank"


def deadzone(value: float, width: float) -> float:
    value = max(-1.0, min(1.0, value))
    if abs(value) <= width:
        return 0.0
    return math.copysign((abs(value) - width) / (1 - width), value)


def wheel_speed(value: float, settings: DriveSettings) -> float:
    """D-pad percentages use the global limit and the sensorless speed floor."""
    if value == 0:
        return 0.0
    return math.copysign(max(settings.min_speed_rpm,
                             min(1.0, abs(value)) * settings.max_speed_rpm), value)


def stick_speed(value: float, settings: DriveSettings) -> float:
    """Map the entire active stick travel linearly over the usable RPM range.

    Clamping max_rpm * travel to min_rpm wastes travel at the bottom. Choosing
    an unreachable full-scale speed also wastes travel at the top. Keep the
    stick's calibrated full-scale separate from the ESC/D-pad global limit.
    The deadzone still stops; sensorless startup below min_rpm is not bypassed.
    """
    travel = deadzone(value, settings.deadzone)
    if travel == 0:
        return 0.0
    speed = settings.min_speed_rpm + abs(travel)*(settings.stick_full_scale_rpm-settings.min_speed_rpm)
    return math.copysign(speed, travel)


def tank_targets(sample: InputState, cfg: DriveSettings) -> tuple[float, float]:
    # D-pad takes priority while held. Ambiguous diagonals command a stop.
    if sample.dpad_x and sample.dpad_y:
        return 0.0, 0.0
    if sample.dpad_y:
        left = right = sample.dpad_y * cfg.dpad_drive_ratio
    elif sample.dpad_x:
        left, right = sample.dpad_x * cfg.dpad_turn_ratio, -sample.dpad_x * cfg.dpad_turn_ratio
    else:
        return stick_speed(sample.left_stick, cfg)*cfg.left_sign, stick_speed(sample.right_stick, cfg)*cfg.right_sign
    return wheel_speed(left, cfg) * cfg.left_sign, wheel_speed(right, cfg) * cfg.right_sign


def controls_neutral(sample: InputState, cfg: DriveSettings) -> bool:
    return (abs(sample.left_stick)<=cfg.deadzone and abs(sample.right_stick)<=cfg.deadzone
            and abs(sample.left_x)<=cfg.steering_deadzone and abs(sample.right_x)<=cfg.steering_deadzone
            and not sample.dpad_x and not sample.dpad_y)


def supported_pair(left: float, right: float, minimum: float, maximum: float) -> tuple[float, float]:
    """Preserve steering ratios on saturation and enforce the ESC speed bounds."""
    scale = max(1.0, abs(left) / maximum, abs(right) / maximum)

    def bounded(value: float) -> float:
        if abs(value) < 1e-9:
            return 0.0
        # Even value / (value / maximum) can round above maximum (e.g.
        # 100.00000000000001 RPM). Bound AFTER scaling so a valid turn never
        # trips EscPair's range check; the protocol limits remain unchanged.
        speed = min(maximum, max(minimum, abs(value) / scale))
        return math.copysign(speed, value)

    return bounded(left), bounded(right)


def joystick_targets(sample: InputState, cfg: DriveSettings) -> tuple[float,float]:
    if sample.dpad_x or sample.dpad_y:
        return tank_targets(sample,cfg)
    forward=stick_speed(sample.left_stick,cfg)
    # Left horizontal steers only while translating. Right horizontal rotates
    # on the spot at neutral throttle, or adds yaw while moving. Right vertical is unused.
    steer=abs(forward)*deadzone(sample.left_x,cfg.steering_deadzone)
    spin=deadzone(sample.right_x,cfg.steering_deadzone)*cfg.stick_full_scale_rpm*cfg.joystick_turn_ratio
    left,right=supported_pair(forward+steer+spin,forward-steer-spin,cfg.min_speed_rpm,cfg.stick_full_scale_rpm)
    return left*cfg.left_sign,right*cfg.right_sign


class DriveController:
    def __init__(self, settings: DriveSettings, *, require_input_updates: bool = True):
        self.settings = settings
        self.mode = settings.control_mode
        self.previous_mode = False
        self.input_seen = False
        self.require_input_updates = require_input_updates
        self.ready = False
        self.motion_since_stop = False
        self.estop_latched = False
        self.previous_reset = False
        self.previous_time: float | None = None
        self.output = Output()
        self.last_signs = [0, 0]
        self.last_motion_at = -math.inf
        self.direction_wait_until: float | None = None

    def _stop(self, reason: str) -> Output:
        if self.motion_since_stop and self.previous_time is not None:
            self.last_motion_at = self.previous_time
        self.motion_since_stop = False
        self.ready = False
        self.direction_wait_until = None
        self.output = Output(reason=reason,control_mode=self.mode)
        return self.output

    def update(self, sample: InputState, now: float) -> Output:
        cfg = self.settings
        dt = 0.0 if self.previous_time is None else now - self.previous_time
        self.previous_time = now
        reset_pressed = sample.reset and not self.previous_reset
        self.previous_reset = sample.reset
        mode_pressed = self.input_seen and sample.mode_toggle and not self.previous_mode
        self.input_seen = True
        self.previous_mode = sample.mode_toggle
        if sample.estop:
            self.estop_latched = True
        if not sample.connected:
            return self._stop("DISCONNECTED")
        if (not all(math.isfinite(x) for x in (sample.left_stick, sample.right_stick, sample.left_x, sample.right_x, now))
                or any(type(x) is not int or x not in (-1, 0, 1) for x in (sample.dpad_x, sample.dpad_y))):
            return self._stop("INVALID_INPUT")
        checked_at = sample.last_input_at if self.require_input_updates else sample.last_device_check_at
        age = now - checked_at
        if not math.isfinite(age) or not 0 <= age < cfg.input_timeout_s:
            return self._stop("INPUT_TIMEOUT" if self.require_input_updates else "GAMEPAD_POLL_TIMEOUT")
        if not 0 <= dt <= cfg.loop_timeout_s:
            return self._stop("LOOP_TIMEOUT")
        neutral = controls_neutral(sample,cfg)
        if self.estop_latched:
            if reset_pressed and not sample.estop and neutral:
                self.estop_latched = False
            else:
                return self._stop("STOP_LATCHED")
        if mode_pressed:
            self.mode = "joystick" if self.mode == "tank" else "tank"
            return self._stop("MODE_CHANGED")
        if not self.ready:
            if not neutral:
                return self._stop("CENTER_CONTROLS")
            self.ready = True
            self.output = Output(reason="READY",control_mode=self.mode)
            return self.output

        targets = (joystick_targets if self.mode=="joystick" else tank_targets)(sample,cfg)
        if not any(targets):
            self.direction_wait_until = None
            # Keep the speed session alive: rpm 0 brakes to rest, and the ESC
            # handles the next direction without an uncontrolled PWM-off stop.
            self.output = Output(enabled=self.motion_since_stop,
                                 reason="DPAD_DIAGONAL_STOP" if sample.dpad_x and sample.dpad_y else "READY",control_mode=self.mode)
            return self.output
        if self.direction_wait_until is not None:
            if now < self.direction_wait_until:
                self.output = Output(reason="DIRECTION_CHANGE_WAIT",control_mode=self.mode)
                return self.output
            self.direction_wait_until = None
        signs = [0 if value == 0 else (1 if value > 0 else -1) for value in targets]
        if not self.motion_since_stop and any(old and new and old != new for old, new in zip(self.last_signs, signs)):
            # After a hard stop (B/input loss), allow the existing ESC cold
            # reversal dwell. Normal neutral/reversal uses controlled braking.
            if now - self.last_motion_at < 2.5:
                self.direction_wait_until = now + 2.5
                self.output = Output(reason="DIRECTION_CHANGE_WAIT",control_mode=self.mode)
                return self.output
        self.output = Output(*targets, enabled=True, reason="DRIVING",
                             synchronize=bool(sample.dpad_x or sample.dpad_y) or
                                 (self.mode=='joystick' and math.isclose(abs(targets[0]),abs(targets[1]))),
                             control_mode=self.mode)
        for index, value in enumerate(targets):
            if value:
                self.motion_since_stop = True
                self.last_signs[index] = 1 if value > 0 else -1
                self.last_motion_at = now
        return self.output

    def inhibit(self, reason: str) -> Output:
        """Power/link interlock: require neutral again before any new movement."""
        return self._stop(reason)
