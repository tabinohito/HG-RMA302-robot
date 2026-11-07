"""Smooth wheel acceleration and coordinate D-pad speeds using ESC telemetry."""

import math


class SmoothTarget:
    """Slew-limit before low-pass filtering; both stages keep speed in bounds.

    Shaping the final per-wheel target also covers feedback corrections, which
    previously bypassed the common acceleration limit at every telemetry frame.
    Zero resets the profile; disable resets all profiles. A sign change is passed
    straight to the ESC's controlled reversal at minimum speed.
    """

    def __init__(self, settings):
        self.settings = settings
        self.direction = None
        self.ramp = self.value = settings.min_speed_rpm
        self.rate = 0.0

    def update(self, target, dt, *, smoothing_s=None, decelerate_now=False):
        if target == 0:
            self.direction = None
            self.ramp = self.value = self.settings.min_speed_rpm
            self.rate = 0.0
            return 0.0
        direction = 1 if target > 0 else -1
        if direction != self.direction:
            self.direction = direction
            self.ramp = self.value = self.settings.min_speed_rpm
            self.rate = 0.0
            return direction*self.value
        goal = abs(target)
        if decelerate_now and goal < self.value:
            # Operator reductions bypass smoothing and cancel all filter lead.
            # The ESC receives the lower target now and controls the braking.
            self.ramp = self.value = goal
            self.rate = 0.0
            return target
        # If a new lower/higher target crosses the current output, discard the
        # old filter lead so it cannot keep accelerating in the old sense.
        if (goal-self.value)*(self.ramp-self.value) < 0:
            self.ramp = self.value
        step = self.settings.acceleration_rpm_s*dt
        self.ramp += max(-step, min(step, goal-self.ramp))
        tau = self.settings.dpad_smoothing_s if smoothing_s is None else smoothing_s
        alpha = -math.expm1(-dt/tau) if tau else 1.0
        change = max(-step, min(step, alpha*(self.ramp-self.value)))
        self.value += change
        self.rate = change/dt if dt else 0.0
        return direction*self.value


class WheelSync:
    # ESC telemetry is normally 10 Hz. Never use a stale/startup speed as proof
    # that a wheel is ready to accelerate. Existing UART timeouts still apply.
    maximum_status_age_s = .25
    speed_margin_rpm = 2.0
    difference_gain = 1.0
    difference_deadband_rpm = .25

    def __init__(self, settings):
        self.settings = settings
        self.reset()

    def reset(self):
        self.profiles = [SmoothTarget(self.settings), SmoothTarget(self.settings)]
        self.directions = None
        self.previous_time = None
        self.speed = self.settings.min_speed_rpm
        self.reason = None
        self.targets = (0.0, 0.0)
        self.requested = (0.0, 0.0)

    @staticmethod
    def _sign(value):
        return 1 if value > 0 else -1

    def _closed_speed(self, wheel, direction, now):
        status = wheel.last_status or {}
        if (wheel.phase != 'running' or status.get('state') != 'closed'
                or status.get('fault') != 'none' or status.get('MOE') != '1'
                or status.get('speed_mode') != '1'
                or not 0 <= now-wheel.last_status_at < self.maximum_status_age_s):
            return None
        try:
            requested = int(status['req_mrpm']) * direction
            measured = int(status['est_mrpm']) * direction / 1000
            if (int(status['uart_direction']) != direction or requested <= 0
                    or (wheel.last_sent_rpm or 0)*direction <= 0 or measured <= 0):
                return None
        except (KeyError, ValueError, TypeError):
            return None
        return measured

    def update(self, output, wheels, now, *, enabled):
        requested = (output.left_rpm, output.right_rpm)
        if not enabled:
            self.reset()
            return self.targets
        dt = (0.0 if self.previous_time is None else
              max(0.0, min(now-self.previous_time, self.settings.loop_timeout_s)))
        self.previous_time = now
        previous_requested, self.requested = self.requested, requested
        if (not output.synchronize or not all(requested)
                or not math.isclose(abs(requested[0]), abs(requested[1]), abs_tol=.001)):
            # Independent sticks retain independent profiles, including across
            # D-pad/stick switches. Never wait for the other wheel to start.
            self.directions = None
            self.speed = self.settings.min_speed_rpm
            self.reason = None
            targets = []
            for index, (profile, target) in enumerate(zip(self.profiles, requested)):
                if target and (index >= len(wheels) or
                        self._closed_speed(wheels[index], self._sign(target), now) is None):
                    target = self._sign(target)*self.settings.min_speed_rpm
                    self.reason = 'WAITING_FOR_WHEEL'
                targets.append(profile.update(target, dt, smoothing_s=self.settings.stick_smoothing_s,
                                              decelerate_now=True))
            self.targets = tuple(targets)
            return self.targets

        directions = tuple(self._sign(value) for value in requested)
        measured = [self._closed_speed(wheel, direction, now)
                    for wheel, direction in zip(wheels, directions)]
        ready = len(measured) == 2 and all(value is not None for value in measured)
        maximum = abs(requested[0])
        minimum = self.settings.min_speed_rpm
        if self.directions != directions:
            # Preserve current profile speeds on a mode switch; a wheel that
            # reverses still resets to the ESC's bounded startup speed.
            self.speed = min(maximum, *(p.value if p.direction == d else minimum
                                       for p, d in zip(self.profiles, directions)))
            self.directions = directions

        if not ready:
            # Both ESCs acquire the observer at their configured startup speed.
            # The early wheel stays at minimum speed until its peer is CLOSED,
            # including the peer's blend and controlled reversal stages.
            self.speed = minimum
            self.reason = 'WAITING_FOR_WHEELS'
            magnitudes = (minimum, minimum)
        else:
            slowest = min(measured)
            # Ramp a common reference; don't keep asking for an unreachable
            # 50 RPM while one wheel is already voltage/current limited.
            # Compensate the filter's lag only while BOTH targets are actually
            # accelerating. Otherwise adding a smoothing filter to this feedback
            # ceiling unnecessarily cuts the sustained acceleration in half.
            lead = max(0.0, min(p.rate for p in self.profiles))*self.settings.dpad_smoothing_s
            ceiling = min(maximum, max(minimum, slowest+self.speed_margin_rpm+lead))
            self.speed = min(ceiling, self.speed+self.settings.acceleration_rpm_s*dt)
            magnitudes = tuple(max(minimum, self.speed-self.difference_gain*
                                   max(0.0, speed-slowest-self.difference_deadband_rpm))
                               for speed in measured)
            self.reason = ('MATCHING_WHEEL_SPEEDS' if self.speed < maximum-.1
                           or max(measured)-slowest > 1 else None)
        goals = tuple(sign*min(maximum, speed) for sign, speed in zip(directions, magnitudes))
        # Do this AFTER every ceiling and differential correction: even a
        # sudden estimate change may not create an unlimited speed command.
        # Smooth feedback corrections, but don't delay an operator reduction.
        self.targets = tuple(profile.update(goal, dt, decelerate_now=abs(new) < abs(old))
                             for profile, goal, new, old in
                             zip(self.profiles, goals, requested, previous_requested))
        return self.targets
