import json
import unittest
from pathlib import Path
from types import SimpleNamespace

from hg_rma302.control import DriveController, DriveSettings, InputState, Output
from hg_rma302.wheel_sync import SmoothTarget, WheelSync


def wheel(speed, now, *, state='closed', target=None):
    target = speed if target is None else target
    return SimpleNamespace(phase='running', last_status_at=now,
                           last_sent_rpm=round(target*1000), last_status={
        'state': state, 'fault': 'none', 'MOE': '1', 'speed_mode': '1',
        'uart_direction': '1' if speed >= 0 else '-1',
        'req_mrpm': str(round(target*1000)), 'est_mrpm': str(round(speed*1000))})


class WheelSyncTests(unittest.TestCase):
    def setUp(self):
        self.settings = DriveSettings(max_speed_rpm=100, right_sign=-1)
        self.sync = WheelSync(self.settings)
        self.forward = Output(50, -50, True, synchronize=True)

    def update(self, wheels, now, output=None, enabled=True):
        return self.sync.update(output or self.forward, wheels, now, enabled=enabled)

    def test_yaw_control_waits_for_both_and_does_not_fight_heading_differential(self):
        out=Output(40,-20,True)
        for i in range(30):
            now=i*.02
            result=self.sync.update(out,[wheel(8,now),wheel(-8,now,state='ramp')],now,enabled=True,yaw_control=True)
            self.assertEqual(result,(8,-8))
        # Once both wheels run, unequal measured speeds must not add a second
        # differential controller over the IMU's correction.
        for i in range(30,400):
            now=i*.02
            result=self.sync.update(self.forward,[wheel(55,now),wheel(-45,now)],now,enabled=True,yaw_control=True)
        self.assertAlmostEqual(result[0],-result[1]);self.assertAlmostEqual(result[0],50,places=3)

    def test_recorded_asymmetric_startup_never_accelerates_early_wheel(self):
        fixture = json.loads((Path(__file__).parent/'fixtures/dpad-startup-20261105.json').read_text())
        held = 0
        for sample in fixture['samples']:
            now = sample['monotonic_s']
            controller = DriveController(self.settings, require_input_updates=False)
            controller.update(InputState(last_device_check_at=now), now)
            output = controller.update(InputState(**sample['input']), now)
            wheels = [SimpleNamespace(phase=w['phase'], last_status=w['status'],
                        last_sent_rpm=w['last_sent_mrpm'], last_status_at=now-w['status_age_s'])
                      for w in sample['wheels'].values()]
            result = self.update(wheels, now, output)
            if any(w.last_status['state'] != 'closed' for w in wheels):
                self.assertEqual(tuple(abs(rpm) for rpm in result), (8, 8))
                self.assertEqual(self.sync.reason, 'WAITING_FOR_WHEELS')
                held += 1
        self.assertGreaterEqual(held, 6)

    def test_blend_stale_missing_and_wrong_direction_are_not_ready(self):
        for state in ('ramp', 'blend', 'decelerate', 'settle'):
            with self.subTest(state=state):
                self.assertEqual(self.update([wheel(30, 1), wheel(-8, 1, state=state)], 1), (8, -8))
        for bad in (wheel(-8, 1), wheel(8, 2), wheel(-8, 2)):
            if bad.last_status_at == 2 and bad.last_status['uart_direction'] == '-1':
                del bad.last_status['est_mrpm']
            self.assertEqual(self.update([wheel(30, 2), bad], 2), (8, -8))
            self.assertEqual(self.sync.reason, 'WAITING_FOR_WHEELS')

    def test_both_closed_ramp_and_neutral_stop_override_immediately(self):
        speeds = (8, -8)
        history = []
        for i in range(50):
            now = i*.02
            result = self.update([wheel(speed, now) for speed in speeds], now)
            self.assertGreaterEqual(result[0], speeds[0])
            self.assertLessEqual(result[0]-speeds[0], .4+1e-9)
            self.assertAlmostEqual(result[1], -result[0])
            history.append(result[0])
            speeds = result
        self.assertLess(history[1]-history[0], .06)
        self.assertGreater(history[-1], 15)
        self.assertEqual(self.update([], 1, Output(0, 0, True)), (0, 0))
        self.assertIsNone(self.sync.reason)
        self.assertEqual(self.update([], 1.02, enabled=False), (0, 0))
        self.assertEqual(self.update([wheel(25, 1.04), wheel(-25, 1.04)], 1.04), (8, -8))

    def test_reversal_waits_for_new_direction_and_uses_latest_input(self):
        for i in range(10):
            self.update([wheel(25, i*.02), wheel(-25, i*.02)], i*.02)
        backwards = Output(-50, 50, True, synchronize=True)
        self.assertEqual(self.update([wheel(25, .2), wheel(-25, .2)], .2, backwards), (-8, 8))
        self.assertEqual(self.sync.reason, 'WAITING_FOR_WHEELS')
        self.assertEqual(self.update([wheel(-8, .22), wheel(8, .22)], .22), (8, -8))
        self.assertEqual(self.sync.reason, 'WAITING_FOR_WHEELS')

    def test_independent_sticks_accelerate_without_waiting_for_the_peer(self):
        previous = (8, -8)
        for i in range(300):
            now = i*.02
            wheels = [wheel(previous[0], now), wheel(-8, now, state='ramp')]
            result = self.update(wheels, now, Output(100, -30, True))
            self.assertLessEqual(result[0]-previous[0], .4+1e-9)
            self.assertEqual(result[1], -8)
            previous = result
        self.assertGreater(result[0], 99)
        self.assertEqual(self.update(wheels, 6, Output(0, -80, True))[0], 0)
        self.assertEqual(self.update(wheels, 6.02, enabled=False), (0, 0))

    def test_delayed_feedback_and_different_motor_limits_remain_matched(self):
        # A separate plant model: unequal time constants/voltage limits, 10 Hz
        # telemetry and a 20 RPM/s motor acceleration limit. Host runs at 50 Hz.
        speeds = [8., 8.]
        history = []
        for i in range(1000):
            now = i*.02
            if i % 5 == 0:
                wheels = [wheel(speeds[0], now), wheel(-speeds[1], now)]
            targets = self.update(wheels, now)
            for j, (limit, tau) in enumerate(((31.1, .08), (26., .18))):
                error = min(abs(targets[j]), limit)-speeds[j]
                speeds[j] += max(-.4, min(.4, error*.02/tau))
            history.append(tuple(speeds))
        self.assertGreater(min(speeds), 25)
        self.assertLess(abs(speeds[0]-speeds[1]), 1.3)
        self.assertLess(max(abs(left-right) for left, right in history), 2)
        self.assertEqual(self.sync.reason, 'MATCHING_WHEEL_SPEEDS')

    def test_smoothed_target_rises_and_settles_without_overshoot(self):
        profile = SmoothTarget(self.settings)
        history = [profile.update(25, .02) for _ in range(300)]
        steps = [b-a for a, b in zip(history, history[1:])]
        self.assertTrue(all(-1e-9 <= step <= .4+1e-9 for step in steps))
        self.assertAlmostEqual(history[-1], 25, places=6)
        self.assertLess(steps[0], .06)
        self.assertLess(steps[-1], .001)
        # Acceleration grows/decays gradually, rather than jumping to the limit.
        self.assertLess(max(abs(b-a)/.02**2 for a, b in zip(steps, steps[1:])), 140)
        previous = history[-1]
        for _ in range(300):
            value = profile.update(8, .02)
            self.assertLessEqual(value, previous)
            self.assertLessEqual(previous-value, .4+1e-9)
            self.assertGreaterEqual(value, 8)
            previous = value
        self.assertAlmostEqual(previous, 8, places=6)

    def test_correction_steps_and_feedback_loss_never_bypass_rate_limit(self):
        previous = (8, -8)
        for i in range(400):
            now = i*.02
            wheels = [wheel(30, now), wheel(-30 if i % 10 < 5 else -15, now)]
            if 200 < i < 220:
                wheels[1].last_status_at = now-1  # Waiting must decelerate smoothly.
            result = self.update(wheels, now)
            for old, new in zip(previous, result):
                self.assertLessEqual(abs(new-old), .4+1e-9)
            previous = result

    def test_operator_reduction_on_turn_transition_is_not_delayed(self):
        for i in range(200):
            now = i*.02
            previous = self.update([wheel(30, now), wheel(-30, now)], now)
        pivot = Output(25, 25, True, synchronize=True)
        result = self.update([wheel(30, 4), wheel(-30, 4)], 4, pivot)
        self.assertEqual(result[0], 8)  # Operator reduced 50 -> 25; peer must restart.
        self.assertEqual(result[1], 8)  # Opposite sign enters ESC-controlled braking.

    def test_rate_limit_holds_with_irregular_loop_timing_and_retargeting(self):
        profile = SmoothTarget(self.settings)
        previous = profile.update(50, 0)
        for i in range(1000):
            dt = (.001, .02, .15, .01, .03)[i % 5]
            value = profile.update(50 if i % 100 < 50 else 8, dt)
            self.assertLessEqual(abs(value-previous), 20*dt+1e-9)
            self.assertTrue(8 <= value <= 50)
            previous = value

    def test_recorded_feedback_steps_are_smoothed_before_uart_targets(self):
        fixture = json.loads((Path(__file__).parent/'fixtures/dpad-feedback-20261105.json').read_text())
        old_jumps, new_jumps = [], []
        previous, old_previous, previous_time = None, None, None
        for row in fixture['samples']:
            now = row['monotonic_s']
            wheels = [SimpleNamespace(phase=w['phase'], last_status=w['status'],
                        last_sent_rpm=w['last_sent_mrpm'], last_status_at=now-w['status_age_s'])
                      for w in row['wheels'].values()]
            result = self.update(wheels, now)
            old = [row['wheel_sync'][side+'_rpm'] for side in ('left', 'right')]
            if previous:
                dt = now-previous_time
                new_jumps.extend(abs(new-old)/dt for old, new in zip(previous, result))
                old_jumps.extend(abs(new-old)/dt for old, new in zip(old_previous, old))
            previous, old_previous, previous_time = result, old, now
        self.assertGreater(max(old_jumps), 90)  # Actual ~2 RPM / 20 ms correction.
        self.assertLessEqual(max(new_jumps), self.settings.acceleration_rpm_s+1e-6)

    def test_stick_profile_waits_for_own_startup_then_accelerates_from_minimum(self):
        command = Output(100, 0, True)
        for i in range(100):
            now = i*.02
            result = self.update([wheel(8, now, state='ramp')], now, command)
            self.assertEqual(result, (8, 0))
        result = self.update([wheel(8, 2)], 2, command)
        self.assertTrue(8 < result[0] < 8.06)
        self.assertEqual(result[1], 0)

    def test_stick_reduction_zero_and_reverse_cancel_filter_lead(self):
        previous = (8, -8)
        for i in range(200):
            now = i*.02
            wheels = [wheel(previous[0], now), wheel(previous[1], now)]
            previous = self.update(wheels, now, Output(100, -100, True))
        self.assertGreater(previous[0], 60)
        result = self.update(wheels, 4, Output(20, -100, True))
        self.assertEqual(result[0], 20)
        self.assertGreaterEqual(abs(result[1]), abs(previous[1]))
        result = self.update(wheels, 4.02, Output(0, -100, True))
        self.assertEqual(result[0], 0)
        result = self.update(wheels, 4.04, Output(-100, -100, True))
        self.assertEqual(result[0], -8)
        self.assertEqual(self.update(wheels, 4.06, Output(0, 0, True)), (0, 0))
        self.assertEqual(self.update(wheels, 4.08, Output(100, -100, True)), (8, -8))

    def test_switching_sticks_and_dpad_preserves_independent_profile_speeds(self):
        previous = (8, -8)
        for i in range(75):
            now = i*.02
            previous = self.update([wheel(previous[0], now), wheel(previous[1], now)], now)
        for i in range(75, 100):
            now = i*.02
            wheels = [wheel(previous[0], now), wheel(previous[1], now)]
            out = Output(100, -100, True) if i < 85 else self.forward
            result = self.update(wheels, now, out)
            self.assertLessEqual(abs(result[0]-previous[0]), .4+1e-9)
            self.assertLessEqual(abs(result[1]-previous[1]), .4+1e-9)
            self.assertGreater(result[0], 15)
            previous = result

    def test_recorded_stick_acceleration_is_shaped_but_release_is_immediate(self):
        fixture = json.loads((Path(__file__).parent/'fixtures/stick-acceleration-20261105.json').read_text())
        previous, old_previous, previous_time = (0, 0), (0, 0), None
        old_rates, new_rates = [], []
        neutral_seen = False
        for row in fixture['samples']:
            now = row['monotonic_s']
            wheels = [SimpleNamespace(phase=w['phase'], last_status=w['status'],
                        last_sent_rpm=w['last_sent_mrpm'], last_status_at=now-w['status_age_s'])
                      for w in row['wheels'].values()]
            output = Output(**row['output'])
            result = self.update(wheels, now, output)
            raw = (output.left_rpm, output.right_rpm)
            if previous_time is not None:
                dt = now-previous_time
                for old, new, before, after in zip(previous, result, old_previous, raw):
                    if old*new > 0 and abs(new)>abs(old):
                        new_rates.append((abs(new)-abs(old))/dt)
                    if before*after>0 and abs(after)>abs(before):
                        old_rates.append((abs(after)-abs(before))/dt)
                    if after == 0:
                        self.assertEqual(new, 0)
                        neutral_seen = True
                    elif old*after>0 and abs(after)<abs(old):
                        self.assertLessEqual(abs(new), abs(after))
            previous, old_previous, previous_time = result, raw, now
        self.assertGreater(max(old_rates), 3000)
        self.assertLessEqual(max(new_rates), 20+1e-6)
        self.assertTrue(neutral_seen)
