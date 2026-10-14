import math
import unittest
from dataclasses import replace

from hg_rma302.control import DriveController, DriveSettings, InputState, mix
from hg_rma302.gamepad import EventDecoder, normalize, EV_ABS, EV_KEY, EV_SYN, SYN_DROPPED, ABS_X, ABS_Y, BTN_B, BTN_LB


class ControlTests(unittest.TestCase):
    def setUp(self):
        self.controller = DriveController(DriveSettings())
        self.t = 0.0

    def step(self, *, dt=.02, **kwargs):
        self.t += dt
        return self.controller.update(InputState(last_input_at=self.t, **kwargs), self.t)

    def arm(self):
        self.step()
        self.assertTrue(self.step(deadman=True).enabled)

    def test_mixer(self):
        self.assertEqual(mix(1, 0), (1, 1))
        self.assertEqual(mix(-1, 0), (-1, -1))
        self.assertEqual(mix(0, 1), (1, -1))
        self.assertEqual(mix(0, -1), (-1, 1))
        self.assertEqual(mix(1, 1), (1, 0))

    def test_boot_with_lb_held_cannot_arm(self):
        self.assertFalse(self.step(deadman=True, throttle=1).enabled)
        self.assertFalse(self.step(deadman=True).enabled)
        self.arm()

    def test_non_neutral_press_cannot_arm(self):
        self.step()
        self.assertFalse(self.step(deadman=True, throttle=.5).enabled)
        self.assertFalse(self.step(deadman=True).enabled)

    def test_speed_range_and_neutral_stop(self):
        self.arm()
        first = self.step(deadman=True, throttle=1)
        self.assertEqual(first.left_rpm, 15)
        for _ in range(100):
            output = self.step(deadman=True, throttle=1)
        self.assertEqual((output.left_rpm, output.right_rpm), (15, 15))
        small = self.step(deadman=True, throttle=.121)
        self.assertGreaterEqual(small.left_rpm, 8)
        self.assertLess(small.left_rpm, 8.01)
        neutral = self.step(deadman=True)
        self.assertEqual((neutral.left_rpm, neutral.right_rpm), (0, 0))

    def test_deadman_release_stops_without_ramp(self):
        self.arm()
        self.step(deadman=True, throttle=1)
        output = self.step(throttle=1)
        self.assertEqual((output.left_rpm, output.right_rpm, output.enabled), (0, 0, False))

    def test_estop_latched_until_neutral_reset_edge(self):
        self.arm()
        self.step(deadman=True, throttle=1, estop=True)
        self.assertFalse(self.step().enabled)
        self.assertFalse(self.step(deadman=True, reset=True).enabled)
        self.step(reset=False)
        self.step(reset=True)
        self.assertTrue(self.step(deadman=True).enabled)

    def test_timeout_does_not_automatically_resume(self):
        self.arm()
        sample = InputState(throttle=1, deadman=True, last_input_at=self.t)
        for _ in range(30):
            self.t += .02
            output = self.controller.update(sample, self.t)
        self.assertEqual(output.reason, "INPUT_TIMEOUT")
        self.assertFalse(self.step(deadman=True, throttle=1).enabled)
        self.arm()

    def test_loop_stall_stops(self):
        self.arm()
        self.assertEqual(self.step(dt=.3, deadman=True, throttle=1).reason, "LOOP_TIMEOUT")

    def test_disconnect_and_nan_stop(self):
        self.arm()
        self.assertEqual(self.step(connected=False).reason, "DISCONNECTED")
        self.arm()
        self.assertEqual(self.step(deadman=True, throttle=math.nan).reason, "INVALID_INPUT")

    def test_reverse_and_pivot_with_lb_held_have_no_host_wait(self):
        self.arm()
        self.step(deadman=True, throttle=1)
        for throttle, steering in ((-1,0),(0,0),(1,0),(0,1),(1,0)):
            output=self.step(deadman=True, throttle=throttle, steering=steering)
            self.assertTrue(output.enabled)
            self.assertIsNone(self.controller.direction_wait_until)
        self.assertGreater(output.left_rpm,0)
        self.assertGreater(output.right_rpm,0)

    def test_turn_preserves_ratio_and_can_stop_inner_wheel(self):
        self.arm()
        right=self.step(deadman=True,throttle=1,steering=.6)
        left=self.step(deadman=True,throttle=1,steering=-.6)
        self.assertEqual((right.left_rpm,right.right_rpm),(15,0))
        self.assertEqual((left.left_rpm,left.right_rpm),(0,15))

    def test_reversal_wait_uses_latest_target_without_shortening_or_extending_wait(self):
        self.arm()
        self.step(deadman=True, throttle=1)
        self.step()
        self.step(deadman=True)
        self.step(deadman=True, throttle=-1)
        deadline = self.controller.direction_wait_until
        for index in range(124):
            # Returning to the old direction during waiting must not restart
            # early, and changed inputs must not keep extending the deadline.
            output = self.step(deadman=True, throttle=1 if index % 2 else -1)
            self.assertFalse(output.enabled)
            self.assertEqual(self.controller.direction_wait_until, deadline)
        self.step(deadman=True, steering=1)
        output = self.step(deadman=True, steering=1)
        self.assertTrue(output.enabled)
        self.assertGreater(output.left_rpm, 0)
        self.assertLess(output.right_rpm, 0)

    def test_neutral_during_wait_does_not_resume_old_motion(self):
        self.arm()
        self.step(deadman=True, throttle=1)
        self.step()
        self.step(deadman=True)
        self.step(deadman=True, throttle=-1)
        for _ in range(130):
            output = self.step(deadman=True)
            self.assertEqual((output.left_rpm, output.right_rpm), (0, 0))
        self.assertIsNone(self.controller.direction_wait_until)
        self.assertTrue(self.step(deadman=True, throttle=-1).enabled)

    def test_lb_release_during_wait_cancels_reversal(self):
        self.arm()
        self.step(deadman=True, throttle=1)
        self.step()
        self.step(deadman=True)
        self.step(deadman=True, throttle=-1)
        self.step(throttle=-1)  # release LB
        self.assertIsNone(self.controller.direction_wait_until)
        for _ in range(130):
            self.assertFalse(self.step(deadman=True, throttle=-1).enabled)
        self.arm()
        self.assertTrue(self.step(deadman=True, throttle=-1).enabled)

    def test_faults_during_reversal_wait_revoke_operator_authorization(self):
        for fault in ({"estop": True}, {"connected": False}, {"dt": .3}, {"throttle": math.nan}):
            with self.subTest(fault=fault):
                self.setUp()
                self.arm()
                self.step(deadman=True, throttle=1)
                self.step(deadman=True, throttle=-1)
                output = self.step(deadman=True, **fault)
                self.assertFalse(output.enabled)
                self.assertIsNone(self.controller.direction_wait_until)
                for _ in range(130):
                    self.assertFalse(self.step(deadman=True, throttle=-1).enabled)

    def test_mirrored_motor_signs(self):
        self.controller = DriveController(DriveSettings(right_sign=-1))
        self.arm()
        output = self.step(deadman=True, throttle=1)
        self.assertGreater(output.left_rpm, 0)
        self.assertLess(output.right_rpm, 0)

    def test_invalid_settings(self):
        for kwargs in ({"max_current_a": math.nan}, {"max_current_a": 2.01},
                       {"input_timeout_s": 0}, {"left_sign": True}, {"deadzone": 1}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                DriveSettings(**kwargs)


class DecoderTests(unittest.TestCase):
    def setUp(self):
        self.decoder = EventDecoder({ABS_X: (0, -32768, 32767), ABS_Y: (0, -32768, 32767)})

    def test_axis_ranges(self):
        self.assertEqual(normalize(-32768, -32768, 32767), -1)
        self.assertEqual(normalize(32767, -32768, 32767), 1)

    def test_requires_whole_frame_and_preserves_timestamp(self):
        self.assertIsNone(self.decoder.feed(EV_ABS, ABS_Y, -32768, 3.0))
        state = self.decoder.feed(EV_SYN, 0, 0, 3.1)
        self.assertEqual(state.throttle, 1)
        self.assertEqual(state.last_input_at, 3.0)
        self.assertIsNone(self.decoder.feed(EV_SYN, 0, 0, 9.0))
        self.assertEqual(self.decoder.state.last_input_at, 3.0)

    def test_repeat_does_not_refresh_deadman(self):
        self.decoder.feed(EV_KEY, BTN_LB, 2, 5)
        self.assertIsNone(self.decoder.feed(EV_SYN, 0, 0, 5))

    def test_brief_stop_edges_are_not_lost(self):
        states = []
        for value in (1, 0):
            self.decoder.feed(EV_KEY, BTN_B, value, 3)
            states.append(self.decoder.feed(EV_SYN, 0, 0, 3))
        self.assertTrue(states[0].estop)
        self.assertFalse(states[1].estop)

    def test_overflow_fails_closed(self):
        with self.assertRaises(OSError):
            self.decoder.feed(EV_SYN, SYN_DROPPED, 0, 0)


class WiredControlTests(unittest.TestCase):
    def setUp(self):
        self.controller = DriveController(DriveSettings(), require_input_updates=False)

    def step(self, now, **kwargs):
        # No new input events, but a successful device poll each control tick.
        return self.controller.update(InputState(last_input_at=0, last_device_check_at=now, **kwargs), now)

    def test_held_stick_continues_and_lb_release_stops(self):
        self.step(0)
        self.step(.02, deadman=True)
        for n in range(2, 102):
            result = self.step(n*.02, deadman=True, throttle=.5)
            self.assertTrue(result.enabled)
        self.assertGreater(result.left_rpm, 0)
        result = self.step(2.04, throttle=.5)
        self.assertEqual((result.left_rpm, result.right_rpm, result.enabled), (0, 0, False))

    def test_missing_device_polls_disarm_without_auto_resume(self):
        self.step(0)
        self.step(.02, deadman=True)
        stale = InputState(throttle=.5, deadman=True, last_device_check_at=.02)
        for n in range(2, 32):
            result = self.controller.update(stale, n*.02)
        self.assertEqual(result.reason, "GAMEPAD_POLL_TIMEOUT")
        self.assertFalse(self.step(.64, deadman=True, throttle=.5).enabled)

    def test_held_lb_can_reverse_without_new_button_events(self):
        self.step(0)
        self.step(.02, deadman=True)
        self.step(.04, deadman=True, throttle=1)
        self.assertEqual(self.step(.06, deadman=True, throttle=-1).reason, "DRIVING")
        for n in range(4, 128):
            self.assertTrue(self.step(n*.02, deadman=True, throttle=-1).enabled)
        output = self.step(2.56, deadman=True, throttle=-1)
        self.assertTrue(output.enabled)
        self.assertLess(output.left_rpm, 0)

    def test_device_poll_timeout_during_wait_cannot_resume_on_reconnect(self):
        self.step(0)
        self.step(.02, deadman=True)
        self.step(.04, deadman=True, throttle=1)
        self.step(.06, deadman=True, throttle=-1)
        stale = InputState(deadman=True, throttle=-1, last_device_check_at=.06)
        for n in range(4, 32):
            output = self.controller.update(stale, n*.02)
        self.assertEqual(output.reason, "GAMEPAD_POLL_TIMEOUT")
        for n in range(32, 200):
            self.assertFalse(self.step(n*.02, deadman=True, throttle=-1).enabled)

    def test_held_lb_at_start_and_loop_stall_cannot_arm(self):
        self.assertFalse(self.step(0, deadman=True).enabled)
        self.assertFalse(self.step(.02, deadman=True).enabled)
        self.step(.04)
        self.assertTrue(self.step(.06, deadman=True).enabled)
        self.assertEqual(self.step(.4, deadman=True).reason, "LOOP_TIMEOUT")


if __name__ == "__main__":
    unittest.main()
