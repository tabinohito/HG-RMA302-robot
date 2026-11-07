import math
import json
import unittest
from pathlib import Path

from hg_rma302.control import DriveController, DriveSettings, InputState, stick_speed, tank_targets
from hg_rma302.gamepad import (EventDecoder, normalize, EV_ABS, EV_KEY, EV_SYN,
                              SYN_DROPPED, ABS_X, ABS_Y, ABS_RY, ABS_HAT0X, ABS_HAT0Y,
                              BTN_B, BTN_LB)


class ControlTests(unittest.TestCase):
    def setUp(self):
        self.controller = DriveController(DriveSettings(max_speed_rpm=100))
        self.t = 0.0

    def step(self, *, dt=.02, **kwargs):
        self.t += dt
        return self.controller.update(InputState(last_input_at=self.t, **kwargs), self.t)

    def arm(self):
        self.assertEqual(self.step().reason, 'READY')

    def test_start_requires_all_controls_neutral(self):
        for controls in ({'left_stick':1}, {'right_stick':-1}, {'dpad_y':1},
                         {'dpad_x':-1}, {'dpad_x':1, 'dpad_y':1}):
            with self.subTest(controls=controls):
                self.setUp()
                for _ in range(10):
                    self.assertEqual(self.step(**controls).reason, 'CENTER_CONTROLS')
                self.arm()
                self.assertTrue(self.step(**controls).enabled if len(controls)==1 else True)

    def test_independent_vertical_sticks_and_motor_signs(self):
        self.arm()
        for controls, expected in (({'left_stick':1}, (100,0)), ({'right_stick':-1}, (0,-100)),
                ({'left_stick':1,'right_stick':-1},(100,-100)),
                ({'left_stick':-1,'right_stick':-1},(-100,-100))):
            out=self.step(**controls)
            self.assertEqual((out.left_rpm,out.right_rpm),expected)
        self.controller=DriveController(DriveSettings(max_speed_rpm=100,right_sign=-1))
        self.arm()
        out=self.step(left_stick=1,right_stick=1)
        self.assertEqual((out.left_rpm,out.right_rpm),(100,-100))
        self.assertFalse(out.synchronize)

    def test_sticks_scale_independently_after_deadzone(self):
        self.arm()
        out=self.step(left_stick=.56,right_stick=1)  # Halfway outside .12 deadzone.
        self.assertAlmostEqual(out.left_rpm,54)
        self.assertEqual(out.right_rpm,100)
        out=self.step(left_stick=.121,right_stick=.1)
        self.assertAlmostEqual(out.left_rpm,8+92/880)
        self.assertEqual(out.right_rpm,0)
        self.assertEqual(self.step(left_stick=2).left_rpm,100)

    def test_calibrated_stick_range_uses_all_active_travel_linearly(self):
        settings = DriveSettings(max_speed_rpm=100, stick_max_speed_rpm=30, right_sign=-1)
        # 8..30 RPM across .12..1 travel: 25 RPM per unit of physical deflection.
        for sign in (-1, 1):
            values = [stick_speed(sign*(.12+i*.0088), settings) for i in range(1, 101)]
            self.assertAlmostEqual(values[0]*sign, 8.22)
            self.assertAlmostEqual(values[-1]*sign, 30)
            for a, b in zip(values, values[1:]):
                self.assertAlmostEqual((b-a)*sign, .22)
        for value in (-.12, -.01, 0, .12):
            self.assertEqual(stick_speed(value, settings), 0)
        self.assertEqual(stick_speed(2, settings), 30)
        self.assertEqual(stick_speed(-2, settings), -30)
        self.assertEqual(tank_targets(InputState(left_stick=.2, right_stick=.56), settings), (10, -19))
        # Calibrating the sticks must not silently scale the D-pad percentages.
        self.assertEqual(tank_targets(InputState(dpad_y=1), settings), (50, -50))
        self.assertEqual(tank_targets(InputState(dpad_x=-1), settings), (-25, -25))

    def test_stick_range_fallback_and_single_speed_config(self):
        self.assertEqual(DriveSettings(max_speed_rpm=25).stick_full_scale_rpm, 25)
        settings = DriveSettings(max_speed_rpm=100, stick_max_speed_rpm=8)
        self.assertEqual(stick_speed(.5, settings), 8)
        self.assertEqual(stick_speed(1, settings), 8)

    def test_recorded_partial_stick_turn_no_longer_requests_two_unreachable_speeds(self):
        fixture = json.loads((Path(__file__).parent/'fixtures/stick-travel-20261105.json').read_text())
        settings = DriveSettings(max_speed_rpm=100, stick_max_speed_rpm=30, right_sign=-1)
        checked = 0
        for row in fixture['samples']:
            if .58 <= row['left_stick'] <= .6 and row['right_stick'] == 1:
                # In the actual log both requests exceed the measured ~30 RPM
                # ceiling: changing the partial left stick cannot make a turn.
                self.assertGreater(row['old_left_rpm'], 50)
                self.assertEqual(row['old_right_rpm'], -100)
                left, right = tank_targets(InputState(left_stick=row['left_stick'], right_stick=1), settings)
                self.assertTrue(19.5 <= left <= 20)
                self.assertEqual(right, -30)
                checked += 1
        self.assertGreaterEqual(checked, 2)  # Two distinct matching input frames in this fixture.

    def test_dpad_exact_percentages_all_directions(self):
        self.arm()
        for controls, expected in (({'dpad_y':1},(50,50)),({'dpad_y':-1},(-50,-50)),
                                  ({'dpad_x':1},(25,-25)),({'dpad_x':-1},(-25,25))):
            with self.subTest(controls=controls):
                out=self.step(**controls)
                self.assertEqual((out.left_rpm,out.right_rpm),expected)
                self.assertTrue(out.synchronize)

    def test_dpad_overrides_sticks_and_release_uses_current_sticks(self):
        self.arm()
        out=self.step(left_stick=-1,right_stick=-1,dpad_y=1)
        self.assertEqual((out.left_rpm,out.right_rpm),(50,50))
        out=self.step(left_stick=1,right_stick=0)
        self.assertEqual((out.left_rpm,out.right_rpm),(100,0))
        self.assertIsNone(self.controller.direction_wait_until)

    def test_dpad_diagonal_stops_even_when_sticks_deflected(self):
        self.arm()
        self.step(left_stick=1,right_stick=1)
        for x in (-1,1):
            for y in (-1,1):
                out=self.step(left_stick=1,right_stick=1,dpad_x=x,dpad_y=y)
                self.assertEqual((out.left_rpm,out.right_rpm,out.reason),(0,0,'DPAD_DIAGONAL_STOP'))
        self.assertEqual(self.step(dpad_x=1).left_rpm,25)

    def test_configurable_dpad_ratios_respect_minimum_speed(self):
        self.controller=DriveController(DriveSettings(max_speed_rpm=25,dpad_drive_ratio=.8,dpad_turn_ratio=.25))
        self.arm()
        self.assertEqual(self.step(dpad_y=1).left_rpm,20)
        out=self.step(dpad_x=-1)
        self.assertEqual((out.left_rpm,out.right_rpm),(-8,8))

    def test_neutral_and_reversal_keep_continuous_speed_session(self):
        self.arm()
        for controls in ({'left_stick':1,'right_stick':1},{},{'left_stick':-1,'right_stick':-1},
                         {'dpad_x':1},{'left_stick':1,'right_stick':1}):
            out=self.step(**controls)
            self.assertTrue(out.enabled)
            self.assertIsNone(self.controller.direction_wait_until)
            if not controls: self.assertEqual((out.left_rpm,out.right_rpm),(0,0))

    def test_b_stops_immediately_and_start_requires_neutral_edge(self):
        self.arm()
        self.step(left_stick=1)
        self.assertFalse(self.step(left_stick=1,estop=True).enabled)
        self.assertEqual(self.step(left_stick=1,reset=True).reason,'STOP_LATCHED')
        self.assertEqual(self.step(reset=True).reason,'STOP_LATCHED')
        self.step()
        self.assertEqual(self.step(dpad_y=1,reset=True).reason,'STOP_LATCHED')
        self.step()
        self.assertEqual(self.step(reset=True).reason,'READY')
        self.assertTrue(self.step(left_stick=1).enabled)

    def test_fault_does_not_resume_until_neutral(self):
        for fault,reason in (({'connected':False},'DISCONNECTED'),({'left_stick':math.nan},'INVALID_INPUT'),
                             ({'dpad_x':2},'INVALID_INPUT'),({'dt':.3},'LOOP_TIMEOUT')):
            with self.subTest(fault=fault):
                self.setUp();self.arm();self.step(left_stick=1)
                self.assertEqual(self.step(**fault).reason,reason)
                for _ in range(5): self.assertFalse(self.step(left_stick=1).enabled)
                self.arm()
                self.assertTrue(self.step(left_stick=1).enabled)

    def test_event_timeout_revokes_authorization(self):
        self.arm();self.step(left_stick=1)
        sample=InputState(left_stick=1,last_input_at=self.t)
        for _ in range(30):
            self.t+=.02;out=self.controller.update(sample,self.t)
        self.assertEqual(out.reason,'INPUT_TIMEOUT')
        self.assertFalse(self.step(left_stick=1).enabled)
        self.arm()

    def begin_cold_reverse(self):
        self.arm();self.step(left_stick=1,right_stick=1)
        self.step(estop=True);self.step(reset=True)
        out=self.step(left_stick=-1,right_stick=-1)
        self.assertEqual(out.reason,'DIRECTION_CHANGE_WAIT')

    def test_cold_reversal_after_b_waits_then_uses_latest_target(self):
        self.begin_cold_reverse()
        deadline=self.controller.direction_wait_until
        for n in range(124):
            out=self.step(left_stick=1 if n%2 else -1,right_stick=-1)
            self.assertFalse(out.enabled)
            self.assertEqual(self.controller.direction_wait_until,deadline)
        for _ in range(2): out=self.step(dpad_x=1)
        self.assertEqual((out.left_rpm,out.right_rpm),(25,-25))

    def test_neutral_cancels_cold_reversal_without_replaying_it(self):
        self.begin_cold_reverse()
        for _ in range(130):
            out=self.step()
            self.assertEqual((out.left_rpm,out.right_rpm),(0,0))
        self.assertIsNone(self.controller.direction_wait_until)
        self.assertTrue(self.step(left_stick=-1).enabled)

    def test_b_during_cold_reversal_cancels_pending_movement(self):
        self.begin_cold_reverse();self.step(estop=True)
        self.assertIsNone(self.controller.direction_wait_until)
        for _ in range(130): self.assertFalse(self.step(left_stick=-1).enabled)
        self.step(reset=True)
        self.assertTrue(self.step(left_stick=-1).enabled)

    def test_invalid_settings(self):
        for kwargs in ({'max_current_a':math.nan},{'max_current_a':2.01},{'input_timeout_s':0},
                       {'left_sign':True},{'deadzone':1},{'max_speed_rpm':100.001},
                       {'acceleration_rpm_s':30.001},
                       {'dpad_smoothing_s':-1},{'dpad_smoothing_s':1.01},
                       {'dpad_smoothing_s':True},{'dpad_smoothing_s':math.nan},
                       {'stick_smoothing_s':-1},{'stick_smoothing_s':1.01},
                       {'stick_smoothing_s':True},{'stick_smoothing_s':math.inf},
                       {'stick_max_speed_rpm':True},{'stick_max_speed_rpm':math.inf},
                       {'stick_max_speed_rpm':math.nan},{'stick_max_speed_rpm':'30'},
                       {'stick_max_speed_rpm':7.99},{'stick_max_speed_rpm':15.01},
                       {'dpad_turn_ratio':0},{'dpad_drive_ratio':1.01},{'dpad_turn_ratio':True}):
            with self.subTest(kwargs=kwargs),self.assertRaises(ValueError): DriveSettings(**kwargs)


class DecoderTests(unittest.TestCase):
    def setUp(self):
        self.decoder=EventDecoder({ABS_Y:(0,-32768,32767),ABS_RY:(0,-32768,32767),
                                   ABS_HAT0X:(0,-1,1),ABS_HAT0Y:(0,-1,1)})

    def test_axis_ranges(self):
        self.assertEqual(normalize(-32768,-32768,32767),-1)
        self.assertEqual(normalize(32767,-32768,32767),1)

    def test_complete_frames_keep_both_axes_and_timestamp(self):
        self.assertIsNone(self.decoder.feed(EV_ABS,ABS_Y,-32768,3))
        self.decoder.feed(EV_ABS,ABS_RY,32767,3.01)
        state=self.decoder.feed(EV_SYN,0,0,3.1)
        self.assertEqual((state.left_stick,state.right_stick),(1,-1))
        self.assertEqual(state.last_input_at,3.01)
        self.assertIsNone(self.decoder.feed(EV_SYN,0,0,9))
        self.assertEqual(self.decoder.state.last_input_at,3.01)

    def test_hat_up_is_forward_and_right_is_positive(self):
        self.decoder.feed(EV_ABS,ABS_HAT0Y,-1,3)
        self.decoder.feed(EV_ABS,ABS_HAT0X,1,3)
        state=self.decoder.feed(EV_SYN,0,0,3)
        self.assertEqual((state.dpad_x,state.dpad_y),(1,1))
        self.decoder.feed(EV_ABS,ABS_HAT0Y,0,4)
        self.assertEqual(self.decoder.feed(EV_SYN,0,0,4).dpad_y,0)

    def test_horizontal_axes_and_lb_do_not_affect_commands(self):
        before=self.decoder.state
        for kind,code,value in ((EV_ABS,ABS_X,32767),(EV_ABS,3,-32768),
                                (EV_KEY,BTN_LB,1),(EV_KEY,BTN_LB,2),(EV_KEY,BTN_LB,0)):
            self.decoder.feed(kind,code,value,5)
            self.assertIsNone(self.decoder.feed(EV_SYN,0,0,5))
        self.assertEqual(self.decoder.state,before)

    def test_brief_stop_edges_are_not_lost(self):
        states=[]
        for value in (1,0):
            self.decoder.feed(EV_KEY,BTN_B,value,3)
            states.append(self.decoder.feed(EV_SYN,0,0,3))
        self.assertTrue(states[0].estop);self.assertFalse(states[1].estop)

    def test_initial_snapshot_includes_both_sticks_hat_and_b(self):
        state=EventDecoder({ABS_Y:(-32768,-32768,32767),ABS_RY:(32767,-32768,32767),
                            ABS_HAT0X:(-1,-1,1),ABS_HAT0Y:(1,-1,1)},[BTN_B]).state
        self.assertEqual((state.left_stick,state.right_stick,state.dpad_x,state.dpad_y),(1,-1,-1,-1))
        self.assertTrue(state.estop)

    def test_overflow_fails_closed(self):
        with self.assertRaises(OSError): self.decoder.feed(EV_SYN,SYN_DROPPED,0,0)


class WiredControlTests(unittest.TestCase):
    def setUp(self):
        self.controller=DriveController(DriveSettings(max_speed_rpm=100),require_input_updates=False)

    def step(self,now,**kwargs):
        return self.controller.update(InputState(last_input_at=0,last_device_check_at=now,**kwargs),now)

    def test_held_sticks_and_hat_continue_without_lb_or_new_events(self):
        self.step(0)
        for n in range(1,200):
            out=self.step(n*.02,left_stick=1,right_stick=1)
            self.assertEqual((out.left_rpm,out.right_rpm),(100,100))
        for n in range(200,400):
            out=self.step(n*.02,dpad_y=-1)
            self.assertEqual((out.left_rpm,out.right_rpm),(-50,-50))
        out=self.step(8)
        self.assertEqual((out.left_rpm,out.right_rpm),(0,0))
        self.assertTrue(out.enabled)  # Controlled deceleration, no queued restart.

    def test_device_poll_failure_requires_neutral_before_resume(self):
        self.step(0);self.step(.02,left_stick=1)
        stale=InputState(left_stick=1,last_device_check_at=.02)
        for n in range(2,32): out=self.controller.update(stale,n*.02)
        self.assertEqual(out.reason,'GAMEPAD_POLL_TIMEOUT')
        for n in range(32,100): self.assertFalse(self.step(n*.02,left_stick=1).enabled)
        self.step(2)
        self.assertTrue(self.step(2.02,left_stick=1).enabled)


if __name__=='__main__': unittest.main()
