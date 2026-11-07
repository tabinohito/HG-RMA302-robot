import math
import struct
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from hg_rma302.control import DriveController,DriveSettings,InputState,Output,joystick_targets
from hg_rma302.gamepad import EventDecoder,EV_ABS,EV_KEY,EV_SYN,ABS_X,ABS_RX,ABS_Y,ABS_RY,ABS_HAT0X,ABS_HAT0Y,BTN_MODE
from hg_rma302.imu import WitDecoder,ImuSettings,ImuLink,YawFeedback,ImuError,wrap_degrees


def frame(kind,values):
    data=bytes([0x55,kind])+struct.pack('<hhhh',*values,0)
    return data+bytes([sum(data)&255])


class ModeTests(unittest.TestCase):
    def test_recorded_turns_stay_within_speed_limit(self):
        cfg=DriveSettings(control_mode='joystick',max_speed_rpm=100,right_sign=-1)
        # Inputs immediately preceding the three 2026-11-08 range errors.
        cases=(
            (InputState(left_stick=-1,left_x=.48439765011062796),(-47.54735035037853,100.)),
            (InputState(left_stick=.7500267032883192,right_x=-1),(19.267874631341943,-100.)),
            (InputState(left_stick=.6328374151216907,left_x=1),(100.,0.)),
        )
        for sample,expected in cases:
            with self.subTest(sample=sample):
                targets=joystick_targets(sample,cfg)
                for actual,want in zip(targets,expected):
                    self.assertTrue(actual==0 or cfg.min_speed_rpm<=abs(actual)<=cfg.max_speed_rpm)
                    self.assertAlmostEqual(actual,want)

    def test_crossing_steering_deadband_progressively_increases_curve(self):
        cfg=DriveSettings(control_mode='joystick',max_speed_rpm=100,right_sign=-1)
        for forward in (-1,-.5,.5,1):
            for turn in (-1,1):
                previous=0.
                for step in range(1001):
                    x=turn*(cfg.steering_deadzone+step*.0001)
                    left,right=joystick_targets(InputState(left_stick=forward,left_x=x),cfg)
                    differential=turn*(left+right)  # Undo the right motor's wiring sign.
                    self.assertGreater(left*forward,0)
                    self.assertGreater(-right*forward,0)
                    self.assertGreaterEqual(differential,previous)
                    self.assertLess(differential-previous,.0251)
                    self.assertTrue(all(8<=abs(v)<=100 for v in (left,right)))
                    previous=differential

    def test_left_horizontal_deadband_and_right_spin_and_reverse(self):
        cfg=DriveSettings(control_mode='joystick',max_speed_rpm=100,right_sign=-1)
        for x in (-.2,-.1,0,.1,.2):
            self.assertEqual(joystick_targets(InputState(left_stick=1,left_x=x),cfg),(100,-100))
        l,r=joystick_targets(InputState(left_stick=.5,left_x=.201),cfg)
        self.assertGreater(l,-r);self.assertLess(l+r,.2)
        self.assertEqual(joystick_targets(InputState(left_x=1),cfg),(0,0))
        self.assertEqual(joystick_targets(InputState(right_x=1),cfg),(50,50))
        self.assertEqual(joystick_targets(InputState(right_x=-1),cfg),(-50,-50))
        l,r=joystick_targets(InputState(left_stick=-1,left_x=1),cfg)
        self.assertGreater(l,-r) # Right yaw also when reversing.
        self.assertEqual(joystick_targets(InputState(right_stick=1),cfg),(0,0))
        self.assertEqual(joystick_targets(InputState(dpad_y=1,right_x=-1),cfg),(50,-50))
        for y in (-1,-.5,-.121,0,.121,.5,1):
            for x in (-1,-.201,0,.201,1):
                for rx in (-1,0,1):
                    for v in joystick_targets(InputState(left_stick=y,left_x=x,right_x=rx),cfg):
                        self.assertTrue(v==0 or 8<=abs(v)<=100)

    def test_logo_edge_stops_and_requires_neutral_even_with_batched_frames(self):
        c=DriveController(DriveSettings(max_speed_rpm=100),require_input_updates=False)
        def step(t,**kw):return c.update(InputState(last_device_check_at=t,**kw),t)
        step(0)
        self.assertTrue(step(.01,left_stick=1,right_stick=1).enabled)
        out=step(.02,left_stick=1,mode_toggle=True)
        self.assertEqual(out.control_mode,'joystick');self.assertFalse(out.enabled)
        self.assertEqual(step(.03,left_stick=1,mode_toggle=True).reason,'CENTER_CONTROLS')
        step(.04,mode_toggle=True);step(.05,mode_toggle=False)
        self.assertTrue(step(.06,left_stick=1).enabled)
        self.assertEqual(step(.07,mode_toggle=True).control_mode,'tank')
        self.assertFalse(step(.08,estop=True).enabled)
        self.assertEqual(step(.09,estop=True,mode_toggle=True).reason,'STOP_LATCHED')
        self.assertEqual(c.mode,'tank')
        # A button held when the process starts is not a new press.
        c=DriveController(DriveSettings())
        c.update(InputState(mode_toggle=True,last_input_at=0),0)
        self.assertEqual(c.mode,'tank')

    def test_decoder_publishes_horizontal_and_brief_logo_frames(self):
        axes={k:(0,-32768,32767) for k in (ABS_X,ABS_Y,ABS_RX,ABS_RY)}
        axes.update({ABS_HAT0X:(0,-1,1),ABS_HAT0Y:(0,-1,1)})
        d=EventDecoder(axes)
        d.feed(EV_ABS,ABS_X,32767,1);d.feed(EV_ABS,ABS_RX,-32768,1)
        s=d.feed(EV_SYN,0,0,1);self.assertEqual((s.left_x,s.right_x),(1,-1))
        for value in (1,0):
            d.feed(EV_KEY,BTN_MODE,value,2)
            self.assertEqual(d.feed(EV_SYN,0,0,2).mode_toggle,bool(value))


class MemorySerial:
    def __init__(self,**kw):self.buffer=b'';self.closed=False
    def open(self):pass
    def close(self):self.closed=True
    def reset_input_buffer(self):self.buffer=b''
    @property
    def in_waiting(self):return len(self.buffer)
    def read(self,n):data=self.buffer[:n];self.buffer=self.buffer[n:];return data


class ImuTests(unittest.TestCase):
    def test_fragmentation_corruption_signed_conversion_and_resync(self):
        d=WitDecoder();good=frame(0x53,(0,0,-16384))
        damaged=bytearray(frame(0x52,(0,0,100)));damaged[-1]^=1
        data=b'noise'+bytes(damaged)+good+frame(0x52,(0,0,-16384))
        for b in data:d.feed(bytes([b]),1)
        self.assertEqual(d.angles[2],-90);self.assertEqual(d.gyro[2],-1000)
        self.assertGreater(d.bad_checksum,0);self.assertLess(len(d.buffer),11)
        self.assertEqual(wrap_degrees(-358),2)

    def test_calibration_freshness_bad_packets_backlog_and_tilt(self):
        cfg=ImuSettings(port='/fake',enabled=True)
        link=ImuLink(cfg,serial_factory=MemorySerial,now=0)
        for n in range(121):
            link.port.buffer=frame(0x51,(0,0,2048))+frame(0x52,(0,0,2))+frame(0x53,(0,0,100))
            link.poll(n*.01)
        self.assertIsNone(link.reason(1.2));self.assertAlmostEqual(link.rate,0)
        self.assertEqual(link.reason(3),'IMU_STALE')
        link.port.buffer=b'x'*513;link.poll(3)
        self.assertEqual(link.reason(3),'IMU_BACKLOG')
        link.close();self.assertTrue(link.port.closed)
        link=ImuLink(cfg,serial_factory=MemorySerial,now=0)
        link.port.buffer=frame(0x51,(0,0,2048))+frame(0x52,(0,0,0))+frame(0x53,(10000,0,0))
        link.poll(0);self.assertEqual(link.reason(0),'IMU_TILT')

    def test_recorded_wt61_uart_capture(self):
        path=Path('artifacts/hardware/2026-11-08/imu/passive-115200.bin')
        if not path.exists():self.skipTest('Local hardware capture unavailable')
        d=WitDecoder();d.feed(path.read_bytes(),1)
        self.assertGreater(d.frames,300);self.assertIsNotNone(d.angles);self.assertIsNotNone(d.gyro)

    def feedback(self,right_sign=-1):
        link=SimpleNamespace(settings=ImuSettings(),yaw=0.,rate=0.,reason=lambda now:None)
        cfg=DriveSettings(max_speed_rpm=100,right_sign=right_sign,acceleration_rpm_s=30)
        f=YawFeedback(link,cfg)
        wheels=[SimpleNamespace(phase='running',last_status_at=0.,last_status={'state':'closed','fault':'none','req_mrpm':str(v*1000)}) for v in (40,40*right_sign)]
        return f,link,wheels

    def test_heading_sign_wrap_reverse_saturation_neutral_and_fault(self):
        f,link,wheels=self.feedback()
        out=Output(40,-40,True)
        f.correct((40,-40),out,wheels,0)
        link.yaw=5;link.rate=2 # Unwanted right yaw: reduce left / boost right.
        t=f.correct((40,-40),out,wheels,.05)
        self.assertLess(t[0],40);self.assertGreater(-t[1],40)
        f.reset();link.yaw=179;link.rate=0
        f.correct((40,-40),out,wheels,0);link.yaw=-179
        f.correct((40,-40),out,wheels,.05);self.assertEqual(f.heading_error,-2)
        f.reset();link.yaw=0
        for w in wheels:w.last_status['req_mrpm']=str(-int(w.last_status['req_mrpm']))
        out=Output(-40,40,True)
        f.correct((-40,40),out,wheels,0);link.yaw=5;link.rate=2
        t=f.correct((-40,40),out,wheels,.05)
        self.assertLess(t[0],-40);self.assertLess(t[1],40)
        self.assertEqual(f.correct((0,0),Output(),wheels,.06),(0,0));self.assertEqual(f.correction,0)
        link.reason=lambda now:'IMU_STALE'
        with self.assertRaises(ImuError):f.correct((-40,40),out,wheels,.07)

    def test_rate_target_uses_shaped_speed_and_cannot_start_or_reverse_wheel(self):
        f,link,wheels=self.feedback(right_sign=1)
        wheels[1].last_status['req_mrpm']='-40000'
        out=Output(100,-100,True)
        f.correct((40,-40),out,wheels,0)
        self.assertEqual(f.target_rate,36)
        link.rate=-300
        previous=(40,-40)
        for i in range(1,20):
            for w in wheels:w.last_status_at=i*.01
            t=f.correct((40,-40),out,wheels,i*.01)
            self.assertTrue(all(8<=abs(v)<=100 for v in t))
            self.assertLessEqual(t[0]-previous[0],.300001);previous=t
        self.assertEqual(f.correct((40,0),Output(40,0,True),wheels,.21)[1],0)
        f.reset();wheels[0].last_status['state']='ramp'
        self.assertEqual(f.correct((8,-8),out,wheels,.22),(8,-8))
        self.assertEqual(f.reason,'WAITING_FOR_WHEELS')

    def test_feedback_reduces_heading_disturbance_in_simple_plant(self):
        def simulate(enabled):
            f,link,wheels=self.feedback(right_sign=1)
            targets=(40,40);out=Output(40,40,True)
            for i in range(1000):
                now=i*.01
                for w in wheels:w.last_status_at=now
                targets=f.correct((40,40),out,wheels,now) if enabled else (40,40)
                desired=(targets[0]-targets[1])+2 # Constant unequal rolling drag.
                link.rate+=(desired-link.rate)*.08
                link.yaw=wrap_degrees(link.yaw+link.rate*.01)
            return abs(link.yaw)
        self.assertLess(simulate(True),simulate(False)*.5)

if __name__=='__main__':unittest.main()
