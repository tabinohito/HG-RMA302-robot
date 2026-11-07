import json
import tempfile
import time
import unittest
from dataclasses import replace
from pathlib import Path

from hg_rma302.config import Config, load_config
from hg_rma302.control import DriveSettings, Output
from hg_rma302.esc import EscError, EscPair
from hg_rma302.esc_settings import EscSettings
from test_serial_integration import Firmware


class RuntimeConfigTests(unittest.TestCase):
    def load(self, data):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'config.json'; path.write_text(json.dumps(data))
            return load_config(str(path))

    def test_common_values_and_per_wheel_overrides(self):
        cfg=self.load({'esc':{'bus_max_v':25.5,'ntc_trip_raw':2900},
                       'left_esc':{'ntc_trip_raw':2800}, 'right_esc':{'wheel_electrical_cycles':200}})
        self.assertEqual(cfg.esc.ntc_trip_raw,2900)
        self.assertEqual(cfg.left_esc.ntc_trip_raw,2800)
        self.assertEqual(cfg.right_esc.ntc_trip_raw,2900)
        self.assertEqual(cfg.right_esc.bus_max_v,25.5)
        wire=cfg.left_esc.wire(cfg.drive)
        self.assertEqual(len(wire),19)
        self.assertEqual(wire['bus_max_mv'],25500)
        self.assertEqual(wire['max_mrpm'],15000)
        self.assertEqual(wire['max_iq_ma'],2000)

    def test_invalid_types_unknown_fields_and_inconsistent_settings(self):
        for data in (
            {'esc':None}, {'left_esc':None}, {'right_esc':{'unknown':1}},
            {'esc':{'ntc_trip_raw':True}}, {'esc':{'ntc_trip_raw':2900.0}},
            {'esc':{'ntc_trip_raw':3201}}, {'esc':{'bus_max_v':26.001}},
            {'esc':{'bus_min_v':26}}, {'esc':{'startup_current_a':float('nan')}},
            {'esc':{'phase_trip_a':2}}, {'esc':{'current_sum_soft_a':1.2,'current_sum_hard_a':1.1}},
            {'esc':{'startup_speed_rpm':9}}, {'esc':{'wheel_electrical_cycles':1}},
            {'esc':{'overspeed_raw_ratio':1.2}}, {'esc':{'startup_lock_score':0}},
            {'esc':{'current_sum_trip_samples':41}}, {'esc':{'startup_current_a':.60001}},
            {'drive':{'max_speed_rpm':15.0001}},
            {'esc':{'acceleration_ff_a':-0.1}}, {'esc':{'acceleration_ff_a':1.001}},
            {'esc':{'acceleration_ff_a':float('nan')}}, {'esc':{'acceleration_ff_a':True}},
            {'esc':{'acceleration_ff_a':.30001}},
        ):
            with self.subTest(data=data), self.assertRaises((ValueError,TypeError)):
                self.load(data)

    def test_existing_json_gets_explicit_default_runtime_values(self):
        cfg=self.load({})
        self.assertEqual(cfg.esc.ntc_trip_raw,2873)
        cfg=load_config('config.example.json')
        self.assertEqual((cfg.right_esc or cfg.esc).wire(cfg.drive)['ntc_trip_raw'],2873)

    def test_two_wheel_settings_reapplied_after_reopen_without_flash(self):
        boards=[Firmware(bus_mv=25200,bus_max_mv=22000),Firmware(bus_mv=25200)]
        pair=None
        try:
            cfg=Config(DriveSettings(max_speed_rpm=25),left_port=boards[0].path,right_port=boards[1].path,
                       left_esc=EscSettings(ntc_trip_raw=2800,acceleration_ff_a=.3),
                       right_esc=EscSettings(ntc_trip_raw=3000,acceleration_ff_a=.5))
            pair=EscPair(cfg)
            tokens=[b.config_id for b in boards]
            self.assertEqual([b.settings['ntc_trip_raw'] for b in boards],[2800,3000])
            self.assertEqual([b.settings['accel_ff_ma'] for b in boards],[300,500])
            self.assertEqual([b.settings['max_mrpm'] for b in boards],[25000,25000])
            self.assertTrue(all(b.settings['bus_max_mv']==26000 for b in boards))
            pair.close(); pair=None
            cfg=replace(cfg,right_esc=EscSettings(ntc_trip_raw=2900),drive=DriveSettings(max_speed_rpm=12))
            pair=EscPair(cfg)
            self.assertNotEqual(tokens,[b.config_id for b in boards])
            self.assertEqual([b.settings['ntc_trip_raw'] for b in boards],[2800,2900])
            self.assertEqual([b.settings['accel_ff_ma'] for b in boards],[300,0])
            self.assertEqual([b.settings['max_mrpm'] for b in boards],[12000,12000])
            self.assertFalse(any('arm' in b.commands for b in boards))
        finally:
            if pair: pair.close()
            for b in boards: b.close()

    def test_incomplete_rejected_ignored_or_stale_configuration_stops_both(self):
        for mode in ('reject','ignore','stale','timeout'):
            boards=[Firmware(),Firmware(config_result=mode)]
            try:
                cfg=Config(DriveSettings(),left_port=boards[0].path,right_port=boards[1].path)
                with self.subTest(mode=mode),self.assertRaises(EscError): EscPair(cfg)
                time.sleep(.02)
                self.assertFalse(any('arm' in b.commands for b in boards))
                self.assertTrue(all('stop' in b.commands and not b.running for b in boards))
            finally:
                for b in boards: b.close()

    def test_old_firmware_requires_update(self):
        boards=[Firmware(config_protocol=1),Firmware()]
        try:
            with self.assertRaisesRegex(EscError,'runtime configuration unsupported'):
                EscPair(Config(DriveSettings(),left_port=boards[0].path,right_port=boards[1].path))
            self.assertFalse(any('arm' in b.commands for b in boards))
        finally:
            for b in boards: b.close()

    def test_config_token_loss_stops_both_during_operation(self):
        boards=[Firmware(),Firmware()]; pair=None
        try:
            pair=EscPair(Config(DriveSettings(),left_port=boards[0].path,right_port=boards[1].path))
            for _ in range(10): pair.update(Output(10,10,True),time.monotonic()); time.sleep(.02)
            self.assertTrue(all(b.running for b in boards))
            boards[1].config_id=0; boards[1].report(); time.sleep(.02)
            with self.assertRaisesRegex(EscError,'configuration changed or reset'):
                pair.update(Output(10,10,True),time.monotonic())
            time.sleep(.03)
            self.assertFalse(any(b.running for b in boards))
        finally:
            if pair: pair.close()
            for b in boards: b.close()

    def test_100_rpm_settings_and_tank_commands_reach_both_uarts(self):
        from hg_rma302.control import DriveController, InputState
        boards=[Firmware(),Firmware()];pair=None
        try:
            drive=DriveSettings(max_speed_rpm=100,right_sign=-1)
            pair=EscPair(Config(drive,left_port=boards[0].path,right_port=boards[1].path))
            self.assertEqual([b.settings['max_mrpm'] for b in boards],[100000,100000])
            control=DriveController(drive,require_input_updates=False)
            def ticks(controls,count=8):
                for _ in range(count):
                    now=time.monotonic()
                    out=control.update(InputState(**controls,last_device_check_at=now),now)
                    pair.update(out,now);time.sleep(.02)
            ticks({},1)
            ticks({'left_stick':1,'right_stick':1})
            self.assertIn('runrpm 8.000 2 20',boards[0].commands)
            self.assertIn('runrpm -8.000 2 20',boards[1].commands)
            self.assertTrue(all(abs(b.target_rpm)<10 for b in boards))
            ticks({'left_stick':1,'right_stick':1}, 270)
            self.assertAlmostEqual(boards[0].target_rpm, 100, delta=.1)
            self.assertAlmostEqual(boards[1].target_rpm, -100, delta=.1)
            for controls,expected in (({'dpad_y':1},(50,-50)),({'dpad_y':-1},(-50,50)),
                    ({'dpad_x':1},(25,25)),({'dpad_x':-1},(-25,-25)),({},(0,0))):
                # D-pad commands now wait for both observers and share an
                # acceleration ramp; their final operator targets are unchanged.
                ticks(controls, 160 if controls else 8)
                for b,rpm in zip(boards,expected):
                    sent = float([cmd for cmd in b.commands if cmd.startswith('rpm ')][-1].split()[1])
                    self.assertAlmostEqual(sent, rpm, delta=.5)
            ticks({'estop':True})
            self.assertFalse(any(b.running for b in boards))
        finally:
            if pair:pair.close()
            for b in boards:b.close()

    def test_25_rpm_firmware_rejects_100_before_settings_or_arm(self):
        boards=[Firmware(supported_max_mrpm=25000),Firmware()]
        try:
            cfg=Config(DriveSettings(max_speed_rpm=100),left_port=boards[0].path,right_port=boards[1].path)
            with self.assertRaisesRegex(EscError,'firmware supports up to 25 RPM'):
                EscPair(cfg)
            self.assertFalse(any(cmd=='arm' or cmd.startswith('cfgbegin') for b in boards for cmd in b.commands))
        finally:
            for b in boards:b.close()

    def test_100_rpm_electrical_ceiling_is_validated(self):
        self.load({'drive':{'max_speed_rpm':100}})
        for data in ({'drive':{'max_speed_rpm':100.001}},
                     {'drive':{'max_speed_rpm':100},'esc':{'wheel_electrical_cycles':217}}):
            with self.subTest(data=data),self.assertRaises(ValueError):self.load(data)

    def test_ntc_preflight_uses_configured_threshold(self):
        for threshold, accepted in ((2873,False),(2900,True)):
            boards=[Firmware(ntc=2874),Firmware()]; pair=None
            try:
                cfg=Config(DriveSettings(),left_port=boards[0].path,right_port=boards[1].path,
                           esc=EscSettings(ntc_trip_raw=threshold))
                if accepted: pair=EscPair(cfg)
                else:
                    with self.assertRaisesRegex(EscError,'NTC.*2873; measured=2874'): EscPair(cfg)
                self.assertFalse(any('arm' in b.commands for b in boards))
            finally:
                if pair: pair.close()
                for b in boards: b.close()
