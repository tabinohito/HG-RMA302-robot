import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from hg_rma302.config import Config, MotorSettings
from hg_rma302.control import DriveSettings, Output
from hg_rma302.diagnostics import write_fault_events
from hg_rma302.esc import EscError, EscFault, EscPair
from test_esc import status


class Board:
    def __init__(self, path):
        self.path = path
        self.state, self.fault, self.direction = 'idle', 'none', 1
        self.count = 0
        self.bus = '20000'
        self.resistance = '1911'
        self.sent, self.lines = [], []

    def snapshot(self, **overrides):
        self.count += 100
        off = self.state in ('idle', 'fault')
        fields = dict(state=self.state, fault=self.fault, count=self.count, direction=self.direction,
                      MOE='0' if off else '1', CCER='0x0' if off else '0x555',
                      bus_mV=self.bus, R_mOhm=self.resistance)
        fields.update(overrides)
        return status(**fields)

    def read_lines(self):
        lines, self.lines = self.lines, []
        return lines + [self.snapshot()]

    def send(self, command):
        self.sent.append(command)
        if command == 'arm':
            self.lines.append('armed for 5 s; ping required every <500 ms during drive')
        elif command.startswith('runrpm '):
            self.state, self.fault = 'closed', 'none'
            self.direction = 1 if float(command.split()[1]) > 0 else -1
            self.lines.append('drive=started')
        elif command.startswith('rpm '):
            self.lines.append(f'OK wheel_target_mrpm={round(float(command.split()[1])*1000)}')

    def stop(self):
        self.sent.append('stop')
        self.state, self.fault = 'idle', 'none'

    def fail(self, reason='observer_lock'):
        self.state, self.fault = 'fault', reason

    def close(self):
        pass


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        self.boards = [Board('left'), Board('right')]
        self.config = Config(DriveSettings(), left_serial='L', right_serial='R')
        with patch('hg_rma302.esc.open_ports', return_value=self.boards), patch('hg_rma302.esc.prepare'):
            self.pair = EscPair(self.config)
        self.t = 0
        self.output = Output(15, -15, True)
        for _ in range(4):
            self.tick()
        self.assertTrue(all(w.phase == 'running' for w in self.pair.wheels))

    def tearDown(self):
        self.pair.close()

    def tick(self, output=None):
        self.t += .02
        if output is not None:
            self.output = output
        self.pair.update(self.output, self.t)

    def advance(self, seconds, output=None):
        if output is not None:
            self.output = output
        for _ in range(round(seconds/.02)):
            self.tick()

    def starts(self, board):
        return [s for s in board.sent if s.startswith('runrpm ')]

    def test_recovery_stops_both_and_restarts_with_latest_target(self):
        self.boards[0].fail()
        self.tick()
        self.assertEqual(self.pair.retry_count, 1)
        self.assertEqual(self.pair.recovery_display(self.t)[0], 'RECOVERING')
        self.assertTrue(all(b.sent[-2:] == ['stop', 'status'] for b in self.boards))
        self.advance(2.4, Output(-9, 12, True))
        self.assertTrue(all(len(self.starts(b)) == 1 for b in self.boards))
        self.advance(.2, Output(-10, 13, True))
        self.assertEqual(self.starts(self.boards[0])[-1], 'runrpm -10.000 2 20')
        self.assertEqual(self.starts(self.boards[1])[-1], 'runrpm 13.000 2 20')
        event = self.pair.fault_events[0]
        self.assertEqual(event['previous_status']['state'], 'closed')
        self.assertEqual((event['side'], event['usb_serial']), ('left', 'L'))
        self.assertIsNone(self.pair.recovery_display(self.t))

    def test_cancellation_does_not_replay_movement(self):
        self.boards[0].fail()
        self.tick()
        self.advance(.4)
        self.tick(Output())  # LB release/B/input fault all revoke enable
        self.advance(4)
        self.assertTrue(all(len(self.starts(b)) == 1 for b in self.boards))
        self.assertEqual(self.pair.retry_count, 0)
        self.assertEqual(self.pair.retry_after, 0)
        self.advance(.1, Output(9, -9, True))  # a fresh authorized hold
        self.assertTrue(all(len(self.starts(b)) == 2 for b in self.boards))

    def test_neutral_stays_stopped_then_uses_new_command(self):
        self.boards[0].fail()
        self.tick()
        self.advance(3, Output(0, 0, True))
        self.assertTrue(all(len(self.starts(b)) == 1 for b in self.boards))
        self.advance(.1, Output(-9, 9, True))
        self.assertTrue(all(len(self.starts(b)) == 2 for b in self.boards))

    def test_persistent_failure_holds_without_exiting_or_repeated_arming(self):
        for attempt in range(4):
            self.boards[0].fail()
            self.tick()
            self.advance(2.7)
            self.assertEqual(len(self.starts(self.boards[0])), min(attempt+2, 4))
        self.assertTrue(self.pair.retry_blocked)
        self.assertEqual(self.pair.recovery_display(self.t)[0], 'RECOVERY_LIMIT')
        self.advance(12)
        self.assertTrue(all(len(self.starts(b)) == 4 for b in self.boards))
        self.tick(Output())
        self.advance(.1, Output(10, -10, True))
        self.assertTrue(all(len(self.starts(b)) == 5 for b in self.boards))

    def test_healthy_closed_loop_resets_consecutive_retry_budget(self):
        self.boards[0].fail()
        self.tick()
        self.advance(2.7)
        self.assertEqual(self.pair.retry_count, 1)
        self.advance(10.1)
        self.assertEqual(self.pair.retry_count, 0)

    def test_other_fault_on_peer_prevents_observer_recovery(self):
        self.boards[0].fail()
        self.boards[1].fail('current')
        with self.assertRaisesRegex(EscFault, 'current'):
            self.tick()
        self.assertEqual(self.pair.retry_count, 0)
        self.assertTrue(all(b.sent[-1] == 'stop' for b in self.boards))

    def test_new_hardware_fault_while_clearing_observer_is_not_ignored(self):
        self.boards[0].fail()
        self.tick()
        self.boards[0].fail('bus')
        with self.assertRaisesRegex(EscFault, 'bus'):
            self.tick()

    def test_later_hard_fault_in_same_read_batch_prevents_recovery(self):
        self.boards[0].fail()
        self.boards[0].lines.append(self.boards[0].snapshot())
        self.boards[0].fail('current')
        with self.assertRaisesRegex(EscFault, 'current'):
            self.tick()
        self.assertEqual(self.pair.retry_count, 0)
        self.assertEqual(self.pair.fault_events[0]['status']['fault'], 'current')

    def test_old_observer_frame_does_not_spend_second_retry(self):
        self.boards[0].fail()
        self.tick()
        self.boards[0].lines.append(self.boards[0].snapshot(state='fault', fault='observer_lock'))
        self.tick()
        self.assertEqual(self.pair.retry_count, 1)
        self.assertEqual(self.pair.wheels[0].phase, 'stopped')

    def test_recovery_refuses_changed_parameters(self):
        self.pair.config = Config(DriveSettings(), left_motor=MotorSettings(1.911,.000949))
        self.boards[0].fail()
        self.tick()
        self.boards[0].resistance = '999'
        with self.assertRaisesRegex(EscError, 'parameters changed'):
            self.advance(2.7)
        self.assertTrue(all(len(self.starts(b)) == 1 for b in self.boards))

    def test_supply_fault_and_reboot_during_recovery_are_fatal(self):
        self.boards[0].fail()
        self.tick()
        self.boards[0].bus = '23000'
        with self.assertRaisesRegex(EscError, 'supply'):
            self.advance(2.7)
        self.boards[0].count = 0
        with self.assertRaisesRegex(EscError, 'rebooted'):
            self.tick()

    def test_recovery_requires_pwm_off(self):
        self.boards[0].lines.append(self.boards[0].snapshot(state='fault', fault='observer_lock', MOE='1'))
        with self.assertRaises(EscFault):
            self.tick()

    def test_recovery_can_be_disabled(self):
        self.pair.config = Config(DriveSettings(observer_retry_limit=0))
        self.boards[0].fail()
        with self.assertRaises(EscFault):
            self.tick()

    def test_fault_diagnostics_append_complete_frames(self):
        self.boards[0].fail()
        self.tick()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'logs'/'faults.jsonl'
            write_fault_events(self.pair.fault_events, path)
            write_fault_events(self.pair.fault_events, path)
            lines = [json.loads(line) for line in path.read_text().splitlines()]
        self.assertEqual(len(lines), 2)
        self.assertEqual(lines[0]['status']['fault'], 'observer_lock')
        self.assertEqual(lines[0]['previous_status']['state'], 'closed')
        self.assertIn('logged_at', lines[0])

    def test_settings_reject_unsafe_or_unbounded_retries(self):
        for kwargs in ({'observer_retry_limit': 6}, {'observer_retry_limit': True},
                       {'observer_retry_delay_s': 0}, {'observer_retry_delay_s': float('nan')}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                DriveSettings(**kwargs)
