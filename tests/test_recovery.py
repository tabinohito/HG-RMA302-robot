import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from hg_rma302.config import Config, MotorSettings
from hg_rma302.control import DriveSettings, Output
from hg_rma302.diagnostics import write_fault_events
from hg_rma302.esc import EscError, EscFault, EscPair, PreparedEsc
from test_esc import status


class Board:
    def __init__(self, path):
        self.path = path
        self.state, self.fault, self.direction = 'idle', 'none', 1
        self.count = 0
        self.target_rpm = 0.0
        self.bus = '20000'
        self.resistance = '1911'
        self.sent, self.lines = [], []

    def snapshot(self, **overrides):
        self.count += 100
        off = self.state in ('idle', 'fault')
        fields = dict(state=self.state, fault=self.fault, count=self.count, direction=self.direction,
                      MOE='0' if off else '1', CCER='0x0' if off else '0x555',
                      bus_mV=self.bus, R_mOhm=self.resistance,
                      req_mrpm=str(round(self.target_rpm*1000)), est_mrpm=str(round(self.target_rpm*1000)))
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
            self.target_rpm = float(command.split()[1])
            self.state, self.fault = 'closed', 'none'
            self.direction = 1 if float(command.split()[1]) > 0 else -1
            self.lines.append('drive=started')
        elif command.startswith('rpm '):
            self.target_rpm = float(command.split()[1])
            if self.target_rpm: self.direction = 1 if self.target_rpm > 0 else -1
            self.lines.append(f'OK wheel_target_mrpm={round(float(command.split()[1])*1000)}')

    def stop(self):
        self.sent.append('stop')
        self.state, self.fault = 'idle', 'none'
        self.target_rpm = 0.0

    def fail(self, reason='observer_lock'):
        self.state, self.fault = 'fault', reason

    def close(self):
        pass


class RecoveryTests(unittest.TestCase):
    rejection = 'ERR rpm requires active runrpm; min_mrpm=8000 max_mrpm=100000 fault=observer_lock'
    def setUp(self):
        self.boards = [Board('left'), Board('right')]
        self.config = Config(DriveSettings(), left_serial='L', right_serial='R')
        with patch('hg_rma302.esc.open_ports', return_value=self.boards), \
                patch('hg_rma302.esc.prepare', return_value=PreparedEsc({}, (18000, 22000))):
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
        self.assertEqual(self.starts(self.boards[0])[-1], 'runrpm -8.000 2 20')
        self.assertEqual(self.starts(self.boards[1])[-1], 'runrpm 8.000 2 20')
        self.assertEqual(self.pair.speed_sync.requested, (-10, 13))
        self.advance(1.5)
        self.assertAlmostEqual(self.boards[0].target_rpm, -10, delta=.01)
        self.assertAlmostEqual(self.boards[1].target_rpm, 13, delta=.01)
        event = self.pair.fault_events[0]
        self.assertEqual(event['previous_status']['state'], 'closed')
        self.assertEqual((event['side'], event['usb_serial']), ('left', 'L'))
        self.assertIsNone(self.pair.recovery_display(self.t))

    def test_fault_then_rpm_rejection_in_same_batch_still_recovers(self):
        self.boards[1].fail()
        self.boards[1].lines += [self.boards[1].snapshot(), self.rejection]
        self.tick()
        self.assertEqual(self.pair.retry_count, 1)
        self.assertTrue(all(b.state=='idle' for b in self.boards))
        self.advance(2.7, Output(-10, 13, True))
        self.assertTrue(all(len(self.starts(b))==2 for b in self.boards))

    def test_rpm_rejection_before_fault_in_same_batch_still_recovers(self):
        self.boards[1].fail()
        self.boards[1].lines.append(self.rejection)
        self.tick()
        self.assertEqual(self.pair.retry_count, 1)
        self.assertTrue(all(b.state=='idle' for b in self.boards))

    def begin_split_rejection(self):
        self.boards[1].fail()
        with patch.object(self.boards[1], 'read_lines', return_value=[self.rejection]):
            self.tick()
        self.assertIsNotNone(self.pair.wheels[1].awaiting_fault_since)
        self.assertEqual(self.boards[1].sent[-1], 'status')
        self.assertEqual(self.boards[0].state, 'idle')
        self.assertEqual(self.pair.retry_count, 0)  # No fabricated PWM-off proof.

    def test_split_rejection_stops_peer_then_confirms_fault_before_recovery(self):
        self.begin_split_rejection()
        count=len(self.boards[1].sent)
        with patch.object(self.boards[1], 'read_lines', return_value=[]):
            self.advance(.12)
        self.assertEqual(len(self.boards[1].sent),count)  # No run/rpm/ping while waiting.
        self.tick()
        self.assertEqual(self.pair.retry_count, 1)
        self.assertIsNone(self.pair.wheels[1].awaiting_fault_since)
        self.advance(2.7, Output(-10, 13, True))
        self.assertTrue(all(len(self.starts(b))==2 for b in self.boards))

    def test_unconfirmed_rejection_times_out_and_never_rearms(self):
        self.begin_split_rejection()
        with patch.object(self.boards[1], 'read_lines', return_value=[]):
            with self.assertRaisesRegex(EscError, 'without confirmed fault status'):
                self.advance(.3)
        self.assertTrue(all(b.state=='idle' for b in self.boards))
        self.assertTrue(all(len(self.starts(b))==1 for b in self.boards))

    def test_b_cancels_split_rejection_wait(self):
        self.begin_split_rejection()
        with patch.object(self.boards[1], 'read_lines', return_value=[]):
            self.tick(Output())
        self.advance(3)
        self.assertTrue(all(b.state=='idle' for b in self.boards))
        self.assertTrue(all(len(self.starts(b))==1 for b in self.boards))

    def test_late_rejection_during_clear_does_not_spend_another_retry(self):
        self.boards[1].fail();self.tick()
        self.boards[1].lines.append(self.rejection)
        self.tick()  # This same read also acknowledges the stop with idle.
        self.assertEqual(self.pair.retry_count,1)
        self.assertIsNone(self.pair.wheels[1].awaiting_fault_since)

    def test_other_error_or_hard_fault_is_not_hidden_by_recoverable_rejection(self):
        self.boards[1].lines += [self.rejection, 'ERR invalid command']
        self.boards[1].fail()
        with self.assertRaisesRegex(EscError,'invalid command'):
            self.tick()
        self.assertEqual(self.pair.retry_count,0)


    def test_cancellation_does_not_replay_movement(self):
        self.boards[0].fail()
        self.tick()
        self.advance(.4)
        self.tick(Output())  # B/input faults revoke enable
        self.advance(4)
        self.assertTrue(all(len(self.starts(b)) == 1 for b in self.boards))
        self.assertEqual(self.pair.retry_count, 0)
        self.assertEqual(self.pair.retry_after, 0)
        self.advance(.1, Output(9, -9, True))  # a new command after neutral
        self.assertTrue(all(len(self.starts(b)) == 2 for b in self.boards))

    def test_neutral_stays_stopped_then_uses_new_command(self):
        self.boards[0].fail()
        self.tick()
        self.advance(3, Output(0, 0, True))
        self.assertEqual(self.pair.retry_count, 0)
        self.assertEqual(self.pair.retry_after, 0)
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

    def test_six_cell_recovery_uses_each_boards_advertised_voltage_limit(self):
        for wheel, board in zip(self.pair.wheels, self.boards):
            wheel.bus_limits_mv = (18000, 26000)
            board.bus = '25200'
        self.boards[0].fail()
        self.tick()
        self.advance(2.7)
        self.assertTrue(all(len(self.starts(b)) == 2 for b in self.boards))
        # One old ESC must retain its lower limit even with a new peer.
        self.pair.wheels[1].bus_limits_mv = (18000, 22000)
        self.boards[0].fail()
        self.tick()
        with self.assertRaisesRegex(EscError, '22000.*measured=25200.*overvoltage'):
            self.advance(2.7)

    def test_six_cell_recovery_still_rejects_overvoltage(self):
        for wheel in self.pair.wheels:
            wheel.bus_limits_mv = (18000, 26000)
        self.boards[0].fail()
        self.tick()
        self.boards[0].bus = '26001'
        with self.assertRaisesRegex(EscError, '26000.*measured=26001.*overvoltage'):
            self.advance(2.7)
        self.assertTrue(all(len(self.starts(b)) == 1 for b in self.boards))

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
