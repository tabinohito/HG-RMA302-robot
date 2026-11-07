import io
import json
import math
from pathlib import Path
import signal
from tempfile import TemporaryDirectory
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from hg_rma302.cli import main, run
from hg_rma302.config import Config
from hg_rma302.control import DriveSettings, InputState, Output
from hg_rma302.diagnostics import DriveLog


def records(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines()]


class LogTests(unittest.TestCase):
    def test_snapshot_copies_telemetry_and_encodes_initial_times_as_null(self):
        with TemporaryDirectory() as folder:
            path = Path(folder) / 'nested' / 'drive.jsonl'
            logger = DriveLog(path)
            state = {'state': 'ramp', 'req_mrpm': '-50000', 'est_mrpm': '-8000', 'iq_mA': '600'}
            wheel = SimpleNamespace(port=SimpleNamespace(path='right'), phase='running',
                                    last_sent_rpm=-50000, pending_rpm=None,
                                    last_status_at=3, last_status=state)
            pair = SimpleNamespace(wheels=[wheel], retry_count=1, retry_blocked=False, retry_after=4)
            logger.snapshot(InputState(dpad_y=1), Output(50, -50, True), pair, 3.1)
            state['est_mrpm'] = '-30000'
            self.assertIsNone(logger.close())
            row, end = records(path)
            self.assertIsNone(row['input']['last_input_at'])
            self.assertIsNone(row['input']['last_device_check_at'])
            self.assertEqual(row['output']['right_rpm'], -50)
            self.assertEqual(row['wheels']['left']['status']['est_mrpm'], '-8000')
            self.assertAlmostEqual(row['wheels']['left']['status_age_s'], .1)
            self.assertEqual(end, {'type': 'log_end', 'records_written': 1, 'dropped_records': 0})

    def test_explicit_path_is_never_overwritten_and_auto_paths_are_unique(self):
        with TemporaryDirectory() as folder:
            path = Path(folder) / 'existing.jsonl'
            path.write_text('previous recording\n')
            with self.assertRaises(FileExistsError):
                DriveLog(path)
            self.assertEqual(path.read_text(), 'previous recording\n')
            # Resolve auto-generated paths into the temporary directory.
            original_open = Path.open
            opened = []

            def open_under_tmp(path, *args, **kwargs):
                actual = Path(folder) / path.name
                opened.append(actual)
                return original_open(actual, *args, **kwargs)

            with patch('hg_rma302.diagnostics.Path.open', open_under_tmp), \
                    patch('hg_rma302.diagnostics.Path.mkdir'):
                first, second = DriveLog(), DriveLog()
            first.close(); second.close()
            self.assertNotEqual(first.path, second.path)
            self.assertTrue(all(path.exists() for path in opened))

    def test_slow_disk_never_blocks_producer_and_dropped_records_are_reported(self):
        entered, release = threading.Event(), threading.Event()

        class SlowFile(io.StringIO):
            def write(self, text):
                entered.set()
                if not release.wait(3):
                    raise OSError('test writer was not released')
                return super().write(text)

            def close(self):
                self.saved = self.getvalue()
                super().close()

        sink = SlowFile()
        with patch('hg_rma302.diagnostics.Path.open', return_value=sink), \
                patch('hg_rma302.diagnostics.Path.mkdir'):
            logger = DriveLog('unused', queue_size=2)
        try:
            logger.record('first')
            self.assertTrue(entered.wait(1))
            finished = threading.Event()

            def produce():
                for n in range(100):
                    logger.record('sample', n=n)
                finished.set()

            producer = threading.Thread(target=produce, daemon=True)
            producer.start()
            self.assertTrue(finished.wait(1), 'control producer blocked on slow disk')
            self.assertEqual(logger.dropped_records, 98)
            self.assertIn('省略', logger.warning())
        finally:
            release.set()
            warning = logger.close()
        self.assertIn('98', warning)
        self.assertEqual(json.loads(sink.saved.splitlines()[-1])['dropped_records'], 98)

    def test_write_failure_is_reported_without_raising_in_control_thread(self):
        class BrokenFile(io.StringIO):
            def write(self, text):
                raise OSError('disk full')

        with patch('hg_rma302.diagnostics.Path.open', return_value=BrokenFile()), \
                patch('hg_rma302.diagnostics.Path.mkdir'):
            logger = DriveLog('unused')
        logger.record('sample', value=math.nan)
        warning = logger.close()
        self.assertIn('disk full', warning)
        logger.record('later')  # A failed logger must not break the controller.


class FakePad:
    def __init__(self, *args, **kwargs):
        self.device = SimpleNamespace(name='F310', path='/dev/input/test',
                                      info=SimpleNamespace(vendor=0x045e, product=0x028e))
        self.requires_event_updates = False
        self.index = -1
        self.checked_at = time.monotonic()
        self.closed = False

    @property
    def state(self):
        return InputState(dpad_y=int(self.index >= 1), last_device_check_at=self.checked_at)

    def read(self):
        self.index += 1
        self.checked_at = time.monotonic()
        return [self.state]

    def close(self):
        self.closed = True


class FakePair:
    def __init__(self, config, *, fail=False):
        self.ports = [SimpleNamespace(path=side) for side in ('left', 'right')]
        self.wheels = [SimpleNamespace(port=port, phase='stopped', last_sent_rpm=None,
                                      pending_rpm=None, last_status_at=0, last_status=None)
                       for port in self.ports]
        self.retry_count, self.retry_after, self.retry_blocked = 0, 0, False
        self.notices, self.fault_events = [], []
        self.closed = False
        self.fail = fail

    def update(self, output, now):
        for wheel, rpm in zip(self.wheels, (output.left_rpm, output.right_rpm)):
            wheel.phase = 'running' if rpm else 'stopped'
            wheel.last_sent_rpm = round(rpm * 1000)
            wheel.last_status_at = now
            wheel.last_status = {'state': 'closed' if rpm else 'idle', 'fault': 'none',
                                 'req_mrpm': str(round(rpm * 1000)),
                                 'est_mrpm': str(30000 if rpm > 0 else -8000 if rpm < 0 else 0)}
        if output.left_rpm and self.fail:
            self.wheels[1].last_status['fault'] = 'observer_lock'
            self.fault_events.append({'side': 'right', 'status': dict(self.wheels[1].last_status)})
            self.notices.append('test recovery notice')
            raise RuntimeError('test ESC error')

    def recovery_display(self, now):
        return None

    def close(self):
        self.closed = True
        return []


class DriveLoggingTests(unittest.TestCase):
    def test_input_polling_remains_fast_without_quadrupling_periodic_log(self):
        clock = SimpleNamespace(now=100.)
        def sleep(seconds):
            clock.now += seconds
        cfg = Config(DriveSettings())
        pad, pair = FakePad(), FakePair(cfg)
        with TemporaryDirectory() as folder:
            path = Path(folder) / 'fast.jsonl'
            with patch('hg_rma302.cli.time', SimpleNamespace(monotonic=lambda: clock.now, sleep=sleep)), \
                    patch('hg_rma302.cli.Gamepad', return_value=pad), \
                    patch('hg_rma302.cli.EscPair', return_value=pair), \
                    patch('sys.stdout', new=io.StringIO()):
                run(cfg, live=True, seconds=.12, log_path=str(path))
            rows = records(path)
            frames = [r for r in rows if r['type'] == 'input_frame']
            samples = [r for r in rows if r['type'] == 'sample']
            self.assertGreaterEqual(len(frames), 23)
            self.assertLessEqual(len(frames), 25)
            self.assertGreaterEqual(len(samples), 5)
            self.assertLessEqual(len(samples), 6)
            self.assertEqual(len(frames), pad.index + 1)

    def test_cli_log_forms_and_drive_only_validation(self):
        cfg = Config(DriveSettings())
        for arguments, expected in (([], None), (['--log'], ''), (['--log', 'run.jsonl'], 'run.jsonl')):
            with patch('hg_rma302.cli.load_config', return_value=cfg), patch('hg_rma302.cli.run') as runner:
                self.assertEqual(main(['drive', *arguments]), 0)
                self.assertEqual(runner.call_args.kwargs['log_path'], expected)
        with patch('sys.stderr', new=io.StringIO()), self.assertRaises(SystemExit):
            main(['check', '--log'])

    def test_drive_saves_both_wheels_inputs_settings_and_cleanup_on_success_or_error(self):
        cfg = Config(DriveSettings(max_speed_rpm=100, right_sign=-1), left_serial='L', right_serial='R')
        for fail in (False, True):
            with self.subTest(fail=fail), TemporaryDirectory() as folder:
                path = Path(folder) / 'drive.jsonl'
                pad, pair = FakePad(), FakePair(cfg, fail=fail)
                real_close = DriveLog.close

                def checked_close(logger):
                    self.assertTrue(pair.closed, 'logger drained before motor stop')
                    self.assertTrue(pad.closed)
                    return real_close(logger)

                with patch('hg_rma302.cli.Gamepad', return_value=pad), \
                        patch('hg_rma302.cli.EscPair', return_value=pair), \
                        patch('hg_rma302.cli.write_fault_events'), \
                        patch.object(DriveLog, 'close', checked_close), \
                        patch('sys.stdout', new=io.StringIO()):
                    if fail:
                        with self.assertRaisesRegex(RuntimeError, 'test ESC error'):
                            run(cfg, live=True, seconds=.12, log_path=str(path))
                    else:
                        run(cfg, live=True, seconds=.12, log_path=str(path))
                rows = records(path)
                self.assertEqual(rows[0]['type'], 'session_start')
                self.assertEqual(rows[0]['config']['drive']['max_speed_rpm'], 100)
                self.assertEqual(rows[0]['config']['drive']['right_sign'], -1)
                samples = [row for row in rows if row['type'] in ('sample', 'error_snapshot') and
                           row['input']['dpad_y'] == 1]
                self.assertTrue(samples)
                row = samples[-1]
                self.assertEqual(row['output']['left_rpm'], 50)
                self.assertEqual(row['output']['right_rpm'], -50)
                self.assertEqual(row['wheels']['left']['status']['est_mrpm'], '30000')
                self.assertEqual(row['wheels']['right']['status']['est_mrpm'], '-8000')
                self.assertEqual(rows[-1]['type'], 'log_end')
                self.assertEqual(rows[-1]['dropped_records'], 0)
                if fail:
                    self.assertTrue(any(row['type'] == 'esc_fault' for row in rows))
                    self.assertTrue(any(row['type'] == 'error' and row['message'] == 'test ESC error' for row in rows))
                else:
                    self.assertTrue(any(row['type'] == 'drive_exit' and row['reason'] == 'duration' for row in rows))

    def test_log_open_failure_happens_before_hardware_access(self):
        with TemporaryDirectory() as folder:
            path = Path(folder) / 'existing.jsonl'
            path.touch()
            with patch('hg_rma302.cli.Gamepad') as pad, patch('hg_rma302.cli.EscPair') as pair:
                with self.assertRaises(FileExistsError):
                    run(Config(DriveSettings()), live=True, log_path=str(path))
                pad.assert_not_called()
                pair.assert_not_called()

    def test_disabled_logging_does_not_open_a_file(self):
        with patch('hg_rma302.cli.DriveLog') as logger, patch('hg_rma302.cli._run'):
            run(Config(DriveSettings()))
            logger.assert_not_called()

    def test_ctrl_c_saves_exit_reason_and_restores_signal_handler(self):
        class InterruptingPad(FakePad):
            def read(self):
                frames = super().read()
                if self.index == 2:
                    signal.raise_signal(signal.SIGINT)
                return frames

        original_handler = signal.getsignal(signal.SIGINT)
        cfg = Config(DriveSettings())
        pad, pair = InterruptingPad(), FakePair(cfg)
        with TemporaryDirectory() as folder:
            path = Path(folder) / 'interrupted.jsonl'
            with patch('hg_rma302.cli.Gamepad', return_value=pad), \
                    patch('hg_rma302.cli.EscPair', return_value=pair), \
                    patch('sys.stdout', new=io.StringIO()):
                run(cfg, live=True, seconds=1, log_path=str(path))
            rows = records(path)
            self.assertTrue(pair.closed and pad.closed)
            self.assertTrue(any(row['type'] == 'drive_exit' and row['reason'] == 'SIGINT' for row in rows))
            self.assertEqual(rows[-1]['type'], 'log_end')
            self.assertEqual(signal.getsignal(signal.SIGINT), original_handler)


if __name__ == '__main__':
    unittest.main()
