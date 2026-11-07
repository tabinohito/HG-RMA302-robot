"""Actual pyserial -> two Linux pseudo terminals -> a UART firmware simulator."""

import os
import pty
import select
import threading
import time
import unittest
from unittest.mock import patch
from types import SimpleNamespace

from hg_rma302.config import Config, MotorSettings
from hg_rma302.esc_settings import EscSettings
from hg_rma302.control import DriveController, DriveSettings, InputState, Output
from hg_rma302.esc import EscPair, EscError, SerialPort, exchange
from test_esc import status


class Firmware:
    def __init__(self, old=False, *, stale_on_params=False, params_result="apply", protocol=4,
                 bus_mv=20000, bus_min_mv=18000, bus_max_mv=26000, config_protocol=2,
                 config_result='apply', ntc=2000, supported_max_mrpm=100000, cap_result='reply'):
        self.master, self.slave = pty.openpty()
        self.path = os.ttyname(self.slave)
        self.supported_max_mrpm = supported_max_mrpm
        self.cap_result = cap_result
        self.old = old
        self.protocol = protocol
        self.bus_mv, self.bus_min_mv, self.bus_max_mv = bus_mv, bus_min_mv, bus_max_mv
        self.acceleration = 20.0
        self.config_protocol, self.config_result = config_protocol, config_result
        self.settings = EscSettings().wire(DriveSettings(max_speed_rpm=25))
        self.config_id, self.pending_id, self.staged = 0, 0, {}
        self.ntc = ntc
        self.commands = []
        self.timeline = []
        self.running = False
        self.fault = 'none'
        self.inject_observer = threading.Event()
        self.direction = 1
        self.target_rpm = 0.0
        self.startup_pending = False
        self.count = 0
        self.resistance_mohm = 1911
        self.inductance_uh = 949
        self.stale_on_params = stale_on_params
        self.params_result = params_result
        self.ending = threading.Event()
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def line(self, line):
        # Fragment writes across arbitrary boundaries like the ST-Link VCP.
        data = (line + "\r\n").encode()
        middle = len(data) // 2
        os.write(self.master, data[:middle])
        os.write(self.master, data[middle:])

    def report(self):
        self.count += 2000
        state = "ramp" if self.startup_pending else "closed"
        self.line(status("fault" if self.fault != 'none' else state if self.running else "idle",
                         fault=self.fault, MOE='1' if self.running else '0',
                         CCER='0x555' if self.running else '0x0',
                         count=self.count, direction=self.direction,
                         req_mrpm=str(round(self.target_rpm*1000)),
                         est_mrpm=str(self.direction*8000 if self.startup_pending else round(self.target_rpm*1000)),
                         bus_mV=str(self.bus_mv),
                         ntc=str(self.ntc), config_id=str(self.config_id),
                         R_mOhm=str(self.resistance_mohm), L_uH=str(self.inductance_uh),
                         cart_protocol=str(self.protocol), slew_mrpm_s=str(round(self.acceleration*1000))))

    def loop(self):
        buffer = b""
        last_report = time.monotonic()
        while not self.ending.is_set():
            if self.inject_observer.is_set():
                self.inject_observer.clear()
                self.running, self.fault = False, 'observer_lock'
                self.report()
            if select.select([self.master], [], [], .01)[0]:
                data = os.read(self.master, 4096).replace(b"\x03", b"")
                buffer += data
                while b"\n" in buffer:
                    raw, buffer = buffer.split(b"\n", 1)
                    command = raw.decode().strip()
                    if not command:
                        continue
                    self.commands.append(command)
                    self.timeline.append((time.monotonic(), command))
                    if command == "cartcap":
                        if self.cap_result == 'silent':
                            continue
                        if self.cap_result == 'partial':
                            os.write(self.master, b'cart_protocol=4')
                            continue
                        if self.cap_result == 'unexpected':
                            self.line('different_device=yes')
                            continue
                        self.line("ERR command; use help" if self.old else
                                  f"cart_protocol={self.protocol} signed_rpm=1 continuous_speed=1 controlled_reverse=1 controlled_zero=1 reverse_dwell_ms=2000 watchdog_ms=500 "
                                  f"supported_max_mrpm={self.supported_max_mrpm} config_protocol={self.config_protocol} bus_min_mV={self.bus_min_mv} bus_max_mV={self.bus_max_mv} "
                                  "max_iq_mA=1000 max_speed_iq_mA=2000 phase_trip_mA=4000 min_mrpm=8000 max_mrpm=25000 startup_mA=600 slew_mrpm_s=20000 max_slew_mrpm_s=20000 startup_target_mrpm=8000 live_target=1")
                    elif command.startswith('cfgbegin '):
                        self.pending_id = int(command.split()[1]); self.staged = {}; self.config_id=0
                        self.line(f'config_begin={self.pending_id}')
                    elif command.startswith('cfgset '):
                        _, key, value = command.split()
                        if self.config_result == 'reject' and key == 'ntc_trip_raw':
                            self.line('ERR configuration rejected')
                        elif self.config_result != 'timeout':
                            self.staged[key] = int(value)
                            self.line(f'config_set={key} value={value}')
                    elif command.startswith('cfgapply '):
                        self.config_id=int(command.split()[1])
                        if self.config_result != 'ignore': self.settings=dict(self.staged)
                        self.line(f'config_applied={self.config_id}')
                    elif command == 'cartcfg':
                        token = self.config_id-1 if self.config_result=='stale' else self.config_id
                        self.line(f'config_version={self.config_protocol} config_id={token} pending=0 ' +
                                  ' '.join(f'{k}={v}' for k,v in self.settings.items()))
                    elif command == "zero":
                        self.line("calibration=started")
                    elif command == "stop":
                        self.running = False
                        self.target_rpm = 0.0
                        self.fault = 'none'
                    elif command == "status":
                        self.report()
                    elif command.startswith("params "):
                        if self.stale_on_params:
                            # A periodic status was already in flight when the
                            # host sent params. UART command processing is later.
                            self.report()
                            time.sleep(.04)
                        if self.params_result == "reject":
                            self.line("ERR parameters")
                        elif self.params_result == "apply":
                            _, resistance, inductance = command.split()
                            self.resistance_mohm = round(float(resistance) * 1000)
                            self.inductance_uh = round(float(inductance) * 1e6)
                        # Successful params has no acknowledgment, like the MCU.
                    elif command == "hfi off":
                        self.line("hfi=off valid=0")
                    elif command == "arm":
                        self.line("armed for 5 s; ping required every <500 ms during drive")
                    elif command.startswith("runrpm "):
                        self.target_rpm = float(command.split()[1])
                        self.direction = 1 if float(command.split()[1]) > 0 else -1
                        self.acceleration = float(command.split()[3])
                        self.running = True
                        self.line("drive=started")
                        self.report()
                    elif command.startswith("rpm "):
                        value=float(command.split()[1])
                        self.target_rpm = value
                        if value: self.direction=1 if value>0 else -1
                        self.line(f"OK wheel_target_mrpm={round(float(command.split()[1]) * 1000)}")
            if self.running and time.monotonic() - last_report >= .1:
                self.report()
                last_report = time.monotonic()

    def close(self):
        self.ending.set()
        self.thread.join(1)
        os.close(self.master)
        os.close(self.slave)


class SerialIntegrationTests(unittest.TestCase):
    def test_usb_standby_never_arms_until_both_bus_voltages_are_valid(self):
        boards=[Firmware(bus_mv=5000),Firmware(bus_mv=5000)]
        pair=None;services=[]
        try:
            cfg=Config(DriveSettings(),left_port=boards[0].path,right_port=boards[1].path)
            pair=EscPair(cfg,allow_standby=True,service=lambda:services.append(time.monotonic()))
            requested=Output(left_rpm=10,right_rpm=10,enabled=True,reason='DRIVING')
            def ticks(n):
                for _ in range(n):
                    now=time.monotonic();pair.poll_standby(now);pair.update(requested,now);time.sleep(.01)
            ticks(25)
            boards[0].bus_mv=24000
            ticks(25)
            self.assertFalse(any(c=='arm' or c.startswith('runrpm') for b in boards for c in b.commands))
            self.assertGreater(len(services),40)
            self.assertLess(max(b-a for a,b in zip(services,services[1:])),.15)
            boards[1].bus_mv=24000
            ticks(40)
            self.assertTrue(all('arm' in b.commands and any(c.startswith('runrpm') for c in b.commands) for b in boards))
        finally:
            if pair:pair.close()
            for board in boards:board.close()

    def test_timeout_distinguishes_silence_partial_and_unexpected_reply(self):
        for reply, expected in (
                ('silent', 'received_bytes=0, complete_lines=0, partial_bytes=0'),
                ('partial', 'received_bytes=15, complete_lines=0, partial_bytes=15'),
                ('unexpected', 'received_bytes=22, complete_lines=1, partial_bytes=0')):
            with self.subTest(reply=reply):
                board = Firmware(cap_result=reply)
                port = SerialPort(board.path)
                try:
                    with self.assertRaises(EscError) as caught:
                        exchange(port, 'cartcap', lambda ls: any(l.startswith('cart_protocol=') for l in ls), timeout=.08)
                    self.assertIn(expected, str(caught.exception))
                    if reply == 'partial':
                        self.assertIn("partial=b'cart_protocol=4'", str(caught.exception))
                    if reply == 'unexpected':
                        self.assertIn("last_line='different_device=yes'", str(caught.exception))
                finally:
                    port.close()
                    board.close()

    def test_silent_right_preflight_names_identity_and_closes_both_ports(self):
        boards = [Firmware(), Firmware(cap_result='silent')]
        ports = []
        def factory(path, baudrate):
            port = SerialPort(path, baudrate)
            ports.append(port)
            return port
        discovered = [SimpleNamespace(device=b.path, vid=0x0483, serial_number=s)
                      for b, s in zip(boards, ('LEFT-UNIQUE', 'RIGHT-UNIQUE'))]
        try:
            cfg = Config(DriveSettings(), left_serial='LEFT-UNIQUE', right_serial='RIGHT-UNIQUE')
            with patch('hg_rma302.serial_devices.list_serial_ports', return_value=discovered):
                with self.assertRaisesRegex(EscError, 'right ESC serial=RIGHT-UNIQUE: .*received_bytes=0'):
                    EscPair(cfg, port_factory=factory)
            self.assertEqual(len(ports), 2)
            self.assertTrue(all(not p.serial.is_open for p in ports))
            self.assertFalse(any(c == 'arm' or c.startswith('runrpm') for b in boards for c in b.commands))
        finally:
            for p in ports: p.close()
            for b in boards: b.close()

    def test_calibrated_stick_travel_reaches_uart_and_reductions_do_not_wait(self):
        boards = [Firmware(), Firmware()]
        pair = None
        try:
            settings = DriveSettings(max_speed_rpm=100, stick_max_speed_rpm=30, right_sign=-1)
            pair = EscPair(Config(settings, left_port=boards[0].path, right_port=boards[1].path))
            controller = DriveController(settings, require_input_updates=False)

            def ticks(travel, count):
                for _ in range(count):
                    now = time.monotonic()
                    output = controller.update(InputState(left_stick=travel, right_stick=travel,
                                                          last_device_check_at=now), now)
                    pair.update(output, now)
                    time.sleep(.02)

            ticks(0, 1)
            ticks(1, 110)
            for board, target in zip(boards, (30, -30)):
                self.assertAlmostEqual(board.target_rpm, target, delta=.01)
            for travel, rpm in ((.56, 19), (.2, 10), (.121, 8.025), (0, 0)):
                ticks(travel, 4)
                self.assertEqual([b.target_rpm for b in boards], [rpm, -rpm])
            # Starts are bounded; no hidden run command bypasses the profile.
            self.assertIn('runrpm 8.000 2 20', boards[0].commands)
            self.assertIn('runrpm -8.000 2 20', boards[1].commands)
        finally:
            if pair:
                pair.close()
            for board in boards:
                board.close()

    def test_stick_reduction_and_neutral_are_sent_without_filter_tail(self):
        boards = [Firmware(), Firmware()]
        pair = None
        try:
            pair = EscPair(Config(DriveSettings(max_speed_rpm=100),
                                 left_port=boards[0].path, right_port=boards[1].path))
            def ticks(output, count):
                for _ in range(count):
                    pair.update(output, time.monotonic())
                    time.sleep(.02)
            ticks(Output(100, -100, True), 75)
            self.assertTrue(all(abs(b.target_rpm)>25 for b in boards))
            ticks(Output(10, 0, True), 4)
            self.assertEqual([b.target_rpm for b in boards], [10, 0])
            ticks(Output(0, 0, True), 4)
            self.assertEqual([b.target_rpm for b in boards], [0, 0])
            ticks(Output(), 4)
            self.assertFalse(any(b.running for b in boards))
        finally:
            if pair:
                pair.close()
            for board in boards:
                board.close()

    def test_dpad_waits_for_late_observer_over_fragmented_uart(self):
        boards = [Firmware(), Firmware()]
        boards[1].startup_pending = True
        pair = None
        try:
            pair = EscPair(Config(DriveSettings(max_speed_rpm=100),
                                 left_port=boards[0].path, right_port=boards[1].path))
            output = Output(50, -50, True, synchronize=True)

            def drive_for(seconds, command):
                deadline = time.monotonic()+seconds
                while time.monotonic() < deadline:
                    pair.update(command, time.monotonic())
                    time.sleep(.02)

            drive_for(.7, output)
            self.assertTrue(all(b.running for b in boards))
            self.assertEqual([b.target_rpm for b in boards], [8, -8])
            self.assertEqual(pair.speed_sync.reason, 'WAITING_FOR_WHEELS')
            boards[1].startup_pending = False
            drive_for(.7, output)
            self.assertGreater(boards[0].target_rpm, 12)
            self.assertAlmostEqual(boards[0].target_rpm, -boards[1].target_rpm, delta=.5)
            drive_for(.1, Output())  # B/input fault disables PWM immediately.
            self.assertFalse(any(b.running for b in boards))
            self.assertEqual(pair.speed_sync.targets, (0, 0))
        finally:
            if pair:
                pair.close()
            for board in boards:
                board.close()

    def test_voltage_preflight_uses_json_limits_and_reports_actual_voltage(self):
        cases = ((25200, 26000, None), (26000, 26000, None),
                 (26001, 26000, 'overvoltage'), (17999, 26000, 'undervoltage'),
                 (25200, 22000, 'overvoltage'))
        for bus, maximum, error in cases:
            with self.subTest(bus=bus, maximum=maximum):
                boards = [Firmware(bus_mv=bus, bus_max_mv=maximum), Firmware()]
                pair = None
                try:
                    cfg = Config(DriveSettings(), left_port=boards[0].path, right_port=boards[1].path,
                                 esc=EscSettings(bus_max_v=maximum/1000))
                    if error:
                        with self.assertRaisesRegex(EscError, error) as caught:
                            EscPair(cfg)
                        if error != 'unsupported':
                            self.assertIn(f'measured={bus} mV', str(caught.exception))
                    else:
                        pair = EscPair(cfg)
                        self.assertEqual(pair.wheels[0].bus_limits_mv, (18000, maximum))
                    self.assertFalse(any('arm' in b.commands for b in boards))
                finally:
                    if pair:
                        pair.close()
                    for board in boards:
                        board.close()

    def test_observer_recovery_over_fragmented_uart_uses_latest_command(self):
        boards = [Firmware(bus_mv=25200), Firmware(bus_mv=25200)]
        pair = None
        try:
            pair = EscPair(Config(DriveSettings(), left_port=boards[0].path, right_port=boards[1].path))

            def drive_for(seconds, output):
                deadline = time.monotonic() + seconds
                while time.monotonic() < deadline:
                    pair.update(output, time.monotonic())
                    time.sleep(.02)

            drive_for(.3, Output(15, -15, True))
            self.assertTrue(all(b.running for b in boards))
            failed_at = time.monotonic()
            boards[0].inject_observer.set()
            drive_for(.3, Output(15, -15, True))
            self.assertFalse(any(b.running for b in boards))
            self.assertEqual(pair.retry_count, 1)
            drive_for(2.4, Output(-12, 9, True))
            self.assertTrue(all(b.running for b in boards))
            self.assertEqual([b.direction for b in boards], [-1, 1])
            for board, target in zip(boards, (-12, 9)):
                starts = [(at, cmd) for at, cmd in board.timeline if cmd.startswith('runrpm ')]
                self.assertEqual(len(starts), 2)
                self.assertGreaterEqual(starts[1][0] - failed_at, 2.5)
                self.assertEqual(starts[1][1], f'runrpm {8 if target>0 else -8:.3f} 2 20')
                self.assertTrue(any(cmd == 'stop' and starts[0][0] < at < starts[1][0]
                                    for at, cmd in board.timeline))
            drive_for(1.5, Output(-12, 9, True))
            self.assertAlmostEqual(boards[0].target_rpm, -12, delta=.01)
            self.assertAlmostEqual(boards[1].target_rpm, 9, delta=.01)
            drive_for(.1, Output())
            self.assertFalse(any(b.running for b in boards))
        finally:
            if pair:
                pair.close()
            for board in boards:
                board.close()

    def test_protocol_two_cannot_start_with_responsive_host(self):
        boards = [Firmware(protocol=2), Firmware()]
        try:
            with self.assertRaisesRegex(EscError, 'protocol 4'):
                EscPair(Config(DriveSettings(), left_port=boards[0].path, right_port=boards[1].path))
            self.assertFalse(any('arm' in board.commands for board in boards))
        finally:
            for board in boards:
                board.close()

    def test_usb_identity_reconnect_keeps_motor_parameters_with_correct_wheel(self):
        boards = [Firmware(), Firmware()]
        pair = None
        try:
            # Deliberately stale legacy paths: serial identity must win.
            config = Config(DriveSettings(), left_port=boards[1].path, right_port=boards[0].path,
                            left_serial='LEFT', right_serial='RIGHT',
                            left_motor=MotorSettings(1.887,.000881), right_motor=MotorSettings(1.85,.000930))
            for left_index, right_index in ((0,1),(1,0)):
                devices=[SimpleNamespace(device=boards[left_index].path,vid=0x0483,serial_number='LEFT'),
                         SimpleNamespace(device=boards[right_index].path,vid=0x0483,serial_number='RIGHT')]
                with patch('hg_rma302.serial_devices.list_serial_ports',return_value=devices):
                    pair=EscPair(config)
                self.assertEqual([p.path for p in pair.ports],[boards[left_index].path,boards[right_index].path])
                self.assertEqual((boards[left_index].resistance_mohm,boards[left_index].inductance_uh),(1887,881))
                self.assertEqual((boards[right_index].resistance_mohm,boards[right_index].inductance_uh),(1850,930))
                self.assertFalse(any('arm' in b.commands for b in boards))
                self.assertFalse(any(b.running for b in boards))
                pair.close()
                pair=None
        finally:
            if pair:
                pair.close()
            for board in boards:
                board.close()

    def test_measured_params_ignore_inflight_status_from_before_application(self):
        boards = [Firmware(stale_on_params=True), Firmware(stale_on_params=True)]
        pair = None
        try:
            config = Config(DriveSettings(), left_port=boards[0].path, right_port=boards[1].path,
                            left_motor=MotorSettings(1.887, .000881), right_motor=MotorSettings(1.85, .00093))
            pair = EscPair(config)
            self.assertEqual([(b.resistance_mohm, b.inductance_uh) for b in boards], [(1887, 881), (1850, 930)])
            for board in boards:
                self.assertNotIn("arm", board.commands)
                self.assertFalse(board.running)
        finally:
            if pair:
                pair.close()
            for board in boards:
                board.close()

    def test_rejected_params_cannot_be_masked_by_old_matching_status(self):
        boards = [Firmware(stale_on_params=True, params_result="reject"), Firmware()]
        try:
            config = Config(DriveSettings(), left_port=boards[0].path, right_port=boards[1].path,
                            left_motor=MotorSettings(1.911, .000949))
            with self.assertRaisesRegex(EscError, "ERR parameters"):
                EscPair(config)
            for board in boards:
                self.assertNotIn("arm", board.commands)
        finally:
            for board in boards:
                board.close()

    def test_unapplied_params_fail_with_expected_and_actual_values(self):
        boards = [Firmware(params_result="ignore"), Firmware()]
        try:
            config = Config(DriveSettings(), left_port=boards[0].path, right_port=boards[1].path,
                            left_motor=MotorSettings(1.887, .000881))
            with self.assertRaisesRegex(EscError, "expected R=1887 mOhm L=881 uH; actual R=1911 mOhm L=949 uH"):
                EscPair(config)
            for board in boards:
                self.assertNotIn("arm", board.commands)
        finally:
            for board in boards:
                board.close()

    def test_tank_pivot_updates_reversing_wheel_without_stopping_other(self):
        boards = [Firmware(), Firmware()]
        pair = None
        try:
            settings = DriveSettings(right_sign=-1)
            pair = EscPair(Config(settings, left_port=boards[0].path, right_port=boards[1].path))
            controller = DriveController(settings, require_input_updates=False)

            def tick(**controls):
                now = time.monotonic()
                # A wired pad can hold sticks without new events.
                output = controller.update(InputState(last_input_at=0, last_device_check_at=now, **controls), now)
                pair.update(output, now)
                time.sleep(.02)
                return output

            tick()
            for _ in range(20):
                tick(left_stick=1,right_stick=1)
            self.assertTrue(all(board.running for board in boards))
            self.assertEqual([board.direction for board in boards], [1, -1])
            # Only the right wheel changes direction; the left must continue.
            output = tick(left_stick=1,right_stick=-1)
            self.assertEqual(output.reason, "DRIVING")
            for _ in range(20):
                tick(left_stick=1,right_stick=-1)
            self.assertTrue(all(board.running for board in boards))
            self.assertEqual([board.direction for board in boards], [1, 1])
            for board in boards:
                starts = [(at, cmd) for at, cmd in board.timeline if cmd.startswith("runrpm ")]
                self.assertEqual(len(starts), 1)
                self.assertFalse(any(cmd=="stop" and at>starts[0][0] for at,cmd in board.timeline))
            for _ in range(5):
                tick(estop=True)  # B must stop both immediately.
            self.assertFalse(any(board.running for board in boards))
        finally:
            if pair:
                pair.close()
            for board in boards:
                board.close()

    def test_reopen_discards_partial_old_status_before_handshake(self):
        boards = [Firmware(), Firmware()]
        pair = None
        def open_stale(path, baudrate):
            port = SerialPort(path, baudrate)
            port.buffer.extend(b"state=idle fault=none cut-off-old-line")
            return port
        try:
            pair = EscPair(Config(DriveSettings(), left_port=boards[0].path, right_port=boards[1].path),
                           port_factory=open_stale)
            self.assertEqual(len(pair.wheels), 2)
            self.assertFalse(any(board.running for board in boards))
        finally:
            if pair:
                pair.close()
            for board in boards:
                board.close()

    def test_two_signed_wheels_then_stop(self):
        boards = [Firmware(), Firmware()]
        pair = None
        try:
            pair = EscPair(Config(DriveSettings(), left_port=boards[0].path, right_port=boards[1].path))
            for _ in range(30):
                pair.update(Output(8.5, -8.5, True), time.monotonic())
                time.sleep(.02)
            self.assertTrue(all(board.running for board in boards))
            self.assertIn("runrpm 8.000 2 20", boards[0].commands)
            self.assertIn("runrpm -8.000 2 20", boards[1].commands)
            self.assertAlmostEqual(boards[0].target_rpm, 8.5, delta=.05)
            self.assertAlmostEqual(boards[1].target_rpm, -8.5, delta=.05)
            for _ in range(10):
                pair.update(Output(), time.monotonic())
                time.sleep(.02)
            self.assertFalse(any(board.running for board in boards))
        finally:
            if pair:
                pair.close()
            for board in boards:
                board.close()

    def test_old_firmware_cannot_arm_either_board(self):
        boards = [Firmware(old=True), Firmware()]
        try:
            with self.assertRaises(EscError):
                EscPair(Config(DriveSettings(), left_port=boards[0].path, right_port=boards[1].path))
            time.sleep(.05)
            for board in boards:
                self.assertNotIn("arm", board.commands)
                self.assertIn("stop", board.commands)
        finally:
            for board in boards:
                board.close()
