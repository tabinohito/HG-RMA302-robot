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
from hg_rma302.control import DriveController, DriveSettings, InputState, Output
from hg_rma302.esc import EscPair, EscError, SerialPort
from test_esc import status


class Firmware:
    def __init__(self, old=False, *, stale_on_params=False, params_result="apply", protocol=4):
        self.master, self.slave = pty.openpty()
        self.path = os.ttyname(self.slave)
        self.old = old
        self.protocol = protocol
        self.acceleration = 20.0
        self.commands = []
        self.timeline = []
        self.running = False
        self.direction = 1
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
        self.line(status("closed" if self.running else "idle", count=self.count, direction=self.direction,
                         R_mOhm=str(self.resistance_mohm), L_uH=str(self.inductance_uh),
                         cart_protocol=str(self.protocol), slew_mrpm_s=str(round(self.acceleration*1000))))

    def loop(self):
        buffer = b""
        last_report = time.monotonic()
        while not self.ending.is_set():
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
                        self.line("ERR command; use help" if self.old else
                                  f"cart_protocol={self.protocol} signed_rpm=1 continuous_speed=1 controlled_reverse=1 controlled_zero=1 reverse_dwell_ms=2000 watchdog_ms=500 "
                                  "bus_min_mV=18000 bus_max_mV=22000 max_iq_mA=1000 max_speed_iq_mA=2000 phase_trip_mA=4000 min_mrpm=8000 max_mrpm=25000 startup_mA=600 slew_mrpm_s=20000 max_slew_mrpm_s=20000 startup_target_mrpm=8000 live_target=1")
                    elif command == "zero":
                        self.line("calibration=started")
                    elif command == "stop":
                        self.running = False
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
                        self.direction = 1 if float(command.split()[1]) > 0 else -1
                        self.acceleration = float(command.split()[3])
                        self.running = True
                        self.line("drive=started")
                        self.report()
                    elif command.startswith("rpm "):
                        value=float(command.split()[1])
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

    def test_held_lb_pivot_updates_reversing_wheel_without_stopping_other(self):
        boards = [Firmware(), Firmware()]
        pair = None
        try:
            settings = DriveSettings(right_sign=-1)
            pair = EscPair(Config(settings, left_port=boards[0].path, right_port=boards[1].path))
            controller = DriveController(settings, require_input_updates=False)

            def tick(**controls):
                now = time.monotonic()
                # One initial button event; held LB has no repeated events.
                output = controller.update(InputState(last_input_at=0, last_device_check_at=now, **controls), now)
                pair.update(output, now)
                time.sleep(.02)
                return output

            tick()
            tick(deadman=True)
            for _ in range(20):
                tick(deadman=True, throttle=1)
            self.assertTrue(all(board.running for board in boards))
            self.assertEqual([board.direction for board in boards], [1, -1])
            # Only the right wheel changes direction; the left must continue.
            output = tick(deadman=True, steering=1)
            self.assertEqual(output.reason, "DRIVING")
            for _ in range(20):
                tick(deadman=True, steering=1)
            self.assertTrue(all(board.running for board in boards))
            self.assertEqual([board.direction for board in boards], [1, 1])
            for board in boards:
                starts = [(at, cmd) for at, cmd in board.timeline if cmd.startswith("runrpm ")]
                self.assertEqual(len(starts), 1)
                self.assertFalse(any(cmd=="stop" and at>starts[0][0] for at,cmd in board.timeline))
            for _ in range(5):
                tick(steering=1)  # LB release must still stop both.
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
            self.assertIn("runrpm 8.500 2 20", boards[0].commands)
            self.assertIn("runrpm -8.500 2 20", boards[1].commands)
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
