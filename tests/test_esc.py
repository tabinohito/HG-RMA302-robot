import unittest
from unittest.mock import patch

from hg_rma302.config import Config, MotorSettings, load_config
from hg_rma302.control import DriveSettings, Output
from hg_rma302.esc import EscError, EscPair, Wheel, parse_status, prepare


def status(state="idle", count=100, direction=1, fault="none", **overrides):
    fields = dict(state=state, fault=fault, cal="ok", n=str(count), MOE="0" if state == "idle" else "1",
                  CCER="0x0" if state == "idle" else "0x555", bus_mV="20000", R_mOhm="1911", L_uH="949",
                  speed_mode="0" if state == "idle" else "1", uart_direction=str(direction), cart_protocol="4", iq_max_mA="2000", slew_mrpm_s="20000")
    fields.update(overrides)
    return " ".join(f"{key}={value}" for key, value in fields.items())


class Port:
    path = "fake"

    def __init__(self):
        self.sent, self.lines = [], []
        self.fail_stop = False
        self.closed = False

    def send(self, command):
        self.sent.append(command)

    def read_lines(self):
        lines, self.lines = self.lines, []
        return lines

    def stop(self):
        self.sent.append("stop")
        if self.fail_stop:
            raise OSError("disconnected")

    def close(self):
        self.closed = True


class WheelTests(unittest.TestCase):
    def setUp(self):
        self.port = Port()
        self.wheel = Wheel(self.port, Config(DriveSettings()))

    def start(self, target=8.5):
        self.wheel.update(target, 0)
        self.assertEqual(self.port.sent[0], "arm")
        self.port.lines = ["armed for 5 s; ping required every <500 ms during drive"]
        self.wheel.receive(.02)
        self.wheel.update(target, .02)
        self.port.lines = ["drive=started", status("align", direction=1 if target > 0 else -1)]
        self.wheel.receive(.04)
        self.wheel.update(target, .04)

    def test_signed_start_and_speed(self):
        self.start(-8.5)
        self.assertIn("runrpm -8.500 2 20", self.port.sent)
        self.assertFalse(any(c.startswith("rpm ") for c in self.port.sent))
        self.port.lines = [status("closed", count=200, direction=-1)]
        self.wheel.receive(.06)
        self.wheel.update(-10, .06)
        self.assertIn("rpm -10.000", self.port.sent)
        self.port.lines = ["OK wheel_target_mrpm=-10000"]
        self.wheel.receive(.06)
        self.assertIsNone(self.wheel.pending_rpm)

    def test_zero_stops_instead_of_restarting(self):
        self.start()
        self.wheel.update(0, .06, enabled=False)
        self.assertEqual(self.port.sent[-2:], ["stop", "status"])
        self.port.lines = [status(count=200)]
        self.wheel.receive(.08)
        self.wheel.update(0, .08, enabled=False)
        self.assertEqual(self.wheel.phase, "stopped")

    def test_speed_updates_during_startup_and_ping_continues(self):
        self.start()
        for index, state in enumerate(("align", "ramp", "blend", "closed"), 1):
            now = .1 * index
            target = 9 + index
            self.port.lines = [status(state, count=100 + index)]
            self.wheel.receive(now)
            self.wheel.update(target, now)
            self.assertIn(f"rpm {target:.3f}", self.port.sent)
            self.port.lines = [f"OK wheel_target_mrpm={target * 1000}"]
            self.wheel.receive(now + .01)
        self.assertGreaterEqual(self.port.sent.count("ping"), 3)

    def test_only_latest_target_sent_after_delayed_ack(self):
        self.start()
        self.wheel.update(10, .06)
        self.wheel.update(11, .08)
        self.wheel.update(12, .10)
        self.port.lines = ["OK wheel_target_mrpm=10000"]
        self.wheel.receive(.12)
        self.wheel.update(13, .12)
        self.assertEqual([c for c in self.port.sent if c.startswith("rpm ")],
                         ["rpm 10.000", "rpm 13.000"])
        self.port.lines = ["OK wheel_target_mrpm=13000"]
        self.wheel.receive(.14)
        for now in (.14,.16,.18,.20):
            self.wheel.update(13, now)
        self.assertEqual(self.port.sent.count("rpm 13.000"), 1)

    def test_wrong_acceleration_fails(self):
        self.start()
        self.port.lines = [status("align", count=200, slew_mrpm_s="2000")]
        with self.assertRaisesRegex(EscError,"acceleration"):
            self.wheel.receive(.06)

    def test_wrong_current_limit_and_speed_ack_fail(self):
        self.start()
        self.port.lines = [status("closed", count=200, iq_max_mA="500")]
        with self.assertRaisesRegex(EscError, "current limit"):
            self.wheel.receive(.1)
        self.setUp()
        self.start()
        self.port.lines = [status("closed", count=200)]
        self.wheel.receive(.1)
        self.wheel.update(9, .1)
        self.port.lines = ["OK wheel_target_mrpm=8501"]
        with self.assertRaisesRegex(EscError, "acknowledgment"):
            self.wheel.receive(.12)

    def test_speed_outside_configured_range_rejected(self):
        for target in (7.999, 15.001, -7.999, float("nan")):
            with self.subTest(target=target), self.assertRaises(EscError):
                self.wheel.update(target, 0)
        self.assertEqual(self.port.sent, [])

    def test_direction_change_sent_to_firmware_without_host_stop(self):
        self.start()
        self.wheel.update(-10, .06)
        self.assertIn("rpm -10.000", self.port.sent)
        self.assertNotIn("stop", self.port.sent)

    def test_neutral_brakes_and_ready_accepts_latest_reverse(self):
        self.start()
        self.wheel.update(0, .06)
        self.assertIn("rpm 0.000", self.port.sent)
        self.assertNotIn("stop", self.port.sent)
        self.port.lines=["OK wheel_target_mrpm=0", status("ready", count=200, MOE="0", CCER="0x0")]
        self.wheel.receive(.08)
        self.wheel.update(-10,.1)
        self.assertIn("rpm -10.000",self.port.sent)
        self.wheel.update(0,.12,enabled=False)
        self.assertIn("stop",self.port.sent)
        self.assertIsNone(self.wheel.pending_rpm)

    def test_missing_speed_ack_fails(self):
        self.start()
        self.port.lines = [status("closed", count=200)]
        self.wheel.receive(.06)
        self.wheel.update(9, .06)
        self.port.lines = [status("closed", count=300)]
        self.wheel.receive(.32)
        with self.assertRaisesRegex(EscError, "acknowledgment"):
            self.wheel.update(8.5, .32)

    def test_unexpected_idle_fails(self):
        self.start()
        self.port.lines = [status(count=200)]
        with self.assertRaisesRegex(EscError, "stopped unexpectedly"):
            self.wheel.receive(.1)

    def test_fault_and_wrong_direction_fail(self):
        for kwargs in ({"fault": "observer"}, {"direction": 0}):
            self.setUp()
            self.start()
            self.port.lines = [status("closed", count=200, **kwargs)]
            with self.subTest(kwargs=kwargs), self.assertRaises(EscError):
                self.wheel.receive(.1)

    def test_missing_arm_ack_and_telemetry(self):
        self.wheel.update(8.5, 0)
        with self.assertRaisesRegex(EscError, "arming timeout"):
            self.wheel.update(8.5, .5)
        self.setUp()
        self.start()
        self.port.lines = ["OK wheel_target_mrpm=8500"]
        self.wheel.receive(.1)
        with self.assertRaisesRegex(EscError, "telemetry timeout"):
            self.wheel.update(8.5, .5)

    def test_duplicate_status_cannot_mask_dead_adc(self):
        self.start()
        self.port.lines = [status("align", count=100), "OK wheel_target_mrpm=8500"]
        self.wheel.receive(.3)
        with self.assertRaisesRegex(EscError, "telemetry timeout"):
            self.wheel.update(8.5, .5)

    def test_stop_of_other_wheel_attempted_if_one_fails(self):
        pair = EscPair.__new__(EscPair)
        left, right = Port(), Port()
        left.fail_stop = True
        pair.ports = [left, right]
        errors = pair.close()
        self.assertEqual(len(errors), 1)
        self.assertEqual(right.sent, ["stop"])
        self.assertTrue(left.closed and right.closed)

    def test_pair_reads_both_before_any_new_drive(self):
        pair = EscPair.__new__(EscPair)
        left, right = Port(), Port()
        pair.ports = [left, right]
        pair.config = Config(DriveSettings())
        pair._initialize_recovery()
        pair.wheels = [Wheel(port, Config(DriveSettings())) for port in pair.ports]
        right.lines = [status(fault="current")]
        with self.assertRaises(EscError):
            pair.update(Output(8.5, 8.5, True), 0)
        self.assertEqual(left.sent, ["stop"])
        self.assertEqual(right.sent, ["stop"])


class ConfigurationTests(unittest.TestCase):
    def test_example_loads(self):
        self.assertEqual(load_config("config.example.json").drive.max_current_a, 2.0)

    def test_incomplete_status_rejected(self):
        with self.assertRaises(EscError):
            parse_status("state=closed fault=none")

    def test_same_port_rejected_before_open(self):
        with self.assertRaises(ValueError):
            EscPair(Config(DriveSettings(), left_port="/dev/ttyACM0", right_port="/dev/ttyACM0"))

    def test_current_cannot_exceed_two_amps(self):
        with self.assertRaises(ValueError):
            DriveSettings(max_current_a=2.01)

    def test_acceleration_range(self):
        for value in (0, 20.01, True, float('nan')):
            with self.subTest(value=value), self.assertRaises(ValueError):
                DriveSettings(acceleration_rpm_s=value)
        self.assertEqual(DriveSettings(acceleration_rpm_s=20).acceleration_rpm_s, 20)

    def test_motor_parameters_must_be_a_pair(self):
        with self.assertRaises(ValueError):
            MotorSettings(resistance_ohm=1)


if __name__ == "__main__":
    unittest.main()
