"""Input-device selection and decoding across the supported controller IDs."""

from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from hg_rma302.gamepad import Gamepad


def device(path, vendor, product, *, keys=(305, 310, 315)):
    result = Mock()
    result.path, result.name, result.phys, result.fd = path, "Test controller", "usb-test", 99
    result.info = SimpleNamespace(vendor=vendor, product=product)
    result.capabilities.return_value = {1: list(keys), 3: [0, 1]}
    result.absinfo.return_value = SimpleNamespace(value=0, min=-32768, max=32767)
    result.active_keys.return_value = []
    result.read.return_value = [
        SimpleNamespace(type=1, code=310, value=1, timestamp=lambda: 12.5),
        SimpleNamespace(type=3, code=1, value=-32768, timestamp=lambda: 12.5),
        SimpleNamespace(type=0, code=0, value=0, timestamp=lambda: 12.5),
    ]
    return result


class GamepadTests(unittest.TestCase):
    def test_autodetect_and_read_f310_f710_and_connected_xbox_id(self):
        for vendor, product in ((0x046d, 0xc21d), (0x046d, 0xc21f), (0x045e, 0x028e)):
            with self.subTest(usb=f"{vendor:04x}:{product:04x}"):
                opened = []

                def open_device(path):
                    pad = device(path, vendor, product)
                    opened.append(pad)
                    return pad

                evdev = SimpleNamespace(list_devices=lambda: ["/dev/input/test"], InputDevice=open_device)
                with patch("hg_rma302.gamepad._evdev", return_value=evdev), patch("hg_rma302.gamepad.fcntl.ioctl"), patch("hg_rma302.gamepad.time.monotonic", return_value=12.6):
                    pad = Gamepad()
                    try:
                        self.assertEqual(pad.device.path, "/dev/input/test")
                        frames = pad.read()
                        self.assertEqual(len(frames), 1)
                        self.assertTrue(frames[0].deadman)
                        self.assertEqual(frames[0].throttle, 1)
                        self.assertEqual(frames[0].last_input_at, 12.5)
                    finally:
                        pad.close()
                self.assertEqual(len(opened), 2)  # Discovery then the selected reader.
                for opened_device in opened:
                    opened_device.close.assert_called_once()
                opened[-1].grab.assert_called_once()

    def open_pad(self, vendor=0x046d, product=0xc21d):
        raw = device("/dev/input/test", vendor, product)
        backend = patch("hg_rma302.gamepad._evdev", return_value=SimpleNamespace(InputDevice=lambda path: raw))
        clock_ioctl = patch("hg_rma302.gamepad.fcntl.ioctl")
        backend.start()
        clock_ioctl.start()
        self.addCleanup(backend.stop)
        self.addCleanup(clock_ioctl.stop)
        pad = Gamepad(raw.path)
        self.addCleanup(pad.close)
        return pad, raw

    def test_wired_poll_keeps_event_timestamp_separate(self):
        pad, raw = self.open_pad()
        with patch("hg_rma302.gamepad.time.monotonic", return_value=12.6):
            pad.read()
        raw.read.side_effect = BlockingIOError()
        with patch("hg_rma302.gamepad.time.monotonic", return_value=20):
            self.assertEqual(pad.read(), [])
        self.assertFalse(pad.requires_event_updates)
        self.assertEqual(pad.state.last_input_at, 12.5)
        self.assertEqual(pad.state.last_device_check_at, 20)
        self.assertTrue(pad.state.deadman)

    def test_wired_poll_failure_is_not_ignored(self):
        pad, raw = self.open_pad()
        raw.read.side_effect = BlockingIOError()
        raw.active_keys.side_effect = OSError("device disconnected")
        with self.assertRaisesRegex(OSError, "disconnected"):
            pad.read()
        self.assertEqual(pad.state.last_device_check_at, -float("inf"))

    def test_wired_stale_button_queue_is_rejected(self):
        pad, raw = self.open_pad()
        with patch("hg_rma302.gamepad.time.monotonic", return_value=15):
            with self.assertRaisesRegex(OSError, "queue is stale"):
                pad.read()

    def test_wireless_pad_does_not_get_wired_liveness(self):
        pad, raw = self.open_pad(0x046d, 0xc21f)
        pad.read()
        raw.read.side_effect = BlockingIOError()
        pad.read()
        self.assertTrue(pad.requires_event_updates)
        self.assertEqual(pad.state.last_device_check_at, -float("inf"))
        self.assertEqual(pad.state.last_input_at, 12.5)
        raw.active_keys.assert_called_once()  # Initial snapshot only.

    def test_directinput_and_unrelated_devices_rejected_before_grab(self):
        for vendor, product in ((0x046d, 0xc216), (0x1234, 0x5678)):
            with self.subTest(usb=(vendor, product)):
                pad = device("/dev/input/test", vendor, product)
                evdev = SimpleNamespace(InputDevice=lambda path: pad)
                with patch("hg_rma302.gamepad._evdev", return_value=evdev):
                    with self.assertRaisesRegex(ValueError, "Unsupported gamepad"):
                        Gamepad(pad.path)
                pad.grab.assert_not_called()
                pad.close.assert_called_once()

    def test_missing_deadman_button_rejected_before_grab(self):
        pad = device("/dev/input/test", 0x046d, 0xc21d, keys=(305, 315))
        with patch("hg_rma302.gamepad._evdev", return_value=SimpleNamespace(InputDevice=lambda path: pad)):
            with self.assertRaisesRegex(ValueError, "mapping is unsupported"):
                Gamepad(pad.path)
        pad.grab.assert_not_called()
        pad.close.assert_called_once()

    def test_multiple_controllers_require_explicit_selection(self):
        pads = {"/dev/input/a": device("/dev/input/a", 0x046d, 0xc21d),
                "/dev/input/b": device("/dev/input/b", 0x045e, 0x028e)}
        evdev = SimpleNamespace(list_devices=lambda: list(pads), InputDevice=lambda path: pads[path])
        with patch("hg_rma302.gamepad._evdev", return_value=evdev):
            with self.assertRaisesRegex(RuntimeError, "found 2"):
                Gamepad()
        for pad in pads.values():
            pad.grab.assert_not_called()
            pad.close.assert_called_once()
