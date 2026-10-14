import errno
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from hg_rma302.config import Config
from hg_rma302.control import DriveSettings
from hg_rma302.serial_devices import PortsUnavailable, open_ports, resolve_ports, serial_device_report


def usb(path, serial, vid=0x0483):
    return SimpleNamespace(device=path, serial_number=serial, vid=vid, pid=0x374b, description="ST-Link")


def config(**kwargs):
    return Config(DriveSettings(), left_serial="LEFT-UNIQUE", right_serial="RIGHT-UNIQUE",
                  **kwargs)


class OpenPort:
    def __init__(self, path, baudrate):
        self.path, self.closed = path, False

    def close(self):
        self.closed = True


class Clock:
    now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class SerialIdentityTests(unittest.TestCase):
    def test_reenumeration_and_unrelated_boards_do_not_swap_wheels(self):
        first = [usb('/dev/ttyACM0','RIGHT-UNIQUE'), usb('/dev/ttyACM1','LEFT-UNIQUE')]
        second = [usb('/dev/ttyACM0','XIAO',0x2886), usb('/dev/ttyACM1','RIGHT-UNIQUE'),
                  usb('/dev/ttyACM2','LEFT-UNIQUE')]
        cfg = config(left_port='/dev/ttyACM1', right_port='/dev/ttyACM0')
        self.assertEqual(resolve_ports(cfg,first),('/dev/ttyACM1','/dev/ttyACM0'))
        self.assertEqual(resolve_ports(cfg,second),('/dev/ttyACM2','/dev/ttyACM1'))

    def test_missing_identity_never_falls_back_to_stale_path(self):
        cfg = config(left_port='/dev/ttyACM0',right_port='/dev/ttyACM1',connect_timeout_s=0)
        with patch('hg_rma302.serial_devices.list_serial_ports', return_value=[usb('/dev/ttyACM0','RIGHT-UNIQUE')]), \
                patch('hg_rma302.serial_devices.OpenPort',create=True) as factory:
            with self.assertRaisesRegex(PortsUnavailable,'LEFT-UNIQUE'):
                open_ports(cfg,factory)
            factory.assert_not_called()

    def test_wrong_vendor_suffix_and_ambiguous_identity_rejected(self):
        for ports in ([usb('/dev/ttyACM0','LEFT-UNIQUE',0x2886)],
                      [usb('/dev/ttyACM0','UNIQUE')]):
            with self.assertRaises(PortsUnavailable):
                resolve_ports(config(),ports)
        with self.assertRaisesRegex(ValueError,'複数'):
            resolve_ports(config(),[usb('/dev/ttyACM0','LEFT-UNIQUE'),usb('/dev/ttyACM2','LEFT-UNIQUE')])

    def test_duplicate_identity_and_same_resolved_device_rejected(self):
        with self.assertRaises(ValueError):
            Config(DriveSettings(),left_serial='SAME',right_serial='SAME')
        cfg = Config(DriveSettings(),left_serial='LEFT-UNIQUE',right_port='/dev/ttyACM1')
        with self.assertRaises(ValueError):
            resolve_ports(cfg,[usb('/dev/ttyACM1','LEFT-UNIQUE')])

    def test_waits_for_both_then_rechecks_after_open(self):
        ports = [usb('/dev/ttyACM2','LEFT-UNIQUE'),usb('/dev/ttyACM1','RIGHT-UNIQUE')]
        clock = Clock()
        with patch('hg_rma302.serial_devices.list_serial_ports',side_effect=[ports[:1],ports,ports]), \
                patch('hg_rma302.serial_devices.time.monotonic',clock.monotonic), \
                patch('hg_rma302.serial_devices.time.sleep',clock.sleep):
            opened = open_ports(config(),OpenPort)
        self.assertEqual([p.path for p in opened],['/dev/ttyACM2','/dev/ttyACM1'])
        self.assertAlmostEqual(clock.now,.1)

    def test_renumber_during_open_closes_both_before_retry(self):
        old = [usb('/dev/ttyACM0','LEFT-UNIQUE'),usb('/dev/ttyACM1','RIGHT-UNIQUE')]
        new = [usb('/dev/ttyACM2','LEFT-UNIQUE'),usb('/dev/ttyACM3','RIGHT-UNIQUE')]
        all_opened = []
        def factory(*args):
            port=OpenPort(*args)
            all_opened.append(port)
            return port
        with patch('hg_rma302.serial_devices.list_serial_ports',side_effect=[old,new,new,new]), \
                patch('hg_rma302.serial_devices.time.sleep'):
            opened = open_ports(config(),factory)
        self.assertTrue(all(p.closed for p in all_opened[:2]))
        self.assertEqual([p.path for p in opened],['/dev/ttyACM2','/dev/ttyACM3'])

    def test_failed_second_open_closes_first_and_resolves_again(self):
        old = [usb('/dev/ttyACM0','LEFT-UNIQUE'),usb('/dev/ttyACM1','RIGHT-UNIQUE')]
        new = [usb('/dev/ttyACM2','LEFT-UNIQUE'),usb('/dev/ttyACM3','RIGHT-UNIQUE')]
        first = OpenPort('/dev/ttyACM0',115200)
        with patch('hg_rma302.serial_devices.list_serial_ports',side_effect=[old,new,new]), \
                patch('hg_rma302.serial_devices.time.sleep'):
            from unittest.mock import Mock
            factory=Mock(side_effect=[first,OSError(errno.ENODEV,'unplugged'),
                                     OpenPort('/dev/ttyACM2',115200),OpenPort('/dev/ttyACM3',115200)])
            opened=open_ports(config(),factory)
        self.assertTrue(first.closed)
        self.assertEqual([p.path for p in opened],['/dev/ttyACM2','/dev/ttyACM3'])

    def test_busy_port_is_not_retried_and_permissions_are_explained(self):
        ports=[usb('/dev/ttyACM0','LEFT-UNIQUE'),usb('/dev/ttyACM1','RIGHT-UNIQUE')]
        for number,message in ((errno.EAGAIN,'使用中'),(errno.EACCES,'アクセス権限')):
            with self.subTest(number=number), \
                    patch('hg_rma302.serial_devices.list_serial_ports',return_value=ports), \
                    patch('hg_rma302.serial_devices.time.sleep') as sleep:
                def factory(*args):
                    raise OSError(number,'blocked')
                with self.assertRaisesRegex(RuntimeError,message):
                    open_ports(config(connect_timeout_s=0),factory)
                sleep.assert_not_called()

    def test_missing_timeout_is_bounded(self):
        clock=Clock()
        with patch('hg_rma302.serial_devices.list_serial_ports',return_value=[]), \
                patch('hg_rma302.serial_devices.time.monotonic',clock.monotonic), \
                patch('hg_rma302.serial_devices.time.sleep',clock.sleep):
            with self.assertRaisesRegex(PortsUnavailable,'タイムアウト'):
                open_ports(config(connect_timeout_s=.25),OpenPort)
        self.assertEqual(clock.now,.25)

    def test_legacy_paths_and_report_roles(self):
        self.assertEqual(resolve_ports(Config(DriveSettings(),left_port='/dev/pts/10',right_port='/dev/pts/11')),
                         ('/dev/pts/10','/dev/pts/11'))
        ports=[usb('/dev/ttyACM2','LEFT-UNIQUE'),usb('/dev/ttyACM0','XIAO',0x2886),usb('/dev/ttyACM1','RIGHT-UNIQUE')]
        with patch('hg_rma302.serial_devices.list_serial_ports',return_value=ports):
            self.assertEqual([p['role'] for p in serial_device_report(config())],[None,'right','left'])

    def test_invalid_identity_and_timeout_configuration(self):
        for value in ('',123,True,' X','X\n'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                Config(DriveSettings(),left_serial=value)
        for value in (-1,31,True,float('nan'),'5'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                config(connect_timeout_s=value)
