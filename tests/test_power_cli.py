import io
import unittest
from types import SimpleNamespace
from unittest.mock import patch
from dataclasses import replace
from tempfile import TemporaryDirectory
from pathlib import Path
import json
from hg_rma302.config import Config, load_config
from hg_rma302.control import DriveSettings, InputState, Output
from hg_rma302.power import monitor_power
from hg_rma302.power_settings import PowerSettings
from hg_rma302.diagnostics import DriveLog
from hg_rma302.cli import main,run
from test_power import status,Board
import test_power

class PowerCliTests(unittest.TestCase):
    def test_read_only_monitor_does_not_claim_or_release_or_switch_power(self):
        clock=SimpleNamespace(now=0)
        board=Board('/pico',115200);board.relay=1
        def sleep(n):
            clock.now+=n
            if board.sent:board.lines=[status(relay=1,power=1)]
        with patch('hg_rma302.power.SerialPort',return_value=board), \
             patch('hg_rma302.power.time',SimpleNamespace(monotonic=lambda:clock.now,sleep=sleep)), \
             patch('sys.stdout',new=io.StringIO()) as output:
            monitor_power(PowerSettings(port='/pico'),seconds=.4)
        self.assertTrue(output.getvalue())
        self.assertTrue(board.sent);self.assertEqual(set(board.sent),{'status'})
        self.assertTrue(board.closed);self.assertEqual(board.relay,1)

    def test_power_fields_are_recorded_and_copied(self):
        link=test_power.PowerTests().connect()
        with TemporaryDirectory() as folder:
            path=Path(folder)/'power.jsonl'
            log=DriveLog(path)
            log.snapshot(InputState(),Output(),None,.2,power=link)
            link.status['mv']='24000'
            self.assertIsNone(log.close())
            row=json.loads(path.read_text().splitlines()[0])
            self.assertEqual(row['power']['voltage_v'],5)
            self.assertEqual(row['power']['status']['mv'],'5000')

    def test_config_load_and_dry_run_remains_offline(self):
        cfg=Config(DriveSettings(),power=PowerSettings(enabled=True,port='/pico'))
        with patch('hg_rma302.cli._run') as offline,patch('hg_rma302.power_drive.run_powered') as powered:
            run(cfg)
            offline.assert_called_once();powered.assert_not_called()
        with patch('hg_rma302.cli.load_config',return_value=cfg), \
             patch('hg_rma302.power.monitor_power') as monitor:
            self.assertEqual(main(['power','--config','/fake','--seconds','1']),0)
            monitor.assert_called_once_with(cfg.power,seconds=1)
        with TemporaryDirectory() as folder:
            path=Path(folder)/'config.json'
            path.write_text(json.dumps({'power':{'enabled':True,'serial':'ABC'}}))
            loaded=load_config(path)
            self.assertTrue(loaded.power.enabled);self.assertEqual(loaded.power.serial,'ABC')

if __name__=='__main__':unittest.main()
