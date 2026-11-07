#!/usr/bin/env python3
"""Hardware integration check using synthetic neutral/B/Start input. Never arms ESCs.

Requires installed power_protocol=1 firmware. Switches SSR OFF and finishes OFF.
Stops both ESCs, exercises the real host loop, then verifies Pico heartbeat timeout.
"""
import argparse
from dataclasses import replace
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from hg_rma302.cli import run
from hg_rma302.config import load_config
from hg_rma302.control import InputState
from hg_rma302.esc import EscPair,SerialPort,query_status,check_stopped
from hg_rma302.power import PowerLink,parse_power_status
from hg_rma302.power_settings import PowerSettings
from hg_rma302.serial_devices import open_ports


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',default='config.local.json')
    parser.add_argument('--serial',required=True,help='Exact Pico USB serial')
    parser.add_argument('--output',required=True,help='New directory for test records')
    args=parser.parse_args()
    folder=Path(args.output);folder.mkdir(parents=True,exist_ok=False)
    cfg=load_config(args.config)
    cfg=replace(cfg,power=PowerSettings(enabled=True,serial=args.serial))
    commands=[]
    class StoppedPort(SerialPort):
        def send(self,command):
            for line in command.splitlines():
                if line.split() and line.split()[0] in ('arm','runrpm','rpm','run','current','ping'):
                    raise AssertionError('Rotation command prohibited in standby verification: '+command)
            commands.append({'time':time.monotonic(),'path':self.path,'command':command})
            super().send(command)
    class NeutralPad:
        requires_event_updates=False
        device=SimpleNamespace(name='SYNTHETIC NEUTRAL / B / START (no motion)',path='test-input')
        def __init__(self,*a,**kw):self.started=time.monotonic();self.state=InputState()
        def read(self):
            now=time.monotonic();elapsed=now-self.started
            self.state=InputState(estop=3.5<elapsed<3.6,reset=4.5<elapsed<4.6,
                                  last_device_check_at=now,last_input_at=now)
            return [self.state]
        def close(self):pass
    def poll_for(link,seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            link.poll(time.monotonic(),'USB_ONLY');time.sleep(.005)
    def open_link():
        link=PowerLink(cfg.power)
        try:
            poll_for(link,.5)
            assert link.ready
            return link
        except BaseException:
            link.close();raise
    # Exclusively acquire both ESCs before touching the power controller.
    ports=open_ports(cfg,StoppedPort)
    link=None
    try:
        for p in ports:p.stop()
        for p in ports:check_stopped(query_status(p))
        link=open_link();link.estop();poll_for(link,.15)
        assert link.status['estop']=='1' and link.status['relay']=='0'
        link.clear();poll_for(link,.15)
        assert not link.estopped and link.status['relay']=='0'
    finally:
        if link:
            errors=link.close();assert not errors,errors
        for p in ports:
            p.stop();p.close()
    with patch('hg_rma302.power_drive.Gamepad',NeutralPad), \
         patch('hg_rma302.power_drive.EscPair',side_effect=lambda config,**kw:EscPair(config,port_factory=StoppedPort,**kw)):
        run(cfg,live=True,seconds=7,log_path=str(folder/'neutral-drive.jsonl'))
    link=open_link()
    try:
        assert link.status['relay']=='0'
        link.clear();poll_for(link,.2)
        link.port.read_lines()
        time.sleep(.9) # Deliberately withhold host heartbeat; SSR already OFF.
        frames=[parse_power_status(line) for line in link.port.read_lines() if line.startswith('online=')]
        assert frames and frames[-1]['relay']=='0' and frames[-1]['estop']=='1'
        assert frames[-1]['stop_reason']=='host_timeout',frames[-1]
        (folder/'watchdog.json').write_text(json.dumps(frames,indent=2)+'\n')
    finally:
        errors=link.close();assert not errors,errors
    link=open_link()
    try:
        link.clear();poll_for(link,.2)
        assert not link.estopped and link.status['relay']=='0'
        final_power=link.snapshot(time.monotonic())
    finally:
        errors=link.close();assert not errors,errors
    ports=open_ports(cfg,StoppedPort)
    try:
        final_esc=[]
        for p in ports:
            p.stop();s=query_status(p);check_stopped(s);final_esc.append(s)
    finally:
        for p in ports:p.close()
    (folder/'result.json').write_text(json.dumps({'finished_at':datetime.now(timezone.utc).isoformat(),
        'motor_commands':0,'power':final_power,'escs':final_esc,'commands':commands},indent=2)+'\n')
    print('PASS: neutral host loop, B/Start, SSR OFF, watchdog latch, both PWM off; no motor command sent')

if __name__=='__main__':main()
