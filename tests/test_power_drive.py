"""End-to-end host-loop interlocks with scripted USB devices; no hardware."""
import io
import unittest
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import patch
from hg_rma302.config import Config
from hg_rma302.control import DriveSettings, InputState
from hg_rma302.power import PowerLink, PowerError
from hg_rma302.power_settings import PowerSettings
from hg_rma302.power_drive import run_powered
from test_power import Board


class PoweredLoopTests(unittest.TestCase):
    def scenario(self, input_at, board_at, *, duration=1.5, preflight=.08, failure=None):
        clock=SimpleNamespace(now=0.)
        def sleep(n): clock.now+=max(n,.00001)
        timeline=[]; pairs=[]; links=[]; rows=[]
        class ClockBoard(Board):
            def read_lines(self):
                self.now=clock.now;board_at(self,clock.now)
                return super().read_lines()
            def send(self,command):
                self.now=clock.now;timeline.append((clock.now,'pico',command))
                if failure and failure(command): raise OSError('USB write failed')
                super().send(command)
        class Pad:
            device=SimpleNamespace(name='F310',path='/fake/pad')
            requires_event_updates=False
            closed=False
            def __init__(self):self.state=InputState();self.last=-1
            def read(self):
                frames=input_at(clock.now,self.last)
                self.last=clock.now
                if not isinstance(frames,list):frames=[frames]
                frames=[replace(f,last_device_check_at=clock.now) for f in frames]
                self.state=frames[-1]
                return frames
            def close(self):self.closed=True
        class Pair:
            def __init__(self,cfg,*,service,allow_standby):
                assert allow_standby
                self.ports=[SimpleNamespace(path=s) for s in ('left','right')]
                self.wheels=[SimpleNamespace(port=p,phase='stopped',last_status_at=clock.now,
                    last_status={'bus_mV':'5000','state':'idle'},bus_limits_mv=(18000,26000)) for p in self.ports]
                self.speed_sync=SimpleNamespace(reason=None)
                self.notices=[];self.fault_events=[];self.closed=False
                pairs.append(self)
                until=clock.now+preflight
                while clock.now<until:
                    service();sleep(.005)
            def poll_standby(self,now):
                for w in self.wheels:
                    w.last_status_at=now
                    w.last_status['bus_mV']='24000' if links[0].port.relay else '5000'
            def update(self,out,now):
                timeline.append((now,'esc',out))
                for w in self.wheels:
                    w.last_status_at=now
                    w.phase='running' if out.enabled and (out.left_rpm or out.right_rpm) else 'stopped'
            def recovery_display(self,now):return None
            def close(self):
                self.closed=True;timeline.append((clock.now,'esc','close'));return []
        def power_factory(settings):
            link=PowerLink(settings,port_factory=ClockBoard,now=clock.now)
            links.append(link);return link
        class Logger:
            def snapshot(self,sample,out,pair,now,**kw):
                rows.append((now,out,kw['power'].snapshot(now)))
            def record(self,*a,**kw):pass
            def warning(self):return None
        pad=Pad();caught=None
        cfg=Config(DriveSettings(),power=PowerSettings(enabled=True,port='/fake/pico'))
        stdout=io.StringIO()
        with patch('hg_rma302.power_drive.time',SimpleNamespace(monotonic=lambda:clock.now,sleep=sleep)), \
             patch('hg_rma302.power_drive.Gamepad',return_value=pad), \
             patch('hg_rma302.power_drive.PowerLink',side_effect=power_factory), \
             patch('hg_rma302.power_drive.EscPair',Pair),patch('sys.stdout',new=stdout):
            try:run_powered(cfg,seconds=duration,logger=Logger())
            except Exception as exc:caught=exc
        self.assertTrue(pad.closed)
        self.assertTrue(all(p.closed for p in pairs))
        self.assertTrue(all(p.port.closed for p in links))
        return timeline,rows,links[0],caught,stdout.getvalue()

    def test_usb_only_never_moves_and_restoring_power_requires_neutral(self):
        def inputs(now,last):
            return InputState(left_stick=1 if now<.9 or now>1.05 else 0)
        def board(b,now):
            if .6<now<.7:b.relay=1 # Physical button; not a host power-on command.
        events,rows,link,exc,_=self.scenario(inputs,board)
        self.assertIsNone(exc)
        moving=[t for t,device,out in events if device=='esc' and out!='close' and out.enabled and out.left_rpm]
        self.assertTrue(moving)
        self.assertGreater(min(moving),1.05)
        self.assertTrue(any(out.reason=='USB_ONLY' for _,out,_ in rows))
        self.assertEqual(link.port.relay,0)

    def test_brief_b_press_during_slow_preflight_latches_and_start_does_not_power_on(self):
        def inputs(now,last):
            if last<.28<=now:return [InputState(estop=True),InputState()]
            return InputState(reset=.85<now<.9)
        def board(b,now):
            if now<.27:b.relay=1
        events,rows,link,exc,_=self.scenario(inputs,board,preflight=.35)
        self.assertIsNone(exc)
        stops=[t for t,d,c in events if d=='pico' and c=='estop']
        self.assertEqual(len(stops),1);self.assertLess(stops[0],.29)
        self.assertFalse(any(d=='esc' and o!='close' and o.enabled and (o.left_rpm or o.right_rpm) for _,d,o in events))
        self.assertTrue(any(d=='pico' and c.startswith('clear ') for _,d,c in events))
        self.assertEqual(link.port.relay,0)
        self.assertFalse(link.port.latched)

    def test_usb_disconnect_and_lost_telemetry_stop_both_paths(self):
        for cause in ('pad','telemetry'):
            def inputs(now,last):return InputState(connected=not(cause=='pad' and now>.8),left_stick=int(now>.5))
            def board(b,now):
                if now<.75:b.relay=1
                if cause=='telemetry' and now>.8:b.telemetry=False
            events,rows,link,exc,_=self.scenario(inputs,board,duration=1.6)
            if cause=='telemetry':self.assertIsInstance(exc,PowerError)
            else:self.assertIsNone(exc)
            self.assertIn('estop',link.port.sent)
            self.assertEqual(link.port.relay,0)
            stop=next(t for t,d,c in events if d=='pico' and c=='estop')
            self.assertFalse(any(t>stop and d=='esc' and o!='close' and o.enabled and o.left_rpm for t,d,o in events))

    def test_failed_pico_stop_write_still_closes_esc(self):
        def inputs(now,last):return InputState(estop=now>.6)
        events,rows,link,exc,_=self.scenario(inputs,lambda b,n:None,failure=lambda c:c=='estop')
        self.assertIsInstance(exc,OSError)
        self.assertTrue(any(d=='esc' and c=='close' for _,d,c in events))

if __name__=='__main__':unittest.main()
