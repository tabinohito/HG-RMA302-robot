import unittest
from dataclasses import replace
from types import SimpleNamespace
from hg_rma302.power import PowerLink, PowerError, parse_power_status, resolve_power_port
from hg_rma302.power_settings import PowerSettings, supply_state
from hg_rma302.power_drive import power_gate
from hg_rma302.config import Config
from hg_rma302.control import DriveSettings, DriveController, InputState


def status(**changes):
    d=dict(online=1,power=0,relay=0,ina=1,mode='V',mv=5000,ua=-100,mw=0,sw=1,dropped=0,
           power_protocol=1,session=17,seq=1,uptime_ms=1000,estop=0,host_state='USB_ONLY',
           stop_reason='none',sample_age_ms=50,current_valid=1,shunt_raw=-4,led_rgb=8,keys=0,drive_mode="tank",mode_led_rgb=2060)
    d.update(changes)
    return ' '.join(f'{k}={v}' for k,v in d.items())


class Board:
    def __init__(self,path,baudrate):
        self.path=path;self.now=0.;self.sent=[];self.lines=[];self.closed=False
        self.session=0;self.seq=0;self.relay=0;self.latched=0
        self.telemetry=True;self.ack=True;self.state='BOOT';self.last_report=-1
    def snapshot(self):
        return status(session=self.session,seq=self.seq,uptime_ms=round(self.now*1000),
                      relay=self.relay,power=self.relay,estop=self.latched,host_state=self.state)
    def send(self,command):
        self.sent.append(command);parts=command.split()
        if command=='cap':
            self.lines.append('power_protocol=1 watchdog_ms=750 remote_on=0 led_state=1 estop_latch=1 drive_modes=1 shunt_uohm=100000')
        elif parts[0]=='claim':
            self.session=int(parts[1]);self.lines.append(f'OK claim={self.session}')
        elif parts[0]=='host':
            self.seq=int(parts[2]);self.state=parts[3]
            if self.ack:self.lines.append(f'OK host={self.session} seq={self.seq}')
        elif command=='estop':
            self.relay=0;self.latched=1;self.lines.append('OK estop')
        elif parts[0]=='clear':
            self.latched=0;self.lines.append(f'OK clear={self.session}')
        elif parts[0]=='release':
            self.relay=0
            self.lines.append(f'OK release={self.session}')
            self.session=0
            self.lines.append(self.snapshot())
        if self.telemetry and self.session:
            self.lines.append(self.snapshot());self.last_report=self.now
    def read_lines(self):
        if self.telemetry and self.session and self.now-self.last_report>=.1:
            self.lines.append(self.snapshot());self.last_report=self.now
        lines,self.lines=self.lines,[]
        return lines
    def close(self):self.closed=True


class PowerTests(unittest.TestCase):
    def connect(self):
        settings=PowerSettings(enabled=True,port='/fake/pico')
        link=PowerLink(settings,port_factory=Board,now=0)
        for i in range(21):
            link.port.now=i*.01;link.poll(i*.01)
        self.assertTrue(link.ready)
        return link

    def test_complete_identity_no_enumeration_fallback(self):
        cfg=PowerSettings(enabled=True,serial='PICO-EXACT',port='/dev/old')
        good=SimpleNamespace(device='/dev/new',vid=0x2e8a,pid=0x000a,serial_number='PICO-EXACT')
        self.assertEqual(resolve_power_port(cfg,[good]),'/dev/new')
        with self.assertRaises(PowerError):resolve_power_port(cfg,[])
        with self.assertRaises(PowerError):resolve_power_port(cfg,[replace_ns(good,vid=0x0483)])
        with self.assertRaises(PowerError):resolve_power_port(cfg,[good,replace_ns(good,device='/dev/other')])

    def test_signed_measurements_and_saturation_not_reported_as_valid_current(self):
        link=self.connect();link.status=parse_power_status(status(session=link.session))
        self.assertEqual(link.snapshot(.2)['current_a'],-.0001)
        link.status=parse_power_status(status(session=link.session,shunt_raw=32767,ua=819175,current_valid=0))
        snapshot=link.snapshot(.2)
        self.assertTrue(snapshot['current_saturated']);self.assertIsNone(snapshot['current_a'])
        self.assertEqual(snapshot['voltage_v'],5)
        link.status=parse_power_status(status(session=link.session,ina=0,mv='NA',ua='NA',mw='NA',current_valid=0))
        self.assertIsNone(link.snapshot(.2)['voltage_v'])
        for values in ({'mv':'nan'}, {'estop':1,'relay':1,'power':1},{'seq':-1},
                       {'current_valid':1,'shunt_raw':32767},{'power_protocol':0}):
            with self.subTest(values=values),self.assertRaises(PowerError):parse_power_status(status(**values))

    def test_estop_clear_never_energizes_and_close_releases(self):
        link=self.connect();board=link.port
        board.relay=1
        link.estop();self.assertEqual(board.relay,0);self.assertTrue(link.estopped)
        board.now=.21;link.poll(.21)
        link.clear();self.assertEqual(board.relay,0)
        board.now=.22;link.poll(.22);self.assertFalse(link.estopped)
        link.close();self.assertTrue(board.closed)
        self.assertTrue(board.sent[-1].startswith('release '))
        self.assertFalse(any(c=='on' or c.startswith('on ') for c in board.sent))

    def test_missing_ack_and_frozen_status_are_bounded(self):
        link=self.connect();board=link.port;board.ack=False
        with self.assertRaisesRegex(PowerError,'heartbeat acknowledgment timeout'):
            for i in range(21,80): board.now=i*.01;link.poll(board.now)
        link=self.connect();board=link.port;board.telemetry=False;board.lines=[]
        with self.assertRaisesRegex(PowerError,'telemetry timeout'):
            for i in range(21,90): board.now=i*.01;link.poll(board.now)

    def test_wrong_session_ack_reboot_and_old_firmware_fail(self):
        link=self.connect();link.port.lines=[status(session=0,uptime_ms=1)]
        with self.assertRaisesRegex(PowerError,'session changed'):link.poll(.21)
        link=self.connect();link.port.lines=[f'OK host={link.session} seq=99']
        with self.assertRaisesRegex(PowerError,'acknowledgment'):link.poll(.21)
        with self.assertRaisesRegex(PowerError,'firmware'):parse_power_status('online=1 power=0 relay=0 ina=1')

    def test_voltage_gate_usb_only_mixed_and_drive_power_and_neutral(self):
        cfg=Config(DriveSettings(),power=PowerSettings(enabled=True,port='/pico'))
        link=SimpleNamespace(ready=True,estopped=False,status=parse_power_status(status()))
        pair=SimpleNamespace(wheels=[SimpleNamespace(last_status={'bus_mV':'5000'},last_status_at=1.,bus_limits_mv=(20000,26000)) for _ in range(2)])
        self.assertEqual(power_gate(cfg,link,pair,1.1),'USB_ONLY')
        link.status=parse_power_status(status(power=1,relay=1,mv=24000))
        pair.wheels[0].last_status['bus_mV']='24000'
        self.assertEqual(power_gate(cfg,link,pair,1.1),'USB_ONLY')
        pair.wheels[1].last_status['bus_mV']='24000'
        self.assertIsNone(power_gate(cfg,link,pair,1.1))
        self.assertEqual(power_gate(cfg,link,pair,1.6),'WAIT_ESC_STATUS')
        pair.wheels[0].last_status['bus_mV']='27000'
        with self.assertRaises(PowerError):power_gate(cfg,link,pair,1.1)
        controller=DriveController(cfg.drive)
        controller.update(InputState(last_input_at=0),0)
        self.assertTrue(controller.update(InputState(left_stick=1,last_input_at=.01),.01).enabled)
        controller.inhibit('USB_ONLY')
        self.assertFalse(controller.update(InputState(left_stick=1,last_input_at=.02),.02).enabled)
        controller.update(InputState(last_input_at=.03),.03)
        self.assertTrue(controller.update(InputState(left_stick=1,last_input_at=.04),.04).enabled)

    def test_configuration_bounds(self):
        for kw in ({'enabled':1},{'enabled':True},{'telemetry_timeout_s':.8},
                   {'usb_max_v':18},{'serial':' X'},{'usb_min_v':True}):
            with self.subTest(kw=kw),self.assertRaises(ValueError):PowerSettings(**kw)
        self.assertEqual(supply_state(5000,20000,26000,PowerSettings()),'USB_ONLY')
        self.assertEqual(supply_state(18000,20000,26000,PowerSettings()),'WAIT_POWER')
        self.assertEqual(supply_state(27000,20000,26000,PowerSettings()),'OVERVOLTAGE')


def replace_ns(value,**kw):
    return SimpleNamespace(**(vars(value)|kw))

if __name__=='__main__':unittest.main()
