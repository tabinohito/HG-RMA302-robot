"""Nonblocking RP2040 telemetry/LED/SSR link. Never sends a power-on command."""
import math
import secrets
import time
from .esc import SerialPort
from .serial_devices import list_serial_ports

STATES = {'BOOT','USB_ONLY','WAIT_POWER','READY','DRIVING','RECOVERING','STOPPED','FAULT'}

class PowerError(RuntimeError):
    pass


def resolve_power_port(settings, ports=None):
    if settings.serial:
        ports = list_serial_ports() if ports is None else ports
        matches = {p.device for p in ports if p.vid == 0x2e8a and p.pid == 0x000a
                   and p.serial_number == settings.serial}
        if len(matches) != 1:
            raise PowerError(f'Pico power serial={settings.serial}: expected one device, found {sorted(matches)}')
        return matches.pop()
    if settings.port:
        return settings.port
    raise PowerError('Configure power.serial or power.port')


def fields(line):
    result = {}
    for item in line.split():
        if '=' not in item:
            continue
        key,value = item.split('=',1)
        if key in result:
            raise PowerError(f'Duplicate power field: {key}')
        result[key] = value
    return result


def parse_power_status(line):
    if not line.startswith('online='):
        return None
    d=fields(line)
    required={'online','power','relay','ina','mode','mv','ua','mw','sw','dropped',
              'power_protocol','session','seq','uptime_ms','estop','host_state','stop_reason',
              'sample_age_ms','current_valid','shunt_raw','led_rgb','keys','drive_mode','mode_led_rgb'}
    if not required <= d.keys() or d['power_protocol']!='1':
        raise PowerError('Pico needs the embedded/pico_power_master firmware (power_protocol=1)')
    try:
        for key in ('online','power','relay','ina','sw','estop','current_valid'):
            if d[key] not in ('0','1'): raise ValueError(key)
        for key in ('session','seq','uptime_ms','sample_age_ms','dropped','led_rgb','keys'):
            if not d[key].isascii() or not d[key].isdecimal() or not 0 <= int(d[key]) <= 0xffffffff:
                raise ValueError(key)
        if d['mode'] not in ('V','I','P') or d['host_state'] not in STATES or d['drive_mode'] not in ('tank','joystick'):
            raise ValueError('mode/state')
        if not -32768 <= int(d['shunt_raw']) <= 32767 or not 0<=int(d['mode_led_rgb'])<=0xffffff or int(d['led_rgb']) > 0xffffff or int(d['keys']) > 7:
            raise ValueError('sensor/LED/keys')
        if d['power'] != d['relay'] or (d['estop']=='1' and d['relay']!='0'):
            raise ValueError('relay/estop mismatch')
        if d['ina']=='1':
            if not 0 <= int(d['mv']) <= 40959 or not -819200 <= int(d['ua']) <= 819175:
                raise ValueError('measurement')
            if not -33554432 <= int(d['mw']) <= 33554432:
                raise ValueError('power')
        elif any(d[k]!='NA' for k in ('mv','ua','mw')) or d['current_valid']!='0':
            raise ValueError('invalid sensor data')
        if d['current_valid']=='1' and int(d['shunt_raw']) in (-32768,32767):
            raise ValueError('saturated current marked valid')
    except (TypeError,ValueError) as exc:
        raise PowerError(f'Invalid Pico power status: {exc}') from exc
    return d


class PowerLink:
    def __init__(self, settings, *, port_factory=SerialPort, now=None):
        self.settings=settings
        self.port=port_factory(resolve_power_port(settings),115200)
        self.session=secrets.randbelow(0xffffffff)+1
        self.phase='opening'
        now=time.monotonic() if now is None else now
        self.deadline=now+1.5
        self.opened_at=now
        self.last_poll_at=now
        self.last_status_at=now
        self.last_heartbeat=-math.inf
        self.pending_at=0.
        self.pending_seq=None
        self.sequence=0
        self.acknowledged=0
        self.status=None
        self.local_estop=False
        self.clear_pending=False
        self.clear_sent_at=now
        self.closed=False
        self.notices=[]

    @property
    def ready(self):
        return self.phase=='active' and self.acknowledged>0 and self.status is not None

    @property
    def estopped(self):
        return self.local_estop or bool(self.status and self.status['estop']=='1')

    def estop(self):
        if not self.local_estop:
            self.local_estop=True
            self.clear_pending=False
            self.port.send('estop')

    def clear(self):
        if self.ready and self.estopped and not self.clear_pending and self.status['relay']=='0':
            self.port.send(f'clear {self.session}')
            self.clear_pending=True
            self.clear_sent_at=self.last_poll_at

    def poll(self, now, state='BOOT', mode='tank'):
        self.last_poll_at=now
        if mode not in ('tank','joystick'):raise ValueError('Unknown drive mode')
        if state not in STATES:
            raise ValueError('Unknown host LED state')
        for line in self.port.read_lines():
            if line=='ERR clear':
                self.clear_pending=False
                self.notices.append('非常停止解除を拒否：電源ボタンを離し、中立でStartを押し直してください。')
                continue
            if line.startswith('ERR'):
                raise PowerError(f'{self.port.path}: {line}')
            if line.startswith('power_protocol='):
                cap=fields(line)
                if self.phase!='hello': continue
                if any(cap.get(k)!=v for k,v in {'power_protocol':'1','watchdog_ms':'750',
                        'remote_on':'0','led_state':'1','estop_latch':'1','drive_modes':'1','shunt_uohm':'100000'}.items()):
                    raise PowerError('Incompatible Pico power firmware')
                self.port.send(f'claim {self.session}')
                self.phase='claiming';self.deadline=now+1
            elif line.startswith('OK claim='):
                if self.phase!='claiming' or fields(line).get('claim')!=str(self.session):
                    raise PowerError('Wrong Pico claim acknowledgment')
                self.phase='active';self.last_status_at=now
            elif line.startswith('OK host='):
                ack=fields(line)
                if (ack.get('host')!=str(self.session) or self.pending_seq is None or
                        ack.get('seq')!=str(self.pending_seq)):
                    raise PowerError('Wrong Pico heartbeat acknowledgment')
                self.acknowledged=self.pending_seq;self.pending_seq=None
            elif line.startswith('OK clear='):
                if not self.clear_pending or fields(line).get('clear')!=str(self.session):
                    raise PowerError('Wrong Pico clear acknowledgment')
                self.clear_pending=False;self.local_estop=False
            elif line.startswith('online='):
                d=parse_power_status(line)
                if self.phase!='active': continue
                if int(d['session'])!=self.session or int(d['seq'])>self.sequence:
                    raise PowerError('Pico reset or host session changed')
                if self.status and (int(d['uptime_ms'])<int(self.status['uptime_ms']) or
                                    int(d['seq'])<int(self.status['seq'])):
                    raise PowerError('Pico telemetry went backwards')
                if self.status is None or int(d['uptime_ms'])>int(self.status['uptime_ms']):
                    self.last_status_at=now
                self.status=d
        if self.phase=='opening' and now-self.opened_at>=.15:
            self.port.send('cap');self.phase='hello'
        if self.phase!='active':
            if now>self.deadline:
                raise PowerError(f'{self.port.path}: no power_protocol=1 handshake; install embedded/pico_power_master firmware')
            return
        if now-self.last_status_at>self.settings.telemetry_timeout_s:
            raise PowerError('Pico power telemetry timeout')
        if self.pending_seq is not None and now-self.pending_at>.3:
            raise PowerError('Pico heartbeat acknowledgment timeout')
        if self.clear_pending and now-self.clear_sent_at>.3:
            raise PowerError('Pico clear acknowledgment timeout')
        if self.pending_seq is None and now-self.last_heartbeat>=.1:
            if self.sequence==0xffffffff:
                raise PowerError('Pico host sequence exhausted; restart drive')
            self.sequence+=1
            self.port.send(f'host {self.session} {self.sequence} {state} {mode}')
            self.pending_seq=self.sequence;self.pending_at=now;self.last_heartbeat=now

    def snapshot(self,now):
        d=self.status
        valid=bool(d and d['ina']=='1' and int(d['sample_age_ms'])<=300)
        current_valid=valid and d['current_valid']=='1'
        return {'path':self.port.path,'serial':self.settings.serial,'phase':self.phase,
                'status_age_s':now-self.last_status_at if d else None,
                'voltage_v':int(d['mv'])/1000 if valid else None,
                'current_a':int(d['ua'])/1e6 if current_valid else None,
                'power_w':int(d['mw'])/1000 if current_valid else None,
                'current_saturated':valid and d['current_valid']=='0',
                'ssr_command_on':bool(d and d['relay']=='1'),
                'estop':self.estopped,'status':dict(d) if d else None}

    def close(self):
        if self.closed: return []
        self.closed=True
        errors=[]
        try:
            # End ownership and switch off; an emergency latch is not cleared.
            self.port.send(f'release {self.session}' if self.phase=='active' else 'off')
            if self.phase=='active':
                deadline=time.monotonic()+.25
                acknowledged=False
                off=False
                while time.monotonic()<deadline:
                    for line in self.port.read_lines():
                        acknowledged |= line==f'OK release={self.session}'
                        if acknowledged and line.startswith('online='):
                            d=parse_power_status(line)
                            off=d['session']=='0' and d['relay']=='0'
                    if acknowledged and off: break
                    time.sleep(.005)
                if not (acknowledged and off):
                    errors.append('Pico SSR OFF acknowledgment missing; host watchdog expires after 750 ms')
        except Exception as exc:
            errors.append(str(exc))
        finally:
            self.port.close()
        return errors


def monitor_power(settings, *, seconds=None):
    """Read telemetry without claiming the controller or changing SSR/LEDs."""
    import json
    port=SerialPort(resolve_power_port(settings),115200)
    started=time.monotonic()
    last_report=started
    next_query=started+.15
    try:
        while seconds is None or time.monotonic()-started<seconds:
            now=time.monotonic()
            if now>=next_query:
                port.send('status');next_query=now+.2
            for line in port.read_lines():
                d=parse_power_status(line)
                if d is not None:
                    print(json.dumps(d,ensure_ascii=False),flush=True)
                    last_report=now
            if now-last_report>1:
                raise PowerError(f'{port.path}: no power telemetry')
            time.sleep(.01)
    finally:
        port.close()
