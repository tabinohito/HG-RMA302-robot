"""Read-only WIT 11-byte UART telemetry and bounded yaw feedback."""
from collections import deque
from dataclasses import dataclass
import math
import statistics
import struct
import time


@dataclass(frozen=True)
class ImuSettings:
    enabled: bool = False
    serial: str | None = None
    port: str = ''
    baudrate: int = 115200
    yaw_sign: int = -1  # Body convention: clockwise/right positive; sensor +Z up.
    timeout_s: float = .15
    calibration_s: float = 1.0
    heading_kp: float = 1.5  # desired deg/s per heading error degree
    rate_kp: float = .12  # wheel differential RPM correction per deg/s error
    max_yaw_rate_dps: float = 90.0
    max_correction_rpm: float = 8.0
    correction_slew_rpm_s: float = 20.0

    def __post_init__(self):
        if type(self.enabled) is not bool:raise ValueError('imu.enabled must be boolean')
        if self.serial is not None and (not isinstance(self.serial,str) or not self.serial or self.serial.strip()!=self.serial):
            raise ValueError('Invalid imu.serial')
        if not isinstance(self.port,str):raise ValueError('imu.port must be a device path')
        if self.enabled and not (self.serial or self.port):raise ValueError('Set imu.serial or imu.port')
        if type(self.baudrate) is not int or self.baudrate not in (9600,19200,38400,57600,115200):raise ValueError('Unsupported IMU baudrate')
        if type(self.yaw_sign) is not int or self.yaw_sign not in (-1,1):raise ValueError('imu.yaw_sign must be -1 or 1')
        for key,lo,hi in [('timeout_s',.05,.3),('calibration_s',.5,5),('heading_kp',0,5),
            ('rate_kp',0,1),('max_yaw_rate_dps',5,180),('max_correction_rpm',0,15),('correction_slew_rpm_s',1,30)]:
            v=getattr(self,key)
            if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or not lo<=v<=hi:
                raise ValueError(f'imu.{key} must be in [{lo}, {hi}]')


class ImuError(RuntimeError):pass


def wrap_degrees(angle):return (angle+180)%360-180


def resolve_imu_port(settings):
    from .serial_devices import list_serial_ports
    if settings.serial:
        matches={p.device for p in list_serial_ports() if p.serial_number==settings.serial}
        if len(matches)!=1:raise ImuError(f'IMU serial={settings.serial}: expected one port, found {sorted(matches)}')
        return matches.pop()
    return settings.port


class WitDecoder:
    def __init__(self):
        self.buffer=bytearray();self.frames=0;self.bad_checksum=0
        self.accel=None;self.gyro=None;self.angles=None
        self.accel_at=self.gyro_at=self.angle_at=-math.inf
        self.angle_sequence=0;self.gyro_sequence=0
    def feed(self,data,now):
        self.buffer.extend(data)
        while len(self.buffer)>=11:
            if self.buffer[0]!=0x55 or not 0x50<=self.buffer[1]<=0x5f:
                del self.buffer[0];continue
            if (sum(self.buffer[:10])&255)!=self.buffer[10]:
                self.bad_checksum+=1;del self.buffer[0];continue
            kind=self.buffer[1];v=struct.unpack_from('<hhh',self.buffer,2)
            del self.buffer[:11];self.frames+=1
            if kind==0x51:self.accel=tuple(x*16/32768 for x in v);self.accel_at=now
            elif kind==0x52:
                self.gyro=tuple(x*2000/32768 for x in v);self.gyro_at=now;self.gyro_sequence+=1
            elif kind==0x53:
                self.angles=tuple(x*180/32768 for x in v);self.angle_at=now;self.angle_sequence+=1


class ImuLink:
    def __init__(self,settings,*,serial_factory=None,now=None):
        if serial_factory is None:
            import serial
            serial_factory=serial.Serial
        self.settings=settings;self.path=resolve_imu_port(settings)
        # Configure without asserting modem outputs; no sensor commands are sent.
        self.port=serial_factory(port=None,baudrate=settings.baudrate,timeout=0,exclusive=True)
        try:
            self.port.dtr=False;self.port.rts=False;self.port.port=self.path;self.port.open();self.port.reset_input_buffer()
        except BaseException:self.port.close();raise
        self.decoder=WitDecoder();self.opened_at=time.monotonic() if now is None else now
        self.calibration=deque();self.bias_dps=None;self.discarded=0
        self.last_angle_sequence=0;self.last_gyro_sequence=0
        self.previous_yaw=None;self.previous_angle_at=None;self.fault=None
    def poll(self,now):
        n=self.port.in_waiting
        if n>512:
            self.port.reset_input_buffer();self.decoder=WitDecoder();self.calibration.clear()
            self.discarded+=1;self.fault='IMU_BACKLOG'
            return
        d=self.decoder;old_gyro=d.gyro_sequence
        d.feed(self.port.read(n) if n else b'',now)
        if d.gyro_sequence!=old_gyro and self.bias_dps is None and d.accel is not None:
            magnitude=math.sqrt(sum(x*x for x in d.accel))
            if (now-d.accel_at>self.settings.timeout_s or not .9<magnitude<1.1 or
                    max(abs(x) for x in d.gyro)>3):self.calibration.clear()
            else:
                self.calibration.append((now,d.gyro[2]))
                while len(self.calibration)>1 and now-self.calibration[1][0]>self.settings.calibration_s:
                    self.calibration.popleft()
                if len(self.calibration)>=10 and now-self.calibration[0][0]>=self.settings.calibration_s:
                    values=[x[1] for x in self.calibration]
                    if max(values)-min(values)<.5:self.bias_dps=statistics.mean(values)
                    else:self.calibration.clear()
        if d.angle_sequence!=self.last_angle_sequence:
            yaw=d.angles[2]
            if self.previous_yaw is not None:
                dt=d.angle_at-self.previous_angle_at
                if dt>0 and abs(wrap_degrees(yaw-self.previous_yaw))>720*dt+5:
                    self.fault='IMU_ANGLE_JUMP'
            self.previous_yaw=yaw;self.previous_angle_at=d.angle_at;self.last_angle_sequence=d.angle_sequence
    def reason(self,now):
        if self.fault:return self.fault
        d=self.decoder
        if any(now-t>self.settings.timeout_s for t in (d.accel_at,d.gyro_at,d.angle_at)):
            return 'IMU_WAIT' if now-self.opened_at<2 else 'IMU_STALE'
        if abs(d.angles[0])>35 or abs(d.angles[1])>35:return 'IMU_TILT'
        if self.bias_dps is None:return 'IMU_CALIBRATING'
        return None
    def snapshot(self,now):
        d=self.decoder
        return {'path':self.path,'reason':self.reason(now),'yaw_deg':self.yaw,
                'yaw_rate_dps':self.rate,'bias_dps':self.bias_dps,'accel_g':d.accel,
                'roll_pitch_yaw_deg':d.angles,'gyro_dps':d.gyro,
                'angle_age_s':now-d.angle_at if d.angles else None,
                'gyro_age_s':now-d.gyro_at if d.gyro else None,
                'frames':d.frames,'bad_checksum':d.bad_checksum,'discarded_backlogs':self.discarded}
    @property
    def yaw(self):
        return None if self.decoder.angles is None else self.settings.yaw_sign*self.decoder.angles[2]
    @property
    def rate(self):
        return None if self.decoder.gyro is None else self.settings.yaw_sign*(self.decoder.gyro[2]-(self.bias_dps or 0))
    def close(self):self.port.close()


class YawFeedback:
    """Heading hold for straight travel; yaw-rate control for all intentional turns.

    The ESC's own speed/startup controller remains authoritative. Correction
    cannot start a stopped wheel, reverse a wheel, or create sub-minimum RPM.
    """
    def __init__(self,link,drive):
        self.link=link;self.drive=drive;self.reset()
    def reset(self):
        self.heading=None;self.correction=0.;self.previous_time=None;self.previous_key=None
        self.target_rate=0.;self.heading_error=0.;self.reason='INACTIVE';self.targets=(0.,0.);self.last_targets=None
    def correct(self,targets,output,wheels,now):
        self.targets=targets
        if not output.enabled or not any(targets):self.reset();return targets
        reason=self.link.reason(now)
        if reason:raise ImuError(reason)
        signs=(self.drive.left_sign,self.drive.right_sign)
        left,right=(v*s for v,s in zip(targets,signs))
        requested=tuple(v*s for v,s in zip((output.left_rpm,output.right_rpm),signs))
        key=(output.control_mode,tuple(0 if x==0 else (1 if x>0 else -1) for x in requested))
        if key!=self.previous_key:self.reset();self.previous_key=key
        # Corrections start only once both commanded wheels are in closed loop.
        for wheel,target in zip(wheels,targets):
            d=wheel.last_status or {}
            if target and (wheel.phase!='running' or d.get('state')!='closed' or d.get('fault')!='none'
                or now-wheel.last_status_at>.25 or int(d.get('req_mrpm','0'))*target<=0):
                self.heading=None;self.correction=0.;self.previous_time=now;self.reason='WAITING_FOR_WHEELS'
                self.last_targets=targets
                return targets
        dt=0. if self.previous_time is None else max(0.,min(now-self.previous_time,.05))
        self.previous_time=now
        cfg=self.link.settings
        straight=requested[0]*requested[1]>0 and abs(requested[0]-requested[1])<.01
        if straight:
            if self.heading is None:self.heading=self.link.yaw
            self.heading_error=wrap_degrees(self.heading-self.link.yaw)
            self.target_rate=max(-30.,min(30.,self.heading_error*cfg.heading_kp))
            self.reason='HEADING_HOLD'
        else:
            self.heading=None;self.heading_error=0.
            self.target_rate=(left-right)/(2*self.drive.max_speed_rpm)*cfg.max_yaw_rate_dps
            self.reason='YAW_RATE'
        correction=max(-cfg.max_correction_rpm,min(cfg.max_correction_rpm,
                    cfg.rate_kp*(self.target_rate-self.link.rate)))
        step=cfg.correction_slew_rpm_s*dt
        self.correction+=max(-step,min(step,correction-self.correction))
        adjusted=[]
        for index,(base,delta,sign) in enumerate(zip((left,right),(self.correction,-self.correction),signs)):
            if base==0:adjusted.append(0.);continue
            direction=1 if base>0 else -1
            speed=max(self.drive.min_speed_rpm,min(self.drive.max_speed_rpm,(base+delta)*direction))
            if self.last_targets is not None and self.last_targets[index]*base*sign>0:
                # Feedback plus feed-forward acceleration shares the same total
                # slew limit; operator reductions still pass through immediately.
                speed=min(speed,abs(self.last_targets[index])+self.drive.acceleration_rpm_s*dt)
            adjusted.append(direction*speed*sign)
        self.targets=tuple(adjusted);self.last_targets=self.targets
        return self.targets
    def snapshot(self,now):
        return {**self.link.snapshot(now),'feedback':self.reason,'heading_target_deg':self.heading,
                'heading_error_deg':self.heading_error,'target_rate_dps':self.target_rate,
                'correction_rpm':self.correction,'targets_rpm':self.targets}


def monitor_imu(settings,*,seconds=None):
    import json
    link=ImuLink(settings)
    started=last_print=time.monotonic()
    try:
        while seconds is None or time.monotonic()-started<seconds:
            now=time.monotonic();link.poll(now)
            if now-last_print>=.1:
                print(json.dumps(link.snapshot(now),ensure_ascii=False),flush=True);last_print=now
            time.sleep(.005)
    finally:link.close()
