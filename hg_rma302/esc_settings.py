"""Startup tuning and exact UART representation; no flash writes per run."""
from dataclasses import asdict, dataclass
from decimal import Decimal
import math


@dataclass(frozen=True)
class EscSettings:
    bus_min_v: float = 18.0
    bus_max_v: float = 26.0
    phase_trip_a: float = 4.0
    ntc_trip_raw: int = 2873
    wheel_electrical_cycles: float = 199.0
    startup_speed_rpm: float = 8.0
    startup_current_a: float = .6
    acceleration_ff_a: float = 0.0
    current_sum_soft_a: float = 1.2
    current_sum_hard_a: float = 2.0
    current_sum_trip_samples: int = 10
    startup_lock_bad_cost: int = 5
    startup_lock_score: int = 2000
    overspeed_filtered_ratio: float = 1.5
    overspeed_raw_ratio: float = 3.0

    def __post_init__(self):
        ranges = {
            'bus_min_v': (7,26), 'bus_max_v': (7,26), 'phase_trip_a': (1,4),
            'ntc_trip_raw': (1000,3200), 'wheel_electrical_cycles': (1,1000),
            'startup_speed_rpm': (8,25), 'startup_current_a': (.1,1),
            'acceleration_ff_a': (0,1),
            'current_sum_soft_a': (.3,1.5), 'current_sum_hard_a': (.3,2),
            'current_sum_trip_samples': (1,40), 'startup_lock_bad_cost': (1,100),
            'startup_lock_score': (500,10000),
            'overspeed_filtered_ratio': (1.1,2), 'overspeed_raw_ratio': (1.1,3),
        }
        for key, (low, high) in ranges.items():
            value = getattr(self,key)
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not low<=value<=high:
                raise ValueError(f'esc.{key} must be in [{low}, {high}]')
            scale = 1 if key in INTEGER_FIELDS else 1000
            scaled = Decimal(str(value))*scale
            if scaled != scaled.to_integral_value() or (scale==1 and type(value) is not int):
                raise ValueError(f'esc.{key} requires ' + ('an integer' if scale==1 else 'at most 3 decimal places'))
        if self.bus_min_v >= self.bus_max_v:
            raise ValueError('esc.bus_min_v must be below bus_max_v')
        if self.current_sum_soft_a >= self.current_sum_hard_a:
            raise ValueError('esc.current_sum_soft_a must be below current_sum_hard_a')
        if self.overspeed_raw_ratio < self.overspeed_filtered_ratio:
            raise ValueError('esc.overspeed_raw_ratio must be >= overspeed_filtered_ratio')

    def wire(self, drive):
        if 2*max(drive.max_current_a,self.startup_current_a)>self.phase_trip_a:
            raise ValueError('esc.phase_trip_a must be >= 2 * max(drive.max_current_a, esc.startup_current_a)')
        if self.startup_speed_rpm>drive.min_speed_rpm:
            raise ValueError('esc.startup_speed_rpm must be <= drive.min_speed_rpm')
        hz = lambda rpm: rpm*self.wheel_electrical_cycles/60
        if hz(self.startup_speed_rpm)<10 or hz(drive.max_speed_rpm)>360 or hz(drive.acceleration_rpm_s)>100:
            raise ValueError('esc wheel conversion exceeds supported electrical speed/acceleration')
        result = {}
        for public, wire in WIRE_FIELDS.items():
            scale = 1 if public in INTEGER_FIELDS else 1000
            result[wire] = int(Decimal(str(getattr(self,public)))*scale)
        for public, wire in DRIVE_FIELDS.items():
            scaled = Decimal(str(getattr(drive,public)))*1000
            if scaled != scaled.to_integral_value():
                raise ValueError(f'drive.{public} requires at most 3 decimal places')
            result[wire] = int(scaled)
        return result

    def overlay(self, values):
        if not isinstance(values,dict):
            raise ValueError('ESC overrides must be a JSON object')
        return EscSettings(**(asdict(self)|values))


INTEGER_FIELDS = {'ntc_trip_raw','current_sum_trip_samples','startup_lock_bad_cost','startup_lock_score'}
WIRE_FIELDS = {
    'bus_min_v':'bus_min_mv', 'bus_max_v':'bus_max_mv', 'phase_trip_a':'phase_trip_ma',
    'ntc_trip_raw':'ntc_trip_raw', 'wheel_electrical_cycles':'wheel_cycles_milli',
    'startup_speed_rpm':'startup_mrpm', 'startup_current_a':'startup_ma',
    'acceleration_ff_a':'accel_ff_ma',
    'current_sum_soft_a':'sum_soft_ma', 'current_sum_hard_a':'sum_hard_ma',
    'current_sum_trip_samples':'sum_trip_samples', 'startup_lock_bad_cost':'lock_bad_cost',
    'startup_lock_score':'lock_score', 'overspeed_filtered_ratio':'overspeed_filtered_milli',
    'overspeed_raw_ratio':'overspeed_raw_milli',
}
DRIVE_FIELDS = {'min_speed_rpm':'min_mrpm','max_speed_rpm':'max_mrpm',
               'max_current_a':'max_iq_ma','acceleration_rpm_s':'max_slew_mrpm_s'}
