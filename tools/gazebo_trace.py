"""Versioned loopback observation packets; never confer control authority."""
import math
import struct

HEADER = struct.Struct('<4sIQQIIIIQ')
HEADER_V2 = struct.Struct('<4sIQQIIIIQQ')
VALUES = struct.Struct('<34f')
FIELDS = [('position_world_m',3),('pose_wxyz',4),('velocity_world_m_s',3),
          ('gyro_body_rad_s',3),('acceleration_body_m_s2',3),('estimate_wxyz',4),
          ('estimated_rates_rad_s',3),('target_rates_rad_s',3),('target_wxyz',4),('motors',4)]

def decode(data):
    if data[:4] == b'FLXC':
        if len(data) < 24:
            raise ValueError('Command trace header truncated')
        _, version, now, accepted, length = struct.unpack_from('<4sIQII', data)
        if version != 1 or accepted > 1 or length > 280 or len(data) != 24+length:
            raise ValueError('Invalid command trace')
        return dict(kind='command',version=version,sim_us=now,accepted=bool(accepted),frame_hex=data[24:].hex())
    if len(data)<8:raise ValueError('Trace length')
    version=struct.unpack_from('<I',data,4)[0]
    header=HEADER if version==1 else HEADER_V2
    if version not in (1,2,3) or len(data) != header.size + VALUES.size + (28 if version==3 else 0):
        raise ValueError('Trace length')
    fields=header.unpack_from(data)
    magic, version, now, generation, state, reason, source, valid, sequence = fields[:9]
    if magic != b'FLXT' or state > 5 or reason > 13 or source not in (0,1,2,0xffffffff) or valid > 1:
        raise ValueError('Trace header')
    values = VALUES.unpack_from(data, header.size)
    record = dict(kind='tick',version=version,sim_us=now,generation=generation,
                  state=state,reason=reason,source=None if source==0xffffffff else source,
                  target_attitude_valid=bool(valid),sequence=sequence,numeric_valid=all(map(math.isfinite,values)))
    record.update(sample_us=now if version==1 else fields[9],sample_time_provenance='assumed-tick-v1' if version==1 else 'input-v2')
    cursor = 0
    for field,count in FIELDS:
        record[field] = [value if math.isfinite(value) else None for value in values[cursor:cursor+count]]
        cursor += count
    if version==3:
        raw=struct.unpack_from('<6fI',data,192)
        if raw[6]>63:raise ValueError('Sensor range mask')
        record.update(raw_gyro_body_rad_s=[v if math.isfinite(v) else None for v in raw[:3]],
                      raw_acceleration_body_m_s2=[v if math.isfinite(v) else None for v in raw[3:6]],sensor_limit_mask=raw[6])
    return record

def tilt(quaternion):
    w,x,y,z=quaternion
    return math.acos(max(-1.,min(1.,1-2*(x*x+y*y))))
