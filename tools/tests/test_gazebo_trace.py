import math
import struct
import unittest
from gazebo_trace import HEADER, HEADER_V2, VALUES, decode, tilt

class TraceTests(unittest.TestCase):
    def packet(self, **changes):
        fields=dict(magic=b'FLXT',version=1,now=1000,generation=3,state=3,reason=0,source=0,valid=1,sequence=7)
        fields.update(changes)
        values=[0.]*34;values[3]=values[16]=values[26]=1.;values[-4:]=[.4]*4
        return HEADER.pack(*fields.values())+VALUES.pack(*values)
    def test_layout_and_units(self):
        self.assertEqual(len(self.packet()),184)
        record=decode(self.packet())
        self.assertEqual(record['estimate_wxyz'],[1,0,0,0])
        self.assertEqual(record['sim_us'],1000);self.assertEqual(record['sequence'],7)
        self.assertEqual(record['target_wxyz'],[1,0,0,0])
        self.assertTrue(record['numeric_valid'])
        self.assertAlmostEqual(tilt([0,1,0,0]),math.pi)
    def test_bad_packets_and_nonfinite_observation(self):
        for data in [self.packet()[:-1],self.packet()+b'0',self.packet(version=2),self.packet(state=6),self.packet(source=3)]:
            with self.assertRaises(ValueError):decode(data)
        data=bytearray(self.packet());struct.pack_into('<f',data,HEADER.size+10*4,float('nan'))
        record=decode(data)
        self.assertFalse(record['numeric_valid']);self.assertIsNone(record['gyro_body_rad_s'][0])
    def test_command_boundary_is_plugin_time(self):
        data=struct.pack('<4sIQII',b'FLXC',1,123456,1,2)+b'xx'
        self.assertEqual(decode(data),dict(kind='command',version=1,sim_us=123456,accepted=True,frame_hex='7878'))
        for invalid in [data[:-1],data+b'x',b'FLXC']:
            with self.assertRaises(ValueError):decode(invalid)
    def test_v2_sensor_capture_time_and_power_fault(self):
        values=VALUES.unpack_from(self.packet(),HEADER.size)
        frame=HEADER_V2.pack(b'FLXT',2,20000,4,5,11,0xffffffff,0,8,9999)+VALUES.pack(*values)
        record=decode(frame)
        self.assertEqual(record['sample_us'],9999)
        self.assertEqual(record['reason'],11)
        self.assertEqual(record['sample_time_provenance'],'input-v2')
        self.assertEqual(decode(self.packet())['sample_time_provenance'],'assumed-tick-v1')

    def test_v3_preserves_raw_contact_and_sensor_limits(self):
        values=VALUES.unpack_from(self.packet(),HEADER.size)
        frame=HEADER_V2.pack(b'FLXT',3,20000,4,3,0,0,1,8,20000)+VALUES.pack(*values)
        frame+=struct.pack('<6fI',1,2,3,135,-139,132,56)
        record=decode(frame)
        self.assertEqual(record['sensor_limit_mask'],56)
        self.assertEqual(record['raw_acceleration_body_m_s2'],[135,-139,132])
        with self.assertRaises(ValueError):decode(frame[:-1])
