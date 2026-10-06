import struct,unittest
from log import download,to_csv
class LogTests(unittest.TestCase):
    def test_offsets_and_roundtrip(self):
        data=struct.pack('<4sHHI',b'FLXR',1,64,2)+struct.pack('<Q14f',1000,*([0.]*14))+struct.pack('<Q14f',2000,*([1.]*14))
        class Client:
            def log_chunk(self,offset,count):
                chunk=data[offset:offset+count];return struct.pack('<IHB',offset,0,len(chunk))+chunk
        self.assertEqual(download(Client()),data);self.assertEqual(len(to_csv(data).splitlines()),3)
        with self.assertRaises(ValueError):to_csv(data[:-1])
    def test_missing_chunk_and_nonfinite_rejected(self):
        class Client:
            def log_chunk(self,offset,count):return struct.pack('<IHB',offset,0,0)
        with self.assertRaises(ValueError):download(Client())
        data=struct.pack('<4sHHI',b'FLXR',1,64,1)+struct.pack('<Q14f',1000,*([float('nan')]*14))
        with self.assertRaises(ValueError):to_csv(data)
