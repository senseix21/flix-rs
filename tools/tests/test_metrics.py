import pathlib
import queue
import struct
import subprocess
import unittest
from pyflix.client import Decoder, Flix, UdpTransport, pack

ROOT=pathlib.Path(__file__).resolve().parents[2]
class MetricsTransport:
    network=False
    def __init__(self, mode):self.mode=mode;self.incoming=queue.Queue()
    def send(self,frame):
        for message,payload in Decoder().feed(frame):
            if message!=126:continue
            text=payload[9:9+payload[8]].decode('ascii')
            nonce=int(text.split()[1],16)
            values=[1]+([1,123,0,0,1,0,0,0,0,0]*5)+[0]
            def chunk(index,nonce=nonce,corrupt=False):
                part=values[index*7:(index+1)*7]
                if corrupt:part[0]+=1
                data=b'FLXM'+struct.pack('<IBB',nonce,index,8)+struct.pack('<%dQ'%len(part),*part)
                payload=struct.pack('<IHBBB70s',0,0,10,0,len(data),data)
                self.incoming.put(pack(126,payload,system=1,component=1))
            chunk(0,nonce^1) # An older request must not satisfy this one.
            if self.mode=='conflict':chunk(0);chunk(0,corrupt=True);return
            for index in reversed(range(8)):
                if self.mode=='missing' and index==3:continue
                chunk(index)
            chunk(7) # A consistent duplicate is harmless.
    def receive(self):
        try:return self.incoming.get(timeout=.01)
        except queue.Empty:return b''
    def close(self):pass

class MetricsTests(unittest.TestCase):
    def test_reordered_chunks_and_wrong_nonce(self):
        with Flix(MetricsTransport('valid')) as client:
            result=client.metrics()
            self.assertEqual(result['histograms']['owner_tick']['maximum_us'],123)
            self.assertFalse(result['hardware_qualified'])
    def test_missing_and_conflicting_chunks_never_succeed(self):
        with Flix(MetricsTransport('missing')) as client:
            with self.assertRaises(TimeoutError):client.metrics(timeout=.05)
            self.assertEqual(client._metrics_chunks,{})
        with Flix(MetricsTransport('conflict')) as client:
            with self.assertRaisesRegex(RuntimeError,'conflicting'):client.metrics()
    def test_actual_rust_owner_maintenance_snapshot(self):
        process=subprocess.Popen([str(ROOT/'target/release/bench'),'0'],stdout=subprocess.PIPE,text=True)
        try:
            endpoint=process.stdout.readline().strip();port=int(endpoint.rsplit(':',1)[1])
            with Flix(UdpTransport(port=port),simulation=True) as client:
                with self.assertRaisesRegex(RuntimeError,'maintenance'):client.metrics()
                client.console('maintenance');self.assertEqual(client.status(),'maintenance')
                result=client.metrics()
                self.assertGreater(result['histograms']['sample_age']['count'],0)
                client.console('exit')
        finally:process.terminate();process.wait(timeout=2);process.stdout.close()
