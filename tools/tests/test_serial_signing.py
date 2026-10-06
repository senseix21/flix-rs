import os
import pty
import struct
import unittest
import pathlib
import subprocess
import secrets
import tempfile
import time
from pyflix.client import Decoder, Flix, pack
from pyflix.serial import SerialTransport

class SerialSigningTests(unittest.TestCase):
    def test_signed_python_client_roundtrip_with_actual_rust_owner(self):
        from pyflix.client import UdpTransport
        root=pathlib.Path(__file__).resolve().parents[2]
        key=secrets.token_bytes(32)
        stamp=int((time.time()-1420070400)*100000)
        with tempfile.TemporaryDirectory() as directory:
            enrollment=pathlib.Path(directory)/'session.enrollment.frame'
            enrollment.write_bytes(pack(256,struct.pack('<QBB32s',stamp,1,1,key)))
            enrollment.chmod(0o600)
            process=subprocess.Popen([str(root/'target/release/bench'),'0',str(enrollment)],stdout=subprocess.PIPE,text=True)
            try:
                endpoint=process.stdout.readline().strip();self.assertTrue(endpoint.startswith('127.0.0.1:'))
                port=int(endpoint.rsplit(':',1)[1])
                with Flix(UdpTransport(port=port),key=key) as client:
                    client.set_mode(2);client.manual(thrust=0);client.arm();client.disarm()
                    self.assertEqual(len(client.get_parameter(0)),25)
                with Flix(UdpTransport(port=port),key=secrets.token_bytes(32)) as wrong:
                    with self.assertRaises(TimeoutError):wrong.get_parameter(0)
            finally:
                process.terminate();process.wait(timeout=2);process.stdout.close()
    def test_signed_reader_rejects_tampering_replay_unsigned_and_wrong_source(self):
        key=bytes(range(32)); reader=Decoder(key=key,system=1,component=1)
        frame=pack(77,struct.pack('<HB',400,0),system=1,component=1,key=key)
        self.assertEqual(len(list(reader.feed(frame))),1)
        self.assertEqual(list(reader.feed(frame)),[])
        corrupted=bytearray(frame);corrupted[-1]^=1
        self.assertEqual(list(reader.feed(corrupted)),[])
        self.assertEqual(list(reader.feed(pack(77,b'\x90\x01\0',system=1,component=1))),[])
        self.assertEqual(list(reader.feed(pack(77,b'\x90\x01\0',system=2,component=1,key=key))),[])
        self.assertEqual(reader.rejected,4)
    def test_actual_posix_serial_and_proxy_cannot_enroll(self):
        master,slave=pty.openpty()
        try:
            transport=SerialTransport(os.ttyname(slave),proxy=True)
            frame=pack(0,b'\0'*9)
            transport.send(frame)
            self.assertEqual(os.read(master,280),frame)
            os.write(master,frame[:5]);os.write(master,frame[5:])
            decoder=Decoder(); packets=[]
            for _ in range(10):
                packets+=list(decoder.feed(transport.receive()))
                if packets:break
            self.assertEqual(len(packets),1)
            with Flix(transport,key=b"x"*32) as client:
                with self.assertRaises(ValueError):client.enroll_radio(b'x'*32,123)
        finally:os.close(master);os.close(slave)
