"""Real Python serial client -> Rust bridge -> signed Rust owner (radio mocked by UDP)."""
import os
import pathlib
import pty
import secrets
import struct
import subprocess
import tempfile
import time
import unittest
from pyflix.client import Flix, pack
from pyflix.serial import SerialTransport

ROOT=pathlib.Path(__file__).resolve().parents[2]

class ProxyIntegrationTests(unittest.TestCase):
    def test_signed_serial_roundtrip_and_disconnect(self):
        key=secrets.token_bytes(32)
        stamp=int((time.time()-1420070400)*100000)
        with tempfile.TemporaryDirectory() as directory:
            enrollment=pathlib.Path(directory)/'session.enrollment.frame'
            enrollment.write_bytes(pack(256,struct.pack('<QBB32s',stamp,1,1,key)))
            enrollment.chmod(0o600)
            owner=subprocess.Popen([str(ROOT/'target/release/bench'),'0',str(enrollment)],stdout=subprocess.PIPE,text=True)
            master,slave=pty.openpty();bridge=None;transport=None
            try:
                endpoint=owner.stdout.readline().strip();self.assertTrue(endpoint.startswith('127.0.0.1:'))
                transport=SerialTransport(os.ttyname(slave),proxy=True)
                bridge=subprocess.Popen([str(ROOT/'target/release/proxy-bench'),'--fd',str(master),endpoint.rsplit(':',1)[1]],pass_fds=(master,),stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
                self.assertEqual(bridge.stdout.readline().strip(),'ready')
                time.sleep(.1) # connect after several telemetry periods; first ACK must survive
                transport.send(pack(0,b'\0'*9)) # unsigned input cannot poison subsequent valid framing
                with Flix(transport,key=key) as client:
                    client.set_mode(2);client.manual(thrust=0);client.arm();client.disarm()
                    self.assertEqual(len(client.get_parameter(0)),25)
                    with self.assertRaises(ValueError):client.enroll_radio(key,stamp)
                transport=None
                with Flix(SerialTransport(os.ttyname(slave),proxy=True),key=secrets.token_bytes(32)) as wrong:
                    with self.assertRaises(TimeoutError):wrong.get_parameter(0)
                os.close(slave);slave=-1
                bridge.wait(timeout=2)
                self.assertIn(bridge.returncode,(0,1)) # orderly EOF or POSIX EIO after last slave closes
            finally:
                if transport is not None:transport.close()
                if bridge is not None:
                    if bridge.poll() is None:bridge.terminate();bridge.wait(timeout=2)
                    bridge.stdout.close();bridge.stderr.close()
                os.close(master)
                if slave>=0:os.close(slave)
                owner.terminate();owner.wait(timeout=2);owner.stdout.close()
