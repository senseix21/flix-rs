import pathlib,queue,struct,threading,time,unittest,subprocess
from pyflix.client import Flix,Closed,Busy,pack,Decoder,UdpTransport
ROOT=pathlib.Path(__file__).resolve().parents[2]
class Mock:
    network=False
    def __init__(self):self.incoming=queue.Queue();self.closed=False;self.reply=True
    def send(self,data):
        for message,payload in Decoder().feed(data):
            if self.reply and message==76:
                command=struct.unpack_from('<H',payload,28)[0]
                self.incoming.put(pack(77,struct.pack('<HBBiBB',command,0,255,0,255,190),system=1,component=1))
    def receive(self):
        try:return self.incoming.get(timeout=.01)
        except queue.Empty:return b''
    def close(self):self.closed=True
class ClientTests(unittest.TestCase):
    def test_immediate_ack_and_slow_callback(self):
        mock=Mock()
        with Flix(mock,callback=lambda *_:time.sleep(.2)) as client:
            for _ in range(10):client.command(400,0)
        self.assertTrue(mock.closed)
        self.assertFalse(client._threads[0].is_alive())
        self.assertFalse(client._threads[1].is_alive())
    def test_close_wakes_pending_request(self):
        mock=Mock();mock.reply=False;client=Flix(mock);errors=[]
        def request():
            try:client.command(400,0)
            except Exception as error:errors.append(error)
        worker=threading.Thread(target=request);worker.start()
        deadline=time.monotonic()+1
        while not client._pending and time.monotonic()<deadline:time.sleep(.001)
        client.close();worker.join(1);self.assertFalse(worker.is_alive());self.assertIsInstance(errors[0],Closed)
    def test_duplicate_request_key_is_rejected(self):
        mock=Mock();client=Flix(mock)
        with client._pending_lock:client._pending[('ack',400)]=queue.Queue(1)
        with self.assertRaises(Busy):client.command(400,0)
        client.close()
    def test_udp_requires_signing_except_explicit_loopback_simulation(self):
        transport=UdpTransport()
        try:
            with self.assertRaises(ValueError):Flix(transport)
        finally:transport.close()
    def test_actual_rust_owner_roundtrip(self):
        process=subprocess.Popen([str(ROOT/'target/release/bench'),'0'],stdout=subprocess.PIPE,text=True)
        try:
            endpoint=process.stdout.readline().strip();self.assertTrue(endpoint.startswith('127.0.0.1:'));port=int(endpoint.rsplit(':',1)[1]);events=[]
            with Flix(UdpTransport(port=port),simulation=True,callback=lambda m,p:events.append((m,p))) as client:
                client.set_mode(2);client.manual(thrust=0);client.arm()
                deadline=time.monotonic()+2
                while time.monotonic()<deadline:
                    client.manual(thrust=.4)
                    if any(m==140 and any(v>0 for v in struct.unpack_from('<4f',p,8)) for m,p in list(events)):break
                    time.sleep(.01)
                self.assertTrue(any(m==140 and any(v>0 for v in struct.unpack_from('<4f',p,8)) for m,p in list(events)))
                client.disarm();events.clear();deadline=time.monotonic()+2
                while time.monotonic()<deadline:
                    motors=[struct.unpack_from('<4f',p,8) for m,p in list(events) if m==140]
                    if motors and motors[-1]==(0.,0.,0.,0.):break
                    time.sleep(.01)
                self.assertTrue(motors);self.assertEqual(motors[-1],(0.,0.,0.,0.))
                parameter=client.get_parameter(0);self.assertEqual(struct.unpack_from('<f',parameter)[0],struct.unpack('<f',struct.pack('<f',.05))[0])
        finally:
            process.terminate();process.wait(timeout=2);process.stdout.close()
if __name__=='__main__':unittest.main()
class HighresImuTests(unittest.TestCase):
    def test_decodes_full_range_gyro_from_owner_payload(self):
        # Third oracle case: +/-1999.9 deg/s, beyond SCALED_IMU's i16 mrad/s range.
        lines=[l for l in (ROOT/'fixtures/telemetry/highres_imu.txt').read_text().splitlines() if not l.startswith('#')]
        fields=lines[2].split()
        payload=bytes.fromhex(fields[-1])
        frames=list(Decoder().feed(pack(105,payload,system=1,component=1)))
        self.assertEqual(len(frames),1)
        message,decoded=frames[0]
        self.assertEqual(message,105)
        time_us,ax,ay,az,gx,gy,gz=struct.unpack_from('<Q6f',decoded)
        self.assertEqual(time_us,int(fields[0]))
        self.assertGreater(gx,32.767)  # SCALED_IMU would clip at 32767 mrad/s
        self.assertEqual(struct.unpack_from('<H',decoded,60)[0],0x3f)
        # Trailing zero truncation (MAVLink 2) must decode to the same padded payload.
        trimmed=payload.rstrip(b'\0')
        self.assertEqual(list(Decoder().feed(pack(105,trimmed,system=1,component=1)))[0][1],decoded)
