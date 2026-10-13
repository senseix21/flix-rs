"""Independent client implementation. Reader never runs user callbacks or waits for requests."""
import hashlib
import hmac
import json
import pathlib
import queue
import secrets
import socket
import struct
import threading
import time

_ROOT = pathlib.Path(__file__).resolve().parents[2]
_DIALECT = {int(k): tuple(v) for k, v in json.loads((_ROOT / 'protocol/common-subset.json').read_text())['messages'].items()}
PARAMETER_COUNT = len(json.loads((_ROOT / 'protocol/parameters.json').read_text())['parameters'])

class Closed(RuntimeError): pass
class Busy(RuntimeError): pass

def crc(data, extra):
    result = 0xffff
    for value in bytes(data) + bytes([extra]):
        tmp = value ^ (result & 255)
        tmp = (tmp ^ (tmp << 4)) & 255
        result = ((result >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 65535
    return result

def pack(message, payload, sequence=0, system=255, component=190, key=None, timestamp=None):
    extra, _, maximum = _DIALECT[message]
    if len(payload) > maximum: raise ValueError('payload length')
    header = bytes([253, len(payload), 1 if key else 0, 0, sequence, system, component]) + message.to_bytes(3, 'little')
    frame = header + payload
    frame += struct.pack('<H', crc(frame[1:], extra))
    if key:
        if len(key) != 32: raise ValueError('signing key length')
        if timestamp is None: timestamp = int((time.time() - 1420070400) * 100000)
        trailer = bytes([0]) + timestamp.to_bytes(6, 'little')
        frame += trailer + hashlib.sha256(key + frame + trailer).digest()[:6]
    return frame

class Decoder:
    def __init__(self, *, key=None, system=None, component=None):
        if key is not None and len(key) != 32: raise ValueError("signing key length")
        self.buffer = bytearray(); self.key = key; self.system = system; self.component = component
        self.streams = {}; self.rejected = 0
    def feed(self, data):
        for byte in data:
            if not self.buffer and byte != 253: continue
            self.buffer.append(byte)
            if len(self.buffer) < 3: continue
            size = self.buffer[1] + 12 + (13 if self.buffer[2] & 1 else 0)
            if len(self.buffer) < size: continue
            frame = bytes(self.buffer); self.buffer.clear()
            if frame[2] & ~1: continue
            message = int.from_bytes(frame[7:10], 'little')
            if message not in _DIALECT: continue
            extra, _, maximum = _DIALECT[message]
            length = frame[1]
            if length > maximum or crc(frame[1:10+length], extra) != int.from_bytes(frame[10+length:12+length], 'little'): continue
            if self.system is not None and frame[5] != self.system or self.component is not None and frame[6] != self.component:
                self.rejected += 1; continue
            if self.key is not None:
                if not frame[2] & 1:
                    self.rejected += 1; continue
                timestamp = int.from_bytes(frame[-12:-6], 'little')
                stream = (frame[5], frame[6], frame[-13])
                expected = hashlib.sha256(self.key + frame[:-6]).digest()[:6]
                floor = max(0, int((time.time()-1420070400)*100000)-6000000)
                if not hmac.compare_digest(expected, frame[-6:]) or timestamp < floor or timestamp <= self.streams.get(stream, 0) or stream not in self.streams and len(self.streams) >= 4:
                    self.rejected += 1; continue
                self.streams[stream] = timestamp
            yield message, frame[10:10+length].ljust(maximum, b'\0')

class UdpTransport:
    network = True
    def __init__(self, host='127.0.0.1', port=14550):
        self.endpoint = (socket.gethostbyname(host), port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('0.0.0.0', 0)); self.socket.settimeout(.1)
    def send(self, data): self.socket.sendto(data, self.endpoint)
    def receive(self):
        try:
            data, endpoint = self.socket.recvfrom(281)
            return data if endpoint == self.endpoint and len(data) <= 280 else b''
        except socket.timeout: return b''
    def close(self): self.socket.close()

class Flix:
    def __init__(self, transport, *, key=None, simulation=False, callback=None):
        if (getattr(transport, 'network', False) or getattr(transport, 'requires_signing', False)) and key is None and not simulation:
            raise ValueError('wireless control needs signing; simulation=True is loopback bench only')
        if simulation and getattr(transport, 'requires_signing', False): raise ValueError('proxy requires signed control')
        if simulation and getattr(transport, 'network', False) and transport.endpoint[0] != '127.0.0.1':
            raise ValueError('simulation transport must be loopback')
        if key is not None and (len(key)!=32 or not any(key)): raise ValueError("nonzero 32-byte signing key required")
        self.transport = transport; self.key = key; self.callback = callback
        self._write = threading.Lock(); self._pending_lock = threading.Lock(); self._pending = {}
        self._stop = threading.Event(); self._callbacks = queue.Queue(32); self.dropped_callbacks = 0
        self._sequence = 0; self._timestamp = 0
        self._metrics_chunks = {}
        self._threads = [threading.Thread(target=f, daemon=True) for f in (self._reader, self._heartbeat, self._callback_worker)]
        for thread in self._threads: thread.start()
    def __enter__(self): return self
    def __exit__(self, *_): self.close()
    def send(self, message, payload):
        with self._write:
            if self._stop.is_set(): raise Closed('connection closed')
            self._timestamp = max(self._timestamp + 1, int((time.time()-1420070400)*100000))
            frame = pack(message, payload, self._sequence, key=self.key, timestamp=self._timestamp)
            self._sequence = (self._sequence+1) & 255
            self.transport.send(frame)
    def request(self, key, message, payload, timeout=1.0):
        pending = queue.Queue(1)
        with self._pending_lock:
            if self._stop.is_set(): raise Closed('connection closed')
            if key in self._pending: raise Busy('one in-flight request per response key')
            self._pending[key] = pending
        try:
            self.send(message, payload)  # waiter already registered, even for immediate replies
            try: reply = pending.get(timeout=timeout)
            except queue.Empty: raise TimeoutError(key) from None
            if isinstance(reply, Exception): raise reply
            return reply
        finally:
            with self._pending_lock:
                if self._pending.get(key) is pending: self._pending.pop(key)
    def command(self, command, *parameters):
        if len(parameters)>7: raise ValueError('at most seven parameters')
        payload = struct.pack('<7fHBBB', *(list(parameters)+[0.]*(7-len(parameters))), command, 1, 1, 0)
        reply = self.request(('ack', command), 76, payload)
        if reply[2] != 0: raise RuntimeError('command denied: result %d' % reply[2])
    def arm(self): self.command(400, 1)
    def disarm(self): self.command(400, 0)
    def set_mode(self, mode): self.command(176, 1, mode)
    def manual(self, roll=0, pitch=0, yaw=0, thrust=0):
        if any(not -1<=v<=1 for v in (roll,pitch,yaw)) or not 0<=thrust<=1: raise ValueError('pilot bounds')
        self.send(69, struct.pack('<hhhhHB', int(pitch*1000),int(roll*1000),int(thrust*1000),int(yaw*1000),0,1))
    def attitude(self, quaternion, thrust, rates=(0,0,0), rates_only=False):
        if len(quaternion)!=4 or len(rates)!=3: raise ValueError('shape')
        w,x,y,z=quaternion;r,p,q=rates
        self.send(82,struct.pack('<I8fBBB',0,w,x,-y,-z,r,-p,-q,thrust,1,1,128 if rates_only else 0))
    def get_parameter(self, index):
        if not 0<=index<PARAMETER_COUNT: raise ValueError('parameter index')
        return self.request(('parameter',index),20,struct.pack('<hBB16s',index,1,1,b''))
    def console(self, command):
        allowed={'status','disarm','maintenance','exit','clear-fault','save','calibrate-gyro'}
        if command not in allowed: raise ValueError('console allowlist')
        encoded=command.encode('ascii'); self.send(126,struct.pack('<IHBBB70s',0,0,10,0,len(encoded),encoded))
    def status(self):
        encoded=b'status'
        payload=self.request(('console','status'),126,struct.pack('<IHBBB70s',0,0,10,0,len(encoded),encoded))
        lines=payload[9:9+payload[8]].decode('ascii').strip().split('\n')
        return lines[-1]
    def metrics(self, timeout=2.0):
        """Frozen v1 snapshot. Missing, mixed or conflicting chunks never succeed."""
        nonce=secrets.randbits(32)
        with self._pending_lock:
            if self._metrics_chunks: raise Busy('one metrics snapshot at a time')
            self._metrics_chunks[nonce]={}
        try:
            text=('metrics %08x'%nonce).encode('ascii')
            values=self.request(('metrics',nonce),126,struct.pack('<IHBBB70s',0,0,10,0,len(text),text),timeout)
            if len(values)!=52 or values[0]!=1: raise RuntimeError('unsupported metrics snapshot')
            names=('acquisition','sample_age','owner_tick','acquisition_and_owner','schedule_jitter')
            result={'version':1,'hardware_qualified':False,'histograms':{},'invalid_sample_age':values[51]}
            for i,name in enumerate(names):
                h=values[1+i*10:11+i*10]
                if min(sum(h[2:]),(1<<64)-1)!=h[0]:raise RuntimeError('inconsistent histogram')
                result['histograms'][name]={'count':h[0],'maximum_us':h[1],'buckets':list(h[2:]),'upper_us':[50,100,250,500,1000,2000,5000,None]}
            return result
        finally:
            with self._pending_lock:self._metrics_chunks.pop(nonce,None)
    def enroll_radio(self, key, timestamp):
        if getattr(self.transport, 'network', False) or not getattr(self.transport, 'trusted_wired', False):
            raise ValueError('enrollment requires the direct trusted wired flight-board connection')
        if self.key is not None or len(key)!=32 or not any(key) or not 0<timestamp<(1<<48):
            raise ValueError('fresh key and 48-bit enrollment timestamp required')
        if self.status()!='maintenance': raise RuntimeError('enter stopped maintenance before enrollment')
        payload=self.request(('console',),256,struct.pack('<QBB32s',timestamp,1,1,key))
        if payload[9:9+payload[8]]!=b'radio-enrolled': raise RuntimeError('radio enrollment denied')
    def revoke_radio(self):
        if not getattr(self.transport, 'trusted_wired', False) or getattr(self.transport, 'network', False):
            raise ValueError('revocation requires direct trusted wired connection')
        if self.status()!='maintenance': raise RuntimeError('enter stopped maintenance before revocation')
        payload=self.request(('console',),256,struct.pack('<QBB32s',0,1,1,b'\0'*32))
        if payload[9:9+payload[8]]!=b'radio-revoked': raise RuntimeError('revocation denied')
    def log_chunk(self, offset, count=90):
        if not 0<=offset<=64012 or not 0<count<=90: raise ValueError('log bounds')
        return self.request(('log',offset),119,struct.pack('<IIHBB',offset,count,0,1,1))
    def _reader(self):
        decoder=Decoder(key=self.key, system=1, component=1)
        try:
            while not self._stop.is_set():
                for message,payload in decoder.feed(self.transport.receive()):
                    key=None
                    if message==77: key=('ack',int.from_bytes(payload[:2],'little'))
                    elif message==22: key=('parameter',int.from_bytes(payload[6:8],'little'))
                    elif message==120: key=('log',int.from_bytes(payload[:4],'little'))
                    elif message==126:
                        data=payload[9:9+payload[8]]
                        if len(data)>=10 and data[:4]==b'FLXM':
                            nonce=int.from_bytes(data[4:8],'little');key=('metrics',nonce)
                            with self._pending_lock:
                                chunks=self._metrics_chunks.get(nonce)
                                waiter=self._pending.get(key)
                                if chunks is not None and waiter is not None:
                                    index,total=data[8:10]
                                    error=None
                                    if total==0:error=RuntimeError('metrics require stopped maintenance')
                                    elif total!=8 or index>=8 or len(data)!=10+8*(7 if index<7 else 3):error=RuntimeError('malformed metrics chunk')
                                    elif index in chunks and chunks[index]!=data[10:]:error=RuntimeError('conflicting metrics chunk')
                                    else:chunks[index]=data[10:]
                                    result=error
                                    if error is None and len(chunks)==8:result=struct.unpack('<52Q',b''.join(chunks[i] for i in range(8)))
                                    if result is not None:
                                        try:waiter.put_nowait(result)
                                        except queue.Full:pass
                            continue
                        # Echoed command replies ("> status" newline result) answer that command
                        # only; other shell text (enrollment, notices) keeps the generic key.
                        if data[:2]==b'> ':key=('console',data[2:].split(b'\n',1)[0].decode('ascii','replace'))
                        else:key=('console',)
                    with self._pending_lock:
                        waiter=self._pending.get(key)
                        if waiter:
                            try: waiter.put_nowait(payload)
                            except queue.Full: pass
                    if self.callback:
                        try: self._callbacks.put_nowait((message,payload))
                        except queue.Full: self.dropped_callbacks+=1
        except (OSError,Closed):
            self._fail_pending(Closed('transport disconnected'))
            self._stop.set()
    def _heartbeat(self):
        while not self._stop.wait(1.0):
            try: self.send(0,struct.pack('<IBBBBB',0,6,8,0,4,3))
            except (OSError,Closed): break
    def _callback_worker(self):
        while not self._stop.is_set():
            try: message=self._callbacks.get(timeout=.1)
            except queue.Empty: continue
            try: self.callback(*message)
            except Exception: pass  # user callback isolation, reader remains available
    def _fail_pending(self, error):
        with self._pending_lock:
            for waiter in self._pending.values():
                try: waiter.put_nowait(error)
                except queue.Full: pass
    def close(self):
        self._stop.set();self._fail_pending(Closed('connection closed'))
        with self._write: self.transport.close()
        for thread in self._threads:
            if thread is not threading.current_thread(): thread.join(timeout=1.0)
