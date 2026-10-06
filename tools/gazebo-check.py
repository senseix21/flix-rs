"""Exercise the local simulation only; rejects any non-loopback endpoint."""
import json
import struct
import threading
import time
from pyflix.client import Flix, UdpTransport

events = []
lock = threading.Lock()

def observe(message, payload):
    with lock:
        events.append((message, payload))

def motors():
    with lock:
        return [struct.unpack_from('<4f', p, 8) for m, p in events if m == 140]

with Flix(UdpTransport(), simulation=True, callback=observe) as client:
    deadline = time.monotonic() + 60
    while True:
        try:
            parameter = client.get_parameter(0)
            break
        except TimeoutError:
            if time.monotonic() >= deadline:
                raise
    client.disarm()
    client.set_mode(2)
    client.manual(thrust=0)
    armed = False
    stopped = False
    try:
        client.arm()
        armed = True
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            client.manual(thrust=.15)
            if any(any(v > 0 for v in row) for row in motors()):
                break
            time.sleep(.02)
        assert any(any(v > 0 for v in row) for row in motors()), 'no positive motor telemetry'
    finally:
        client.disarm()
    with lock:
        events.clear()
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        rows = motors()
        if rows and rows[-1] == (0., 0., 0., 0.):
            stopped = True
            break
        time.sleep(.02)
    assert stopped, 'no stopped motor telemetry'
    with lock:
        ids = sorted({m for m, _ in events})
    print(json.dumps({'endpoint': '127.0.0.1:14550', 'parameter_0': struct.unpack_from('<f', parameter)[0], 'mode': 'STAB', 'arm_ack': armed, 'positive_motor_telemetry': True, 'disarm_ack': True, 'motors_after_disarm': rows[-1], 'received_message_ids': ids, 'hardware_test': False}, indent=2))
