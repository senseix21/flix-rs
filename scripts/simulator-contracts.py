#!/usr/bin/env python3
"""Deterministic fault replay through the real simulator ABI, without physics truth."""
import ctypes
import json
import math
from pathlib import Path
import random
import struct
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from pyflix.client import pack

class Input(ctypes.Structure):
    _fields_ = [('version', ctypes.c_uint32), ('calibrated', ctypes.c_uint32),
                ('now', ctypes.c_uint64), ('sample', ctypes.c_uint64),
                ('gyro', ctypes.c_float * 3), ('acc', ctypes.c_float * 3)]

class Output(ctypes.Structure):
    _fields_ = [('version', ctypes.c_uint32), ('state', ctypes.c_uint32),
                ('reason', ctypes.c_uint32), ('reserved', ctypes.c_uint32),
                ('now', ctypes.c_uint64), ('generation', ctypes.c_uint64),
                ('motors', ctypes.c_float * 4)]

class Instance:
    def __init__(self, library):
        self.lib = library
        self.handle = library.flix_create()
        assert self.handle, 'instance capacity exceeded'
        self.now = 0
        self.sequence = 0
        self.tick(now=0)

    def close(self):
        assert self.lib.flix_destroy(self.handle) == 0

    def receive(self, message, payload, accepted=True):
        frame = pack(message, payload, self.sequence)
        self.sequence = (self.sequence + 1) & 255
        result = self.lib.flix_receive(self.handle, frame, len(frame), self.now + 1000)
        assert (result == 0) == accepted, (message, result)

    def command(self, command, *args, accepted=True):
        self.receive(76, struct.pack('<7fHBBB', *(list(args) + [0.] * (7-len(args))),
                                    command, 1, 1, 0), accepted)

    def setpoint(self, mode, thrust, axes=(0, 0, 0)):
        if mode in (1, 2):
            self.receive(69, struct.pack('<hhhhHB', axes[1], axes[0], int(thrust*1000), axes[2], 0, 1))
        else:
            self.receive(82, struct.pack('<I8fBBB', 0, 1, 0, 0, 0,
                                        *axes, thrust, 1, 1, 128 if mode == 4 else 0))

    def tick(self, *, now=None, sample=None, gyro=(0, 0, 0), acc=(0, 0, 9.80665), calibrated=1):
        self.now = self.now + 1000 if now is None else now
        sensor = Input(1, calibrated, self.now, self.now if sample is None else sample,
                       (ctypes.c_float * 3)(*gyro), (ctypes.c_float * 3)(*acc))
        out = Output()
        assert self.lib.flix_tick(self.handle, ctypes.byref(sensor), ctypes.byref(out)) == 0
        assert all(math.isfinite(v) and 0 <= v <= 1 for v in out.motors)
        buffer = ctypes.create_string_buffer(280)
        for _ in range(4):
            if self.lib.flix_reply(self.handle, buffer, 280) == 0:
                break
        self.output = out
        return out

    def arm(self, mode=2):
        self.command(176, 1, min(mode, 3))
        self.tick()
        self.setpoint(mode, 0)
        self.command(400, 1)
        assert self.tick().state == 3
        self.setpoint(mode, .4)
        assert any(v > 0 for v in self.tick().motors)

    def stopped(self, state=None, reason=None):
        assert list(self.output.motors) == [0]*4
        if state is not None:
            assert self.output.state == state, (self.output.state, state)
        if reason is not None:
            assert self.output.reason == reason, (self.output.reason, reason)


def main():
    filename = 'libflight_gazebo.dylib' if sys.platform == 'darwin' else 'libflight_gazebo.so'
    lib = ctypes.CDLL(str(ROOT / 'target/release' / filename))
    lib.flix_create.restype = ctypes.c_uint64
    lib.flix_destroy.argtypes = lib.flix_reset.argtypes = [ctypes.c_uint64]
    lib.flix_tick.argtypes = [ctypes.c_uint64, ctypes.POINTER(Input), ctypes.POINTER(Output)]
    lib.flix_receive.argtypes = [ctypes.c_uint64, ctypes.c_void_p, ctypes.c_uint32, ctypes.c_uint64]
    lib.flix_reply.argtypes = [ctypes.c_uint64, ctypes.c_void_p, ctypes.c_uint32]
    passed = []
    for mode, name in [(1, 'ACRO'), (2, 'STAB'), (3, 'AUTO attitude'), (4, 'AUTO rates')]:
        instance = Instance(lib)
        try:
            instance.arm(mode)
            instance.command(400, 0)
            instance.tick(); instance.stopped(1)
            passed.append(name + ' explicit arm and stop')
        finally:
            instance.close()
    for name, kwargs, reason in [
        ('nonfinite IMU', {'gyro': (float('nan'), 0, 0)}, 5),
        ('stale IMU', {'sample': 0}, 4),
        ('calibration lost', {'calibrated': 0}, 2),
        ('clock rollback', {'now': 1}, 6),
        ('clock gap', {'now': 100000}, 6),
    ]:
        instance = Instance(lib)
        try:
            instance.arm(); instance.tick(**kwargs); instance.stopped(5, reason)
            instance.command(400, 1); instance.tick(); instance.stopped(5)
            passed.append(name + ' stops and denies rearm')
        finally:
            instance.close()
    instance = Instance(lib)
    try:
        instance.receive(126, struct.pack('<IHBBB70s', 0, 0, 10, 0, 11, b'maintenance'))
        assert instance.tick().state == 2
        instance.command(209, 1, 0, 10, .05, 1)
        instance.tick(); instance.stopped(2)
        assert any(v > 0 for v in instance.tick().motors)
        instance.command(400, 0); instance.tick(); instance.stopped(1)
        passed.append('direct diagnostic output obeys explicit stop')
    finally:
        instance.close()
    instance, other = Instance(lib), Instance(lib)
    try:
        instance.arm()
        for _ in range(1100):
            instance.receive(0, struct.pack('<IBBBBB', 0, 6, 8, 0, 4, 3))
            instance.receive(69, struct.pack('<hhhhHB', 0, 0, 2000, 0, 0, 1), accepted=False)
            instance.tick()
        instance.stopped(4, 3)
        passed.append('heartbeat and rejected inputs do not renew command lease')
        other.arm()
        instance.setpoint(2, 0)
        instance.command(400, 1)
        assert lib.flix_reset(instance.handle) == 0
        instance.tick(now=0); instance.stopped(1)
        # Resets discard queued authority and never reset a second instance.
        instance.command(400, 1); instance.tick(); instance.stopped(1)
        other.setpoint(2, .4); assert other.tick().state == 3
        passed.append('reset clears authority and preserves independent instance')
        for _ in range(8): other.command(400, 1)
        other.command(400, 1, accepted=False)
        other.command(400, 0); other.tick(); other.stopped(1)
        passed.append('reserved stop dominates full command queue')
        other.arm(1)
        rng = random.Random(0x5eed)
        for _ in range(1000):
            other.setpoint(1, .8, tuple(rng.randint(-1000, 1000) for _ in range(3)))
            other.tick()
        other.command(400, 0); other.tick(); other.stopped(1)
        passed.append('seeded extreme ACRO inputs stay bounded and disarm')
    finally:
        instance.close(); other.close()
    print(json.dumps(dict(profile='simulator-ABI-replay', seed=0x5eed,
                         physics_test=False, hardware_test=False, passed=passed), indent=2))

if __name__ == '__main__':
    main()
