"""ctypes binding to the Flix owner ABI (`adapters/gazebo/flix.h`) and the MAVLink frames we send.

The library is the same `flight-gazebo` build Gazebo links, so each instance runs the real
`flight_esp32::Owner` around `flight_core::Flight`. Commands enter only as MAVLink frames through
`flix_receive`, under the simulator's explicit unauthenticated bench policy.
"""
import ctypes
import os
import pathlib
import struct
import sys

ROOT = pathlib.Path(__file__).resolve().parents[3]
# pyflix owns the pinned dialect (CRC extras and lengths); reuse it rather than copy it.
sys.path.insert(0, str(ROOT / 'tools'))
from pyflix.client import Decoder, pack  # noqa: E402

BOOT, STANDBY, MAINTENANCE, ARMED, FAILSAFE, FAULT = range(6)
STATES = ('boot', 'standby', 'maintenance', 'armed', 'failsafe', 'fault')
REASONS = (
    'none', 'disarmed', 'not-ready', 'command-expired', 'sensor-expired', 'invalid-numeric',
    'clock', 'generation', 'tilt', 'maintenance-expired', 'configuration', 'power', 'sensor-init',
    'position-expired',
)
# SET_POSITION_TARGET_LOCAL_NED masks Flix accepts (flight_protocol::POSITION_MASK*).
POSITION_MASK_WITH_VELOCITY = 0x09C0
MAV_FRAME_LOCAL_NED = 1
AUTO_MODE = 3


class Capacity(RuntimeError):
    """The owner registry refused another instance (SWM-11)."""


class _Input(ctypes.Structure):
    _fields_ = [
        ('version', ctypes.c_uint32), ('calibrated', ctypes.c_uint32),
        ('now_us', ctypes.c_uint64), ('sample_us', ctypes.c_uint64),
        ('gyro', ctypes.c_float * 3), ('acceleration', ctypes.c_float * 3),
        ('saturation', ctypes.c_uint32), ('reserved', ctypes.c_uint32),
    ]


class _Output(ctypes.Structure):
    _fields_ = [
        ('version', ctypes.c_uint32), ('state', ctypes.c_uint32), ('reason', ctypes.c_uint32),
        ('reserved', ctypes.c_uint32), ('now_us', ctypes.c_uint64), ('generation', ctypes.c_uint64),
        ('motors', ctypes.c_float * 4),
    ]


def _load():
    name = 'libflight_gazebo.dylib' if sys.platform == 'darwin' else 'libflight_gazebo.so'
    path = os.environ.get('FLIX_LIBRARY') or str(ROOT / 'target/release' / name)
    library = ctypes.CDLL(path)
    library.flix_create.restype = ctypes.c_uint64
    library.flix_destroy.argtypes = [ctypes.c_uint64]
    library.flix_tick2.argtypes = [ctypes.c_uint64, ctypes.POINTER(_Input), ctypes.POINTER(_Output)]
    library.flix_receive.argtypes = [ctypes.c_uint64, ctypes.c_char_p, ctypes.c_uint32, ctypes.c_uint64]
    library.flix_reply.argtypes = [ctypes.c_uint64, ctypes.c_char_p, ctypes.c_uint32]
    for function in (library.flix_destroy, library.flix_tick2, library.flix_receive, library.flix_reply):
        function.restype = ctypes.c_int32
    return library


_library = None


class Owner:
    """One simulated flight controller. Close it to free its registry slot."""

    def __init__(self):
        global _library
        if _library is None:
            _library = _load()
        self.handle = _library.flix_create()
        if not self.handle:
            raise Capacity('owner registry full')
        self._buffer = ctypes.create_string_buffer(280)

    def tick(self, now_us, gyro, acceleration, saturation=0, calibrated=True, sample_us=None):
        sample = _Input(
            2, 1 if calibrated else 0, now_us, now_us if sample_us is None else sample_us,
            (ctypes.c_float * 3)(*gyro), (ctypes.c_float * 3)(*acceleration), saturation, 0,
        )
        output = _Output()
        status = _library.flix_tick2(self.handle, ctypes.byref(sample), ctypes.byref(output))
        # A rejected input still writes a stopped output; the status is reported, not hidden.
        return status, output.state, output.reason, output.generation, tuple(output.motors)

    def receive(self, frame, now_us):
        return _library.flix_receive(self.handle, frame, len(frame), now_us) == 0

    def replies(self):
        frames = []
        while True:
            length = _library.flix_reply(self.handle, self._buffer, len(self._buffer))
            if length <= 0:
                return frames
            frames.append(self._buffer.raw[:length])

    def close(self):
        if self.handle:
            _library.flix_destroy(self.handle)
            self.handle = 0


class Messages:
    """MAVLink frames from one ground sender (bench policy: system 255, component 190)."""

    def __init__(self):
        self.sequence = 0

    def _frame(self, message, payload):
        frame = pack(message, payload, self.sequence)
        self.sequence = (self.sequence + 1) & 255
        return frame

    def command(self, target, number, *parameters):
        values = list(parameters) + [0.0] * (7 - len(parameters))
        return self._frame(76, struct.pack('<7fHBBB', *values, number, target, 1, 0))

    def arm(self, target):
        return self.command(target, 400, 1.0)

    def stop(self, target):
        """COMMAND_LONG disarm; the owner routes it to the reserved stop path (SAF-04)."""
        return self.command(target, 400, 0.0)

    def auto_mode(self, target):
        return self.command(target, 176, 1.0, float(AUTO_MODE))

    def console(self, target, text):
        # Use the MAVLink 2 target extension: an unaddressed console line would reach every
        # drone sharing a link (SWM-01).
        data = text.encode()
        return self._frame(126, struct.pack('<IHBBB70sBB', 0, 0, 10, 0, len(data), data, target, 1))

    def parameter(self, target, name, value):
        return self._frame(23, struct.pack('<fBB16sB', value, target, 1, name.encode().ljust(16, b'\0'), 9))

    def position_target(self, target, position_nwu, velocity_nwu, yaw_nwu):
        """SET_POSITION_TARGET_LOCAL_NED, converted from Flix world NWU to NED."""
        x, y, z = position_nwu
        vx, vy, vz = velocity_nwu
        return self._frame(84, struct.pack('<I11fHBBB', 0, x, -y, -z, vx, -vy, -vz, 0.0, 0.0, 0.0,
                                           -yaw_nwu, 0.0, POSITION_MASK_WITH_VELOCITY, target, 1,
                                           MAV_FRAME_LOCAL_NED))

    def mocap(self, position_nwu):
        """ATT_POS_MOCAP. No target field: it must travel on this drone's own link."""
        x, y, z = position_nwu
        return self._frame(138, struct.pack('<Q7f', 0, 1.0, 0.0, 0.0, 0.0, x, -y, -z))

    def attitude(self, target, attitude_flu, thrust):
        """SET_ATTITUDE_TARGET. Body rates are sent as zero feedforward (mask 0)."""
        w, x, y, z = attitude_flu
        # The protocol converts NED/FRD to FLU by negating y and z; send the inverse.
        return self._frame(82, struct.pack('<I8fBBB', 0, w, x, -y, -z, 0.0, 0.0, 0.0, thrust, target, 1, 0))


def parse_replies(decoder, frames):
    """Decode owner replies into (kind, value) pairs.

    kinds: 'ack' (command, result), 'notice' text, 'parameter' (name, value),
    'heartbeat' (armed, critical).
    """
    events = []
    for frame in frames:
        for message, payload in decoder.feed(frame):
            if message == 77:
                events.append(('ack', struct.unpack_from('<HB', payload)))
            elif message == 253:
                events.append(('notice', payload[1:51].split(b'\0', 1)[0].decode('ascii', 'replace')))
            elif message == 22:
                value, _, _, name = struct.unpack_from('<fHH16s', payload)
                events.append(('parameter', (name.split(b'\0', 1)[0].decode(), value)))
            elif message == 0:
                events.append(('heartbeat', (bool(payload[6] & 128), payload[7] == 5)))
    return events


__all__ = ['Capacity', 'Decoder', 'Messages', 'Owner', 'parse_replies']
