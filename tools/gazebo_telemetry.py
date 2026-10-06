"""Observe existing simulator replies without introducing control traffic."""
import json
import math
import os
import pathlib
import struct
import time
from pyflix.client import Decoder

class Telemetry:
    def __init__(self, destination):
        self.destination = pathlib.Path(destination)
        self.decoder = Decoder()
        self.state = {}
        self.last_write = 0.0

    def feed(self, data):
        for message, payload in self.decoder.feed(data):
            now = time.time()
            if message == 140:
                values = struct.unpack_from('<4f', payload, 8)
                if not all(math.isfinite(v) and 0 <= v <= 1 for v in values):
                    continue
                self.state.update(motors=list(values), motor_unix=now)
            elif message == 0:
                self.state.update(armed=bool(payload[6] & 128), fault=payload[7] == 5, heartbeat_unix=now)
            elif message == 31:
                values = struct.unpack_from('<4f', payload, 4)
                if all(math.isfinite(v) for v in values):
                    self.state['estimate_wxyz'] = list(values)
            else:
                continue
            self.state['received_unix'] = now
        if self.state and time.monotonic() - self.last_write >= .05:
            self.last_write = time.monotonic()
            try:
                self.destination.parent.mkdir(parents=True, exist_ok=True)
                temporary = self.destination.with_suffix('.tmp')
                temporary.write_text(json.dumps(self.state, allow_nan=False))
                os.replace(temporary, self.destination)
            except OSError:
                # Observation failure must not interrupt the simulated control transport.
                pass
