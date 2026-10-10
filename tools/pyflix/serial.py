"""POSIX raw UART transport. Direct USB/UART and proxy roles are explicit."""
import os
import select
import termios
import threading
import time

class SerialTransport:
    network = False
    def __init__(self, path, *, baud=115200, proxy=False):
        from .client import Closed
        self._closed_error = Closed
        self.trusted_wired = not proxy
        self.requires_signing = proxy
        # Reads and writes use independent locks: the reader thread polls receive() in a
        # tight loop, and one shared non-fair lock let it starve send() past its deadline.
        self._read_lock = threading.Lock(); self._write_lock = threading.Lock()
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            settings = termios.tcgetattr(self.fd)
            speed = getattr(termios, 'B'+str(baud), None)
            if speed is None: raise ValueError('unsupported baud rate')
            settings[0] = 0; settings[1] = 0
            settings[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            settings[3] = 0; settings[4] = speed; settings[5] = speed
            settings[6][termios.VMIN] = 0; settings[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, settings)
        except Exception:
            os.close(self.fd); self.fd = -1; raise
    def _check(self):
        if self.fd < 0: raise self._closed_error('serial closed')
    def send(self, data):
        if not 0<len(data)<=280: raise ValueError('one bounded MAVLink frame per write')
        deadline = time.monotonic()+.5
        with self._write_lock:
            self._check(); remaining = memoryview(data)
            while remaining:
                wait = deadline-time.monotonic()
                if wait <= 0: raise OSError('serial write deadline')
                if not select.select([], [self.fd], [], wait)[1]: raise OSError('serial write deadline')
                try: count = os.write(self.fd, remaining)
                except BlockingIOError: continue
                if count <= 0: raise self._closed_error('serial disconnected')
                remaining = remaining[count:]
    def receive(self):
        with self._read_lock:
            self._check()
            if not select.select([self.fd], [], [], .01)[0]: return b''
            try: data = os.read(self.fd, 1120)
            except BlockingIOError: return b''
            if not data: raise self._closed_error('serial disconnected')
            return data
    def close(self):
        with self._read_lock, self._write_lock:
            if self.fd >= 0: os.close(self.fd); self.fd = -1
