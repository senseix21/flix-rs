"""Single local simulation control client; bounded commands and browser input lease."""
import math
import queue
import subprocess
import threading
import time
from pyflix.client import Flix, UdpTransport

class BrowserControl:
    def __init__(self, container='flix-rust-gazebo'):
        self.container = container
        self.lock = threading.Lock()
        self.commands = queue.Queue(8)
        self.stop_requested = threading.Event()
        self.closed = threading.Event()
        self.ready = self.armed = False
        self.error = 'Connecting to simulation'
        self.inputs = dict(roll=0., pitch=0., yaw=0., thrust=0.)
        self.expires = 0.
        self.disarmed_at = 0.
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def snapshot(self):
        with self.lock:
            return dict(ready=self.ready, armed=self.armed, error=self.error)

    def input(self, values):
        if set(values) != {'roll','pitch','yaw','thrust'}:
            raise ValueError('Invalid control fields')
        for key,value in values.items():
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not (-1 <= value <= 1 if key != 'thrust' else 0 <= value <= 1):
                raise ValueError('Control value outside limits')
        with self.lock:
            if not self.armed:
                raise ValueError('Arm the simulation first')
            self.inputs = dict(values)
            self.expires = time.monotonic() + .5

    def request(self, action):
        if action == 'disarm':
            with self.lock:
                self.armed = False
                self.disarmed_at = time.monotonic()
                self.inputs = dict(roll=0.,pitch=0.,yaw=0.,thrust=0.)
            self.stop_requested.set()
            return
        if action not in {'arm','clear-fault','reset'}:
            raise ValueError('Unknown action')
        reply = queue.Queue(1)
        with self.lock:
            if not self.ready:
                raise ValueError('Simulation is still connecting')
            try:
                self.commands.put_nowait((action,time.monotonic(),reply))
            except queue.Full:
                raise ValueError('Command queue full') from None
        try:
            error = reply.get(timeout=15 if action == 'reset' else 2)
        except queue.Empty:
            raise ValueError('Command acknowledgement timed out') from None
        if error:
            raise ValueError(error)

    def stop(self, client):
        with self.lock:
            self.armed = False
            self.inputs = dict(roll=0.,pitch=0.,yaw=0.,thrust=0.)
        client.disarm()

    def run(self):
        while not self.closed.is_set():
            self.run_session()
            if not self.closed.is_set():
                self.closed.wait(.5)

    def run_session(self):
        try:
            with Flix(UdpTransport(),simulation=True) as client:
                while not self.closed.is_set():
                    try:
                        client.get_parameter(0)
                        break
                    except TimeoutError:
                        self.closed.wait(.25)
                if self.closed.is_set():
                    return
                self.stop(client)
                with self.lock:
                    self.ready, self.error = True, None
                try:
                    while not self.closed.is_set():
                        if self.stop_requested.is_set():
                            self.stop_requested.clear()
                            self.stop(client)
                        try:
                            action,at,reply = self.commands.get_nowait()
                        except queue.Empty:
                            action = None
                        if action:
                            error = None
                            try:
                                if time.monotonic()-at > .5:
                                    raise ValueError('Command expired; try again')
                                if action == 'reset':
                                    self.stop(client)
                                    with self.lock:
                                        self.ready, self.error = False, 'Resetting simulation'
                                        self.disarmed_at = time.monotonic()
                                    for _ in range(8):
                                        try:
                                            cancelled = self.commands.get_nowait()
                                            cancelled[2].put_nowait('Simulation reset cancelled queued command')
                                        except queue.Empty:
                                            break
                                    subprocess.run(['docker','restart',self.container], check=True,
                                                   timeout=12, capture_output=True)
                                elif action == 'clear-fault':
                                    self.stop(client)
                                    client.console('clear-fault')
                                    client.set_mode(2)
                                else:
                                    with self.lock:
                                        if at <= self.disarmed_at:
                                            raise ValueError('Disarm cancelled queued arm command')
                                        if self.armed or self.inputs['thrust'] != 0:
                                            raise ValueError('Disarm and set zero thrust first')
                                    client.set_mode(2)
                                    client.manual(thrust=0)
                                    client.arm()
                                    with self.lock:
                                        self.armed = True
                                        self.expires = time.monotonic()+.5
                            except (ValueError,RuntimeError,TimeoutError,OSError,subprocess.SubprocessError) as exc:
                                error = str(exc)
                            reply.put_nowait(error)
                            if action == 'reset':
                                return
                        with self.lock:
                            armed,expires,values = self.armed,self.expires,dict(self.inputs)
                        if armed and (self.stop_requested.is_set() or time.monotonic() >= expires):
                            self.stop(client)
                        elif armed:
                            client.manual(**values)
                        self.closed.wait(.02)
                finally:
                    self.stop(client)
        except (OSError,RuntimeError,TimeoutError) as error:
            with self.lock:
                self.ready,self.armed,self.error = False,False,str(error)

    def close(self):
        self.closed.set()
        self.thread.join(timeout=3)
