"""Headless Gazebo and bounded UDP relay for Docker's loopback-only Rust plugin.

Publish container UDP 14551 only on host 127.0.0.1. This relay is a development
transport, not an authenticated flight gateway. One client endpoint at a time.
"""
import pathlib
import selectors
import signal
import socket
import subprocess
from gazebo_telemetry import Telemetry

root = pathlib.Path(__file__).resolve().parents[1]
server = subprocess.Popen(['gzserver', '--verbose', str(root/'adapters/gazebo/bench.world')])
telemetry = Telemetry(root/'.local/gazebo-telemetry.json')
running = True

def stop(*_):
    global running
    running = False

signal.signal(signal.SIGTERM, stop)
signal.signal(signal.SIGINT, stop)
external = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
internal = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
selector = selectors.DefaultSelector()
try:
    external.bind(('0.0.0.0', 14551))
    internal.bind(('127.0.0.1', 0))
    internal.connect(('127.0.0.1', 14550))
    for sock in (external, internal):
        sock.setblocking(False)
        selector.register(sock, selectors.EVENT_READ)
    client = None
    print('Gazebo starting; host loopback UDP 14550 forwards to Rust plugin', flush=True)
    while running:
        if server.poll() is not None:
            raise RuntimeError('gzserver exited with status %s' % server.returncode)
        for key, _ in selector.select(timeout=.1):
            try:
                data, source = key.fileobj.recvfrom(281)
                if not 0 < len(data) <= 280:
                    continue
                if key.fileobj is external:
                    if data[0] != 253:
                        continue
                    client = source
                    internal.send(data)
                else:
                    telemetry.feed(data)
                    if client is not None:
                        external.sendto(data, client)
            except (BlockingIOError, ConnectionRefusedError):
                # During world startup the plugin's socket may not be ready.
                continue
finally:
    selector.close()
    external.close()
    internal.close()
    if server.poll() is None:
        server.terminate()
        try:
            server.wait(timeout=5)
        except subprocess.TimeoutExpired:
            server.kill()
            server.wait()
