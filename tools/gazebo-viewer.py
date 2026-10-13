#!/usr/bin/env python3
"""Local browser viewer and authenticated simulation-only control endpoints."""
import argparse
import json
import math
import pathlib
import re
import secrets
import signal
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit
from gazebo_browser_control import BrowserControl

ROOT = pathlib.Path(__file__).resolve().parents[1]
WEB = ROOT/'tools/viewer'

# `flix` in the bench world; flix1..flix8 in the swarm world (scripts/gazebo-swarm-world.py).
MODEL = re.compile(r'flix[1-8]?')

def parse_pose(block):
    name = re.search(r'name:\s*"([^"]+)"', block)
    if len(block) > 16384 or not name or not MODEL.fullmatch(name.group(1)):
        return None
    values = []
    for section, axes in [('position', 'xyz'), ('orientation', 'xyzw')]:
        found = re.search(section + r'\s*\{([^}]+)\}', block)
        if not found:
            return None
        for axis in axes:
            field = re.search(r'\b' + axis + r':\s*([^\s}]+)', found.group(1))
            if not field:
                return None
            try:
                value = float(field.group(1))
            except ValueError:
                return None
            if not math.isfinite(value):
                return None
            values.append(value)
    if not .9 <= sum(v*v for v in values[3:]) <= 1.1:
        return None
    return {'name': name.group(1), 'position': values[:3], 'quaternion_xyzw': values[3:]}

class Snapshot:
    def __init__(self, swarm=False):
        self.lock = threading.Lock()
        self.poses = {}
        self.swarm = swarm
        self.error = 'Waiting for Gazebo pose stream'

    def read(self):
        with self.lock:
            poses = dict(self.poses)
            state = {'pose': poses.get('flix'), 'poses': poses, 'error': self.error}
        try:
            path = ROOT/'.local/gazebo-telemetry.json'
            if path.stat().st_size > 16384:
                raise ValueError('telemetry size')
            state['telemetry'] = json.loads(path.read_text())
        except (OSError, ValueError):
            state['telemetry'] = None
        state['swarm'] = None
        if self.swarm:
            try:
                path = ROOT/'.local/gazebo-swarm.json'
                if path.stat().st_size > 65536:
                    raise ValueError('swarm state size')
                state['swarm'] = json.loads(path.read_text())
            except (OSError, ValueError):
                pass
        state['server_unix'] = time.time()
        return state

    def listen(self, container, stop):
        while not stop.is_set():
            try:
                # One-minute subscriptions avoid frequent transport reconnect gaps and bound orphan lifetime.
                process = subprocess.Popen(['docker', 'exec', container, 'stdbuf', '-oL', 'gz', 'topic', '-e', '/gazebo/default/pose/info', '-d', '60'], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
                block, depth = [], 0
                for line in process.stdout:
                    if stop.is_set():
                        break
                    if not block and line.strip() != 'pose {':
                        continue
                    block.append(line)
                    depth += line.count('{') - line.count('}')
                    if len(block) > 128:
                        block, depth = [], 0
                        continue
                    if depth == 0:
                        pose = parse_pose(''.join(block))
                        if pose:
                            pose['received_unix'] = time.time()
                            with self.lock:
                                self.poses[pose['name']], self.error = pose, None
                        block = []
                process.stdout.close()
                if process.poll() is None:
                    process.terminate()
                process.wait(timeout=7)
                if process.returncode and not stop.is_set():
                    with self.lock:
                        self.error = 'Gazebo stream unavailable; check the simulation container'
            except (OSError, subprocess.SubprocessError) as error:
                with self.lock:
                    self.error = str(error)
            stop.wait(.25)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--container', default='flix-rust-gazebo')
    parser.add_argument('--view-only', action='store_true')
    parser.add_argument('--swarm', action='store_true', help='show every drone of scripts/gazebo-swarm.sh (view only)')
    args = parser.parse_args()
    if args.swarm:
        args.view_only = True
        if args.container == 'flix-rust-gazebo':
            args.container = 'flix-rust-gazebo-swarm'
    snapshot, stop = Snapshot(args.swarm), threading.Event()
    controls, token = (None if args.view_only else BrowserControl(args.container)), secrets.token_urlsafe(24)

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = urlsplit(self.path).path
            if path == '/api/session':
                data,mime = json.dumps(dict(token=token,control=controls.snapshot() if controls else dict(enabled=False,ready=False,armed=False,error='View only; use the terminal control client'))).encode(),'application/json'
            elif path == '/api/state':
                data, mime = json.dumps(snapshot.read(), allow_nan=False).encode(), 'application/json'
            elif path in ('/', '/viewer.js', '/viewer.css'):
                file = {'/': 'index.html', '/viewer.js': 'viewer.js', '/viewer.css': 'viewer.css'}[path]
                data = (WEB/file).read_bytes()
                mime = {'/': 'text/html; charset=utf-8', '/viewer.js': 'text/javascript', '/viewer.css': 'text/css'}[path]
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'")
            self.end_headers()
            self.wfile.write(data)

        def do_POST(self):
            host=self.headers.get('Host','')
            allowed={f'127.0.0.1:{args.port}',f'localhost:{args.port}'}
            if host not in allowed or self.headers.get('Origin') != 'http://'+host or not secrets.compare_digest(self.headers.get('X-Flix-Token',''),token):
                self.send_error(403)
                return
            if urlsplit(self.path).path != '/api/control':
                self.send_error(404)
                return
            try:
                length=int(self.headers.get('Content-Length','0'))
                if not 0 < length <= 2048 or self.headers.get('Content-Type') != 'application/json':
                    raise ValueError('Invalid control request')
                request=json.loads(self.rfile.read(length))
                if controls is None:
                    raise ValueError('Browser flight control is disabled')
                if request.get('action') == 'input':
                    controls.input(request.get('values',{}))
                else:
                    controls.request(request.get('action'))
                result,status=dict(ok=True,control=controls.snapshot() if controls else dict(enabled=False,ready=False,armed=False,error='View only; use the terminal control client')),200
            except (ValueError,TypeError,AttributeError) as error:
                result,status=dict(ok=False,error=str(error)),409
            data=json.dumps(result).encode()
            self.send_response(status)
            self.send_header('Content-Type','application/json')
            self.send_header('Content-Length',str(len(data)))
            self.send_header('Cache-Control','no-store')
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, *_):
            pass

    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    server.daemon_threads = True
    thread = threading.Thread(target=snapshot.listen, args=(args.container, stop), daemon=True)
    thread.start()
    def shutdown(*_):
        stop.set()
        threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    print(f'Gazebo browser viewer: http://127.0.0.1:{args.port}', flush=True)
    try:
        server.serve_forever()
    finally:
        stop.set()
        server.server_close()
        if controls:
            controls.close()
        thread.join(timeout=8)

if __name__ == '__main__':
    main()
