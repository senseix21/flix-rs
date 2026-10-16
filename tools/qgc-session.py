#!/usr/bin/env python3
"""R2-04: operator-driven QGroundControl session against the Rust bench, with a traffic record.

Starts the bench with a fresh per-boot signing key, starts QGC with a private settings directory
holding the same key, and relays every frame through a recorder. An operator follows the
checklist in `progress.txt`; the recorder ticks steps off from the traffic itself, so the result
does not depend on what anyone reports. Once the drone is armed with throttle above 20 %, the
recorder cuts the link for 3 s to test loss and reconnect.

    python3 tools/qgc-session.py --executable .local/qgc/QGroundControl.app/Contents/MacOS/QGroundControl \\
        --output .local/qualification/r2/qgc-session

Signing keys never reach stdout, `progress.txt` or `result.json`. Private files are 0600 and the
enrollment frame is deleted at exit.

With `--gazebo 127.0.0.1:14550` the vehicle is the Rust owner inside Gazebo (scripts/gazebo-run.sh)
instead of the bench. That link is the plugin's unsigned loopback development policy, so no key is
made; the recorder sends heartbeats until the plugin answers, so it learns where to send telemetry.
"""
import argparse
import hashlib
import json
import os
import pathlib
import secrets
import socket
import struct
import subprocess
import sys
import threading
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from pyflix.client import pack  # noqa: E402

RELAY_PORT = 14554
QGC_PORT = 14553
CUT_SECONDS = 3.0
STEPS = [
    ('connected', 'QGC shows the vehicle and finishes loading parameters'),
    ('stab', 'Analyze Tools > MAVLink Console: type  stab  and press Enter (reply "> stab" "ok")'),
    ('joystick', 'Application Settings > Fly View: enable Virtual Joystick, Auto-Center Throttle OFF'),
    ('armed', 'Throttle stick fully down, then Arm (toolbar "Disarmed" > Arm, slide to confirm)'),
    ('throttle', 'Raise the throttle stick above 20 % for a few seconds'),
    ('cut', '(automatic) the recorder cuts the link for 3 s: expect the drone to stop and disarm'),
    ('reconnected', 'QGC reconnects by itself; the drone stays disarmed'),
    ('rearmed', 'Throttle down, type  stab  in the console again if needed, then Arm again'),
    ('disarmed', 'Disarm from QGC (toolbar "Armed" > Disarm)'),
]


def frames(data):
    """Yield (system, component, message, payload, signed) for each MAVLink 2 frame."""
    i = 0
    while i + 12 <= len(data):
        if data[i] != 253:
            i += 1
            continue
        length, flags = data[i + 1], data[i + 2]
        size = 12 + length + (13 if flags & 1 else 0)
        frame = data[i:i + size]
        if len(frame) < size:
            return
        yield frame[5], frame[6], int.from_bytes(frame[7:10], 'little'), frame[10:10 + length], bool(flags & 1)
        i += size


class Session:
    def __init__(self, output):
        self.output = output
        self.lock = threading.Lock()
        self.done = {}
        self.events = []
        self.armed = False
        self.counts = {'to_drone': {}, 'to_qgc': {}}
        self.manual = 0
        self.max_throttle = 0.0
        self.throttle_since = None
        self.cut_until = None
        self.last_qgc = 0.0
        self.parameters = set()
        self.start = time.monotonic()

    def now(self):
        return round(time.monotonic() - self.start, 3)

    def mark(self, step, detail=''):
        with self.lock:
            if step in self.done:
                return
            # Steps are ordered: recovery steps only count after the cut.
            names = [s for s, _ in STEPS]
            if any(n not in self.done for n in names[:names.index(step)] if n not in ('joystick',)):
                return
            self.done[step] = self.now()
            self.events.append((self.now(), 'step', step, detail))
        self.write()

    def note(self, kind, detail):
        with self.lock:
            self.events.append((self.now(), kind, detail))

    def to_drone(self, data):
        self.last_qgc = time.monotonic()
        for system, component, message, payload, signed in frames(data):
            key = f'{system}/{component}/{message}/{"signed" if signed else "unsigned"}'
            self.counts['to_drone'][key] = self.counts['to_drone'].get(key, 0) + 1
            if message == 69 and len(payload) >= 8:
                self.manual += 1
                throttle = struct.unpack_from('<h', payload, 4)[0] / 1000
                self.max_throttle = max(self.max_throttle, throttle)
                self.mark('joystick', f'MANUAL_CONTROL from {system}/{component}')
                if self.armed and throttle > 0.2:
                    self.throttle_since = self.throttle_since or time.monotonic()
                    if time.monotonic() - self.throttle_since > 2.0:
                        self.mark('throttle', f'throttle {throttle:.2f}')
                else:
                    self.throttle_since = None
            elif message == 76 and len(payload) >= 30:
                command = struct.unpack_from('<H', payload, 28)[0]
                self.note('command', f'COMMAND_LONG {command} param1 {struct.unpack_from("<f", payload, 0)[0]:g}')
            elif message == 126 and len(payload) >= 9:
                count = min(payload[8], 70)
                text = bytes(payload[9:9 + count]).split(b'\0', 1)[0].decode('ascii', 'replace').strip()
                self.note('console', f'to drone: {text!r} (device {payload[6]})')

    def to_qgc(self, data):
        for system, component, message, payload, signed in frames(data):
            key = f'{system}/{component}/{message}'
            self.counts['to_qgc'][key] = self.counts['to_qgc'].get(key, 0) + 1
            if message == 22 and len(payload) >= 25:
                total, index = struct.unpack_from('<HH', payload, 4)
                self.parameters.add(index)
                if len(self.parameters) == total:
                    self.mark('connected', f'{total} parameters sent')
            elif message == 0 and len(payload) >= 9:
                armed = bool(payload[6] & 128)
                if armed != self.armed:
                    self.note('heartbeat', 'armed' if armed else 'disarmed')
                self.armed = armed
                if armed:
                    self.mark('armed')
                    if 'reconnected' in self.done:
                        self.mark('rearmed')
                elif 'cut' in self.done and 'reconnected' not in self.done and self.cut_until is None:
                    # Link back and the drone reports itself disarmed.
                    if time.monotonic() - self.last_qgc < 1.0:
                        self.mark('reconnected', 'heartbeat disarmed after the cut')
                elif 'rearmed' in self.done:
                    self.mark('disarmed')
            elif message == 77 and len(payload) >= 3:
                command, result = struct.unpack_from('<HB', payload)
                self.note('ack', f'COMMAND_ACK {command} result {result}')
            elif message == 126 and len(payload) >= 9:
                text = bytes(payload[9:9 + payload[8]]).decode('ascii', 'replace')
                self.note('console', f'from drone: {text!r}')
                if text.startswith('> stab\nok'):
                    self.mark('stab')
            elif message == 253 and len(payload) >= 2:
                self.note('statustext', bytes(payload[1:51]).split(b'\0', 1)[0].decode('ascii', 'replace'))

    def write(self):
        with self.lock:
            lines = [f'QGC session, {self.now():.0f} s', '']
            for step, text in STEPS:
                lines.append(f'[{"x" if step in self.done else " "}] {text}')
            lines += ['', f'drone armed: {self.armed}   MANUAL_CONTROL frames: {self.manual}   '
                          f'max throttle: {self.max_throttle:.2f}',
                      f'frames to QGC: {sum(self.counts["to_qgc"].values())}   '
                      f'frames to drone: {sum(self.counts["to_drone"].values())}   '
                      f'parameters sent: {len(self.parameters)}']
            lines += ['', 'recent:'] + [f'  {e[0]:7.2f} {" ".join(map(str, e[1:]))}' for e in self.events[-8:]]
            (self.output / 'progress.txt').write_text('\n'.join(lines) + '\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--executable', type=pathlib.Path, required=True)
    parser.add_argument('--output', type=pathlib.Path, required=True)
    parser.add_argument('--minutes', type=float, default=12.0)
    parser.add_argument('--gazebo', metavar='HOST:PORT',
                        help='use a running Gazebo simulation (unsigned development link) instead of the bench')
    args = parser.parse_args()
    if subprocess.run(['pgrep', '-x', 'QGroundControl'], capture_output=True).returncode == 0:
        raise SystemExit('close any running QGroundControl first')
    args.output.mkdir(parents=True, exist_ok=False)
    if args.gazebo:
        return gazebo_session(args)
    phrase = secrets.token_hex(32)
    key = hashlib.sha256(phrase.encode()).digest()
    stamp = int((time.time() - 1420070400) * 100000)
    enrollment = args.output / 'private.enrollment.frame'
    with os.fdopen(os.open(enrollment, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600), 'wb') as f:
        f.write(pack(256, struct.pack('<QBB32s', stamp, 1, 1, key)))
    config = args.output.resolve() / 'private-config'
    ini = config / 'qgroundcontrol.com/QGroundControl.ini'
    ini.parent.mkdir(parents=True)
    with os.fdopen(os.open(ini, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600), 'w') as f:
        f.write(f'[General]\nmavlink2SigningKey={phrase}\ntelemetrySave=false\n\n[AutoConnect]\nudpListenPort={QGC_PORT}\n')
    session = Session(args.output)
    stop = threading.Event()
    relay = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    relay.bind(('127.0.0.1', RELAY_PORT))
    relay.settimeout(0.1)
    environment = dict(os.environ, XDG_CONFIG_HOME=str(config), XDG_CACHE_HOME=str(config.parent / 'private-cache'),
                       XDG_DATA_HOME=str(config.parent / 'private-data'),
                       # QGC v5.0.8 derives its signing timestamp from local time; outside UTC it
                       # takes the bench's correctly signed frames for stale ones and drops them.
                       TZ='UTC',
                       FLIX_BENCH_ENDPOINT=f'127.0.0.1:{RELAY_PORT}',
                       FLIX_BENCH_TICKS=str(int(args.minutes * 60_000)))
    bench = qgc = None
    try:
        with (args.output / 'bench.log').open('w') as log:
            bench = subprocess.Popen([str(ROOT / 'target/release/bench'), '0', str(enrollment)],
                                     stdout=subprocess.PIPE, stderr=log, text=True, env=environment)
            address, port = bench.stdout.readline().strip().rsplit(':', 1)
            endpoint = (address, int(port))
            qlog = (args.output / 'qgc-private.log').open('w')
            os.chmod(args.output / 'qgc-private.log', 0o600)
            qgc = subprocess.Popen([str(args.executable),
                                    '--logging:qgc.comms.linkinterface,qgc.vehicle.multivehiclemanager,'
                                    'qgc.vehicle.initialconnectstatemachine,qgc.factsystem.parametermanager'],
                                   stdout=qlog, stderr=qlog, env=environment)

            def pump():
                while not stop.is_set():
                    try:
                        data, source = relay.recvfrom(4096)
                    except socket.timeout:
                        continue
                    cutting = session.cut_until is not None and time.monotonic() < session.cut_until
                    if source == endpoint:
                        session.to_qgc(data)
                        if not cutting:
                            relay.sendto(data, ('127.0.0.1', QGC_PORT))
                    elif source == ('127.0.0.1', QGC_PORT):
                        if not cutting:
                            session.to_drone(data)
                            relay.sendto(data, endpoint)
                    else:
                        session.note('unknown-source', f'{source} {len(data)} bytes')

            worker = threading.Thread(target=pump, daemon=True)
            worker.start()
            while bench.poll() is None and qgc.poll() is None:
                if 'throttle' in session.done and 'cut' not in session.done and session.cut_until is None:
                    session.cut_until = time.monotonic() + CUT_SECONDS
                    session.note('cut', f'link cut for {CUT_SECONDS:g} s while armed: {session.armed}')
                if session.cut_until is not None and time.monotonic() >= session.cut_until:
                    session.cut_until = None
                    session.mark('cut', f'link restored; drone armed: {session.armed}')
                session.write()
                if 'disarmed' in session.done:
                    break
                time.sleep(0.2)
    finally:
        stop.set()
        relay.close()
        for child in (qgc, bench):
            if child is not None and child.poll() is None:
                child.terminate()
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    child.kill()
        enrollment.unlink(missing_ok=True)
    executable = args.executable.read_bytes()
    result = {
        'client': 'QGroundControl v5.0.8',
        'executable_sha256': hashlib.sha256(executable).hexdigest(),
        'policy': 'signed per-boot session (sender 255/190)',
        'steps': {step: session.done.get(step) for step, _ in STEPS},
        'passed': all(step in session.done for step, _ in STEPS),
        'manual_control_frames': session.manual,
        'max_throttle': session.max_throttle,
        'frames': session.counts,
        'events': [list(e) for e in session.events],
        'bench_counters': ((args.output / 'bench.log').read_text().strip().splitlines() or [''])[-1],
        'physical_hardware': False,
    }
    (args.output / 'result.json').write_text(json.dumps(result, indent=1) + '\n')
    session.write()
    print(json.dumps({k: result[k] for k in ('passed', 'steps', 'manual_control_frames', 'max_throttle')}, indent=1))
    return 0 if result['passed'] else 1


def gazebo_session(args):
    host, port = args.gazebo.rsplit(':', 1)
    endpoint = (host, int(port))
    config = args.output.resolve() / 'qgc-config'
    ini = config / 'qgroundcontrol.com/QGroundControl.ini'
    ini.parent.mkdir(parents=True)
    ini.write_text(f'[General]\ntelemetrySave=false\n\n[AutoConnect]\nudpListenPort={QGC_PORT}\n')
    session = Session(args.output)
    stop = threading.Event()
    heard = threading.Event()
    relay = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    relay.bind(('127.0.0.1', RELAY_PORT))
    relay.settimeout(0.1)
    environment = dict(os.environ, XDG_CONFIG_HOME=str(config), XDG_CACHE_HOME=str(config.parent / 'qgc-cache'),
                       XDG_DATA_HOME=str(config.parent / 'qgc-data'), TZ='UTC')
    deadline = time.monotonic() + args.minutes * 60
    qgc = None
    # GCS heartbeat (type 6, autopilot 8, MAVLink 3): the plugin learns its endpoint from accepted frames.
    hello = pack(0, struct.pack('<IBBBBB', 0, 6, 8, 0, 0, 3))
    try:
        qlog = (args.output / 'qgc.log').open('w')
        qgc = subprocess.Popen([str(args.executable),
                                '--logging:qgc.comms.linkinterface,qgc.vehicle.multivehiclemanager,'
                                'qgc.vehicle.initialconnectstatemachine,qgc.factsystem.parametermanager'],
                               stdout=qlog, stderr=qlog, env=environment)

        def pump():
            while not stop.is_set():
                try:
                    data, source = relay.recvfrom(4096)
                except socket.timeout:
                    continue
                cutting = session.cut_until is not None and time.monotonic() < session.cut_until
                if source == endpoint:
                    heard.set()
                    session.to_qgc(data)
                    if not cutting:
                        relay.sendto(data, ('127.0.0.1', QGC_PORT))
                elif source == ('127.0.0.1', QGC_PORT):
                    if not cutting:
                        session.to_drone(data)
                        relay.sendto(data, endpoint)
                else:
                    session.note('unknown-source', f'{source} {len(data)} bytes')

        worker = threading.Thread(target=pump, daemon=True)
        worker.start()
        last_hello = 0.0
        while qgc.poll() is None and time.monotonic() < deadline:
            # Keep greeting until QGC itself talks: Colima's UDP forwarder drops an idle flow.
            if not session.last_qgc and time.monotonic() - last_hello > 1.0:
                relay.sendto(hello, endpoint)
                last_hello = time.monotonic()
            if 'throttle' in session.done and 'cut' not in session.done and session.cut_until is None:
                session.cut_until = time.monotonic() + CUT_SECONDS
                session.note('cut', f'link cut for {CUT_SECONDS:g} s while armed: {session.armed}')
            if session.cut_until is not None and time.monotonic() >= session.cut_until:
                session.cut_until = None
                session.mark('cut', f'link restored; drone armed: {session.armed}')
            session.write()
            if 'disarmed' in session.done:
                break
            time.sleep(0.2)
    finally:
        stop.set()
        relay.close()
        if qgc is not None and qgc.poll() is None:
            qgc.terminate()
            try:
                qgc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                qgc.kill()
    result = {
        'client': 'QGroundControl v5.0.8',
        'executable_sha256': hashlib.sha256(args.executable.read_bytes()).hexdigest(),
        'vehicle': f'Rust owner in Gazebo 11 (flight-gazebo plugin) at {args.gazebo}',
        'policy': 'unsigned loopback development link (plugin policy)',
        'steps': {step: session.done.get(step) for step, _ in STEPS},
        'passed': all(step in session.done for step, _ in STEPS),
        'manual_control_frames': session.manual,
        'max_throttle': session.max_throttle,
        'frames': session.counts,
        'events': [list(e) for e in session.events],
        'physical_hardware': False,
    }
    (args.output / 'result.json').write_text(json.dumps(result, indent=1) + '\n')
    session.write()
    print(json.dumps({k: result[k] for k in ('passed', 'steps', 'manual_control_frames', 'max_throttle')}, indent=1))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
