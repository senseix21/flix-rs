"""Foreground development session; only containers labeled for this checkout are managed."""
import argparse
import json
import pathlib
import platform
import signal
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
import webbrowser

ROOT = pathlib.Path(__file__).resolve().parents[1]
NAME = 'flix-rust-gazebo-browser'
LABEL = 'dev.flix.browser-root'


def docker(*args, check=True, capture=False):
    return subprocess.run(['docker', *args], check=check, text=True,
                          stdout=subprocess.PIPE if capture else None, timeout=60)


def inspect():
    result = docker('container', 'inspect', NAME, check=False, capture=True)
    if result.returncode:
        return None
    item = json.loads(result.stdout)[0]
    if item['Config'].get('Labels', {}).get(LABEL) != str(ROOT):
        raise RuntimeError(f'{NAME} belongs to another session; refusing to manage it')
    return item


def build(image, arm):
    subprocess.run(['docker', 'build', '-t', image, '-f',
                    'adapters/gazebo/Dockerfile.arm64' if arm else 'adapters/gazebo/Dockerfile',
                    '.'], cwd=ROOT, check=True)


def poses_ready(snapshot, drones):
    """Require every requested model before opening the swarm viewer."""
    if drones is None:
        return snapshot.get('pose') is not None
    poses = snapshot.get('poses', {})
    return all(poses.get(f'flix{i}') is not None for i in range(1, drones + 1))


def start(args, image, arm):
    legacy = pathlib.Path(args.legacy).expanduser().resolve() if args.legacy else next(
        (p for p in (ROOT/'.local/legacy-git', ROOT.parent/'flix')
         if (p/'gazebo/models/flix/flix.sdf').is_file()), None)
    if legacy is None or not (legacy/'gazebo/models/flix/flix.sdf').is_file():
        raise RuntimeError('Missing legacy plant assets; set SIM_LEGACY to the Flix checkout (see docs/legacy-source.md)')
    # Reserve no unrelated listener: fail before starting the container if the port is occupied.
    with socket.socket() as probe:
        # Match HTTPServer's reuse policy so a just-stopped viewer's TIME_WAIT
        # sockets do not block switching worlds; live listeners still fail.
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        probe.bind(('127.0.0.1', args.port))
    old = inspect()
    if old:
        if old['State']['Running']:
            raise RuntimeError('A browser simulation is already running; use make sim-stop first')
        docker('rm', NAME)
    if docker('image', 'inspect', image, check=False, capture=True).returncode:
        build(image, arm)
    viewer = None
    try:
        docker('run', '--init', '-d', '--name', NAME, '--label', f'{LABEL}={ROOT}',
               '-p', '127.0.0.1:14550:14551/udp',
               '-v', f'{ROOT}:/work', '-v', f'{legacy}:/legacy:ro', '-w', '/work',
               image, 'sh', *(['scripts/gazebo-swarm.sh', str(args.drones)]
                             if args.swarm else ['scripts/gazebo-run.sh']))
        viewer = subprocess.Popen([sys.executable, str(ROOT/'tools/gazebo-viewer.py'),
                                   '--container', NAME, '--port', str(args.port),
                                   *(['--swarm'] if args.swarm else [])], cwd=ROOT)
        url = f'http://127.0.0.1:{args.port}'
        deadline = time.monotonic() + 600
        print('Waiting for the Gazebo pose (first build can take several minutes).', flush=True)
        print('Use make sim-logs in another terminal for build output.', flush=True)
        while True:
            if viewer.poll() is not None:
                raise RuntimeError('Browser viewer exited before the session ended')
            state = inspect()
            if not state or not state['State']['Running']:
                raise RuntimeError('Gazebo stopped; inspect make sim-logs')
            try:
                with urllib.request.urlopen(url+'/api/state', timeout=2) as response:
                    ready = poses_ready(json.load(response), args.drones if args.swarm else None)
            except (OSError, ValueError, urllib.error.URLError):
                ready = False
            if ready:
                break
            if time.monotonic() >= deadline:
                raise RuntimeError('Gazebo did not produce a pose within 10 minutes; inspect make sim-logs')
            time.sleep(2)
        print(f'Ready: {url} — Ctrl+C stops the viewer and world.', flush=True)
        if not webbrowser.open(url):
            print(f'Open {url} in your browser.', flush=True)
        while viewer.poll() is None:
            state = inspect()
            if not state or not state['State']['Running']:
                return
            time.sleep(2)
        raise RuntimeError('Browser viewer exited unexpectedly')
    finally:
        if viewer and viewer.poll() is None:
            viewer.terminate()
            try:
                viewer.wait(timeout=12)
            except subprocess.TimeoutExpired:
                viewer.kill()
                viewer.wait()
        state = inspect()
        if state and state['State']['Running']:
            docker('stop', '-t', '10', NAME)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['start', 'stop', 'status', 'logs', 'build'])
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--image', default='')
    parser.add_argument('--legacy', default='')
    parser.add_argument('--swarm', action='store_true', help='run the repeating autonomous swarm demo (view only)')
    parser.add_argument('--drones', type=int, default=3)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error('port must be between 1 and 65535')
    if not 1 <= args.drones <= 8:
        parser.error('drones must be between 1 and 8')
    arm = platform.machine().lower() in ('arm64', 'aarch64')
    image = args.image or ('flix-rust-gazebo:arm64' if arm else 'flix-rust-gazebo:local')
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    try:
        docker('info', capture=True)
        if args.action == 'build':
            build(image, arm)
        elif args.action == 'start':
            start(args, image, arm)
        else:
            state = inspect()
            if not state:
                print('No managed browser simulation.')
            elif args.action == 'status':
                print(state['State']['Status'])
            elif args.action == 'stop':
                docker('stop', '-t', '10', NAME)
            else:
                subprocess.run(['docker', 'logs', '--tail', '80', '-f', NAME], check=True)
    except KeyboardInterrupt:
        pass
    except (OSError, RuntimeError, subprocess.SubprocessError) as error:
        print(f'Simulation: {error}', file=sys.stderr)
        print('Docker must be running. On Apple Silicon with Colima: colima start --port-forwarder grpc', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
