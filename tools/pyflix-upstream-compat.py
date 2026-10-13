#!/usr/bin/env python3
"""R2-04: run the pinned upstream pyflix client against the Rust bench and record what works.

Upstream pyflix is imported from the pinned legacy checkout (`1f5b268`, tools/pyflix) and runs
on a pinned pymavlink in an isolated virtual environment:

    python3 -m venv .local/pyflix-venv && .local/pyflix-venv/bin/pip install pymavlink==2.4.47
    cargo build --release -p flight-replay --bin bench
    .local/pyflix-venv/bin/python tools/pyflix-upstream-compat.py --legacy .local/legacy-git \\
        --output .local/qualification/r2/pyflix

The bench uses the unsigned host-only BenchUsb policy (pyflix has no signing support), so this
covers the wired development path only. Each step's result and the bench's own admission
counters are written to result.json. The tool reports observed behaviour; it does not decide
whether a difference is intended.
"""
import argparse
import json
import os
import pathlib
import socket
import subprocess
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]
BENCH_PORT = 14560


def free(port):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        try:
            s.bind(('127.0.0.1', port))
            return True
        except OSError:
            return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--legacy', type=pathlib.Path, required=True, help='pinned legacy flix checkout')
    parser.add_argument('--output', type=pathlib.Path, required=True)
    parser.add_argument('--sender-component', type=int, default=None,
                        help='override pyflix source component (default: pymavlink default)')
    args = parser.parse_args()
    revision = subprocess.run(['git', '-C', str(args.legacy), 'rev-parse', 'HEAD'],
                              capture_output=True, text=True, check=True).stdout.strip()
    sys.path.insert(0, str(args.legacy / 'tools'))
    import pymavlink
    from pymavlink import mavutil
    from pyflix import flix as upstream
    if not free(14550):
        raise SystemExit('UDP 14550 is busy; pyflix listens there')
    args.output.mkdir(parents=True, exist_ok=True)
    environment = dict(os.environ, FLIX_BENCH_ENDPOINT='127.0.0.1:14550', FLIX_BENCH_TICKS='25000')
    log = (args.output / 'bench.log').open('w')
    bench = subprocess.Popen([str(ROOT / 'target/release/bench'), str(BENCH_PORT)], stdout=subprocess.PIPE,
                             stderr=log, text=True, env=environment)
    bench.stdout.readline()
    steps = []

    def step(name, call):
        started = time.monotonic()
        try:
            value = call()
            steps.append(dict(step=name, ok=True, value=value, seconds=round(time.monotonic() - started, 3)))
        except Exception as error:  # record every client-visible failure verbatim
            steps.append(dict(step=name, ok=False, error=f'{type(error).__name__}: {error}',
                              seconds=round(time.monotonic() - started, 3)))

    drone = None
    try:
        # pyflix waits passively for the vehicle's heartbeat on UDP 14550.
        step('connect (wait for HEARTBEAT)', lambda: bool(globals().__setitem__('_d', upstream.Flix(timeout=5)) or True))
        drone = globals().get('_d')
        if drone is not None:
            if args.sender_component is not None:
                drone.mavlink.srcComponent = args.sender_component
            sender = (drone.mavlink.srcSystem, drone.mavlink.srcComponent)
            step('heartbeat mode/armed decode', lambda: dict(mode=drone.mode, armed=drone.armed))
            step('get_param MAV_SYS_ID', lambda: drone.get_param('MAV_SYS_ID'))
            step('set_param CTL_ATT_MAX (stopped)', lambda: drone.set_param('CTL_ATT_MAX', 0.5))
            step('cli status', lambda: drone.cli('status'))
            step('set_mode STAB', lambda: drone.set_mode('STAB'))
            step('set_controls throttle 0', lambda: drone.set_controls(0, 0, 0, 0))
            step('set_armed True', lambda: drone.set_armed(True))
            step('armed reported', lambda: (time.sleep(0.6), drone.armed)[1])
            step('set_controls throttle 0.4', lambda: drone.set_controls(0, 0, 0, 0.4))
            step('set_armed False', lambda: drone.set_armed(False))
            step('disarmed reported', lambda: (time.sleep(0.6), drone.armed)[1])
            step('cli maintenance', lambda: drone.cli('maintenance'))
            step('set_param CTL_ATT_MAX (maintenance)', lambda: drone.set_param('CTL_ATT_MAX', 0.5))
            step('get_param CTL_ATT_MAX', lambda: drone.get_param('CTL_ATT_MAX'))
            step('cli exit', lambda: drone.cli('exit'))
            step('set_mode RAW (diagnostic, intentionally unsupported)', lambda: drone.set_mode('RAW'))
            step('set_mode AUTO', lambda: drone.set_mode('AUTO'))
            step('set_attitude thrust 0', lambda: drone.set_attitude([0, 0, 0], 0.0))
            step('set_motors (diagnostic)', lambda: drone.set_motors([0.1, 0, 0, 0]))
            step('telemetry seen', lambda: dict(attitude=drone.attitude is not None and len(drone.attitude) == 4,
                                                gyro=bool(getattr(drone, 'gyro', None)),
                                                motors=bool(getattr(drone, 'motors', None))))
    finally:
        # Let the bench finish its fixed run so it prints its admission counters.
        try:
            bench.wait(timeout=60)
        except subprocess.TimeoutExpired:
            bench.kill()
        log.close()
    counters = (args.output / 'bench.log').read_text().strip().splitlines()[-1:] or ['']
    result = dict(
        client='upstream pyflix (flix tools/pyflix)',
        legacy_revision=revision,
        pymavlink=getattr(pymavlink, '__version__', None) or subprocess.run(
            [sys.executable, '-m', 'pip', 'show', 'pymavlink'], capture_output=True, text=True).stdout.split('Version: ')[-1].split()[0],
        python=sys.version.split()[0],
        bench_policy='BenchUsb system 255 component 190 (unsigned, host-only)',
        pyflix_sender=list(sender) if drone is not None else None,
        steps=steps,
        bench_counters=counters[0],
    )
    (args.output / 'result.json').write_text(json.dumps(result, indent=1) + '\n')
    for s in steps:
        print(('ok  ' if s['ok'] else 'FAIL'), s['step'], '->', s.get('value', s.get('error')))
    print(result['bench_counters'])
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
