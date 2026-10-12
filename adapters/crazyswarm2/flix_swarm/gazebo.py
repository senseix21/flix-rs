"""Fly a Flix swarm in Gazebo: one `Ground` per drone over UDP, plugin traces as ground truth.

Runs inside the Gazebo container next to `gzserver` (see `scripts/gazebo-swarm.sh`). Each drone
is a separate Flix model whose Rust plugin binds its own loopback command port and sends its
1 kHz FLXT trace to its own trace port. Traces are observation only: the coordinator reads
position and velocity from them in place of a positioning system (swarm spec S1), and every
command still travels as MAVLink to the drone's own system ID.

    PYTHONPATH=adapters/crazyswarm2:tools python3 -m flix_swarm.gazebo --drones 3 --loop

State for the browser viewer goes to `.local/gazebo-swarm.json` (atomic replace, ~20 Hz).
"""
import argparse
import itertools
import json
import math
import os
import pathlib
import selectors
import socket
import sys
import time

from gazebo_trace import decode

from . import geometry as g
from .planner import Planner
from .swarm import closest_approach
from .vehicle import Config, Ground

ROOT = pathlib.Path(__file__).resolve().parents[3]
COMMAND_BASE = 14600
TRACE_BASE = 14700
SPACING = 0.6
SEPARATION = 0.3
HEIGHT = 0.5
GROUND = 0.0138
# Every step waits at least 1 s after its plan ends; a drone further off than this has not settled.
SETTLED_ERROR_M = 0.05


def layout(count):
    """Start positions: a line along x, SPACING apart (matches scripts/gazebo-swarm-world.py)."""
    return [(i * SPACING, 0.0, 0.0) for i in range(count)]


class Drone:
    def __init__(self, index, start, config):
        self.name = f'flix{index + 1}'
        self.ground = Ground(self.name, 11 + index, config)
        self.planner = Planner(start)
        self.command = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.command.bind(('127.0.0.1', 0))
        self.command.connect(('127.0.0.1', COMMAND_BASE + index))
        self.command.setblocking(False)
        self.trace = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.trace.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self.trace.bind(('127.0.0.1', TRACE_BASE + index))
        self.trace.setblocking(False)
        self.truth = None
        self.sent = self.unsent = 0

    def send(self, frame):
        try:
            self.command.send(frame)
            self.sent += 1
        except OSError:
            # Plugin not bound yet (world still loading): the frame is lost, like radio loss.
            self.unsent += 1

    def view(self):
        t = self.truth or {}
        target = None
        if self.ground.target is not None and t:
            target = [round(v, 4) for v in self.planner.current(t['sim_us'])[0]]
        return {
            'name': self.name,
            'system_id': self.ground.system_id,
            'phase': self.ground.phase,
            'state': t.get('state'),
            'reason': t.get('reason'),
            'position_m': t.get('position_world_m'),
            'target_m': target,
            'motors': t.get('motors'),
            'refusal': self.ground.refusal,
        }


class Coordinator:
    def __init__(self, count, loop, output, onboard=False):
        self.config = Config(onboard=onboard)
        self.drones = [Drone(i, p, self.config) for i, p in enumerate(layout(count))]
        self.loop = loop
        self.output = pathlib.Path(output)
        self.selector = selectors.DefaultSelector()
        for d in self.drones:
            self.selector.register(d.trace, selectors.EVENT_READ, (d, 'trace'))
            self.selector.register(d.command, selectors.EVENT_READ, (d, 'reply'))
        self.step = None
        self.wait_until = 0
        self.cycle = 0
        self.message = 'Waiting for Gazebo'
        self.last_write = 0.0
        self.failures = []

    def now(self):
        times = [d.truth['sim_us'] for d in self.drones if d.truth]
        return min(times) if times else None

    def plan(self, goals, duration):
        """Command every drone at once after checking the whole plan (SWM-12)."""
        now = self.now()
        flying = {d.name: d for d in self.drones if d.ground.phase == 'armed'}
        moves = {n: (d.planner.current(now)[0], goals[n]) for n, d in flying.items() if n in goals}
        for a, b in itertools.combinations(sorted(moves), 2):
            if closest_approach(*moves[a], *moves[b]) < SEPARATION:
                raise ValueError(f'{a} and {b} would pass too close')
        for n, (_, goal) in moves.items():
            flying[n].planner.go_to(goal, duration, now)

    def choreography(self):
        """Yield (message, seconds to wait) steps; runs once, or forever with --loop."""
        count = len(self.drones)
        starts = layout(count)
        while True:
            self.cycle += 1
            yield 'Provisioning system IDs', None
            for d in self.drones:
                d.ground.request_arm(self.now())
            yield 'Arming each drone', 1.0
            refused = [f'{d.name}: {d.ground.refusal}' for d in self.drones if d.ground.phase != 'armed']
            self.failures += [f'not armed: {r}' for r in refused]
            for d in self.drones:
                if d.ground.phase == 'armed':
                    d.ground.target = d.planner.current
            self.plan({d.name: (s[0], s[1], HEIGHT) for d, s in zip(self.drones, starts)}, 2.0)
            yield 'Takeoff' + (f' (not armed: {", ".join(refused)})' if refused else ''), 3.5
            middle = (count - 1) * SPACING / 2
            wave = {d.name: (s[0], 0.35 * (1 if i % 2 else -1), HEIGHT + 0.15 * (i % 2))
                    for i, (d, s) in enumerate(zip(self.drones, starts))}
            self.plan(wave, 2.5)
            yield 'Formation: zigzag', 3.5
            radius = max(SPACING, middle)
            ring = {d.name: (middle + radius * _cos(i, count), radius * _sin(i, count), HEIGHT + 0.2)
                    for i, d in enumerate(self.drones)}
            self.plan(ring, 3.0)
            yield 'Formation: ring', 4.0
            # A full turn, one place at a time, so every drone is back at its own ring slot and
            # the return to the line retraces an already checked move.
            for turn in range(1, count + 1):
                self.plan({d.name: ring[self.drones[(i + turn) % count].name]
                           for i, d in enumerate(self.drones)}, 2.5)
                yield f'Formation: ring, turn {turn} of {count}', 4.0
            self.plan({d.name: (s[0], s[1], HEIGHT) for d, s in zip(self.drones, starts)}, 3.0)
            yield 'Back to the line', 4.0
            self.plan({d.name: (s[0], s[1], GROUND) for d, s in zip(self.drones, starts)}, 2.5)
            yield 'Landing', 3.0
            for d in self.drones:
                if d.ground.phase == 'armed':
                    d.ground.request_stop(self.now())
            yield 'Landed and stopped', 3.0
            for d in self.drones:
                t = d.truth or {}
                if d.ground.phase != 'stopped' or t.get('state') == 3 or any(t.get('motors') or [1]) \
                        or t['position_world_m'][2] > GROUND + 0.02:
                    self.failures.append(f'{d.name} not landed and stopped ({d.ground.phase})')
            if not self.loop:
                return
            for d in self.drones:
                d.ground.release()

    def advance(self):
        now = self.now()
        if now is None:
            return True
        if self.step is None:
            self.step = self.choreography()
            self.wait_until = None
        if self.wait_until is None:
            if not all(d.ground.phase != 'provisioning' for d in self.drones):
                return True
        elif now < self.wait_until:
            return True
        try:
            self.message, seconds = next(self.step)
        except StopIteration:
            return False
        except ValueError as rejected:
            self.message = f'Plan rejected: {rejected}'
            self.failures.append(self.message)
            print(f'[{now / 1e6:8.2f} s] {self.message}', flush=True)
            return False
        print(f'[{now / 1e6:8.2f} s] {self.message}  (errors: {self._errors(now)})', flush=True)
        self.wait_until = None if seconds is None else now + int(seconds * 1e6)
        return True

    def _errors(self, now):
        """Distance from each armed drone to its planned point, at the end of the previous step."""
        parts = []
        for d in self.drones:
            if d.ground.phase == 'armed' and d.ground.target is not None and d.truth:
                error = g.distance(d.truth['position_world_m'], d.planner.current(now)[0])
                if error > SETTLED_ERROR_M:
                    self.failures.append(f'{d.name} {error * 1000:.0f} mm from plan before "{self.message}"')
                parts.append(f'{d.name} {error * 1000:.0f} mm')
            else:
                parts.append(f'{d.name} {d.ground.phase}')
        return ', '.join(parts)

    def run(self):
        running = True
        while running:
            for key, _ in self.selector.select(timeout=0.05):
                drone, kind = key.data
                self._drain(drone, kind)
            running = self.advance()
            self._write()
        self._write(force=True)
        for failure in self.failures:
            print(f'FAIL {failure}', flush=True)
        print('pass gazebo swarm' if not self.failures else 'FAIL gazebo swarm', flush=True)
        return 1 if self.failures else 0

    def _drain(self, drone, kind):
        sock = drone.trace if kind == 'trace' else drone.command
        while True:
            try:
                data = sock.recv(512)
            except (BlockingIOError, ConnectionRefusedError):
                return
            if kind == 'reply':
                if drone.truth:
                    drone.ground.receive(drone.truth['sim_us'], [data])
                continue
            try:
                record = decode(data)
            except ValueError:
                continue
            if record['kind'] != 'tick' or not record['numeric_valid']:
                continue
            drone.truth = record
            now = record['sim_us']
            frame = drone.ground.outgoing(now, tuple(record['position_world_m']), tuple(record['velocity_world_m_s']))
            if frame is not None:
                drone.send(frame)

    def _write(self, force=False):
        if not force and time.monotonic() - self.last_write < 0.05:
            return
        self.last_write = time.monotonic()
        now = self.now()
        state = {
            'written_unix': time.time(),
            'sim_us': now,
            'cycle': self.cycle,
            'message': self.message,
            'position_loop': 'drone' if self.config.onboard else 'ground',
            'drones': [d.view() for d in self.drones],
        }
        try:
            self.output.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.output.with_suffix('.tmp')
            temporary.write_text(json.dumps(state, allow_nan=False))
            os.replace(temporary, self.output)
        except (OSError, ValueError):
            pass


def _cos(i, n):
    return math.cos(2 * math.pi * i / n + math.pi)


def _sin(i, n):
    return math.sin(2 * math.pi * i / n + math.pi)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--drones', type=int, default=3, choices=range(2, 9))
    parser.add_argument('--loop', action='store_true', help='repeat the flight until interrupted')
    parser.add_argument('--output', default=str(ROOT / '.local/gazebo-swarm.json'))
    parser.add_argument('--onboard', action='store_true',
                        help='S2: position loop in flight-core, fed by ATT_POS_MOCAP from traces')
    args = parser.parse_args()
    return Coordinator(args.drones, args.loop, args.output, args.onboard).run()


if __name__ == '__main__':
    sys.exit(main())
