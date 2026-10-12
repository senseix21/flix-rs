"""Swarm runner without ROS: N vehicles, N plants, one 1 kHz clock, Crazyswarm2-like commands.

This is the fast path used by tests and CI scenarios. The Crazyswarm2 integration uses the same
`Vehicle` with the Crazyswarm2 server, firmware planner and a `flix` physics backend.
"""
import itertools

from . import geometry as g
from .plant import HALF_SIZE, Plant
from .planner import Planner
from .vehicle import Config, Vehicle

TICK_US = 1000
GROUND = HALF_SIZE[2]


class PlanRejected(ValueError):
    """A commanded plan would bring two drones closer than the separation limit (SWM-12)."""


def closest_approach(a0, a1, b0, b1):
    """Minimum distance between two drones moving on synchronized straight segments."""
    r0 = g.sub(a0, b0)
    dr = g.sub(g.sub(a1, b1), r0)
    span = g.dot(dr, dr)
    s = 0.0 if span == 0 else max(0.0, min(1.0, -g.dot(r0, dr) / span))
    return g.norm(g.add(r0, g.scale(dr, s)))


class Drone:
    def __init__(self, name, system_id, position, seed, config):
        self.plant = Plant(position)
        self.vehicle = Vehicle(name, system_id, seed, config)
        self.planner = Planner(self.plant.position)
        self.vehicle.target = None

    @property
    def name(self):
        return self.vehicle.name


class Swarm:
    def __init__(self, positions, seed=1, config=None, separation=0.3, first_id=11):
        self.config = config or Config()
        self.separation = separation
        self.now_us = 0
        self.drones = []
        try:
            for i, position in enumerate(positions):
                self.drones.append(Drone(f'cf{i + 1}', first_id + i, position, seed + i, self.config))
        except Exception:
            self.close()
            raise
        for a, b in itertools.combinations(self.drones, 2):
            if g.distance(a.plant.position, b.plant.position) < separation:
                self.close()
                raise PlanRejected(f'{a.name} and {b.name} start closer than {separation} m')
        self.min_separation = None
        self.trace = []

    def close(self):
        for drone in self.drones:
            drone.vehicle.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def by_name(self, name):
        return next(d for d in self.drones if d.name == name)

    def step(self):
        self.now_us += TICK_US
        for drone in self.drones:
            p = drone.plant
            motors = drone.vehicle.control(self.now_us, TICK_US, p.position, p.velocity, p.attitude, p.rates)
            p.step(motors, TICK_US * 1e-6)
        airborne = [d for d in self.drones if d.plant.position[2] > GROUND + 0.05]
        for a, b in itertools.combinations(airborne, 2):
            distance = g.distance(a.plant.position, b.plant.position)
            if self.min_separation is None or distance < self.min_separation[0]:
                self.min_separation = (distance, a.name, b.name, self.now_us)
        if self.now_us % 20_000 == 0:
            self.trace.append(self.snapshot())

    def snapshot(self):
        return {
            'sim_us': self.now_us,
            'drones': {
                d.name: {
                    'phase': d.vehicle.phase,
                    'state': d.vehicle.state,
                    'reason': d.vehicle.reason,
                    'position_m': [round(v, 4) for v in d.plant.position],
                    'target_m': [round(v, 4) for v in d.planner.current(self.now_us)[0]],
                    'tilt_deg': round(g.tilt(d.plant.attitude) * 57.29578, 2),
                    'motors': [round(v, 4) for v in d.vehicle.motors],
                }
                for d in self.drones
            },
        }

    def run(self, seconds, until=None):
        end = self.now_us + int(seconds * 1e6)
        while self.now_us < end:
            self.step()
            if until is not None and until():
                return True
        return until is None

    def provision(self, timeout=1.0):
        if not self.run(timeout, lambda: all(d.vehicle.phase == 'standby' for d in self.drones)):
            raise RuntimeError('provisioning timed out: ' + ', '.join(
                f'{d.name}={d.vehicle.phase}' for d in self.drones))

    def arm(self, names=None, timeout=1.0):
        """Request arming per drone; return {name: None or refusal reason}."""
        chosen = [d for d in self.drones if names is None or d.name in names]
        for drone in chosen:
            drone.vehicle.request_arm(self.now_us)
        self.run(timeout, lambda: all(d.vehicle.phase != 'arming' for d in chosen))
        self.run(0.01)  # let a denial notice that follows its ACK arrive
        result = {}
        for drone in chosen:
            if drone.vehicle.phase == 'armed':
                drone.vehicle.target = self._target(drone)
                result[drone.name] = None
            else:
                result[drone.name] = drone.vehicle.refusal or drone.vehicle.phase
        return result

    def _target(self, drone):
        return lambda now_us: drone.planner.current(now_us)

    def flying(self):
        return [d for d in self.drones if d.vehicle.phase == 'armed']

    def takeoff(self, height, duration):
        for d in self.flying():
            x, y, _ = d.planner.current(self.now_us)[0]
            d.planner.go_to((x, y, height), duration, self.now_us)

    def go_to(self, goals, duration):
        """goals: {name: position}. Checks the whole plan before commanding anyone (SWM-12)."""
        moves = {d.name: (d.planner.current(self.now_us)[0], tuple(goals.get(d.name, d.planner.position)))
                 for d in self.flying()}
        for a, b in itertools.combinations(sorted(moves), 2):
            distance = closest_approach(*moves[a], *moves[b])
            if distance < self.separation:
                raise PlanRejected(f'{a} and {b} would pass within {distance:.3f} m')
        for d in self.flying():
            if d.name in goals:
                d.planner.go_to(goals[d.name], duration, self.now_us)

    def land(self, duration):
        for d in self.flying():
            x, y, _ = d.planner.current(self.now_us)[0]
            d.planner.go_to((x, y, GROUND), duration, self.now_us)
        self.run(duration + 0.5)
        for d in self.flying():
            d.vehicle.request_stop(self.now_us)
        self.run(0.05)

    def stop_all(self):
        """Swarm-wide stop: every drone gets its own stop frame this tick."""
        for d in self.drones:
            d.vehicle.request_stop(self.now_us)

    def report(self):
        return {
            'sim_seconds': self.now_us / 1e6,
            'min_separation': None if self.min_separation is None else {
                'distance_m': round(self.min_separation[0], 4),
                'pair': list(self.min_separation[1:3]),
                'sim_us': self.min_separation[3],
            },
            'drones': {
                d.name: {
                    'system_id': d.vehicle.system_id,
                    'phase': d.vehicle.phase,
                    'refusal': d.vehicle.refusal,
                    'arm_latency_us': d.vehicle.arm_latency_us,
                    'stop_latency_us': None if d.vehicle.stopped_us is None
                    else d.vehicle.stopped_us - d.vehicle.stop_requested_us,
                    'frames_sent': d.vehicle.link.sent,
                    'frames_dropped': d.vehicle.link.dropped,
                    'frames_lost': d.vehicle.link.lost,
                    'frames_rejected': d.vehicle.rejected,
                    'events': [list(e) for e in d.vehicle.events],
                }
                for d in self.drones
            },
        }
