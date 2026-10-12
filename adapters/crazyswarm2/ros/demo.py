"""Crazyswarm2 coordinator script for the S1 Flix scenarios, using the stock crazyflie_py API.

    python3 demo.py --scenario formation --ros-args -p use_sim_time:=true

Marks (named instants used by check.py) go to $FLIX_SWARM_TRACE/marks.jsonl.
"""
import argparse
import itertools
import json
import os
import pathlib
import sys

from crazyflie_py.crazyflie import CrazyflieServer, TimeHelper
import rclpy
from flix_swarm.swarm import closest_approach

SEPARATION = 0.3
START = {'cf1': (0.0, 0.0), 'cf2': (0.6, 0.0), 'cf3': (1.2, 0.0)}


class Swarm:
    """`crazyflie_py.Crazyswarm` without its joystick, which needs /dev/input in a container."""

    def __init__(self):
        rclpy.init()
        self.allcfs = CrazyflieServer()
        self.timeHelper = TimeHelper(self.allcfs)


class Coordinator:
    def __init__(self):
        self.swarm = Swarm()
        self.time = self.swarm.timeHelper
        self.cfs = {cf.prefix[1:]: cf for cf in self.swarm.allcfs.crazyflies}
        self.planned = {name: (x, y, 0.0) for name, (x, y) in START.items()}
        self.marks = pathlib.Path(os.environ['FLIX_SWARM_TRACE']) / 'marks.jsonl'

    def mark(self, label):
        with self.marks.open('a') as file:
            file.write(json.dumps({'label': label, 'sim_us': int(self.time.time() * 1e6)}) + '\n')

    def go_to(self, goals, duration):
        """Check the whole plan against the separation limit before commanding anyone (SWM-12)."""
        for a, b in itertools.combinations(sorted(goals), 2):
            distance = closest_approach(self.planned[a], goals[a], self.planned[b], goals[b])
            if distance < SEPARATION:
                raise ValueError(f'{a} and {b} would pass within {distance:.3f} m')
        for name, goal in goals.items():
            self.cfs[name].goTo(goal, 0.0, duration)
            self.planned[name] = goal

    def takeoff(self, names, height, duration):
        self.swarm.allcfs.takeoff(targetHeight=height, duration=duration)
        for name in names:
            self.planned[name] = self.planned[name][:2] + (height,)


def formation(c):
    c.time.sleep(1.0)  # owners provision their system IDs
    c.swarm.allcfs.arm(True)
    c.time.sleep(0.5)
    c.takeoff(START, 0.5, 2.0)
    c.time.sleep(4.0)
    c.mark('hover')
    try:
        c.go_to({'cf1': (1.2, 0.0, 0.5), 'cf3': (0.0, 0.0, 0.5)}, 2.0)
        c.mark('crossing-accepted')
    except ValueError:
        c.mark('crossing-rejected')
    c.go_to({'cf1': (0.0, -0.3, 0.6), 'cf2': (0.6, 0.45, 0.6), 'cf3': (1.2, -0.3, 0.6)}, 2.5)
    c.time.sleep(3.5)
    c.mark('triangle')
    c.go_to({'cf1': (0.0, 0.0, 0.8), 'cf2': (0.6, 0.0, 0.8), 'cf3': (1.2, 0.0, 0.8)}, 2.5)
    c.time.sleep(3.5)
    c.mark('line')
    c.swarm.allcfs.land(targetHeight=0.0, duration=2.5)
    c.time.sleep(3.5)
    c.mark('landed')


def stop(c):
    c.time.sleep(1.0)
    c.swarm.allcfs.arm(True)
    c.time.sleep(0.5)
    c.takeoff(START, 0.5, 2.0)
    c.time.sleep(4.0)
    c.mark('hover')
    c.swarm.allcfs.emergency()
    c.time.sleep(1.0)
    c.mark('after-stop')


def partial_arm(c):
    c.time.sleep(1.0)
    c.cfs['cf1'].arm(True)
    c.cfs['cf3'].arm(True)
    c.time.sleep(0.5)
    c.takeoff(('cf1', 'cf3'), 0.5, 2.0)
    c.time.sleep(4.0)
    c.mark('hover')
    c.swarm.allcfs.land(targetHeight=0.0, duration=2.0)
    c.time.sleep(3.0)
    c.mark('landed')


SCENARIOS = {'formation': formation, 'stop': stop, 'partial-arm': partial_arm}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', choices=sorted(SCENARIOS), required=True)
    args, _ = parser.parse_known_args()
    coordinator = Coordinator()
    SCENARIOS[args.scenario](coordinator)
    return 0


if __name__ == '__main__':
    sys.exit(main())
