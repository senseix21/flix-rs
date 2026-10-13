"""Crazyswarm2 SIL wrapper that flies a real Flix owner instead of a Crazyflie controller.

Installed into `crazyflie_sim` by `adapters/crazyswarm2/Dockerfile`. The Crazyflie firmware
planner still turns takeoff/goTo/land/trajectory requests into setpoints; a Flix `Vehicle`
turns those setpoints into MAVLink AUTO attitude requests for the Flix owner (swarm spec S1).
Motor outputs leave through the owner's final gate in Flix order RL, RR, FR, FL and are
consumed only by the `flix` backend.
"""
import json
import math
import os
import pathlib

from flix_swarm.plant import HALF_SIZE
from flix_swarm.vehicle import Config, Vehicle

from . import sim_data_types
from .crazyflie_sil import CrazyflieSIL

TICK_US = 1000
TRACE_PERIOD_US = 20_000
# After a land plan ends, stop the motors as the Crazyflie high-level commander does.
LAND_SETTLE_US = 500_000


class FlixSIL(CrazyflieSIL):

    def __init__(self, name, initialPosition, time_func, system_id, seed, logger, trace_dir=None):
        super().__init__(name, initialPosition, 'none', time_func)
        # FLIX_POSITION_LOOP=drone: the position loop runs in flight-core (swarm S2), fed by
        # ATT_POS_MOCAP from simulator truth. Otherwise the ground runs it (S1).
        onboard = os.environ.get('FLIX_POSITION_LOOP', 'ground') == 'drone'
        self.vehicle = Vehicle(name, system_id, seed, Config(onboard=onboard))
        self.logger = logger
        x, y = float(initialPosition[0]), float(initialPosition[1])
        self.flix_state = ((x, y, HALF_SIZE[2]), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        self.land_until_us = None
        self.trace = None
        self.reported = 0
        if trace_dir:
            path = pathlib.Path(trace_dir)
            path.mkdir(parents=True, exist_ok=True)
            self.trace = (path / f'{name}.jsonl').open('w')

    def now_us(self):
        return int(round(self.time_func() * 1e6))

    def _flying(self, request):
        if self.vehicle.phase == 'armed':
            return True
        self.logger.warning(f'[{self.name}] {request} ignored: {self.vehicle.phase}, not armed')
        return False

    def takeoff(self, targetHeight, duration, groupMask=0):
        if self._isGroup(groupMask) and self._flying('takeoff'):
            super().takeoff(targetHeight, duration, groupMask)
            self.vehicle.target = self._target

    def land(self, targetHeight, duration, groupMask=0):
        if self._isGroup(groupMask) and self._flying('land'):
            super().land(targetHeight, duration, groupMask)
            self.land_until_us = self.now_us() + int(duration * 1e6) + LAND_SETTLE_US

    def goTo(self, goal, yaw, duration, relative=False, groupMask=0):
        if self._isGroup(groupMask) and self._flying('go_to'):
            super().goTo(goal, yaw, duration, relative, groupMask)

    def startTrajectory(self, trajectoryId, timescale=1.0, reverse=False, relative=True, groupMask=0):
        if self._isGroup(groupMask) and self._flying('start_trajectory'):
            super().startTrajectory(trajectoryId, timescale, reverse, relative, groupMask)

    def cmdFullState(self, pos, vel, acc, yaw, omega):
        self.logger.warning(f'[{self.name}] cmd_full_state is not supported for Flix in S1')

    def arm(self, arm):
        """Explicit arm or disarm request for this drone (SWM-02)."""
        if not arm:
            self.emergency()
        elif not self.vehicle.request_arm(self.now_us()):
            self.logger.warning(f'[{self.name}] arm ignored: {self.vehicle.phase}')

    def emergency(self):
        self.vehicle.request_stop(self.now_us())
        self.mode = CrazyflieSIL.MODE_IDLE

    def _target(self, now_us):
        s = self.setpoint
        return (
            (s.position.x, s.position.y, s.position.z),
            (s.velocity.x, s.velocity.y, s.velocity.z),
            (s.acceleration.x, s.acceleration.y, s.acceleration.z),
            math.radians(s.attitude.yaw),
        )

    def setState(self, state: sim_data_types.State):
        super().setState(state)
        self.flix_state = tuple(tuple(float(v) for v in part)
                                for part in (state.pos, state.vel, state.quat, state.omega))

    def executeController(self):
        now = self.now_us()
        if self.land_until_us is not None and now >= self.land_until_us:
            self.land_until_us = None
            if self.vehicle.phase == 'armed':
                self.vehicle.request_stop(now)
        motors = self.vehicle.control(now, TICK_US, *self.flix_state)
        self._record(now)
        return sim_data_types.Action(list(motors))

    def _record(self, now):
        if self.trace is None:
            return
        events = self.vehicle.events[self.reported:]
        if now % TRACE_PERIOD_US and not events:
            return
        self.reported = len(self.vehicle.events)
        position = self.flix_state[0]
        record = {
            'sim_us': now,
            'name': self.name,
            'system_id': self.vehicle.system_id,
            'phase': self.vehicle.phase,
            'state': self.vehicle.state,
            'reason': self.vehicle.reason,
            'position_m': [round(v, 4) for v in position],
            'target_m': None if self.vehicle.target is None
            else [round(v, 4) for v in self._target(now)[0]],
            'motors': [round(v, 4) for v in self.vehicle.motors],
            'events': [list(e) for e in events],
            'position_loop': 'drone' if self.vehicle.config.onboard else 'ground',
            'stop_latency_us': None if self.vehicle.stopped_us is None
            else self.vehicle.stopped_us - self.vehicle.stop_requested_us,
        }
        self.trace.write(json.dumps(record, separators=(',', ':')) + '\n')
        self.trace.flush()

    def shutdown(self):
        if self.trace is not None:
            self.trace.close()
        self.vehicle.close()
