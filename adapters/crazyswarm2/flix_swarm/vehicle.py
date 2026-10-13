"""One simulated Flix drone as the ground sees it: owner, IMU, link and ground position loop.

Plant physics lives elsewhere (`plant.Plant` here, the `flix` backend under Crazyswarm2), so the
same vehicle runs in both. All ground-to-drone traffic is MAVLink through a bounded link.
"""
import collections
import math
import random

from . import geometry as g
from .imu import Imu
from .owner import ARMED, STATES, REASONS, Decoder, Messages, Owner, parse_replies
from .position import PositionController

LEVEL = (1.0, 0.0, 0.0, 0.0)
# Default MAV_RATE_SLOW is 2 Hz; three missed heartbeats mean the drone is out of contact.
HEARTBEAT_TIMEOUT_US = 1_500_000


class Config:
    """`onboard`: the drone runs the position loop (swarm S2); the ground sends position samples
    (ATT_POS_MOCAP) and position targets. Otherwise the ground runs it and sends attitudes (S1)."""

    def __init__(self, lease_s=0.3, setpoint_period_us=20_000, link_capacity=64,
                 link_latency_us=0, imu_noise=True, onboard=False, position_period_us=20_000,
                 position_noise_m=0.0, seed=0, heading=True):
        self.onboard = onboard
        # Send the measured heading with each position sample (S2 heading fusion).
        self.heading = heading
        self.position_period_us = position_period_us
        self.position_noise_m = position_noise_m
        self.seed = seed
        self.lease_s = lease_s
        self.setpoint_period_us = setpoint_period_us
        self.link_capacity = link_capacity
        self.link_latency_us = link_latency_us
        self.imu_noise = imu_noise


class Link:
    """Ground to drone, plus the reply path while up.

    Bounded: a frame arriving at a full queue is dropped and counted. `cut()` models a full
    outage: queued frames are lost and replies stop reaching the ground.
    """

    def __init__(self, capacity, latency_us):
        self.queue = collections.deque()
        self.capacity = capacity
        self.latency_us = latency_us
        self.up = True
        self.sent = self.dropped = self.lost = 0

    def send(self, frame, now_us):
        if not self.up:
            self.lost += 1
        elif len(self.queue) >= self.capacity:
            self.dropped += 1
        else:
            self.queue.append((now_us + self.latency_us, frame))
            self.sent += 1

    def cut(self):
        self.up = False
        self.lost += len(self.queue)
        self.queue.clear()

    def restore(self):
        self.up = True

    def due(self, now_us):
        while self.queue and self.queue[0][0] <= now_us:
            yield self.queue.popleft()[1]


class Ground:
    """What the ground knows about one drone, independent of how frames travel.

    Phases: provisioning, standby, arming, armed, refused, lost, stopped. Feed decoded owner
    replies to `receive` and send whatever `outgoing` returns (at most one frame per call).
    """

    def __init__(self, name, system_id, config=None):
        self.config = config or Config()
        self.name = name
        self.system_id = system_id
        self.messages = Messages()
        self.decoder = Decoder()
        self.controller = PositionController()
        self.pending = collections.deque()
        self.phase = 'provisioning'
        self.events = []
        self.target = None
        self.next_setpoint_us = 0
        self.next_position_us = 0
        # Coordinator switch for dropout tests: stop sending position samples to this drone.
        self.positioning = True
        self.noise = random.Random(self.config.seed * 7919 + system_id)
        self.arm_requested_us = None
        self.arm_latency_us = None
        self.stop_requested_us = None
        self.stopped_us = None
        self.refusal = None
        self.last_heartbeat_us = 0
        # The owner starts as system 1; set the lease first, then move to this drone's ID.
        # `exit` goes to the new ID only after both values are echoed, and provisioning ends on
        # its reply: the new ID takes effect a tick later, so an `exit` delivered with the
        # PARAM_SET in one owner tick would be dropped and leave the drone in maintenance.
        lease = self.config.lease_s
        self._expected = {'SF_RC_LOSS_TIME': lease, 'MAV_SYS_ID': float(system_id)}
        self.pending.extend([
            self.messages.console(1, 'maintenance'),
            self.messages.parameter(1, 'SF_RC_LOSS_TIME', lease),
            self.messages.parameter(1, 'MAV_SYS_ID', float(system_id)),
        ])

    def request_arm(self, now_us):
        """Explicit, per drone (SWM-02). Mode; once it is acknowledged, a zero-thrust lease and arm.

        Selecting a source clears every lease, and the owner applies setpoints before queued
        commands. Sending all three at once loses the lease whenever the transport delivers them
        in one owner tick (seen in Gazebo under CI load), so the arm would be refused.
        """
        if self.phase != 'standby':
            return False
        self.phase = 'arming'
        self.arm_requested_us = now_us
        self.controller.reset()
        self.pending.append(self.messages.auto_mode(self.system_id))
        return True

    def request_stop(self, now_us):
        """Stop goes ahead of anything queued and is sent this tick (SWM-03)."""
        self.pending.appendleft(self.messages.stop(self.system_id))
        if self.stop_requested_us is None:
            self.stop_requested_us = now_us
        self.phase = 'stopped'
        self.target = None

    def release(self):
        """Allow a new explicit arm request after a completed stop (repeat flights only).

        A lost or refused drone is not released: it needs operator attention first.
        """
        if self.phase == 'stopped':
            self.phase = 'standby'
            self.stop_requested_us = self.stopped_us = None

    def receive(self, now_us, frames):
        for kind, value in parse_replies(self.decoder, frames):
            self._reply(now_us, kind, value)

    def _reply(self, now_us, kind, value):
        if kind == 'notice':
            self.events.append((now_us, 'notice', value))
            if self.phase == 'refused' and self.refusal == 'denied' and value.startswith('arm denied: '):
                self.refusal = value[len('arm denied: '):]
        elif kind == 'parameter' and self.phase == 'provisioning':
            name, number = value
            if name in self._expected and abs(self._expected[name] - number) < 1e-6:
                del self._expected[name]
                if not self._expected:
                    self.pending.append(self.messages.console(self.system_id, 'exit'))
        elif kind == 'console' and self.phase == 'provisioning' and not self._expected:
            if value == '> exit\nok\n':
                self.phase = 'standby'
                self.events.append((now_us, 'provisioned', self.system_id))
        elif kind == 'ack' and self.phase == 'arming':
            command, result = value
            if result != 0:
                self.phase = 'refused'
                self.refusal = 'denied' if command == 400 else 'mode denied'
                self.events.append((now_us, 'arm-refused', command))
            elif command == 176:
                self.pending.extend([
                    self.messages.attitude(self.system_id, LEVEL, 0.0),
                    self.messages.arm(self.system_id),
                ])
            elif command == 400:
                self.phase = 'armed'
                self.arm_latency_us = now_us - self.arm_requested_us
                self.next_setpoint_us = now_us
                self.last_heartbeat_us = now_us
                self.events.append((now_us, 'armed', self.arm_latency_us))
        elif kind == 'heartbeat':
            self.last_heartbeat_us = now_us
            if self.phase == 'armed' and not value[0]:
                self._lose(now_us, 'reported disarmed')

    def _lose(self, now_us, why):
        """Stop streaming to this drone and record it for the coordinator (SWM-04).

        The ground never re-arms a lost drone; recovery is a new explicit arm request.
        """
        self.phase = 'lost'
        self.target = None
        self.events.append((now_us, 'lost', why))

    def outgoing(self, now_us, position, velocity, attitude=None):
        """The next frame for this drone, if one is due: queued requests first, then setpoints."""
        if self.phase == 'armed' and now_us - self.last_heartbeat_us > HEARTBEAT_TIMEOUT_US:
            self._lose(now_us, 'heartbeat timeout')
        if self.pending:
            return self.pending.popleft()
        if (self.config.onboard and self.positioning and now_us >= self.next_position_us
                and self.phase not in ('provisioning', 'lost')):
            self.next_position_us = now_us + self.config.position_period_us
            sigma = self.config.position_noise_m
            measured = tuple(v + self.noise.gauss(0.0, sigma) for v in position) if sigma else position
            return self.messages.mocap(measured, attitude if self.config.heading else None)
        if self.phase != 'armed' or now_us < self.next_setpoint_us:
            return None
        if self.config.onboard and self.target is not None:
            self.next_setpoint_us = now_us + self.config.setpoint_period_us
            point, speed, _, heading = self.target(now_us)
            return self.messages.position_target(self.system_id, point, speed, heading)
        dt = self.config.setpoint_period_us * 1e-6
        self.next_setpoint_us = now_us + self.config.setpoint_period_us
        if self.target is None:
            attitude, thrust = LEVEL, 0.0
        else:
            attitude, thrust = self.controller.update(self.target(now_us), position, velocity, dt)
        return self.messages.attitude(self.system_id, attitude, thrust)


class Vehicle(Ground):
    """A Ground plus its in-process owner, IMU synthesis and bounded link (fast harness, FlixSIL)."""

    def __init__(self, name, system_id, seed, config=None):
        super().__init__(name, system_id, config)
        self.owner = Owner()
        self.imu = Imu(seed, self.config.imu_noise)
        self.link = Link(self.config.link_capacity, self.config.link_latency_us)
        self.calibrated = True
        self.sensor_fault = None
        self.previous_velocity = None
        self.state = self.reason = None
        self.motors = (0.0, 0.0, 0.0, 0.0)
        self.rejected = 0

    def close(self):
        self.owner.close()

    def control(self, now_us, dt_us, position, velocity, attitude, rates):
        """Run one owner tick and the ground side; return the gated motor outputs."""
        for frame in self.link.due(now_us):
            if not self.owner.receive(frame, now_us):
                self.rejected += 1
        dt = dt_us * 1e-6
        if self.previous_velocity is None:
            acceleration = (0.0, 0.0, 0.0)
        else:
            acceleration = g.scale(g.sub(velocity, self.previous_velocity), 1.0 / dt)
        self.previous_velocity = velocity
        gyro, specific, mask = self.imu.sample(attitude, rates, acceleration)
        sample_us = now_us
        if self.sensor_fault and now_us >= self.sensor_fault[1]:
            if self.sensor_fault[0] == 'nan':
                specific = (math.nan,) + specific[1:]
            else:
                sample_us = now_us - 10_001
        _, state, reason, _, motors = self.owner.tick(now_us, gyro, specific, mask, self.calibrated, sample_us)
        if (state, reason) != (self.state, self.reason):
            self.events.append((now_us, 'state', STATES[state], REASONS[reason]))
            self.state, self.reason = state, reason
        self.motors = motors
        if self.stop_requested_us is not None and self.stopped_us is None and not any(motors):
            self.stopped_us = now_us
        replies = self.owner.replies()
        if self.link.up:
            self.receive(now_us, replies)
        frame = self.outgoing(now_us, position, velocity, attitude)
        if frame is not None:
            self.link.send(frame, now_us)
        return motors
