"""Ground-side position loop for S1: position target to Flix AUTO attitude and thrust.

This runs on the ground computer, not in `flight-core` (swarm spec SD-02). Its outputs are
ordinary SET_ATTITUDE_TARGET requests, so Flix validates, leases and gates them like any other.
"""
import math

from . import geometry as g
from .plant import GRAVITY, MASS, MAX_THRUST, SCALES

AIRBORNE_HEIGHT = 0.05


class PositionController:
    def __init__(self, kp=(6.0, 6.0, 8.0), kd=(4.0, 4.0, 5.0), ki=(1.0, 1.0, 2.0),
                 integral_limit=0.5, max_tilt=math.radians(25), max_thrust=0.9):
        self.kp, self.kd, self.ki = kp, kd, ki
        self.integral_limit = integral_limit
        self.max_tilt = max_tilt
        self.max_thrust = max_thrust
        self.integral = (0.0, 0.0, 0.0)

    def reset(self):
        self.integral = (0.0, 0.0, 0.0)

    def update(self, target, position, velocity, dt):
        """target: (position, velocity, acceleration, yaw). Returns (attitude FLU->world, thrust)."""
        reference, reference_velocity, feedforward, heading = target
        error = g.sub(reference, position)
        velocity_error = g.sub(reference_velocity, velocity)
        # Integrate only in flight; on the ground the contact force carries the error.
        if position[2] > AIRBORNE_HEIGHT:
            self.integral = tuple(
                max(-self.integral_limit, min(self.integral_limit, self.integral[i] + error[i] * dt))
                for i in range(3)
            )
        demand = tuple(
            feedforward[i] + self.kp[i] * error[i] + self.kd[i] * velocity_error[i] + self.ki[i] * self.integral[i]
            for i in range(3)
        )
        force = (demand[0], demand[1], max(demand[2] + GRAVITY, 0.2 * GRAVITY))
        horizontal = math.hypot(force[0], force[1])
        limit = force[2] * math.tan(self.max_tilt)
        if horizontal > limit:
            force = (force[0] * limit / horizontal, force[1] * limit / horizontal, force[2])
        thrust = min(self.max_thrust, MASS * g.norm(force) / (MAX_THRUST * sum(SCALES)))
        return attitude_for(force, heading), thrust


def attitude_for(force, heading):
    """Attitude whose body z points along force, with body x facing heading."""
    z = g.scale(force, 1.0 / g.norm(force))
    y = g.cross(z, (math.cos(heading), math.sin(heading), 0.0))
    y = g.scale(y, 1.0 / g.norm(y))
    x = g.cross(y, z)
    return g.from_axes(x, y, z)
