"""Rigid-body Flix plant matching the Gazebo model and plugin.

Sources, so the two plants stay comparable:
- mass, inertia, collision box: legacy `gazebo/models/flix/flix.sdf` (pinned `1f5b268`).
- arm, thrust, torque, motor asymmetry, motor order: `adapters/gazebo/plugin.cpp`.
- contact stiffness and damping: `scripts/gazebo-contact.py` soft profile (30 Hz, critically
  damped), spread over the four bottom corners of the collision box.
- gravity: Gazebo default world, 9.8 m/s^2.

Not modelled: motor lag, propeller drag, air drag, ground effect.
"""
from . import geometry as g

MASS = 0.065
INERTIA = (3.55e-5, 4.23e-5, 7.47e-5)
GRAVITY = 9.8
ARM = 0.035355
MAX_THRUST = 0.03 * 9.80665
MAX_TORQUE = 0.0024 * 9.80665
# Motor order RL, RR, FR, FL, as in the plugin.
SCALES = (0.9, 1.05, 1.1, 1.0)
MOTOR_X = (-ARM, -ARM, ARM, ARM)
MOTOR_Y = (ARM, -ARM, -ARM, ARM)
MOTOR_YAW = (-1.0, 1.0, -1.0, 1.0)
HALF_SIZE = (0.0475, 0.0475, 0.0138)
CONTACT_KP = 2309.0 / 4
CONTACT_KD = 24.5 / 4
FRICTION = 1.0
FRICTION_DAMPING = 2.0
CORNERS = tuple(
    (sx * HALF_SIZE[0], sy * HALF_SIZE[1], -HALF_SIZE[2]) for sx in (1, -1) for sy in (1, -1)
)
SUBSTEPS = 2


def hover_thrust():
    """Collective command that balances gravity with every motor at the same output."""
    return MASS * GRAVITY / (MAX_THRUST * sum(SCALES))


class Plant:
    def __init__(self, position):
        self.position = (position[0], position[1], HALF_SIZE[2])
        self.velocity = (0.0, 0.0, 0.0)
        self.attitude = (1.0, 0.0, 0.0, 0.0)
        self.rates = (0.0, 0.0, 0.0)

    def step(self, motors, dt):
        """Apply motor outputs (already through the Flix final gate) for dt seconds."""
        if len(motors) != 4 or not all(0.0 <= m <= 1.0 for m in motors):
            raise ValueError('motor outputs must be four values in [0, 1]')
        h = dt / SUBSTEPS
        for _ in range(SUBSTEPS):
            self._substep(motors, h)

    def _substep(self, motors, h):
        thrust = 0.0
        torque = (0.0, 0.0, 0.0)
        for i, m in enumerate(motors):
            f = SCALES[i] * MAX_THRUST * m
            thrust += f
            torque = g.add(
                torque, (MOTOR_Y[i] * f, -MOTOR_X[i] * f, MOTOR_YAW[i] * SCALES[i] * MAX_TORQUE * m)
            )
        force = g.add(g.rotate(self.attitude, (0.0, 0.0, thrust)), (0.0, 0.0, -MASS * GRAVITY))
        contact_force, contact_torque = self._contact()
        force = g.add(force, contact_force)
        torque = g.add(torque, contact_torque)
        # Semi-implicit Euler: velocities first, then positions with the new velocities.
        self.velocity = g.add(self.velocity, g.scale(force, h / MASS))
        self.position = g.add(self.position, g.scale(self.velocity, h))
        w = self.rates
        gyroscopic = g.cross(w, (INERTIA[0] * w[0], INERTIA[1] * w[1], INERTIA[2] * w[2]))
        net = g.sub(torque, gyroscopic)
        self.rates = tuple(w[i] + net[i] / INERTIA[i] * h for i in range(3))
        delta = g.multiply(self.attitude, (0.0,) + g.scale(self.rates, 0.5 * h))
        self.attitude = g.normalize(tuple(a + d for a, d in zip(self.attitude, delta)))

    def _contact(self):
        force = (0.0, 0.0, 0.0)
        torque = (0.0, 0.0, 0.0)
        for corner in CORNERS:
            world = g.add(self.position, g.rotate(self.attitude, corner))
            depth = -world[2]
            if depth <= 0:
                continue
            velocity = g.add(self.velocity, g.rotate(self.attitude, g.cross(self.rates, corner)))
            normal = CONTACT_KP * depth - CONTACT_KD * velocity[2]
            if normal <= 0:
                continue
            # Viscous friction, capped by the Coulomb limit.
            tangential = (-FRICTION_DAMPING * velocity[0], -FRICTION_DAMPING * velocity[1], 0.0)
            limit = FRICTION * normal
            if g.norm(tangential) > limit:
                tangential = g.scale(tangential, limit / g.norm(tangential))
            f = (tangential[0], tangential[1], normal)
            force = g.add(force, f)
            torque = g.add(torque, g.cross(corner, g.unrotate(self.attitude, f)))
        return force, torque
