"""IMU sample synthesis matching the Gazebo plugin's default `mpu6050-4g` profile.

Noise comes from the legacy SDF (gyro 0.1 deg/s, accel 8 mg standard deviation). Range limits
match the MPU6050 signed registers configured by the plugin (+-2000 deg/s, +-4 g); a limited
axis sets its bit in the saturation mask (gyro x/y/z bits 0..2, accel bits 3..5, ABI v2).
"""
import math
import random

from . import geometry as g
from .plant import GRAVITY

GYRO_NOISE = 0.00174533
ACCEL_NOISE = 0.0784
GYRO_SCALE = math.pi / (180.0 * 16.4)
ACCEL_SCALE = 9.80665 / 8192.0


class Imu:
    def __init__(self, seed, noise=True):
        self.random = random.Random(seed)
        self.noise = noise

    def sample(self, attitude, rates, acceleration_world):
        """Return body gyro (rad/s), body specific force (m/s^2) and saturation mask."""
        specific = g.unrotate(attitude, g.add(acceleration_world, (0.0, 0.0, GRAVITY)))
        gyro = list(rates)
        acc = list(specific)
        if self.noise:
            gyro = [v + self.random.gauss(0.0, GYRO_NOISE) for v in gyro]
            acc = [v + self.random.gauss(0.0, ACCEL_NOISE) for v in acc]
        mask = 0
        for kind, values, step in ((0, gyro, GYRO_SCALE), (1, acc, ACCEL_SCALE)):
            for i in range(3):
                limited = min(max(values[i], -32768.0 * step), 32767.0 * step)
                if limited != values[i]:
                    mask |= 1 << (i + 3 * kind)
                values[i] = limited
        return tuple(gyro), tuple(acc), mask
