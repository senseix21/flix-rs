"""Minimal high-level planner for runs without ROS.

It mirrors the Crazyswarm2 high-level commander's semantics (each segment starts from the last
commanded position, not the measured one) with a minimum-jerk profile. Under ROS, the
Crazyflie firmware planner (cffirmware) supplies targets instead.
"""
from . import geometry as g


def _profile(tau):
    """Minimum-jerk position fraction and its first two derivatives in tau."""
    s = tau ** 3 * (10 - 15 * tau + 6 * tau ** 2)
    ds = 30 * tau ** 2 * (1 - tau) ** 2
    dds = 60 * tau * (1 - tau) * (1 - 2 * tau)
    return s, ds, dds


class Planner:
    def __init__(self, position):
        self.position = tuple(position)
        self.heading = 0.0
        self.segment = None

    def go_to(self, goal, duration, now_us, relative=False):
        if duration <= 0:
            raise ValueError('duration must be positive')
        start = self.current(now_us)[0]
        goal = g.add(start, goal) if relative else tuple(goal)
        self.segment = (start, goal, now_us, int(duration * 1e6))
        self.position = goal

    def done(self, now_us):
        return self.segment is None or now_us >= self.segment[2] + self.segment[3]

    def current(self, now_us):
        """(position, velocity, acceleration, heading) at now_us."""
        if self.segment is None:
            return self.position, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), self.heading
        start, goal, begin, length = self.segment
        tau = min(1.0, max(0.0, (now_us - begin) / length))
        s, ds, dds = _profile(tau)
        delta = g.sub(goal, start)
        seconds = length * 1e-6
        return (
            g.add(start, g.scale(delta, s)),
            g.scale(delta, ds / seconds),
            g.scale(delta, dds / seconds ** 2),
            self.heading,
        )
