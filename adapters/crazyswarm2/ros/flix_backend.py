"""Crazyswarm2 physics backend for Flix (installed as `crazyflie_sim/backend/flix.py`).

Steps `flix_swarm.plant.Plant` at 1 kHz, the Flix control rate. Actions from `FlixSIL` carry
normalized motor outputs in Flix order RL, RR, FR, FL in the `rpm` field; this backend must not
be paired with Crazyflie controllers, whose actions are real RPM in Crazyflie order.
"""
from __future__ import annotations

from flix_swarm.plant import Plant
import numpy as np
from rclpy.node import Node
from rclpy.time import Time
from rosgraph_msgs.msg import Clock

from ..sim_data_types import Action, State

TICK_US = 1000


class Backend:

    def __init__(self, node: Node, names: list[str], states: list[State]):
        self.node = node
        self.names = names
        self.clock_publisher = node.create_publisher(Clock, 'clock', 10)
        # Flix treats a zero tick as a clock fault; start one tick in.
        self.now_us = TICK_US
        self.plants = [Plant(tuple(float(v) for v in state.pos)) for state in states]

    def time(self) -> float:
        return self.now_us / 1e6

    def step(self, states_desired: list[State], actions: list[Action]) -> list[State]:
        self.now_us += TICK_US
        states = []
        for plant, action in zip(self.plants, actions):
            plant.step(tuple(float(v) for v in action.rpm), TICK_US * 1e-6)
            states.append(State(np.array(plant.position), np.array(plant.velocity),
                                np.array(plant.attitude), np.array(plant.rates)))
        clock = Clock()
        clock.clock = Time(nanoseconds=self.now_us * 1000).to_msg()
        self.clock_publisher.publish(clock)
        return states

    def shutdown(self):
        pass
