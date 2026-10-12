"""Contract tests for the S1 swarm harness. Run from the repository root:

    PYTHONPATH=adapters/crazyswarm2 python3 -m unittest discover -s adapters/crazyswarm2/tests
"""
import math
import unittest

from flix_swarm import geometry as g
from flix_swarm.owner import ARMED, Capacity, Messages, Owner
from flix_swarm.plant import GRAVITY, HALF_SIZE, Plant, hover_thrust
from flix_swarm.position import PositionController, attitude_for
from flix_swarm.swarm import PlanRejected, Swarm, closest_approach
from flix_swarm.vehicle import Link


class Geometry(unittest.TestCase):
    def test_attitude_points_body_z_along_force(self):
        for force, heading in (((1.0, 0.0, 9.8), 0.0), ((0.0, -2.0, 9.8), 1.0), ((0.0, 0.0, 1.0), -2.5)):
            q = attitude_for(force, heading)
            z = g.rotate(q, (0.0, 0.0, 1.0))
            expected = g.scale(force, 1 / g.norm(force))
            for a, b in zip(z, expected):
                self.assertAlmostEqual(a, b, places=9)
            self.assertAlmostEqual(g.yaw(attitude_for((0.0, 0.0, 1.0), heading)), heading, places=9)

    def test_rotate_and_unrotate_are_inverse(self):
        q = g.normalize((0.9, 0.1, -0.3, 0.2))
        v = (0.3, -1.2, 2.0)
        for a, b in zip(g.unrotate(q, g.rotate(q, v)), v):
            self.assertAlmostEqual(a, b, places=12)

    def test_forward_acceleration_pitches_nose_down(self):
        # FLU body: accelerating toward +x needs the thrust axis tilted toward +x.
        controller = PositionController()
        target = ((1.0, 0.0, 0.5), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0)
        q, _ = controller.update(target, (0.0, 0.0, 0.5), (0.0, 0.0, 0.0), 0.02)
        self.assertGreater(g.rotate(q, (0.0, 0.0, 1.0))[0], 0.0)

    def test_position_output_is_bounded(self):
        controller = PositionController()
        target = ((100.0, -100.0, 100.0), (50.0, 0.0, 50.0), (0.0, 0.0, 100.0), 0.0)
        q, thrust = controller.update(target, (0.0, 0.0, 0.5), (0.0, 0.0, 0.0), 0.02)
        self.assertLessEqual(g.tilt(q), controller.max_tilt + 1e-9)
        self.assertLessEqual(thrust, controller.max_thrust)
        q, thrust = controller.update(((0.0, 0.0, -100.0),) + target[1:], (0.0, 0.0, 0.5), (0.0, 0.0, 0.0), 0.02)
        self.assertGreaterEqual(thrust, 0.0)


class Physics(unittest.TestCase):
    def test_rests_on_the_ground(self):
        plant = Plant((0.0, 0.0, 0.0))
        for _ in range(500):
            plant.step((0.0,) * 4, 0.001)
        self.assertAlmostEqual(plant.position[2], HALF_SIZE[2] - 0.065 * GRAVITY / 2309.0, delta=5e-5)
        self.assertLess(g.norm(plant.velocity), 1e-3)

    def test_hover_thrust_balances_weight(self):
        plant = Plant((0.0, 0.0, 0.0))
        plant.position = (0.0, 0.0, 1.0)
        plant.step((hover_thrust(),) * 4, 0.001)
        self.assertAlmostEqual(plant.velocity[2], 0.0, places=9)

    def test_rejects_out_of_range_motor_values(self):
        with self.assertRaises(ValueError):
            Plant((0, 0, 0)).step((0.0, 0.0, 1.5, 0.0), 0.001)
        with self.assertRaises(ValueError):
            Plant((0, 0, 0)).step((0.0, 0.0, math.nan, 0.0), 0.001)


class Planning(unittest.TestCase):
    def test_closest_approach(self):
        self.assertAlmostEqual(closest_approach((0, 0, 0), (2, 0, 0), (2, 0, 0), (0, 0, 0)), 0.0)
        self.assertAlmostEqual(closest_approach((0, 0, 0), (1, 0, 0), (0, 1, 0), (1, 1, 0)), 1.0)
        self.assertAlmostEqual(closest_approach((0, 0, 0), (0, 0, 0), (3, 4, 0), (3, 4, 0)), 5.0)

    def test_start_positions_must_be_separated(self):
        with self.assertRaises(PlanRejected):
            Swarm([(0, 0, 0), (0.1, 0, 0)])


class Links(unittest.TestCase):
    def test_full_link_drops_new_frames(self):
        link = Link(capacity=2, latency_us=0)
        for frame in (b'a', b'b', b'c'):
            link.send(frame, 0)
        self.assertEqual((link.sent, link.dropped), (2, 1))
        self.assertEqual(list(link.due(0)), [b'a', b'b'])

    def test_cut_loses_queued_frames(self):
        link = Link(capacity=4, latency_us=10)
        link.send(b'a', 0)
        link.cut()
        link.send(b'b', 0)
        self.assertEqual(link.lost, 2)
        self.assertEqual(list(link.due(100)), [])


class Owners(unittest.TestCase):
    def test_registry_bound(self):
        owners = []
        try:
            for _ in range(8):
                owners.append(Owner())
            with self.assertRaises(Capacity):
                Owner()
        finally:
            for owner in owners:
                owner.close()
        Owner().close()

    def test_each_drone_ignores_other_addresses(self):
        with Swarm([(0, 0, 0), (1, 0, 0)]) as swarm:
            swarm.provision()
            messages = Messages()
            first, second = (d.vehicle for d in swarm.drones)
            # Default ID 1 and the other drone's ID are both foreign once provisioned (SWM-01).
            for target in (1, second.system_id):
                self.assertFalse(first.owner.receive(messages.arm(target), swarm.now_us))
                self.assertFalse(first.owner.receive(messages.attitude(target, (1, 0, 0, 0), 0.0), swarm.now_us))
            self.assertTrue(first.owner.receive(messages.attitude(first.system_id, (1, 0, 0, 0), 0.0), swarm.now_us))

    def test_arming_is_explicit_per_drone(self):
        with Swarm([(0, 0, 0), (1, 0, 0)]) as swarm:
            swarm.provision()
            swarm.run(0.5)
            self.assertTrue(all(d.vehicle.state != ARMED for d in swarm.drones))
            result = swarm.arm(['cf2'])
            self.assertEqual(result, {'cf2': None})
            self.assertEqual([d.vehicle.state == ARMED for d in swarm.drones], [False, True])

    def test_takeoff_ignores_unarmed_drones(self):
        with Swarm([(0, 0, 0), (1, 0, 0)]) as swarm:
            swarm.provision()
            swarm.arm(['cf1'])
            swarm.takeoff(0.5, 1.0)
            swarm.run(2.0)
            first, second = swarm.drones
            self.assertGreater(first.plant.position[2], 0.4)
            self.assertLess(second.plant.position[2], HALF_SIZE[2] + 0.001)
            self.assertFalse(any(second.vehicle.motors))


if __name__ == '__main__':
    unittest.main()
