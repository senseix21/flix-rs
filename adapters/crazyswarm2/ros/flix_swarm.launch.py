"""Launch the Crazyswarm2 simulator with Flix drones (S1).

    ros2 launch adapters/crazyswarm2/ros/flix_swarm.launch.py
"""
import os

from launch import LaunchDescription
from launch_ros.actions import Node
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    with open(os.environ.get('FLIX_SWARM_ROBOTS', os.path.join(HERE, 'crazyflies.yaml'))) as file:
        robots = yaml.safe_load(file)
    with open(os.path.join(HERE, 'server.yaml')) as file:
        server = yaml.safe_load(file)['/crazyflie_server']['ros__parameters']
    server['robot_description'] = 'flix $NAME'
    return LaunchDescription([
        Node(
            package='crazyflie_sim',
            executable='crazyflie_server',
            name='crazyflie_server',
            output='screen',
            emulate_tty=True,
            parameters=[robots, server],
        ),
    ])
