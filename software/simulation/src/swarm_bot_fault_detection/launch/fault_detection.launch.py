"""
fault_detection.launch.py
==========================
Launches the heartbeat fault detection node for each robot in the swarm.
Run this AFTER swarm_bringup.launch.py.

Usage:
  ros2 launch swarm_bot_fault_detection fault_detection.launch.py num_robots:=2
  ros2 launch swarm_bot_fault_detection fault_detection.launch.py num_robots:=4

Each robot gets its own HeartbeatNode that:
  - Broadcasts its own heartbeat at 1Hz on /swarm/heartbeat
  - Monitors all other robots independently (no central monitor)
  - Declares a robot FAILED after 3 missed heartbeats (3s window)
  - Publishes alive/failed lists to /swarm/robot_status
  - Exposes /robot_N/fault_detection/inject_fault service for testing

The frontier_explorer automatically reacts to /swarm/robot_status:
  - Removes failed robots from the auction pool
  - Reallocates their frontiers to surviving robots
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def generate_launch_description():

    def launch_setup(context, *args, **kwargs):
        num_robots = int(context.launch_configurations['num_robots'])
        nodes = []

        for i in range(1, num_robots + 1):
            ns = f'robot_{i}'
            nodes.append(
                Node(
                    package='swarm_bot_fault_detection',
                    executable='heartbeat_node.py',
                    name='heartbeat_node',
                    namespace=ns,
                    output='screen',
                    parameters=[{
                        'use_sim_time': False,
                        'robot_id':     ns,
                        'num_robots':   num_robots,
                        'heartbeat_hz': 1.0,
                        'miss_limit':   3,
                    }]
                )
            )

        return nodes

    return LaunchDescription([
        DeclareLaunchArgument(
            'num_robots',
            default_value='4',
            description='Number of robots in the swarm (1-4)'
        ),
        OpaqueFunction(function=launch_setup),
    ])