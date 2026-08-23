"""
frontier_exploration.launch.py
================================
Launches the frontier explorer node for each robot in the swarm.
Run this AFTER swarm_bringup.launch.py — robots must have maps first.

Usage:
  ros2 launch swarm_bot_coordination frontier_exploration.launch.py num_robots:=2
  ros2 launch swarm_bot_coordination frontier_exploration.launch.py num_robots:=4

use_merged_map:=true passes the parameter directly to the frontier_explorer node,
which subscribes to /{ns}/merged_map instead of /{ns}/map. The node handles the
topic selection internally — no remappings needed or used.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def generate_launch_description():

    def launch_setup(context, *args, **kwargs):
        num_robots     = int(context.launch_configurations['num_robots'])
        use_merged_map = context.launch_configurations.get(
            'use_merged_map', 'false').lower() == 'true'
        nodes = []

        for i in range(1, num_robots + 1):
            ns = f'robot_{i}'

            nodes.append(
                Node(
                    package='swarm_bot_coordination',
                    executable='frontier_explorer.py',
                    name='frontier_explorer',
                    namespace=ns,
                    output='screen',
                    parameters=[{
                        'use_sim_time':       True,
                        'robot_id':           ns,
                        'num_robots':         num_robots,
                        'use_merged_map':     use_merged_map,
                        'bid_broadcast_hz':   1.0,
                        'auction_hz':         0.5,
                        'min_frontier_size':  3,
                        'replan_timeout_sec': 30.0,
                    }],
                )
            )

        return nodes

    return LaunchDescription([
        DeclareLaunchArgument(
            'num_robots',
            default_value='4',
            description='Number of robots in the swarm (1-4)'
        ),
        DeclareLaunchArgument(
            'use_merged_map',
            default_value='false',
            description='Use merged map for frontier detection (true/false)'
        ),
        OpaqueFunction(function=launch_setup),
    ])