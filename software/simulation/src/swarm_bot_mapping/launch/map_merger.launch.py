"""
map_merger.launch.py
====================
Launches the map merger node for each robot in the swarm.
Run this AFTER swarm_bringup.launch.py — robots must have maps first.

Usage:
  ros2 launch swarm_bot_mapping map_merger.launch.py num_robots:=2
  ros2 launch swarm_bot_mapping map_merger.launch.py num_robots:=4

Each robot gets its own MapMergerNode that:
  - Subscribes to all robots' /robot_N/map topics
  - Fuses them into a single unified OccupancyGrid
  - Publishes merged map on /robot_N/merged_map (for local use)
  - Publishes merged map on /merged_map (global — for RViz)

To visualize the merged map in RViz, add a Map display with topic /merged_map.

The frontier explorer automatically uses the merged map when the
swarm_bringup.launch.py has use_merged_map:=true — this remaps
each robot's map subscription to its /robot_N/merged_map topic,
giving frontier detection full swarm-wide coverage awareness.
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
                    package='swarm_bot_mapping',
                    executable='map_merger_node.py',
                    name='map_merger_node',
                    namespace=ns,
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'robot_id':     ns,
                        'num_robots':   num_robots,
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