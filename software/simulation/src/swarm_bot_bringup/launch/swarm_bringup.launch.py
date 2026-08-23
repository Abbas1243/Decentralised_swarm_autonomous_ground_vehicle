"""
swarm_bringup.launch.py
=======================
Master launch file for the swarm robotics project.
Starts Gazebo world, spawns N robots, launches SLAM + Nav2,
fault detection, map merging, and optionally frontier exploration.

Usage:
  # 2 robots, no exploration
  ros2 launch swarm_bot_bringup swarm_bringup.launch.py num_robots:=2

  # 2 robots, full autonomous swarm
  ros2 launch swarm_bot_bringup swarm_bringup.launch.py num_robots:=2 auto_explore:=true

  # 2 robots, full swarm with unified map (robots share map knowledge)
  ros2 launch swarm_bot_bringup swarm_bringup.launch.py \
    num_robots:=2 auto_explore:=true use_merged_map:=true

  # 4 robots, full autonomous swarm with merged map
  ros2 launch swarm_bot_bringup swarm_bringup.launch.py \
    num_robots:=4 auto_explore:=true use_merged_map:=true

Only 2 terminals needed:
  Terminal 1 (always first):
    ros2 run rmw_zenoh_cpp rmw_zenohd

  Terminal 2:
    ros2 launch swarm_bot_bringup swarm_bringup.launch.py \
      num_robots:=2 auto_explore:=true use_merged_map:=true

  Optional Terminal 3 (visualization):
    ros2 launch swarm_bot_bringup swarm_rviz.launch.py num_robots:=2

Startup sequence (automatic, 2 robots):
  t=0s    Gazebo world + robot_1 spawned
  t=2s    robot_2 spawned
  t=4s    robot_1 SLAM + Nav2 starts
  t=6s    robot_2 SLAM + Nav2 starts
  t=11s   Fault detection starts
  t=16s   Map merger starts (if use_merged_map:=true)
  t=21s   Frontier exploration starts (if auto_explore:=true)
          Robots explore using unified merged map

Fault injection:
  ros2 service call /robot_2/fault_detection/inject_fault \\
    std_srvs/srv/SetBool '{data: true}'

Visualize merged map in RViz:
  Add Map display, topic: /merged_map
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


ROBOT_SPAWN_POSITIONS = {
    'robot_1': {'x':  0.0, 'y':  0.0},
    'robot_2': {'x':  2.0, 'y':  0.0},
    'robot_3': {'x':  0.0, 'y':  2.0},
    'robot_4': {'x':  2.0, 'y':  2.0},
}


def generate_launch_description():

    pkg_gazebo   = get_package_share_directory('swarm_bot_gazebo')
    pkg_nav      = get_package_share_directory('swarm_bot_navigation')
    pkg_coord    = get_package_share_directory('swarm_bot_coordination')
    pkg_fault    = get_package_share_directory('swarm_bot_fault_detection')
    pkg_mapping  = get_package_share_directory('swarm_bot_mapping')

    gazebo_launch       = os.path.join(pkg_gazebo,   'launch', 'gazebo.launch.py')
    spawn_launch        = os.path.join(pkg_gazebo,   'launch', 'spawn_robot.launch.py')
    slam_nav_launch     = os.path.join(pkg_nav,      'launch', 'slam_nav.launch.py')
    exploration_launch  = os.path.join(pkg_coord,    'launch', 'frontier_exploration.launch.py')
    fault_launch        = os.path.join(pkg_fault,    'launch', 'fault_detection.launch.py')
    merger_launch       = os.path.join(pkg_mapping,  'launch', 'map_merger.launch.py')

    def launch_setup(context, *args, **kwargs):
        num_robots      = int(context.launch_configurations['num_robots'])
        auto_explore    = context.launch_configurations['auto_explore'].lower() == 'true'
        use_merged_map  = context.launch_configurations['use_merged_map'].lower() == 'true'

        if num_robots < 1 or num_robots > 4:
            raise ValueError(f'num_robots must be 1-4, got {num_robots}')

        actions = []

        # ── 1. Gazebo world ──────────────────────────────────────────────── #
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch),
            )
        )

        # ── 2. Per-robot: spawn + SLAM + Nav2 ────────────────────────────── #
        for i in range(1, num_robots + 1):
            ns  = f'robot_{i}'
            pos = ROBOT_SPAWN_POSITIONS[ns]

            spawn_delay = 2.0 * (i - 1)   # 0s, 2s, 4s, 6s
            nav_delay   = spawn_delay + 4.0

            actions.append(TimerAction(
                period=spawn_delay,
                actions=[IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(spawn_launch),
                    launch_arguments={
                        'namespace':    ns,
                        'x_pos':        str(pos['x']),
                        'y_pos':        str(pos['y']),
                        'z_pos':        '0.1',
                        'use_sim_time': 'true',
                    }.items(),
                )]
            ))

            actions.append(TimerAction(
                period=nav_delay,
                actions=[IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(slam_nav_launch),
                    launch_arguments={
                        'namespace':    ns,
                        'use_sim_time': 'true',
                    }.items(),
                )]
            ))

        # ── 3. Fault detection (always on) ───────────────────────────────── #
        last_nav_start  = 2.0 * (num_robots - 1) + 4.0
        fault_delay     = last_nav_start + 5.0
        merger_delay    = last_nav_start + 10.0
        explore_delay   = last_nav_start + 15.0

        actions.append(TimerAction(
            period=fault_delay,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(fault_launch),
                launch_arguments={
                    'num_robots': str(num_robots),
                }.items(),
            )]
        ))

        # ── 4. Map merger (optional) ──────────────────────────────────────── #
        # Starts 10s after last Nav2. Robots need initial maps before merging.
        # Only launched when use_merged_map:=true.
        if use_merged_map:
            actions.append(TimerAction(
                period=merger_delay,
                actions=[IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(merger_launch),
                    launch_arguments={
                        'num_robots': str(num_robots),
                    }.items(),
                )]
            ))

        # ── 5. Frontier exploration (optional) ───────────────────────────── #
        # Starts 15s after last Nav2 (5s after merger) when use_merged_map:=true
        # so frontier explorer sees merged map from first auction cycle.
        # Starts 10s after last Nav2 when use_merged_map:=false.
        if auto_explore:
            effective_explore_delay = explore_delay if use_merged_map \
                else (last_nav_start + 10.0)
            actions.append(TimerAction(
                period=effective_explore_delay,
                actions=[IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(exploration_launch),
                    launch_arguments={
                        'num_robots':     str(num_robots),
                        'use_merged_map': 'true' if use_merged_map else 'false',
                    }.items(),
                )]
            ))

        return actions

    return LaunchDescription([
        DeclareLaunchArgument(
            'num_robots',
            default_value='4',
            description='Number of robots to spawn (1-4)'
        ),
        DeclareLaunchArgument(
            'auto_explore',
            default_value='false',
            description='Start frontier exploration after bringup (true/false)'
        ),
        DeclareLaunchArgument(
            'use_merged_map',
            default_value='false',
            description='Launch map merger and use unified map for exploration (true/false)'
        ),
        OpaqueFunction(function=launch_setup),
    ])