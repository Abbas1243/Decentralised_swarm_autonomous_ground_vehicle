"""
swarm_rviz.launch.py
====================
Launch RViz for visualizing the swarm.
Run this in a separate terminal after swarm_bringup.launch.py.

Usage:
  ros2 launch swarm_bot_bringup swarm_rviz.launch.py
  ros2 launch swarm_bot_bringup swarm_rviz.launch.py num_robots:=2

Displays:
  - Merged map (/merged_map) — unified map from all robots
  - Per-robot individual maps (/robot_N/map)
  - Per-robot laser scans (/robot_N/scan)
  - Per-robot planned paths (/robot_N/plan)
  - Per-robot robot models (/robot_N/robot_description)
  - Per-robot frontier markers (/robot_N/frontiers)
  - Per-robot costmaps (/robot_N/local_costmap/costmap)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def generate_launch_description():

    def launch_setup(context, *args, **kwargs):
        num_robots  = int(context.launch_configurations['num_robots'])
        fixed_frame = 'robot_1/map'   # robot_1's map frame — always exists in TF

        rviz_config = build_rviz_config(num_robots, fixed_frame)

        import tempfile
        tmp = tempfile.NamedTemporaryFile(
            mode='w', suffix='.rviz', delete=False, prefix='swarm_rviz_'
        )
        tmp.write(rviz_config)
        tmp.flush()

        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', tmp.name],
            parameters=[{'use_sim_time': True}],
            output='screen',
        )

        return [rviz_node]

    return LaunchDescription([
        DeclareLaunchArgument(
            'num_robots',
            default_value='4',
            description='Number of robots to visualize (1-4)'
        ),
        OpaqueFunction(function=launch_setup),
    ])


def build_rviz_config(num_robots, fixed_frame):

    displays = []

    # Grid
    displays.append("""\
  - Class: rviz_default_plugins/Grid
    Name: Grid
    Value: true
    Cell Size: 1.0
    Color: 160; 160; 164""")

    # ── Merged map — most important display ─────────────────────────────── #
    displays.append("""\
  - Class: rviz_default_plugins/Map
    Name: MergedMap
    Topic:
      Value: /merged_map
    Alpha: 0.8
    Color Scheme: costmap
    Value: true""")

    # Per-robot colors
    colors = [
        '255; 80; 80',    # red    — robot_1
        '80; 200; 80',    # green  — robot_2
        '80; 120; 255',   # blue   — robot_3
        '255; 200; 0',    # yellow — robot_4
    ]

    for i in range(1, num_robots + 1):
        ns    = f'robot_{i}'
        color = colors[i - 1]

        # Individual map (semi-transparent, under merged map)
        displays.append(f"""\
  - Class: rviz_default_plugins/Map
    Name: Map_{ns}
    Topic:
      Value: /{ns}/map
    Alpha: 0.4
    Value: true""")

        # Laser scan
        displays.append(f"""\
  - Class: rviz_default_plugins/LaserScan
    Name: Scan_{ns}
    Topic:
      Value: /{ns}/scan
    Size (m): 0.05
    Color: {color}
    Value: true""")

        # Planned path
        displays.append(f"""\
  - Class: rviz_default_plugins/Path
    Name: Path_{ns}
    Topic:
      Value: /{ns}/plan
    Color: {color}
    Value: true""")

        # Robot model
        displays.append(f"""\
  - Class: rviz_default_plugins/RobotModel
    Name: Robot_{ns}
    Description Topic:
      Value: /{ns}/robot_description
    Value: true""")

        # Frontier markers (yellow=available, green=assigned, red=blacklisted)
        displays.append(f"""\
  - Class: rviz_default_plugins/Marker
    Name: Frontiers_{ns}
    Topic:
      Value: /{ns}/frontiers
    Value: true""")

        # Local costmap (shows obstacle inflation around each robot)
        displays.append(f"""\
  - Class: rviz_default_plugins/Map
    Name: LocalCostmap_{ns}
    Topic:
      Value: /{ns}/local_costmap/costmap
    Alpha: 0.3
    Color Scheme: costmap
    Value: false""")

    displays_str = '\n'.join(displays)

    config = f"""Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Displays:
{displays_str}
  Global Options:
    Fixed Frame: {fixed_frame}
    Frame Rate: 10
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
    - Class: rviz_default_plugins/SetInitialPose
      Topic:
        Value: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Value: /robot_1/goal_pose
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic:
        Value: /clicked_point
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 18
      Pitch: 0.9
      Yaw: 0.785398
Window Geometry:
  Height: 1000
  Width: 1400
"""
    return config