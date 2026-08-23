import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Package paths
    pkg_gazebo = get_package_share_directory('swarm_bot_gazebo')
    pkg_description = get_package_share_directory('swarm_bot_description')

    # Launch arguments
    namespace = LaunchConfiguration('namespace', default='robot_1')
    x_pos = LaunchConfiguration('x_pos', default='0.0')
    y_pos = LaunchConfiguration('y_pos', default='0.0')
    z_pos = LaunchConfiguration('z_pos', default='0.1')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # World file
    world_file = os.path.join(pkg_gazebo, 'worlds', 'test_room.world')

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            )
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'false'
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot_1',
            description='Robot namespace'),
        DeclareLaunchArgument('x_pos', default_value='0.0',
            description='X position'),
        DeclareLaunchArgument('y_pos', default_value='0.0',
            description='Y position'),
        DeclareLaunchArgument('z_pos', default_value='0.1',
            description='Z position'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
            description='Use simulation time'),
        gazebo,
    ])