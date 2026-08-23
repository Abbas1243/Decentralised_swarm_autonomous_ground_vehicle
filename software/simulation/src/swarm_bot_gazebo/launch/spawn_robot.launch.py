import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    pkg_description = get_package_share_directory('swarm_bot_description')

    # Launch arguments
    namespace = LaunchConfiguration('namespace', default='robot_1')
    x_pos = LaunchConfiguration('x_pos', default='0.0')
    y_pos = LaunchConfiguration('y_pos', default='0.0')
    z_pos = LaunchConfiguration('z_pos', default='0.1')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # We need to use a function to get namespace value at launch time
    def create_robot_description(context):
        ns = context.launch_configurations['namespace']
        urdf_file = os.path.join(
            pkg_description, 'urdf', 'swarm_bot.urdf.xacro'
        )
        robot_desc = xacro.process_file(
            urdf_file,
            mappings={'robot_namespace': ns}
        ).toxml()

        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=ns,
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': True,
            }]
        )

        spawn_robot = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', f'/{ns}/robot_description',
                '-entity', ns,
                '-x', context.launch_configurations['x_pos'],
                '-y', context.launch_configurations['y_pos'],
                '-z', context.launch_configurations['z_pos'],
                '-robot_namespace', ns
            ],
            output='screen'
        )

        return [robot_state_publisher, spawn_robot]

    from launch.actions import OpaqueFunction
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot_1',
            description='Robot namespace'),
        DeclareLaunchArgument('x_pos', default_value='0.0',
            description='X spawn position'),
        DeclareLaunchArgument('y_pos', default_value='0.0',
            description='Y spawn position'),
        DeclareLaunchArgument('z_pos', default_value='0.1',
            description='Z spawn position'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
            description='Use simulation time'),
        OpaqueFunction(function=create_robot_description)
    ])