import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Same pool as swarm_webots/launch/robot_launch.py -- keep in sync.
ROBOT_POOL = ['robot_0', 'robot_1', 'robot_2', 'robot_3', 'robot_4', 'robot_5']


def generate_launch_description():
    webots_launch_dir = os.path.join(get_package_share_directory('swarm_webots'), 'launch')
    params_file = os.path.join(get_package_share_directory('swarm_robot'), 'config', 'swarm_params.yaml')

    webots_and_drivers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(webots_launch_dir, 'robot_launch.py'))
    )

    agent_nodes = []
    for robot_id in ROBOT_POOL:
        agent_nodes.append(Node(
            package='swarm_robot',
            executable='swarm_agent_node',
            namespace=robot_id,
            name='swarm_agent',
            output='screen',
            parameters=[params_file, {'robot_id': robot_id}],
        ))

    return LaunchDescription([webots_and_drivers] + agent_nodes)
