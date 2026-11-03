import os
import pathlib
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.utils import controller_url_prefix

# Must match the `name` field of the robots placed in swarm_world.wbt.
ROBOT_POOL = ['robot_0', 'robot_1', 'robot_2', 'robot_3', 'robot_4', 'robot_5']

# Connecting many extern controllers to Webots in the same instant races
# against Webots still finishing its own startup/world registration, which
# causes "refusing connection attempt from another extern controller" and
# an immediate disconnect for every robot at once. Staggering the driver
# launches fixes it: give Webots a head start, then connect robots one at
# a time with a small gap between each.
INITIAL_DELAY_SEC = 8.0   # time for Webots itself to finish starting up
STAGGER_STEP_SEC = 1.5    # gap between each robot's connection attempt


def generate_launch_description():
    package_dir = get_package_share_directory('swarm_webots')
    world_path = os.path.join(package_dir, 'worlds', 'swarm_world.wbt')
    robot_urdf_path = os.path.join(package_dir, 'resource', 'swarm_robot.urdf')
    supervisor_urdf_path = os.path.join(package_dir, 'resource', 'swarm_supervisor.urdf')

    # This installed version of webots_ros2_driver wants a FILE PATH here,
    # not URDF text content -- it explicitly warns "Passing robot
    # description as a string is deprecated" if given content. (Some older
    # ROS distro docs show passing content instead; that's not what this
    # version wants.)
    robot_description = robot_urdf_path
    supervisor_description = supervisor_urdf_path

    webots = WebotsLauncher(world=world_path)

    actions = [webots]

    for i, robot_name in enumerate(ROBOT_POOL):
        delay = INITIAL_DELAY_SEC + i * STAGGER_STEP_SEC
        driver_node = Node(
            package='webots_ros2_driver',
            executable='driver',
            namespace=robot_name,
            output='screen',
            additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + robot_name},
            parameters=[{'robot_description': robot_description}],
        )
        actions.append(TimerAction(period=delay, actions=[driver_node]))

    supervisor_delay = INITIAL_DELAY_SEC + len(ROBOT_POOL) * STAGGER_STEP_SEC
    supervisor_node = Node(
        package='webots_ros2_driver',
        executable='driver',
        namespace='swarm_supervisor',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'swarm_supervisor'},
        parameters=[{'robot_description': supervisor_description}],
    )
    actions.append(TimerAction(period=supervisor_delay, actions=[supervisor_node]))

    return LaunchDescription(actions)
