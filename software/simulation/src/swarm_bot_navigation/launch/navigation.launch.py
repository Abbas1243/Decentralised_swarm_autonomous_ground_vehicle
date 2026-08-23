import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def generate_launch_description():

    pkg_navigation = get_package_share_directory('swarm_bot_navigation')

    slam_params_file = os.path.join(
        pkg_navigation, 'config', 'slam_toolbox_params.yaml'
    )

    def launch_nodes(context):
        ns = context.launch_configurations['namespace']

        slam_toolbox = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace=ns,
            output='screen',
            parameters=[
                slam_params_file,
                {
                    'use_sim_time': True,
                    'odom_frame': ns + '/odom',
                    'map_frame': ns + '/map',
                    'base_frame': ns + '/base_footprint',
                    'scan_topic': '/' + ns + '/scan',
                }
            ]
            # NO tf remappings — slam_toolbox must publish to global /tf
            # Frame uniqueness is guaranteed by frame names (robot_1/map etc.)
        )

        controller_server = Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace=ns,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'controller_frequency': 20.0,
                'min_x_velocity_threshold': 0.001,
                'min_y_velocity_threshold': 0.5,
                'min_theta_velocity_threshold': 0.001,
                'failure_tolerance': 0.3,
                'progress_checker_plugins': ['progress_checker'],
                'goal_checker_plugins': ['general_goal_checker'],
                'controller_plugins': ['FollowPath'],
                'progress_checker.plugin': 'nav2_controller::SimpleProgressChecker',
                'progress_checker.required_movement_radius': 0.5,
                'progress_checker.movement_time_allowance': 10.0,
                'general_goal_checker.plugin': 'nav2_controller::SimpleGoalChecker',
                'general_goal_checker.stateful': True,
                'general_goal_checker.xy_goal_tolerance': 0.25,
                'general_goal_checker.yaw_goal_tolerance': 0.25,
                'FollowPath.plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
                'FollowPath.desired_linear_vel': 0.3,
                'FollowPath.lookahead_dist': 0.6,
                'FollowPath.min_lookahead_dist': 0.3,
                'FollowPath.max_lookahead_dist': 0.9,
                'FollowPath.lookahead_time': 1.5,
                'FollowPath.rotate_to_heading_angular_vel': 1.8,
                'FollowPath.transform_tolerance': 0.1,
                'FollowPath.use_velocity_scaled_lookahead_dist': False,
                'FollowPath.min_approach_linear_velocity': 0.05,
                'FollowPath.approach_velocity_scaling_dist': 0.6,
                'FollowPath.use_collision_detection': True,
                'FollowPath.max_allowed_time_to_collision_up_to_carrot': 1.0,
                'FollowPath.use_regulated_linear_velocity_scaling': True,
                'FollowPath.use_fixed_curvature_lookahead': False,
                'FollowPath.curvature_feedforward_gain': 1.0,
                'FollowPath.use_cost_regulated_linear_velocity_scaling': False,
                'FollowPath.regulated_linear_scaling_min_radius': 0.9,
                'FollowPath.regulated_linear_scaling_min_speed': 0.25,
                'FollowPath.use_rotate_to_heading': True,
                'FollowPath.allow_reversing': False,
                'FollowPath.rotate_to_heading_min_angle': 0.785,
                'FollowPath.max_angular_accel': 3.2,
                'FollowPath.max_robot_pose_search_dist': 10.0,
                'robot_base_frame': ns + '/base_footprint',
                'local_costmap.global_frame': ns + '/odom',
                'local_costmap.robot_base_frame': ns + '/base_footprint',
                'local_costmap.rolling_window': True,
                'local_costmap.width': 3,
                'local_costmap.height': 3,
                'local_costmap.resolution': 0.05,
                'local_costmap.robot_radius': 0.15,
                'local_costmap.plugins': ['obstacle_layer', 'inflation_layer'],
                'local_costmap.obstacle_layer.plugin': 'nav2_costmap_2d::ObstacleLayer',
                'local_costmap.obstacle_layer.enabled': True,
                'local_costmap.obstacle_layer.observation_sources': 'scan',
                'local_costmap.obstacle_layer.scan.topic': '/' + ns + '/scan',
                'local_costmap.obstacle_layer.scan.data_type': 'LaserScan',
                'local_costmap.obstacle_layer.scan.clearing': True,
                'local_costmap.obstacle_layer.scan.marking': True,
                'local_costmap.inflation_layer.plugin': 'nav2_costmap_2d::InflationLayer',
                'local_costmap.inflation_layer.inflation_radius': 0.35,
            }],
            remappings=[
                ('cmd_vel', '/' + ns + '/cmd_vel'),
                ('odom', '/' + ns + '/odom'),
                ('scan', '/' + ns + '/scan'),
            ]
        )

        smoother_server = Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            namespace=ns,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'smoother_plugins': ['simple_smoother'],
                'simple_smoother.plugin': 'nav2_smoother::SimpleSmoother',
                'simple_smoother.tolerance': 1.0e-10,
                'simple_smoother.max_its': 1000,
                'simple_smoother.do_refinement': True,
            }]
        )

        planner_server = Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace=ns,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'expected_planner_frequency': 20.0,
                'planner_plugins': ['GridBased'],
                'GridBased.plugin': 'nav2_navfn_planner/NavfnPlanner',
                'GridBased.tolerance': 0.5,
                'GridBased.use_astar': False,
                'GridBased.allow_unknown': True,
                'robot_base_frame': ns + '/base_footprint',
                'global_costmap.global_frame': ns + '/map',
                'global_costmap.robot_base_frame': ns + '/base_footprint',
                'global_costmap.robot_radius': 0.15,
                'global_costmap.resolution': 0.05,
                'global_costmap.track_unknown_space': True,
                'global_costmap.plugins': ['static_layer', 'obstacle_layer', 'inflation_layer'],
                'global_costmap.static_layer.plugin': 'nav2_costmap_2d::StaticLayer',
                'global_costmap.static_layer.map_subscribe_transient_local': True,
                'global_costmap.obstacle_layer.plugin': 'nav2_costmap_2d::ObstacleLayer',
                'global_costmap.obstacle_layer.enabled': True,
                'global_costmap.obstacle_layer.observation_sources': 'scan',
                'global_costmap.obstacle_layer.scan.topic': '/' + ns + '/scan',
                'global_costmap.obstacle_layer.scan.data_type': 'LaserScan',
                'global_costmap.obstacle_layer.scan.clearing': True,
                'global_costmap.obstacle_layer.scan.marking': True,
                'global_costmap.inflation_layer.plugin': 'nav2_costmap_2d::InflationLayer',
                'global_costmap.inflation_layer.inflation_radius': 0.35,
            }],
            remappings=[
                ('map', '/' + ns + '/map'),
            ]
        )

        behavior_server = Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            namespace=ns,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'costmap_topic': 'local_costmap/costmap_raw',
                'footprint_topic': 'local_costmap/published_footprint',
                'cycle_frequency': 10.0,
                'behavior_plugins': ['spin', 'backup', 'drive_on_heading',
                                     'assisted_teleop', 'wait'],
                'spin.plugin': 'nav2_behaviors/Spin',
                'backup.plugin': 'nav2_behaviors/BackUp',
                'drive_on_heading.plugin': 'nav2_behaviors/DriveOnHeading',
                'wait.plugin': 'nav2_behaviors/Wait',
                'assisted_teleop.plugin': 'nav2_behaviors/AssistedTeleop',
                'global_frame': ns + '/odom',
                'robot_base_frame': ns + '/base_footprint',
                'transform_tolerance': 0.1,
                'simulate_ahead_time': 2.0,
                'max_rotational_vel': 1.0,
                'min_rotational_vel': 0.4,
                'rotational_acc_lim': 3.2,
            }]
        )

        bt_navigator = Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            namespace=ns,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'global_frame': ns + '/map',
                'robot_base_frame': ns + '/base_footprint',
                'odom_topic': '/' + ns + '/odom',
                'bt_loop_duration': 10,
                'default_server_timeout': 20,
                'default_nav_to_pose_bt_xml': '/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml',
                'default_nav_through_poses_bt_xml': '/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml',
                'navigators': ['navigate_to_pose', 'navigate_through_poses'],
                'navigate_to_pose.plugin': 'nav2_bt_navigator/NavigateToPoseNavigator',
                'navigate_through_poses.plugin': 'nav2_bt_navigator/NavigateThroughPosesNavigator',
                'plugin_lib_names': [
                    'nav2_compute_path_to_pose_action_bt_node',
                    'nav2_compute_path_through_poses_action_bt_node',
                    'nav2_smooth_path_action_bt_node',
                    'nav2_follow_path_action_bt_node',
                    'nav2_spin_action_bt_node',
                    'nav2_wait_action_bt_node',
                    'nav2_assisted_teleop_action_bt_node',
                    'nav2_back_up_action_bt_node',
                    'nav2_drive_on_heading_bt_node',
                    'nav2_clear_costmap_service_bt_node',
                    'nav2_is_stuck_condition_bt_node',
                    'nav2_goal_reached_condition_bt_node',
                    'nav2_goal_updated_condition_bt_node',
                    'nav2_globally_updated_goal_condition_bt_node',
                    'nav2_is_path_valid_condition_bt_node',
                    'nav2_initial_pose_received_condition_bt_node',
                    'nav2_reinitialize_global_localization_service_bt_node',
                    'nav2_distance_traveled_condition_bt_node',
                    'nav2_time_expired_condition_bt_node',
                    'nav2_navigate_to_pose_action_bt_node',
                    'nav2_navigate_through_poses_action_bt_node',
                    'nav2_remove_passed_goals_action_bt_node',
                    'nav2_planner_selector_bt_node',
                    'nav2_controller_selector_bt_node',
                    'nav2_goal_checker_selector_bt_node',
                    'nav2_controller_cancel_bt_node',
                    'nav2_path_longer_on_approach_bt_node',
                    'nav2_wait_cancel_bt_node',
                    'nav2_spin_cancel_bt_node',
                    'nav2_back_up_cancel_bt_node',
                    'nav2_assisted_teleop_cancel_bt_node',
                    'nav2_drive_on_heading_cancel_bt_node',
                    'nav2_is_battery_low_condition_bt_node',
                    'nav2_pipeline_sequence_bt_node',
                    'nav2_recovery_node_bt_node',
                    'nav2_round_robin_node_bt_node',
                    'nav2_rate_controller_bt_node',
                    'nav2_distance_controller_bt_node',
                    'nav2_speed_controller_bt_node',
                    'nav2_truncate_path_action_bt_node',
                    'nav2_truncate_path_local_action_bt_node',
                    'nav2_goal_updater_node_bt_node',
                    'nav2_goal_updated_controller_bt_node',
                    'nav2_single_trigger_bt_node',
                    'nav2_smoother_selector_bt_node',
                    'nav2_progress_checker_selector_bt_node',
                    'nav2_transform_available_condition_bt_node',
                    'nav2_is_battery_charging_condition_bt_node',
                ],
            }],
            remappings=[
                ('goal_pose', '/' + ns + '/goal_pose'),
            ]
        )

        waypoint_follower = Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            namespace=ns,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'loop_rate': 20,
                'stop_on_failure': False,
                'waypoint_task_executor_plugin': 'wait_at_waypoint',
                'wait_at_waypoint.plugin': 'nav2_waypoint_follower::WaitAtWaypoint',
                'wait_at_waypoint.enabled': True,
                'wait_at_waypoint.waypoint_pause_time': 200,
            }]
        )

        velocity_smoother = Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            namespace=ns,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'smoothing_frequency': 20.0,
                'scale_velocities': False,
                'feedback': 'OPEN_LOOP',
                'max_velocity': [0.3, 0.0, 1.0],
                'min_velocity': [-0.3, 0.0, -1.0],
                'max_accel': [2.5, 0.0, 3.2],
                'max_decel': [-2.5, 0.0, -3.2],
                'odom_topic': '/' + ns + '/odom',
                'odom_duration': 0.1,
                'deadband_velocity': [0.0, 0.0, 0.0],
                'velocity_timeout': 1.0,
            }],
            remappings=[
                ('cmd_vel', '/' + ns + '/cmd_vel'),
                ('cmd_vel_smoothed', '/' + ns + '/cmd_vel'),
            ]
        )

        lifecycle_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            namespace=ns,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'smoother_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'velocity_smoother',
                ]
            }]
        )

        return [
            slam_toolbox,
            controller_server,
            smoother_server,
            planner_server,
            behavior_server,
            bt_navigator,
            waypoint_follower,
            velocity_smoother,
            lifecycle_manager,
        ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='robot_1',
            description='Robot namespace'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        OpaqueFunction(function=launch_nodes)
    ])