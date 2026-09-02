#!/usr/bin/env python3
"""
robot_agent_node.py

Each instance of this node IS one robot. It:
  - Knows its own robot_id (0-indexed) and total swarm size via ROS params.
  - Subscribes to /human_pose to get the human's current position.
  - Subscribes to /swarm_size in case robots are added/removed at runtime.
  - Independently computes its own target slot using utils.py (decentralised).
  - Controls its own TurtleSim turtle to reach and hold that slot.

Spawn one instance per robot:
    ros2 run swarm_positioning robot_agent_node \
        --ros-args -p robot_id:=0 -p num_robots:=4 -r __node__:=robot_agent_0
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from turtlesim.msg import Pose
from std_msgs.msg import Int32
import math

from swarm_positioning.utils import (
    compute_formation_positions,
    angle_diff,
    euclidean_distance,
)

# Control gains
LINEAR_KP   = 1.5
ANGULAR_KP  = 6.0
MAX_LINEAR  = 2.0
MAX_ANGULAR = 3.0
ARRIVAL_THRESHOLD = 0.15   # metres — close enough to target slot


class RobotAgentNode(Node):
    def __init__(self):
        super().__init__('robot_agent_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('robot_id',   0)
        self.declare_parameter('num_robots', 1)

        self.robot_id   = self.get_parameter('robot_id').value
        self.num_robots = self.get_parameter('num_robots').value

        # Turtle name for this robot: robot_0 → turtle2, robot_1 → turtle3, …
        # (turtle1 is reserved for the human)
        turtle_index    = self.robot_id + 2
        self.turtle_ns  = f'turtle{turtle_index}'

        self.get_logger().info(
            f'Robot agent {self.robot_id} starting as {self.turtle_ns} '
            f'(swarm size: {self.num_robots})'
        )

        # ── State ────────────────────────────────────────────────────────────
        self.human_pose   = Pose2D()
        self.my_pose      = Pose()
        self.target_x     = None
        self.target_y     = None
        self.target_theta = None

        # ── Publishers ───────────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(
            Twist, f'/{self.turtle_ns}/cmd_vel', 10
        )

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(
            Pose2D, '/human_pose', self.human_pose_callback, 10
        )
        self.create_subscription(
            Pose, f'/{self.turtle_ns}/pose', self.my_pose_callback, 10
        )
        # Optional: listen for dynamic swarm size changes
        self.create_subscription(
            Int32, '/swarm_size', self.swarm_size_callback, 10
        )

        # ── Control loop at 20 Hz ─────────────────────────────────────────
        self.create_timer(0.05, self.control_loop)

    # ── Callbacks ────────────────────────────────────────────────────────────

    def human_pose_callback(self, msg: Pose2D):
        self.human_pose = msg
        self._recompute_target()

    def my_pose_callback(self, msg: Pose):
        self.my_pose = msg

    def swarm_size_callback(self, msg: Int32):
        new_size = msg.data
        if new_size != self.num_robots:
            self.get_logger().info(
                f'Swarm size changed: {self.num_robots} → {new_size}'
            )
            self.num_robots = new_size
            self._recompute_target()

    # ── Core logic ───────────────────────────────────────────────────────────

    def _recompute_target(self):
        """Independently compute this robot's slot in the formation."""
        positions = compute_formation_positions(
            self.human_pose.x,
            self.human_pose.y,
            self.human_pose.theta,
            self.num_robots,
        )
        # Each robot picks its own index — no negotiation needed
        self.target_x, self.target_y, self.target_theta = positions[self.robot_id]

    def control_loop(self):
        if self.target_x is None:
            return   # Haven't received human pose yet

        dist = euclidean_distance(
            self.my_pose.x, self.my_pose.y,
            self.target_x,  self.target_y,
        )

        twist = Twist()

        if dist > ARRIVAL_THRESHOLD:
            # ── Phase 1: Move toward target slot ─────────────────────────
            desired_heading = math.atan2(
                self.target_y - self.my_pose.y,
                self.target_x - self.my_pose.x,
            )
            heading_error = angle_diff(desired_heading, self.my_pose.theta)

            # Turn first if significantly misaligned, then drive
            if abs(heading_error) > 0.3:
                twist.angular.z = max(
                    -MAX_ANGULAR, min(MAX_ANGULAR, ANGULAR_KP * heading_error)
                )
                twist.linear.x = 0.0
            else:
                twist.linear.x  = max(
                    -MAX_LINEAR, min(MAX_LINEAR, LINEAR_KP * dist)
                )
                twist.angular.z = max(
                    -MAX_ANGULAR, min(MAX_ANGULAR, ANGULAR_KP * heading_error)
                )
        else:
            # ── Phase 2: Arrived — rotate to face human ───────────────────
            orient_error = angle_diff(self.target_theta, self.my_pose.theta)
            if abs(orient_error) > 0.05:
                twist.angular.z = max(
                    -MAX_ANGULAR, min(MAX_ANGULAR, ANGULAR_KP * orient_error)
                )

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = RobotAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()