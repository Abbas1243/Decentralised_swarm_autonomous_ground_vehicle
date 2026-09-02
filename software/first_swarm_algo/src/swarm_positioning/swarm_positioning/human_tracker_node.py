#!/usr/bin/env python3
"""
human_tracker_node.py

- Spawns as turtle1 (the default TurtleSim turtle = the human).
- Reads keyboard input (WASD / arrow keys) to move the human turtle manually.
- Continuously publishes the human's current pose on /human_pose so every
  robot agent can read it independently (decentralised).

Run:
    ros2 run swarm_positioning human_tracker_node
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from turtlesim.msg import Pose
import sys
import tty
import termios
import threading


MOVE_SPEED = 2.0
TURN_SPEED = 2.0

KEY_BINDINGS = {
    'w': ( MOVE_SPEED,  0.0),
    's': (-MOVE_SPEED,  0.0),
    'a': (0.0,  TURN_SPEED),
    'd': (0.0, -TURN_SPEED),
    '\x1b[A': ( MOVE_SPEED,  0.0),   # arrow up
    '\x1b[B': (-MOVE_SPEED,  0.0),   # arrow down
    '\x1b[D': (0.0,  TURN_SPEED),    # arrow left
    '\x1b[C': (0.0, -TURN_SPEED),    # arrow right
}


def get_key():
    """Non-blocking single keypress read."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            ch += sys.stdin.read(2)
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


class HumanTrackerNode(Node):
    def __init__(self):
        super().__init__('human_tracker_node')

        # Publish velocity commands to move turtle1 (the human)
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Publish human pose so all robot agents can subscribe
        self.pose_pub = self.create_publisher(Pose2D, '/human_pose', 10)

        # Subscribe to turtle1's actual pose from TurtleSim
        self.pose_sub = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10
        )

        self.current_pose = Pose2D()

        # Timer to publish human pose at 20 Hz
        self.create_timer(0.05, self.publish_human_pose)

        self.get_logger().info('Human Tracker ready. Use W/A/S/D to move the human turtle.')
        self.get_logger().info('Press Ctrl+C to quit.')

        # Keyboard thread
        self.running = True
        self.kb_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.kb_thread.start()

    def pose_callback(self, msg: Pose):
        self.current_pose.x = msg.x
        self.current_pose.y = msg.y
        self.current_pose.theta = msg.theta

    def publish_human_pose(self):
        self.pose_pub.publish(self.current_pose)

    def keyboard_loop(self):
        while self.running:
            key = get_key()
            if key == '\x03':   # Ctrl+C
                self.running = False
                rclpy.shutdown()
                break

            twist = Twist()
            if key in KEY_BINDINGS:
                twist.linear.x, twist.angular.z = KEY_BINDINGS[key]
            self.cmd_pub.publish(twist)

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HumanTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()