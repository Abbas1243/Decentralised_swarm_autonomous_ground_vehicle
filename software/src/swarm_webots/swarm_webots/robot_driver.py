"""Custom webots_ros2_driver plugin, one instance auto-spawned per robot by
webots_ros2_driver based on the <plugin> tag in that robot's URDF.

Subscribes:  cmd_vel (geometry_msgs/Twist)
Publishes:   odom     (nav_msgs/Odometry)  -- ground-truth position from GPS +
             Compass rather than wheel-odometry integration, since for a swarm
             algorithm we care about testing the swarm logic, not sensor drift.
             Swap in real wheel-odometry / a localization node later if needed.

This is the standard "extern controller + custom Python plugin" pattern
documented by webots_ros2_driver: the class is referenced by name from the
robot's URDF, e.g.
    <plugin type="swarm_webots.robot_driver.RobotDriver"/>
"""

import math
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class RobotDriver:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        self.timestep = int(self.robot.getBasicTimeStep())

        # -- these must match the device names in swarm_world.wbt --
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        self.gps = self.robot.getDevice('gps')
        self.compass = self.robot.getDevice('compass')
        self.gps.enable(self.timestep)
        self.compass.enable(self.timestep)

        self.wheel_radius = 0.033   # m -- matches Cylinder radius in swarm_world.wbt
        self.axle_length = 0.13     # m -- distance between the two wheel anchors (0.065 * 2) in swarm_world.wbt
        self.max_wheel_speed = 6.28  # rad/s, motor limit

        self.target_linear = 0.0
        self.target_angular = 0.0
        self.last_position = None

        # NOTE: do NOT call rclpy.init() here -- the webots_ros2_driver
        # 'driver' process already initializes rclpy before loading this
        # plugin. Calling init() again raises and crashes the process.
        self.node = rclpy.create_node('robot_driver_' + str(id(self)))
        # robot_id / namespace comes from the launch file's node namespace,
        # so plain topic names here resolve to <namespace>/cmd_vel etc.
        self.node.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self.odom_pub = self.node.create_publisher(Odometry, 'odom', 10)

    def _on_cmd_vel(self, msg: Twist):
        self.target_linear = msg.linear.x
        self.target_angular = msg.angular.z

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)

        # differential drive inverse kinematics
        left_speed = (self.target_linear - self.target_angular * self.axle_length / 2.0) / self.wheel_radius
        right_speed = (self.target_linear + self.target_angular * self.axle_length / 2.0) / self.wheel_radius
        left_speed = max(-self.max_wheel_speed, min(self.max_wheel_speed, left_speed))
        right_speed = max(-self.max_wheel_speed, min(self.max_wheel_speed, right_speed))
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)

        self._publish_odom()

    def _publish_odom(self):
        gps_val = self.gps.getValues()       # [x, y, z] world frame
        compass_val = self.compass.getValues()
        heading = math.atan2(compass_val[0], compass_val[1])

        now = self.robot.getTime()
        vx, vy = 0.0, 0.0
        if self.last_position is not None:
            last_x, last_y, last_t = self.last_position
            dt = max(now - last_t, 1e-6)
            vx = (gps_val[0] - last_x) / dt
            vy = (gps_val[1] - last_y) / dt
        self.last_position = (gps_val[0], gps_val[1], now)

        msg = Odometry()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.pose.position.x = gps_val[0]
        msg.pose.pose.position.y = gps_val[1]
        msg.pose.pose.orientation.z = math.sin(heading / 2.0)
        msg.pose.pose.orientation.w = math.cos(heading / 2.0)
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        self.odom_pub.publish(msg)
