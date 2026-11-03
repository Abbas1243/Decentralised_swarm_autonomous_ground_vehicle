"""One instance of this node runs per robot (in that robot's namespace).
It is fully decentralized: it only ever reads (a) its own odometry and
(b) the shared /swarm/states topic, which every robot publishes to and
every robot subscribes to -- this is the ROS2 stand-in for a local wireless
broadcast. Filtering by neighbor_radius happens locally in NeighborManager,
so a robot behaves as if it only "heard" broadcasts from nearby peers even
though the ROS topic itself has no physical range limit.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from swarm_interfaces.msg import RobotState, SwarmEvent

from swarm_robot.boids import blended_boids
from swarm_robot.consensus import consensus_step, consensus_heading_step
from swarm_robot.neighbor_manager import NeighborManager
from swarm_robot.motion_controller import MotionController


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return float(np.arctan2(siny_cosp, cosy_cosp))


class SwarmAgentNode(Node):
    def __init__(self):
        super().__init__('swarm_agent')

        # --- parameters (override per-robot via launch file / yaml) ---
        self.declare_parameter('robot_id', 'robot_0')
        self.declare_parameter('neighbor_radius', 3.0)          # m, comm/awareness range
        self.declare_parameter('desired_spacing', 0.6)          # m, separation trigger distance
        self.declare_parameter('critical_distance', 0.3)        # m, emergency stop distance
        self.declare_parameter('max_linear_speed', 0.3)         # m/s
        self.declare_parameter('max_angular_speed', 1.5)        # rad/s
        self.declare_parameter('boids_weight', 0.6)
        self.declare_parameter('consensus_weight', 0.4)
        self.declare_parameter('consensus_gain', 0.15)
        self.declare_parameter('stale_timeout_sec', 2.0)
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('boids_separation_weight', 1.5)
        self.declare_parameter('boids_alignment_weight', 1.0)
        self.declare_parameter('boids_cohesion_weight', 1.0)

        self.robot_id = self.get_parameter('robot_id').value
        self.neighbor_radius = self.get_parameter('neighbor_radius').value
        self.desired_spacing = self.get_parameter('desired_spacing').value
        self.consensus_gain = self.get_parameter('consensus_gain').value
        self.boids_weights = {
            'separation': self.get_parameter('boids_separation_weight').value,
            'alignment': self.get_parameter('boids_alignment_weight').value,
            'cohesion': self.get_parameter('boids_cohesion_weight').value,
        }

        self.neighbor_manager = NeighborManager(
            self_id=self.robot_id,
            neighbor_radius=self.neighbor_radius,
            stale_timeout_sec=self.get_parameter('stale_timeout_sec').value,
        )
        self.motion_controller = MotionController(
            max_linear_speed=self.get_parameter('max_linear_speed').value,
            max_angular_speed=self.get_parameter('max_angular_speed').value,
            critical_distance=self.get_parameter('critical_distance').value,
            boids_weight=self.get_parameter('boids_weight').value,
            consensus_weight=self.get_parameter('consensus_weight').value,
        )

        # own state, updated from odometry (published by the Webots driver)
        self.self_position = np.zeros(2)
        self.self_velocity = np.zeros(2)
        self.self_heading = 0.0
        self.have_odom = False

        # best-effort, keep-last QoS: fine for a high-rate broadcast topic
        # where a dropped sample just means "use the next one"
        swarm_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.state_pub = self.create_publisher(RobotState, '/swarm/states', swarm_qos)
        self.state_sub = self.create_subscription(RobotState, '/swarm/states', self._on_swarm_state, swarm_qos)
        self.event_pub = self.create_publisher(SwarmEvent, '/swarm/events', 10)
        self.event_sub = self.create_subscription(SwarmEvent, '/swarm/events', self._on_swarm_event, 10)

        self.odom_sub = self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        rate = self.get_parameter('control_rate_hz').value
        self.timer = self.create_timer(1.0 / rate, self._control_step)

        self._announce_join()
        self.get_logger().info(f'{self.robot_id} online, neighbor_radius={self.neighbor_radius} m')

    # ------------------------------------------------------------------
    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self.self_position = np.array([p.x, p.y])
        self.self_velocity = np.array([v.x, v.y])
        self.self_heading = yaw_from_quaternion(msg.pose.pose.orientation)
        self.have_odom = True

    def _on_swarm_state(self, msg: RobotState):
        self.neighbor_manager.update_state(
            msg.robot_id,
            [msg.position.x, msg.position.y],
            [msg.velocity.x, msg.velocity.y],
            msg.heading,
        )

    def _on_swarm_event(self, msg: SwarmEvent):
        if msg.robot_id == self.robot_id:
            return
        if msg.event_type == SwarmEvent.LEAVE:
            self.neighbor_manager.remove(msg.robot_id)
            self.get_logger().info(f'{msg.robot_id} left the swarm')
        elif msg.event_type == SwarmEvent.JOIN:
            self.get_logger().info(f'{msg.robot_id} joined the swarm')

    # ------------------------------------------------------------------
    def _control_step(self):
        if not self.have_odom:
            return  # no odometry yet, nothing safe to command

        self._publish_state()

        neighbors = self.neighbor_manager.get_neighbors(self.self_position)
        neighbor_headings = self.neighbor_manager.get_neighbor_headings(self.self_position)

        boids_vector = blended_boids(
            self.self_position, neighbors, self.boids_weights, self.desired_spacing
        )

        neighbor_velocities = [n['velocity'] for n in neighbors]
        consensus_velocity = consensus_step(self.self_velocity, neighbor_velocities, self.consensus_gain)
        # heading consensus is computed for completeness / future use (e.g. broadcasting
        # a smoothed heading) -- the motion controller derives its own steering heading
        # from the desired boids+consensus vector, which is more robust than trusting a
        # slowly-converging consensus heading for immediate collision avoidance.
        _ = consensus_heading_step(self.self_heading, neighbor_headings, self.consensus_gain)

        linear, angular = self.motion_controller.compute_cmd_vel(
            self.self_position, self.self_heading, boids_vector, consensus_velocity, neighbors
        )

        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_vel_pub.publish(cmd)

    def _publish_state(self):
        msg = RobotState()
        msg.robot_id = self.robot_id
        msg.stamp = self.get_clock().now().to_msg()
        msg.position.x, msg.position.y = float(self.self_position[0]), float(self.self_position[1])
        msg.velocity.x, msg.velocity.y = float(self.self_velocity[0]), float(self.self_velocity[1])
        msg.heading = float(self.self_heading)
        msg.comm_radius = float(self.neighbor_radius)
        self.state_pub.publish(msg)

    def _announce_join(self):
        msg = SwarmEvent()
        msg.event_type = SwarmEvent.JOIN
        msg.robot_id = self.robot_id
        self.event_pub.publish(msg)

    def announce_leave(self):
        msg = SwarmEvent()
        msg.event_type = SwarmEvent.LEAVE
        msg.robot_id = self.robot_id
        self.event_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SwarmAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.announce_leave()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
