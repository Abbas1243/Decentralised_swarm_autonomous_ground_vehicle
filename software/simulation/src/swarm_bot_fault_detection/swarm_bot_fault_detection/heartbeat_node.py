#!/usr/bin/env python3
"""
heartbeat_node.py
=================
Decentralized fault detection for swarm robots via heartbeat monitoring.

ARCHITECTURE:
  Each robot runs one instance of this node. Every instance:
    1. Broadcasts its own heartbeat on /swarm/heartbeat at 1Hz
    2. Monitors heartbeats from ALL other robots independently
    3. Declares a robot FAILED if it misses MISS_LIMIT consecutive heartbeats
    4. Publishes updated alive/failed lists to /swarm/robot_status

  There is NO central monitor. Every robot monitors every other robot.
  If this node itself crashes, the other robots still detect faults.
  If any robot dies, this node detects it within MISS_LIMIT seconds.

INTERFACE (matches frontier_explorer.py _status_callback):
  Publishes to:  /swarm/robot_status  (std_msgs/String JSON)
  Subscribes to: /swarm/heartbeat     (std_msgs/String JSON)
  Publishes to:  /swarm/heartbeat     (std_msgs/String JSON)

  Heartbeat message format:
    {"robot_id": "robot_1", "timestamp": 1234567890.123, "seq": 42}

  Status message format (consumed by frontier_explorer._status_callback):
    {"alive_robots": ["robot_1", "robot_2"], "failed_robots": ["robot_3"]}

FAULT INJECTION (for testing):
  Kill any robot process — this node detects it within 3 seconds.
  Or use the ROS 2 service:
    ros2 service call /robot_1/fault_detection/inject_fault std_srvs/srv/SetBool \
      "{data: true}"
  This simulates robot_1 going silent (stops its own heartbeat broadcasts).
  Used for Phase 1 integration test.

THESIS METRICS THIS NODE ENABLES:
  - Fault detection latency: time from robot death to status update
    Target: <5s, Stretch: <3s
  - False positive rate: healthy robots incorrectly declared failed
    Target: <5%, Stretch: <2%
  - System availability: coverage % after 1 robot fails
    Target: >70%
"""

import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool


# ── Constants ────────────────────────────────────────────────────────────── #

# Heartbeat broadcast rate (Hz). 1Hz = 1 beat/second.
HEARTBEAT_HZ = 1.0

# Number of consecutive missed heartbeats before declaring robot failed.
# At 1Hz: 3 misses = 3 second detection window.
# Thesis target: <5s detection latency. 3s gives margin.
MISS_LIMIT = 3

# How often to check for missed heartbeats and publish status (Hz).
MONITOR_HZ = 2.0

# How often to publish /swarm/robot_status even if nothing changed (Hz).
# Ensures frontier_explorer always has fresh status after it starts up.
STATUS_PUBLISH_HZ = 1.0

# A robot that has been silent for this many seconds is definitely dead,
# not just slow. Used to prune robots from tracking entirely.
PRUNE_TIMEOUT_SEC = 30.0


class HeartbeatNode(Node):

    def __init__(self):
        super().__init__('heartbeat_node')

        self.declare_parameter('robot_id', 'robot_1')
        self.declare_parameter('num_robots', 4)
        self.declare_parameter('heartbeat_hz', HEARTBEAT_HZ)
        self.declare_parameter('miss_limit', MISS_LIMIT)
        # use_sim_time must be False for this node.
        # Heartbeat is a real-time watchdog — sim time pauses cause false faults.

        self.robot_id   = self.get_parameter('robot_id').value
        self.num_robots = self.get_parameter('num_robots').value
        self.miss_limit = self.get_parameter('miss_limit').value

        # Heartbeat tracking per robot:
        # {robot_id: {'last_seen': float, 'miss_count': int, 'failed': bool}}
        self._peers = {}

        # Sequence counter for own heartbeat
        self._seq = 0

        # Fault injection flag — when True, this robot stops broadcasting
        # (simulates a silent failure for integration testing)
        self._injected_fault = False

        # ── QoS ──────────────────────────────────────────────────────────── #
        swarm_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # ── Publishers ────────────────────────────────────────────────────── #
        self._heartbeat_pub = self.create_publisher(
            String, '/swarm/heartbeat', swarm_qos)
        self._status_pub = self.create_publisher(
            String, '/swarm/robot_status', reliable_qos)

        # ── Subscriptions ─────────────────────────────────────────────────── #
        self._heartbeat_sub = self.create_subscription(
            String, '/swarm/heartbeat',
            self._heartbeat_callback, swarm_qos)

        # ── Fault injection service ───────────────────────────────────────── #
        # ros2 service call /robot_1/fault_detection/inject_fault
        #   std_srvs/srv/SetBool "{data: true}"   ← inject fault (go silent)
        #   std_srvs/srv/SetBool "{data: false}"  ← recover (resume heartbeat)
        self._inject_srv = self.create_service(
            SetBool,
            'fault_detection/inject_fault',
            self._inject_fault_callback
        )

        # ── Timers ────────────────────────────────────────────────────────── #
        hb_period     = 1.0 / self.get_parameter('heartbeat_hz').value
        monitor_period = 1.0 / MONITOR_HZ
        status_period  = 1.0 / STATUS_PUBLISH_HZ

        self._hb_timer      = self.create_timer(hb_period, self._broadcast_heartbeat)
        self._monitor_timer = self.create_timer(monitor_period, self._monitor_peers)
        self._status_timer  = self.create_timer(status_period, self._publish_status)

        self.get_logger().info(
            f'{self.robot_id}: HeartbeatNode started. '
            f'Monitoring up to {self.num_robots} robots. '
            f'Miss limit: {self.miss_limit} ({self.miss_limit}s detection window).')

    # ── Heartbeat broadcast ───────────────────────────────────────────────── #

    def _broadcast_heartbeat(self):
        """
        Broadcast own heartbeat to all robots.
        Suppressed when fault is injected (simulates silent failure).
        """
        if self._injected_fault:
            return  # Silent — other robots will detect this as a fault

        self._seq += 1
        beat = {
            'robot_id':  self.robot_id,
            'timestamp': time.time(),
            'seq':       self._seq,
        }
        msg      = String()
        msg.data = json.dumps(beat)
        self._heartbeat_pub.publish(msg)

    # ── Heartbeat reception ───────────────────────────────────────────────── #

    def _heartbeat_callback(self, msg: String):
        """
        Receive heartbeat from any robot (including self — ignored).
        Record last-seen time and reset miss counter.
        """
        try:
            beat     = json.loads(msg.data)
            robot_id = beat.get('robot_id')

            if robot_id is None or robot_id == self.robot_id:
                return  # Ignore malformed or own heartbeat

            now = time.time()

            if robot_id not in self._peers:
                # First time seeing this robot
                self._peers[robot_id] = {
                    'last_seen':  now,
                    'miss_count': 0,
                    'failed':     False,
                }
                self.get_logger().info(
                    f'{self.robot_id}: Discovered peer {robot_id}.')
            else:
                peer = self._peers[robot_id]

                # If robot was previously declared failed but is now back
                if peer['failed']:
                    self.get_logger().warn(
                        f'{self.robot_id}: {robot_id} has RECOVERED '
                        f'(was declared failed). Marking alive again.')
                    peer['failed'] = False

                peer['last_seen']  = now
                peer['miss_count'] = 0

        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().warn(f'Bad heartbeat message: {e}')

    # ── Peer monitoring ───────────────────────────────────────────────────── #

    def _monitor_peers(self):
        """
        Check all known peers for missed heartbeats.
        Called at MONITOR_HZ (2Hz) for responsive detection.

        A peer is declared FAILED when miss_count >= MISS_LIMIT.
        miss_count increments every monitor cycle where last_seen is
        older than (1.0 / HEARTBEAT_HZ) seconds = 1 full heartbeat period.
        """
        now              = time.time()
        heartbeat_period = 1.0 / HEARTBEAT_HZ
        newly_failed     = []

        for robot_id, peer in list(self._peers.items()):
            if peer['failed']:
                # Prune robots that have been dead for a long time
                if now - peer['last_seen'] > PRUNE_TIMEOUT_SEC:
                    self.get_logger().info(
                        f'{self.robot_id}: Pruning {robot_id} from peer list '
                        f'(silent for {PRUNE_TIMEOUT_SEC}s).')
                    del self._peers[robot_id]
                continue

            elapsed = now - peer['last_seen']

            if elapsed > heartbeat_period:
                peer['miss_count'] += 1
                self.get_logger().debug(
                    f'{self.robot_id}: {robot_id} miss_count={peer["miss_count"]} '
                    f'(elapsed={elapsed:.2f}s)')

                if peer['miss_count'] >= self.miss_limit:
                    peer['failed'] = True
                    newly_failed.append(robot_id)
                    self.get_logger().error(
                        f'{self.robot_id}: *** FAULT DETECTED *** '
                        f'{robot_id} missed {peer["miss_count"]} heartbeats. '
                        f'Last seen {elapsed:.1f}s ago. Declaring FAILED.')
            else:
                # Reset miss count if heartbeat arrived within window
                if peer['miss_count'] > 0:
                    peer['miss_count'] = 0

        if newly_failed:
            # Publish status immediately on new fault — don't wait for timer
            self._publish_status()

    # ── Status publication ────────────────────────────────────────────────── #

    def _publish_status(self):
        """
        Publish current alive/failed robot lists to /swarm/robot_status.
        The frontier_explorer._status_callback consumes this message and
        removes failed robots from the auction, triggering task reallocation.

        This robot always includes itself in alive_robots (if not injected).
        """
        alive  = [self.robot_id] if not self._injected_fault else []
        failed = []

        for robot_id, peer in self._peers.items():
            if peer['failed']:
                failed.append(robot_id)
            else:
                alive.append(robot_id)

        status = {
            'alive_robots':  sorted(alive),
            'failed_robots': sorted(failed),
            'reported_by':   self.robot_id,
            'timestamp':     time.time(),
        }
        msg      = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)

    # ── Fault injection service ───────────────────────────────────────────── #

    def _inject_fault_callback(self, request, response):
        """
        Service handler for fault injection.
        data=True:  inject fault (robot goes silent, stops heartbeat)
        data=False: recover (resume heartbeat)

        Usage for Phase 1 integration test:
          ros2 service call /robot_2/fault_detection/inject_fault \
            std_srvs/srv/SetBool "{data: true}"

        Other robots will detect robot_2 as failed within 3 seconds.
        Watch /swarm/robot_status and frontier_explorer logs for reallocation.
        """
        self._injected_fault = request.data

        if request.data:
            self.get_logger().warn(
                f'{self.robot_id}: FAULT INJECTED — heartbeat suppressed. '
                f'Other robots will detect failure in ~{self.miss_limit}s.')
        else:
            self.get_logger().info(
                f'{self.robot_id}: Fault injection cleared — resuming heartbeat.')

        response.success = True
        response.message = (
            f'Fault injected on {self.robot_id}'
            if request.data
            else f'Fault cleared on {self.robot_id}'
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()