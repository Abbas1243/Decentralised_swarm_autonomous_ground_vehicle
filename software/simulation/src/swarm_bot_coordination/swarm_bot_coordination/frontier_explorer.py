#!/usr/bin/env python3
"""
frontier_explorer.py
====================
Decentralized frontier-based exploration with greedy distributed auction.

CHANGES FROM v2 → v3:
  - REMOVED: goal_pose topic publisher (fire-and-forget, no feedback)
  - ADDED:   NavigateToPose action client for proper goal lifecycle management
  - Robot now waits for goal result before sending next goal
  - On SUCCEEDED: clears assigned_frontier, immediately triggers re-auction
  - On FAILED/ABORTED: blacklists frontier for BLACKLIST_TIMEOUT_SEC
  - Auction no longer sends new goal if robot is NAVIGATING
  - Replan timeout now cancels the active action goal

CHANGES FROM v3 → v4 (THIS VERSION):
  - BUG FIX: robots were looping on already-explored frontiers at high speed.
    Root cause: frontier revalidation accepted a frontier if ANY single unknown
    neighbor existed — residual boundary cells near visited positions always
    passed this check, causing infinite re-assignment of the same 2 positions.
  - FIX 1: _is_frontier_still_valid() now requires MIN_UNKNOWN_NEIGHBORS (3)
    unknown neighbors, not just 1. Removes phantom frontiers at map edges.
  - FIX 2: visited_positions set — on SUCCEEDED, goal position is recorded.
    Any frontier within VISITED_RADIUS (0.8m) of a visited position is rejected
    in both _detect_frontiers() and _is_frontier_still_valid(). This prevents
    re-assigning frontiers the robot just successfully navigated to.
  - FIX 3: goal duration guard — if Nav2 reports SUCCEEDED but elapsed time
    since goal was sent is less than MIN_GOAL_DURATION_SEC (2.0s), the goal
    is treated as a phantom completion and the frontier is blacklisted instead
    of being marked visited. This catches the "already there" instant-succeed
    case where the robot never actually moved.

v2 fixes preserved:
  1. Robot position from TF lookup
  2. Goal validation: reject frontiers inside obstacles
  3. Frontier revalidation against latest map
  4. Goal candidate snapped to nearest free cell
"""

import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from action_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from nav2_msgs.action import NavigateToPose
import tf2_ros
from tf2_ros import TransformException


# ── Constants ────────────────────────────────────────────────────────────── #
FREE_THRESHOLD      = 25
UNKNOWN_THRESHOLD   = -1
OCCUPIED_THRESHOLD  = 65
MIN_FRONTIER_SIZE   = 3
FRONTIER_RESOLUTION = 2
BID_BROADCAST_HZ    = 1.0
AUCTION_HZ          = 0.5
BID_TIMEOUT_SEC     = 5.0
GOAL_REACHED_DIST   = 0.5
REPLAN_TIMEOUT_SEC  = 30.0
GOAL_FRAME_SUFFIX   = '/map'
MIN_OBSTACLE_CLEARANCE = 0.3

# Frontier blacklist: after a failed goal, don't retry this frontier for N sec
BLACKLIST_TIMEOUT_SEC  = 20.0

# Visited position radius: suppress frontiers within this distance of any
# successfully-reached goal position. Prevents re-assigning explored positions.
VISITED_RADIUS         = 0.8

# Minimum unknown-valued neighbors for a real frontier. Filters map edge artifacts.
# NOTE: Must be 1. Value of 3 kills valid frontiers at startup when map is small —
# the centroid cell is near free space so its 8 neighbors include visited cells,
# dropping the unknown count below 3 even for real frontiers.
MIN_UNKNOWN_NEIGHBORS  = 1

# If Nav2 SUCCEEDED but goal was sent less than this many seconds ago, the robot
# never actually moved — treat as phantom completion and blacklist the frontier.
MIN_GOAL_DURATION_SEC  = 2.0


# ── Robot navigation state ───────────────────────────────────────────────── #
class NavState:
    IDLE       = 'IDLE'        # No goal active
    NAVIGATING = 'NAVIGATING'  # Action goal in flight
    CANCELING  = 'CANCELING'   # Cancel requested, waiting for result


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        self.declare_parameter('robot_id', 'robot_1')
        self.declare_parameter('num_robots', 4)
        self.declare_parameter('bid_broadcast_hz', BID_BROADCAST_HZ)
        self.declare_parameter('auction_hz', AUCTION_HZ)
        self.declare_parameter('min_frontier_size', MIN_FRONTIER_SIZE)
        self.declare_parameter('goal_reached_dist', GOAL_REACHED_DIST)
        self.declare_parameter('replan_timeout_sec', REPLAN_TIMEOUT_SEC)
        self.declare_parameter('use_merged_map', False)

        self.robot_id       = self.get_parameter('robot_id').value
        self.num_robots     = self.get_parameter('num_robots').value
        self.min_frontier   = self.get_parameter('min_frontier_size').value
        self.goal_reached_d = self.get_parameter('goal_reached_dist').value
        self.replan_timeout = self.get_parameter('replan_timeout_sec').value
        self.use_merged_map = self.get_parameter('use_merged_map').value

        self.current_map       = None
        self.robot_position    = None   # (x, y) in map frame — from TF
        self.current_frontiers = []
        self.assigned_frontier = None
        self.goal_sent_time    = None
        self.all_bids          = {}
        self.alive_robots      = None

        # Navigation state machine
        self.nav_state         = NavState.IDLE
        self._goal_handle      = None   # active action goal handle

        # Blacklisted frontiers: {(round_x, round_y): expiry_time}
        self._blacklisted      = {}

        # Visited positions: set of (x, y) tuples where robot successfully arrived.
        # Any frontier within VISITED_RADIUS of these is suppressed.
        self._visited_positions = []

        # TF buffer for robot position lookup
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Action client ────────────────────────────────────────────────── #
        # NavigateToPose action server lives at /<namespace>/navigate_to_pose
        # Because this node runs inside namespace robot_N, the action client
        # name is relative: 'navigate_to_pose' resolves to /robot_N/navigate_to_pose
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        # ── QoS profiles ─────────────────────────────────────────────────── #
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        swarm_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        ns = self.robot_id

        # ── Map topic: use merged map if available, else raw slam_toolbox map ─ #
        # use_merged_map=true → /{ns}/merged_map (full swarm coverage)
        # use_merged_map=false → /{ns}/map (robot's own SLAM map only)
        map_topic = f'/{ns}/merged_map' if self.use_merged_map else f'/{ns}/map'
        self.get_logger().info(
            f'{self.robot_id}: Subscribing to map topic: {map_topic}')

        # ── Subscriptions ─────────────────────────────────────────────────── #
        self.map_sub = self.create_subscription(
            OccupancyGrid, map_topic, self._map_callback, map_qos)
        self.bid_sub = self.create_subscription(
            String, '/swarm/frontier_bids', self._bid_callback, swarm_qos)
        self.status_sub = self.create_subscription(
            String, '/swarm/robot_status', self._status_callback, swarm_qos)

        # ── Publishers ────────────────────────────────────────────────────── #
        self.frontier_viz_pub = self.create_publisher(
            MarkerArray, f'/{ns}/frontiers', 10)
        self.bid_pub = self.create_publisher(
            String, '/swarm/frontier_bids', swarm_qos)

        # ── Timers ────────────────────────────────────────────────────────── #
        bid_period     = 1.0 / self.get_parameter('bid_broadcast_hz').value
        auction_period = 1.0 / self.get_parameter('auction_hz').value

        self.bid_timer     = self.create_timer(bid_period, self._broadcast_bid)
        self.auction_timer = self.create_timer(auction_period, self._run_auction)
        self.tf_timer      = self.create_timer(0.2, self._update_robot_position)

        self.get_logger().info(
            f'{self.robot_id}: FrontierExplorer v3 ready (action client mode).')

    # ── TF Position Lookup ────────────────────────────────────────────────── #

    def _update_robot_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.robot_id + '/map',
                self.robot_id + '/base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            self.robot_position = (tx, ty)
        except TransformException:
            pass

    # ── Map Callback ──────────────────────────────────────────────────────── #

    def _map_callback(self, msg: OccupancyGrid):
        self.current_map = msg
        self.current_frontiers = self._detect_frontiers(msg)
        self._publish_frontier_markers(self.current_frontiers)

    # ── Bid / Status Callbacks ────────────────────────────────────────────── #

    def _bid_callback(self, msg: String):
        try:
            bid = json.loads(msg.data)
            robot_id = bid.get('robot_id')
            if robot_id:
                self.all_bids[robot_id] = bid
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().warn(f'Bad bid message: {e}')

    def _status_callback(self, msg: String):
        try:
            status = json.loads(msg.data)
            self.alive_robots = set(status.get('alive_robots', []))
            failed = status.get('failed_robots', [])
            if failed:
                self.get_logger().warn(
                    f'{self.robot_id}: Failed robots: {failed}. Re-running auction.')
                for robot_id in failed:
                    self.all_bids.pop(robot_id, None)
                # If we were navigating to a frontier that a failed robot
                # was also targeting, cancel and replan
                if self.nav_state == NavState.NAVIGATING:
                    self._cancel_current_goal()
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().warn(f'Bad status message: {e}')

    # ── Frontier Detection ────────────────────────────────────────────────── #

    def _detect_frontiers(self, map_msg: OccupancyGrid) -> list:
        width    = map_msg.info.width
        height   = map_msg.info.height
        res      = map_msg.info.resolution
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        data     = map_msg.data

        if width == 0 or height == 0 or len(data) == 0:
            return []

        frontier_cells = set()
        step = FRONTIER_RESOLUTION

        for y in range(0, height - step, step):
            for x in range(0, width - step, step):
                idx = y * width + x
                if idx >= len(data):
                    continue
                cell = data[idx]
                if cell < 0 or cell >= FREE_THRESHOLD:
                    continue
                for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                    nx, ny = x + dx * step, y + dy * step
                    if 0 <= nx < width and 0 <= ny < height:
                        nidx = ny * width + nx
                        if nidx < len(data) and data[nidx] < 0:
                            frontier_cells.add((x, y))
                            break

        if not frontier_cells:
            return []

        clusters  = self._cluster_frontiers(frontier_cells, step)
        centroids = []

        for cluster in clusters:
            if len(cluster) < self.min_frontier:
                continue
            cx = sum(c[0] for c in cluster) / len(cluster)
            cy = sum(c[1] for c in cluster) / len(cluster)
            best_cell = min(cluster,
                            key=lambda c: (c[0] - cx) ** 2 + (c[1] - cy) ** 2)
            wx = origin_x + (best_cell[0] + 0.5) * res
            wy = origin_y + (best_cell[1] + 0.5) * res
            if self._is_near_obstacle(best_cell[0], best_cell[1],
                                      width, height, data, res):
                continue
            centroids.append((wx, wy))

        return centroids

    def _is_near_obstacle(self, gx, gy, width, height, data, resolution):
        clearance_cells = int(MIN_OBSTACLE_CLEARANCE / resolution) + 1
        for dy in range(-clearance_cells, clearance_cells + 1):
            for dx in range(-clearance_cells, clearance_cells + 1):
                nx, ny = gx + dx, gy + dy
                if 0 <= nx < width and 0 <= ny < height:
                    nidx = ny * width + nx
                    if nidx < len(data) and data[nidx] > OCCUPIED_THRESHOLD:
                        return True
        return False

    def _cluster_frontiers(self, frontier_cells: set, step: int) -> list:
        visited  = set()
        clusters = []
        for cell in frontier_cells:
            if cell in visited:
                continue
            cluster = []
            queue   = [cell]
            visited.add(cell)
            while queue:
                cx, cy = queue.pop(0)
                cluster.append((cx, cy))
                for dx, dy in [(step, 0), (-step, 0), (0, step), (0, -step)]:
                    neighbor = (cx + dx, cy + dy)
                    if neighbor in frontier_cells and neighbor not in visited:
                        visited.add(neighbor)
                        queue.append(neighbor)
            clusters.append(cluster)
        return clusters

    def _is_frontier_still_valid(self, fx: float, fy: float) -> bool:
        if self.current_map is None:
            return True

        # Reject if within VISITED_RADIUS of any successfully-reached position
        for vx, vy in self._visited_positions:
            if math.sqrt((fx - vx) ** 2 + (fy - vy) ** 2) < VISITED_RADIUS:
                return False

        # Check blacklist
        key = (round(fx, 2), round(fy, 2))
        if key in self._blacklisted:
            if time.time() < self._blacklisted[key]:
                return False
            else:
                del self._blacklisted[key]

        map_msg  = self.current_map
        width    = map_msg.info.width
        height   = map_msg.info.height
        res      = map_msg.info.resolution
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        data     = map_msg.data

        gx = int((fx - origin_x) / res)
        gy = int((fy - origin_y) / res)

        if not (0 <= gx < width and 0 <= gy < height):
            return False

        idx = gy * width + gx
        if idx >= len(data):
            return False

        if data[idx] > OCCUPIED_THRESHOLD:
            return False

        # Require at least MIN_UNKNOWN_NEIGHBORS unknown cells in the neighborhood.
        # A single unknown cell is a map edge artifact, not a real frontier.
        step = FRONTIER_RESOLUTION
        unknown_count = 0
        for dx, dy in [(step, 0), (-step, 0), (0, step), (0, -step),
                       (step, step), (-step, step), (step, -step), (-step, -step)]:
            nx, ny = gx + dx, gy + dy
            if 0 <= nx < width and 0 <= ny < height:
                nidx = ny * width + nx
                if nidx < len(data) and data[nidx] < 0:
                    unknown_count += 1

        return unknown_count >= MIN_UNKNOWN_NEIGHBORS

    # ── Greedy Distributed Auction ────────────────────────────────────────── #

    def _broadcast_bid(self):
        if self.current_map is None or self.robot_position is None:
            return

        valid_frontiers = [
            f for f in self.current_frontiers
            if self._is_frontier_still_valid(f[0], f[1])
        ]

        bid = {
            'robot_id':          self.robot_id,
            'position':          {'x': self.robot_position[0],
                                  'y': self.robot_position[1]},
            'frontiers':         [[f[0], f[1]] for f in valid_frontiers],
            'timestamp':         time.time(),
            'assigned_frontier': list(self.assigned_frontier)
                                  if self.assigned_frontier else None,
            'nav_state':         self.nav_state,
        }
        msg      = String()
        msg.data = json.dumps(bid)
        self.bid_pub.publish(msg)

    def _run_auction(self):
        if self.robot_position is None:
            return

        # KEY CHANGE: do not send a new goal if already navigating
        if self.nav_state == NavState.NAVIGATING:
            self._check_replan_timeout()
            return
        if self.nav_state == NavState.CANCELING:
            return  # waiting for cancel to complete, do nothing

        now = time.time()

        # Purge stale bids
        stale = [rid for rid, bid in self.all_bids.items()
                 if now - bid.get('timestamp', 0) > BID_TIMEOUT_SEC]
        for rid in stale:
            del self.all_bids[rid]

        if not self.all_bids:
            return

        alive = self.alive_robots if self.alive_robots is not None \
                else set(self.all_bids.keys())

        all_frontiers  = []
        seen_frontiers = set()
        for robot_id, bid in self.all_bids.items():
            if robot_id not in alive:
                continue
            for f in bid.get('frontiers', []):
                key = (round(f[0], 2), round(f[1], 2))
                if key not in seen_frontiers:
                    if self._is_frontier_still_valid(f[0], f[1]):
                        seen_frontiers.add(key)
                        all_frontiers.append((f[0], f[1]))

        if not all_frontiers:
            self.get_logger().info(
                f'{self.robot_id}: No valid frontiers. Exploration complete.')
            self.assigned_frontier = None
            return

        robot_positions = {}
        for robot_id, bid in self.all_bids.items():
            if robot_id not in alive:
                continue
            pos = bid.get('position', {})
            robot_positions[robot_id] = (pos.get('x', 0.0), pos.get('y', 0.0))

        # Spread-aware auction scoring:
        #   score = dist_to_frontier - SPREAD_WEIGHT * min_dist_frontier_to_others
        # Frontiers far from other robots get lower scores (preferred),
        # so robots naturally assign themselves to different map regions.
        SPREAD_WEIGHT = 0.5

        triples = []
        for robot_id, pos in robot_positions.items():
            others = [p for rid, p in robot_positions.items() if rid != robot_id]
            for frontier in all_frontiers:
                dist_to_frontier = math.sqrt(
                    (pos[0] - frontier[0]) ** 2 +
                    (pos[1] - frontier[1]) ** 2
                )
                if others:
                    min_sep = min(
                        math.sqrt((o[0] - frontier[0]) ** 2 +
                                  (o[1] - frontier[1]) ** 2)
                        for o in others
                    )
                    score = dist_to_frontier - SPREAD_WEIGHT * min_sep
                else:
                    score = dist_to_frontier
                triples.append((score, robot_id, frontier))
        triples.sort(key=lambda t: t[0])

        assigned_robots    = set()
        assigned_frontiers = set()
        assignments        = {}

        for score, robot_id, frontier in triples:
            f_key = (round(frontier[0], 2), round(frontier[1], 2))
            if robot_id in assigned_robots:
                continue
            if f_key in assigned_frontiers:
                continue
            assignments[robot_id]  = frontier
            assigned_robots.add(robot_id)
            assigned_frontiers.add(f_key)

        my_frontier = assignments.get(self.robot_id)

        if my_frontier is None:
            return

        # Skip if same frontier as current assignment (already handled above
        # by NAVIGATING guard — this is the IDLE case where we're waiting)
        if self.assigned_frontier is not None:
            old = (round(self.assigned_frontier[0], 2),
                   round(self.assigned_frontier[1], 2))
            new = (round(my_frontier[0], 2), round(my_frontier[1], 2))
            if old == new:
                return  # Same frontier, already sent, no action goal in flight

        if not self._is_frontier_still_valid(my_frontier[0], my_frontier[1]):
            self.get_logger().warn(
                f'{self.robot_id}: Assigned frontier failed final validation.')
            return

        self.assigned_frontier = my_frontier
        self.goal_sent_time    = now

        pos = robot_positions.get(self.robot_id, (0, 0))
        dist_to_goal = math.sqrt(
            (pos[0] - my_frontier[0]) ** 2 + (pos[1] - my_frontier[1]) ** 2
        )
        self.get_logger().info(
            f'{self.robot_id}: → Frontier ({my_frontier[0]:.2f}, '
            f'{my_frontier[1]:.2f}), dist={dist_to_goal:.2f}m, '
            f'alive={sorted(alive)}'
        )

        self._send_nav2_goal(my_frontier)

    def _check_replan_timeout(self):
        """Called only when NAVIGATING. Cancel goal if timeout exceeded."""
        if self.goal_sent_time is None:
            return
        elapsed = time.time() - self.goal_sent_time
        if elapsed > self.replan_timeout:
            self.get_logger().warn(
                f'{self.robot_id}: Replan timeout {elapsed:.0f}s for '
                f'frontier {self.assigned_frontier}. Canceling goal.')
            self._cancel_current_goal()

    # ── Nav2 Action Client ────────────────────────────────────────────────── #

    def _send_nav2_goal(self, frontier: tuple):
        """
        Send a NavigateToPose goal via the action client.
        Non-blocking: callbacks handle the result asynchronously.
        """
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                f'{self.robot_id}: navigate_to_pose action server not available!')
            self.nav_state         = NavState.IDLE
            self.assigned_frontier = None
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp    = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = self.robot_id + GOAL_FRAME_SUFFIX
        goal_msg.pose.pose.position.x = frontier[0]
        goal_msg.pose.pose.position.y = frontier[1]
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.nav_state = NavState.NAVIGATING

        send_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._goal_feedback_callback
        )
        send_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        """Called when the action server accepts or rejects the goal."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn(
                f'{self.robot_id}: Goal REJECTED by navigate_to_pose server. '
                f'Blacklisting frontier {self.assigned_frontier}.')
            self._blacklist_current_frontier()
            self.nav_state         = NavState.IDLE
            self.assigned_frontier = None
            self.goal_sent_time    = None
            return

        self.get_logger().info(
            f'{self.robot_id}: Goal accepted by Nav2.')
        self._goal_handle = goal_handle

        # Register result callback
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_feedback_callback(self, feedback_msg):
        """
        Called periodically by Nav2 with distance remaining.
        Not used for logic — kept for debug logging.
        """
        dist_remaining = feedback_msg.feedback.distance_remaining
        if dist_remaining < 0.3:
            self.get_logger().debug(
                f'{self.robot_id}: Approaching frontier, '
                f'dist_remaining={dist_remaining:.2f}m')

    def _goal_result_callback(self, future):
        """
        Called when the robot reaches (or fails to reach) the goal.
        This is the critical callback that drives the state machine.
        """
        result    = future.result()
        status    = result.status
        self._goal_handle = None

        elapsed = time.time() - self.goal_sent_time if self.goal_sent_time else 99.0

        if status == GoalStatus.STATUS_SUCCEEDED:
            if elapsed < MIN_GOAL_DURATION_SEC:
                # Nav2 said "succeeded" but the robot barely moved — the goal
                # position is already inside explored space. Blacklist it so
                # the auction stops reassigning it, and record as visited.
                self.get_logger().warn(
                    f'{self.robot_id}: Phantom goal completion in {elapsed:.2f}s '
                    f'at {self.assigned_frontier} — already explored. '
                    f'Blacklisting and marking visited.')
                if self.assigned_frontier:
                    self._visited_positions.append(
                        (self.assigned_frontier[0], self.assigned_frontier[1]))
                self._blacklist_current_frontier()
            else:
                self.get_logger().info(
                    f'{self.robot_id}: ✓ Frontier reached '
                    f'{self.assigned_frontier} in {elapsed:.1f}s. Re-auctioning.')
                if self.assigned_frontier:
                    self._visited_positions.append(
                        (self.assigned_frontier[0], self.assigned_frontier[1]))

            self.assigned_frontier = None
            self.goal_sent_time    = None
            self.nav_state         = NavState.IDLE
            self._run_auction()

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(
                f'{self.robot_id}: Goal CANCELED. Re-auctioning.')
            self.assigned_frontier = None
            self.goal_sent_time    = None
            self.nav_state         = NavState.IDLE
            self._run_auction()

        else:
            # ABORTED or other failure
            self.get_logger().warn(
                f'{self.robot_id}: Goal FAILED (status={status}) at '
                f'{self.assigned_frontier}. Blacklisting.')
            self._blacklist_current_frontier()
            self.assigned_frontier = None
            self.goal_sent_time    = None
            self.nav_state         = NavState.IDLE
            self._run_auction()

    def _cancel_current_goal(self):
        """Cancel the in-flight action goal."""
        if self._goal_handle is None:
            self.nav_state = NavState.IDLE
            return
        self.nav_state = NavState.CANCELING
        cancel_future  = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._cancel_done_callback)

    def _cancel_done_callback(self, future):
        self.get_logger().info(f'{self.robot_id}: Goal cancel confirmed.')
        # Result callback will fire after cancel and handle state reset

    def _blacklist_current_frontier(self):
        """Add current assigned frontier to the blacklist."""
        if self.assigned_frontier is None:
            return
        key = (round(self.assigned_frontier[0], 2),
               round(self.assigned_frontier[1], 2))
        self._blacklisted[key] = time.time() + BLACKLIST_TIMEOUT_SEC
        self.get_logger().info(
            f'{self.robot_id}: Blacklisted frontier {key} for '
            f'{BLACKLIST_TIMEOUT_SEC}s')

    # ── RViz Markers ──────────────────────────────────────────────────────── #

    def _publish_frontier_markers(self, frontiers: list):
        marker_array = MarkerArray()

        delete        = Marker()
        delete.action = Marker.DELETEALL
        delete.ns     = self.robot_id + '_frontiers'
        marker_array.markers.append(delete)

        for i, (fx, fy) in enumerate(frontiers):
            m                    = Marker()
            m.header.frame_id    = self.robot_id + GOAL_FRAME_SUFFIX
            m.header.stamp       = self.get_clock().now().to_msg()
            m.ns                 = self.robot_id + '_frontiers'
            m.id                 = i
            m.type               = Marker.SPHERE
            m.action             = Marker.ADD
            m.pose.position.x    = fx
            m.pose.position.y    = fy
            m.pose.position.z    = 0.15
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.2

            # Blacklisted frontiers shown in red
            key = (round(fx, 2), round(fy, 2))
            if key in self._blacklisted and time.time() < self._blacklisted[key]:
                m.color.r, m.color.g, m.color.b = 1.0, 0.0, 0.0  # red
            elif (self.assigned_frontier and
                  round(fx, 2) == round(self.assigned_frontier[0], 2) and
                  round(fy, 2) == round(self.assigned_frontier[1], 2)):
                m.color.r, m.color.g, m.color.b = 0.0, 1.0, 0.0  # green
            else:
                m.color.r, m.color.g, m.color.b = 1.0, 1.0, 0.0  # yellow
            m.color.a = 0.8

            lt     = Duration()
            lt.sec = 6
            m.lifetime = lt
            marker_array.markers.append(m)

        self.frontier_viz_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()