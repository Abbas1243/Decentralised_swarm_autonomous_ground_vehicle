#!/usr/bin/env python3
"""
map_merger_node.py
==================
Decentralized occupancy grid fusion for swarm robots.

WHAT THIS DOES:
  Each robot runs one instance of this node. Every instance:
    1. Subscribes to ALL robots' /robot_N/map topics
    2. Transforms each map into a common world frame using known spawn offsets
    3. Merges all maps into one unified OccupancyGrid using cell-by-cell fusion
    4. Publishes the merged map on /robot_N/merged_map
    5. Feeds the merged map into the robot's global costmap for better planning

  There is NO central map server. Every robot independently maintains
  its own copy of the unified map. If any robot fails, the others keep
  their merged maps intact.

WHY FIXED OFFSET INSTEAD OF TF:
  Each robot's slam_toolbox builds its map starting from its own spawn
  position as the origin. Robot_1 spawns at (0,0), Robot_2 at (2,0).
  Their map frames (robot_1/map, robot_2/map) are offset by exactly
  (2.0, 0.0) meters in world coordinates.

  In simulation with known spawn positions this offset is exact and
  deterministic. We precompute the pixel offset and apply it during
  grid overlay. No TF lookup needed.

  Phase 2 note: On real hardware, odometry drift means spawn offsets
  become inaccurate over time. Replace with ICP-based alignment using
  the overlapping region of the two maps. The merger logic stays identical
  — only _compute_offset() changes.

CELL FUSION RULES (per cell, taking all robots' values):
  - Any robot sees OCCUPIED (>65):   merged = OCCUPIED (100)
  - All robots agree FREE (<25):     merged = FREE (0)
  - Otherwise (mix of free/unknown): merged = UNKNOWN (-1)
  This is conservative — if any robot sees an obstacle, it's an obstacle.
  Unknown space is only cleared when multiple robots agree it's free.

COORDINATE TRANSFORM:
  Robot_N's grid cell (gx, gy) maps to world position:
    wx = origin_x + (gx + 0.5) * resolution
    wy = origin_y + (gy + 0.5) * resolution
  Where origin_x/y comes from the map's OccupancyGrid.info.origin.

  To overlay Robot_N's map onto the merged canvas:
    canvas_gx = gx + round((robot_origin_x - canvas_origin_x) / resolution)
    canvas_gy = gy + round((robot_origin_y - canvas_origin_y) / resolution)

MERGED MAP FRAME:
  Published in 'map' frame (global world frame).
  Origin at (-1.0, -1.0) — slightly larger than the 10x10m room to
  capture walls. Canvas size: 240x240 cells at 0.05m = 12x12m.

TOPICS:
  Subscribes: /robot_N/map (nav_msgs/OccupancyGrid) for all N robots
  Publishes:  /robot_N/merged_map (nav_msgs/OccupancyGrid)
              /merged_map (nav_msgs/OccupancyGrid) — global view for RViz

FRONTIER EXPLORER INTEGRATION:
  The frontier_explorer subscribes to /{ns}/map. To make it use the
  merged map instead, remap in the launch file:
    remappings=[('/{ns}/map', '/{ns}/merged_map')]
  This gives each robot's frontier detection full swarm map awareness —
  robots stop re-exploring areas already covered by other robots.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import time


# ── Canvas configuration ─────────────────────────────────────────────────── #
# slam_toolbox centers the map around the robot's starting position.
# With a 10x10m room, robots spawning near (0,0) and (2,0), slam_toolbox
# places map origins at approximately (-5, -5) in world coordinates.
# Canvas must fully contain all robots' maps with margin.
#
# Observed origins:
#   robot_1: (-4.97, -4.97)  map size 199x199 → covers (-4.97 to +5.03)
#   robot_2: (-2.24, -4.97)  map size 199x199 → covers (-2.24 to +7.71)
#
# Canvas spans -7.0m to +9.0m in X, -7.0m to +7.0m in Y → 320x280 cells
# This gives margin for all 4 robots' maps regardless of drift.
CANVAS_ORIGIN_X   = -7.0    # meters — world frame
CANVAS_ORIGIN_Y   = -7.0    # meters — world frame
CANVAS_WIDTH      = 320     # cells  (16.0m / 0.05m)
CANVAS_HEIGHT     = 280     # cells  (14.0m / 0.05m)
CANVAS_RESOLUTION = 0.05    # meters per cell

# Cell value thresholds (must match slam_toolbox output)
FREE_THRESHOLD     = 25
OCCUPIED_THRESHOLD = 65
UNKNOWN_VALUE      = -1

# How often to publish the merged map (seconds).
# slam_toolbox publishes every 5s — no point merging faster than source.
MERGE_PUBLISH_HZ = 0.2   # every 5 seconds

# Spawn positions for each robot in world frame (meters).
# Must match ROBOT_SPAWN_POSITIONS in swarm_bringup.launch.py exactly.
SPAWN_POSITIONS = {
    'robot_1': (0.0, 0.0),
    'robot_2': (2.0, 0.0),
    'robot_3': (0.0, 2.0),
    'robot_4': (2.0, 2.0),
}


class MapMergerNode(Node):

    def __init__(self):
        super().__init__('map_merger_node')

        self.declare_parameter('robot_id',   'robot_1')
        self.declare_parameter('num_robots', 4)

        self.robot_id   = self.get_parameter('robot_id').value
        self.num_robots = self.get_parameter('num_robots').value

        # Latest map from each robot: {robot_id: OccupancyGrid}
        self._maps = {}

        # Pre-allocated numpy canvas — reused every merge cycle
        # dtype int16 to hold -1 (unknown), 0-100 (occupancy)
        self._canvas = np.full(
            (CANVAS_HEIGHT, CANVAS_WIDTH), UNKNOWN_VALUE, dtype=np.int16)

        # ── QoS ──────────────────────────────────────────────────────────── #
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # ── Subscriptions — one per robot ─────────────────────────────────── #
        for i in range(1, self.num_robots + 1):
            ns = f'robot_{i}'
            self.create_subscription(
                OccupancyGrid,
                f'/{ns}/map',
                lambda msg, r=ns: self._map_callback(msg, r),
                map_qos
            )
            self.get_logger().info(
                f'{self.robot_id}: Subscribed to /{ns}/map')

        # ── Publishers ────────────────────────────────────────────────────── #
        # Per-robot merged map — published in THIS robot's own map frame
        # so the frontier explorer can subscribe without frame_id mismatch.
        # frame_id = robot_N/map matches what frontier explorer expects.
        self._merged_pub = self.create_publisher(
            OccupancyGrid,
            f'/{self.robot_id}/merged_map',
            map_qos
        )
        # Global merged map in robot_1/map frame — for RViz visualization only
        self._global_pub = self.create_publisher(
            OccupancyGrid,
            '/merged_map',
            map_qos
        )

        # ── Merge timer ───────────────────────────────────────────────────── #
        self._merge_timer = self.create_timer(
            1.0 / MERGE_PUBLISH_HZ, self._merge_and_publish)

        self.get_logger().info(
            f'{self.robot_id}: MapMerger started. '
            f'Watching {self.num_robots} robots. '
            f'Canvas: {CANVAS_WIDTH}x{CANVAS_HEIGHT} cells '
            f'({CANVAS_WIDTH * CANVAS_RESOLUTION:.1f}x'
            f'{CANVAS_HEIGHT * CANVAS_RESOLUTION:.1f}m) '
            f'origin=({CANVAS_ORIGIN_X},{CANVAS_ORIGIN_Y})')

    # ── Map reception ─────────────────────────────────────────────────────── #

    def _map_callback(self, msg: OccupancyGrid, robot_id: str):
        """Store latest map from each robot."""
        self._maps[robot_id] = msg
        self.get_logger().debug(
            f'{self.robot_id}: Received map from {robot_id} '
            f'({msg.info.width}x{msg.info.height})')

    # ── Merge logic ───────────────────────────────────────────────────────── #

    def _merge_and_publish(self):
        """
        Merge all received maps into one canvas and publish.
        Called at MERGE_PUBLISH_HZ (every 5 seconds).

        Algorithm:
          1. Reset canvas to UNKNOWN
          2. For each robot's map:
             a. Compute pixel offset from map origin to canvas origin
             b. Copy cells into canvas with fusion rules
          3. Publish canvas as OccupancyGrid
        """
        if not self._maps:
            return  # No maps received yet

        t_start = time.time()

        # Reset canvas to unknown
        self._canvas[:] = UNKNOWN_VALUE

        # Layer count canvas — tracks how many robots have data for each cell
        # Used for FREE consensus (only mark free if robot has data there)
        layer_count = np.zeros(
            (CANVAS_HEIGHT, CANVAS_WIDTH), dtype=np.int8)

        # Free vote canvas — tracks how many robots see a cell as free
        free_votes = np.zeros(
            (CANVAS_HEIGHT, CANVAS_WIDTH), dtype=np.int8)

        for robot_id, map_msg in self._maps.items():
            self._overlay_map(map_msg, robot_id, layer_count, free_votes)

        # Apply fusion rules:
        #   Occupied: set by _overlay_map directly (any robot sees occupied)
        #   Free: all robots with data agree it's free
        #   Unknown: canvas stays UNKNOWN_VALUE
        free_mask = (layer_count > 0) & (free_votes == layer_count) & \
                    (self._canvas == UNKNOWN_VALUE)
        self._canvas[free_mask] = 0

        elapsed = time.time() - t_start
        self.get_logger().debug(
            f'{self.robot_id}: Merge completed in {elapsed*1000:.1f}ms, '
            f'{len(self._maps)} maps fused.')

        self._publish_canvas()

    def _overlay_map(self, map_msg: OccupancyGrid, robot_id: str,
                     layer_count: np.ndarray, free_votes: np.ndarray):
        """
        Overlay one robot's map onto the canvas.

        Coordinate transform:
          map cell (gx, gy) → world (wx, wy) → canvas cell (cx, cy)
          wx = map_origin_x + (gx + 0.5) * resolution
          cx = round((wx - CANVAS_ORIGIN_X) / CANVAS_RESOLUTION)
             = gx + round((map_origin_x - CANVAS_ORIGIN_X) / CANVAS_RESOLUTION)

        The offset is constant for a given map message (fixed origin).
        """
        w        = map_msg.info.width
        h        = map_msg.info.height
        res      = map_msg.info.resolution
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        data     = map_msg.data

        if w == 0 or h == 0 or len(data) == 0:
            return

        # Pixel offset: how many canvas cells to shift this map's origin
        offset_x = int(round((origin_x - CANVAS_ORIGIN_X) / CANVAS_RESOLUTION))
        offset_y = int(round((origin_y - CANVAS_ORIGIN_Y) / CANVAS_RESOLUTION))

        self.get_logger().info(
            f'{self.robot_id}: Overlaying {robot_id} — '
            f'map_origin=({origin_x:.2f},{origin_y:.2f}) '
            f'map_size=({w}x{h}) '
            f'canvas_offset=({offset_x},{offset_y})')

        # Convert map data to numpy array for vectorized operations
        map_array = np.array(data, dtype=np.int16).reshape(h, w)

        # Compute canvas slice bounds
        c_x0 = max(0, offset_x)
        c_y0 = max(0, offset_y)
        c_x1 = min(CANVAS_WIDTH,  offset_x + w)
        c_y1 = min(CANVAS_HEIGHT, offset_y + h)

        # Corresponding source slice bounds
        s_x0 = max(0, -offset_x)
        s_y0 = max(0, -offset_y)
        s_x1 = s_x0 + (c_x1 - c_x0)
        s_y1 = s_y0 + (c_y1 - c_y0)

        if c_x1 <= c_x0 or c_y1 <= c_y0:
            self.get_logger().warn(
                f'{self.robot_id}: Map from {robot_id} does not overlap canvas. '
                f'offset=({offset_x},{offset_y}), map=({w}x{h}). '
                f'Check spawn positions match SPAWN_POSITIONS constant.')
            return

        # Source and canvas slices
        src   = map_array[s_y0:s_y1, s_x0:s_x1]
        c_view = self._canvas[c_y0:c_y1, c_x0:c_x1]
        lc_view = layer_count[c_y0:c_y1, c_x0:c_x1]
        fv_view = free_votes[c_y0:c_y1, c_x0:c_x1]

        # Known cells (not unknown)
        known_mask = src != UNKNOWN_VALUE
        lc_view[known_mask] += 1

        # Free cells — must exclude unknown (-1). Without this guard,
        # unknown cells satisfy (src < FREE_THRESHOLD) since -1 < 25,
        # which inflates free_votes above layer_count and breaks consensus.
        free_mask = (src >= 0) & (src < FREE_THRESHOLD)
        fv_view[free_mask] += 1

        # Occupied cells — any robot sees occupied → mark occupied immediately
        # This takes priority — occupied cannot be overwritten by free
        occupied_mask = src > OCCUPIED_THRESHOLD
        c_view[occupied_mask] = 100

    def _publish_canvas(self):
        """
        Publish the merged canvas as two OccupancyGrid messages:

        1. /{robot_id}/merged_map — in THIS robot's own map frame
           (robot_N/map). The frontier explorer subscribes to this.
           Frame must match the robot's TF tree for goal sending to work.

        2. /merged_map — in robot_1/map frame for RViz visualization.
           Always uses robot_1/map since that frame always exists.
        """
        flat_data = self._canvas.flatten().astype(np.int8).tolist()

        # ── Per-robot merged map (frontier explorer uses this) ────────────── #
        local_msg                           = OccupancyGrid()
        local_msg.header                    = Header()
        local_msg.header.stamp              = self.get_clock().now().to_msg()
        local_msg.header.frame_id           = self.robot_id + '/map'
        local_msg.info.resolution           = CANVAS_RESOLUTION
        local_msg.info.width                = CANVAS_WIDTH
        local_msg.info.height               = CANVAS_HEIGHT
        local_msg.info.origin.position.x    = CANVAS_ORIGIN_X
        local_msg.info.origin.position.y    = CANVAS_ORIGIN_Y
        local_msg.info.origin.position.z    = 0.0
        local_msg.info.origin.orientation.w = 1.0
        local_msg.data                      = flat_data
        self._merged_pub.publish(local_msg)

        # ── Global merged map (RViz visualization) ────────────────────────── #
        global_msg                           = OccupancyGrid()
        global_msg.header                    = Header()
        global_msg.header.stamp              = self.get_clock().now().to_msg()
        global_msg.header.frame_id           = 'robot_1/map'
        global_msg.info.resolution           = CANVAS_RESOLUTION
        global_msg.info.width                = CANVAS_WIDTH
        global_msg.info.height               = CANVAS_HEIGHT
        global_msg.info.origin.position.x    = CANVAS_ORIGIN_X
        global_msg.info.origin.position.y    = CANVAS_ORIGIN_Y
        global_msg.info.origin.position.z    = 0.0
        global_msg.info.origin.orientation.w = 1.0
        global_msg.data                      = flat_data
        self._global_pub.publish(global_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapMergerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()