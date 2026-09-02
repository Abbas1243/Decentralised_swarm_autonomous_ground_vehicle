#!/usr/bin/env python3
"""
zenoh_rviz_bridge.py

Runs on your PC. Subscribes to the three Zenoh topics published by
map_publisher.c on the Duo S (or PC during testing), converts them to
RViz2 MarkerArray messages, and publishes at ~10 Hz.

Requirements:
    pip install eclipse-zenoh
    ROS2 + rclpy + visualization_msgs installed

Usage:
    # Terminal 1 — run your C slam node (or bag replayer)
    ./build/slam_node --bag room1.bag

    # Terminal 2 — run this bridge
    python3 tools/zenoh_rviz_bridge.py

    # Terminal 3 — RViz2
    rviz2
    # Add MarkerArray display, topics:
    #   /slam/static_map     (green lines = confirmed walls)
    #   /slam/dynamic_lines  (red lines  = live obstacles)
    # Add Axes display for /slam/pose

Zenoh locator (optional):
    python3 tools/zenoh_rviz_bridge.py --locator tcp/192.168.1.50:7447
"""

import argparse
import math
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, TransformStamped
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster

import zenoh

# ── Wire format constants — must match map_publisher.h exactly ────────────────
MAP_MAGIC       = 0x534C414D
HDR_FMT         = '<III'       # magic, n_lines, scan_count   — 12 bytes
HDR_SIZE        = struct.calcsize(HDR_FMT)
LINE_WIRE_FMT   = '<5f4B'     # angle dist length mx my state conf observed pad — 24 bytes
LINE_WIRE_SIZE  = struct.calcsize(LINE_WIRE_FMT)
POSE_WIRE_FMT   = '<3f'        # x y theta                    — 12 bytes

# ── RViz marker colours ───────────────────────────────────────────────────────
COLOR_STATIC   = ColorRGBA(r=0.0,  g=1.0,  b=0.4,  a=0.9)   # green
COLOR_DYNAMIC  = ColorRGBA(r=1.0,  g=0.2,  b=0.1,  a=0.85)  # red
COLOR_UNKNOWN  = ColorRGBA(r=0.6,  g=0.6,  b=0.0,  a=0.5)   # dim yellow
COLOR_OCCLUDED = ColorRGBA(r=0.2,  g=0.4,  b=1.0,  a=0.5)   # blue

LINE_STATE_UNKNOWN  = 0
LINE_STATE_STATIC   = 1
LINE_STATE_DYNAMIC  = 2
LINE_STATE_OCCLUDED = 3

# Thickness of line markers in metres
STATIC_LINE_WIDTH  = 0.04
DYNAMIC_LINE_WIDTH = 0.03
DEFAULT_LINE_WIDTH = 0.02


def parse_map_buffer(data: bytes) -> list | None:
    """
    Deserialize a binary map buffer from map_publisher.c.
    Returns list of dicts, or None on parse error.
    """
    if len(data) < HDR_SIZE:
        return None

    magic, n_lines, scan_count = struct.unpack_from(HDR_FMT, data, 0)
    if magic != MAP_MAGIC:
        return None

    expected = HDR_SIZE + n_lines * LINE_WIRE_SIZE
    if len(data) < expected:
        return None

    lines = []
    offset = HDR_SIZE
    for _ in range(n_lines):
        angle, dist, length, mx, my, state, conf, observed = \
            struct.unpack_from(LINE_WIRE_FMT, data, offset)
        offset += LINE_WIRE_SIZE

        # Reconstruct endpoints from Hough (angle, distance) + midpoint
        # Direction vector along line: (cos(angle), sin(angle))
        # Midpoint: mx, my (already in map frame)
        half = length * 0.5
        cx = math.cos(angle)
        cy = math.sin(angle)

        lines.append(dict(
            angle=angle, distance=dist, length=length,
            mx=mx, my=my,
            x1=mx + cx * half, y1=my + cy * half,
            x2=mx - cx * half, y2=my - cy * half,
            state=state, confidence=conf, observed=observed,
            scan_count=scan_count,
        ))
    return lines


def make_line_marker(marker_id: int, lines: list,
                     color: ColorRGBA, width: float,
                     frame_id: str, stamp) -> Marker:
    """Build a LINE_LIST Marker from a list of line dicts."""
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp    = stamp
    m.ns              = 'slam_lines'
    m.id              = marker_id
    m.type            = Marker.LINE_LIST
    m.action          = Marker.ADD
    m.scale.x         = width          # line width
    m.color           = color
    m.pose.orientation.w = 1.0

    for ln in lines:
        p1, p2 = Point(), Point()
        p1.x, p1.y, p1.z = float(ln['x1']), float(ln['y1']), 0.0
        p2.x, p2.y, p2.z = float(ln['x2']), float(ln['y2']), 0.0
        m.points.append(p1)
        m.points.append(p2)

    return m


def make_delete_marker(marker_id: int, frame_id: str, stamp) -> Marker:
    """Return a DELETE marker to clear a stale overlay."""
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp    = stamp
    m.ns     = 'slam_lines'
    m.id     = marker_id
    m.action = Marker.DELETE
    return m


class ZenohRvizBridge(Node):
    """
    ROS2 node that subscribes to Zenoh SLAM topics and republishes
    as RViz2 MarkerArrays on /slam/static_map, /slam/dynamic_lines.
    """

    FRAME_ID = 'map'   # RViz fixed frame — set this in RViz2 Global Options

    def __init__(self, locator: str | None):
        super().__init__('zenoh_rviz_bridge')

        # ── ROS2 publishers ──────────────────────────────────────────────────
        self._pub_static  = self.create_publisher(
            MarkerArray, '/slam/static_map',    10)
        self._pub_dynamic = self.create_publisher(
            MarkerArray, '/slam/dynamic_lines', 10)

        self._tf_broadcaster = TransformBroadcaster(self)

        # ── Shared state (Zenoh callbacks → ROS publish thread) ──────────────
        self._lock          = threading.Lock()
        self._static_lines  = []
        self._dynamic_lines = []
        self._pose          = (0.0, 0.0, 0.0)    # x, y, theta
        self._scan_count    = 0
        self._last_update   = 0.0

        # ── Zenoh session ────────────────────────────────────────────────────
        conf = zenoh.Config()
        if locator:
            conf.insert_json5('connect/endpoints', f'["{locator}"]')

        self._session = zenoh.open(conf)

        self._sub_static  = self._session.declare_subscriber(
            'slam/static_map',    self._on_static)
        self._sub_dynamic = self._session.declare_subscriber(
            'slam/dynamic_lines', self._on_dynamic)
        self._sub_pose    = self._session.declare_subscriber(
            'slam/pose',          self._on_pose)

        # ── ROS timer — publish at 10Hz ──────────────────────────────────────
        self._timer = self.create_timer(0.1, self._publish_to_rviz)

        self.get_logger().info(
            f'Bridge started — listening on Zenoh'
            f'{" → " + locator if locator else " (multicast)"}'
        )

    # ── Zenoh callbacks (called from Zenoh's thread) ─────────────────────────

    def _on_static(self, sample):
        data = bytes(sample.payload)
        lines = parse_map_buffer(data)
        if lines is not None:
            with self._lock:
                self._static_lines = lines
                self._last_update  = time.time()

    def _on_dynamic(self, sample):
        data = bytes(sample.payload)
        lines = parse_map_buffer(data)
        if lines is not None:
            with self._lock:
                self._dynamic_lines = lines

    def _on_pose(self, sample):
        data = bytes(sample.payload)
        if len(data) >= struct.calcsize(POSE_WIRE_FMT):
            x, y, theta = struct.unpack_from(POSE_WIRE_FMT, data)
            with self._lock:
                self._pose = (x, y, theta)

    # ── ROS publish (called from ROS timer thread) ────────────────────────────

    def _publish_to_rviz(self):
        now = self.get_clock().now().to_msg()

        with self._lock:
            static_lines  = list(self._static_lines)
            dynamic_lines = list(self._dynamic_lines)
            pose          = self._pose
            last_update   = self._last_update

        # ── Static map MarkerArray ────────────────────────────────────────────
        static_ma = MarkerArray()

        if static_lines:
            static_ma.markers.append(
                make_line_marker(0, static_lines, COLOR_STATIC,
                                 STATIC_LINE_WIDTH, self.FRAME_ID, now))
        else:
            # Send a delete marker so stale lines don't linger
            static_ma.markers.append(
                make_delete_marker(0, self.FRAME_ID, now))

        self._pub_static.publish(static_ma)

        # ── Dynamic obstacles MarkerArray ─────────────────────────────────────
        dynamic_ma = MarkerArray()

        if dynamic_lines:
            dynamic_ma.markers.append(
                make_line_marker(1, dynamic_lines, COLOR_DYNAMIC,
                                 DYNAMIC_LINE_WIDTH, self.FRAME_ID, now))
        else:
            dynamic_ma.markers.append(
                make_delete_marker(1, self.FRAME_ID, now))

        self._pub_dynamic.publish(dynamic_ma)

        # ── TF: publish map → base_link transform from pose ──────────────────
        tf              = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id      = 'map'
        tf.child_frame_id       = 'base_link'
        tf.transform.translation.x = float(pose[0])
        tf.transform.translation.y = float(pose[1])
        tf.transform.translation.z = 0.0

        # Convert theta to quaternion (rotation around Z axis)
        half_theta = pose[2] * 0.5
        tf.transform.rotation.z = math.sin(half_theta)
        tf.transform.rotation.w = math.cos(half_theta)

        self._tf_broadcaster.sendTransform(tf)

        # ── Stale data warning ────────────────────────────────────────────────
        if last_update > 0 and time.time() - last_update > 2.0:
            self.get_logger().warn(
                'No data from SLAM for >2s — is slam_node running?',
                throttle_duration_sec=5.0)

    def destroy_node(self):
        self._sub_static.undeclare()
        self._sub_dynamic.undeclare()
        self._sub_pose.undeclare()
        self._session.close()
        super().destroy_node()


def main():
    ap = argparse.ArgumentParser(description='Zenoh → RViz2 bridge for SLAM map')
    ap.add_argument('--locator', default=None,
                    help='Zenoh locator e.g. tcp/192.168.1.50:7447  '
                         '(omit for multicast discovery)')
    args = ap.parse_args()

    rclpy.init()
    node = ZenohRvizBridge(args.locator)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()