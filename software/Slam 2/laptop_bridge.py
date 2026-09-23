#!/usr/bin/env python3
"""
laptop_bridge.py
==================
Runs ON THE LAPTOP ONLY. Connects out to duo_slam_server.py's plain TCP
socket (the Duo S is the server, this is the client) and republishes each
received scan message as ROS2 topics, so RViz sees the same /map, /lines,
/trajectory, /slam_out_pose, and map->laser TF that lidar_visualizer.py
used to produce locally -- except now the actual SLAM computation is
running on the Duo S, and this process is a pure translation layer with
no SLAM logic of its own.

WHY A PLAIN SOCKET INSTEAD OF ZENOH/ROS2 REACHING THE DUO S DIRECTLY
-----------------------------------------------------------------------
Per project constraint: the Duo S cannot run Zenoh (or ROS2/DDS) AND the
SLAM stack at the same time -- not enough headroom on a 512MB/1GHz single
A53 core, especially with the current pure-Python pipeline already
running seconds-per-scan (see slam_progress_compact.md). A raw TCP
socket with small JSON messages has effectively zero CPU/RAM cost on the
Duo S side compared to standing up a full DDS participant. ROS2 itself
only runs HERE, on the laptop -- exactly matching how ROS2 was already
scoped as "PC dev/viz only" everywhere else in this project's docs.

USAGE
-----
    source /opt/ros/humble/setup.bash
    python3 laptop_bridge.py --host <duo-s-ip> --port 9191

    In RViz: Fixed Frame=map, add /map, /lines, /trajectory,
             /slam_out_pose displays (same RViz config lidar_visualizer.py
             already documented).

LIMITATIONS vs the old local lidar_visualizer.py
-----------------------------------------------------------------
- No /scan (raw LaserScan) topic -- raw point data is deliberately not
  sent over the wire (see duo_slam_server.py's wire-format note); only
  the extracted line/arc features are. If you need the raw scan visible
  too, that's an easy addition to the wire message later, not a
  structural blocker.
- No OccupancyGrid free-space rendering (occupancy_grid.
  FreeSpaceAccumulator needs raw scan points, which aren't sent here
  either). /map is published as a MarkerArray of colored line/arc
  segments instead, same visual convention lidar_visualizer.py's /lines
  topic already used for STATIC=green/DYNAMIC=red/UNCLASSIFIED=grey.
"""

import argparse
import json
import math
import socket
import sys
import threading

try:
    import rclpy
    from rclpy.node import Node
    from visualization_msgs.msg import Marker, MarkerArray
    from geometry_msgs.msg import Point, TransformStamped, Quaternion, PoseStamped
    from nav_msgs.msg import Path
    from std_msgs.msg import ColorRGBA
    from builtin_interfaces.msg import Duration
    from tf2_ros import TransformBroadcaster
except ImportError:
    print("ERROR: source /opt/ros/humble/setup.bash first")
    sys.exit(1)

MAX_PATH_POSES = 5000

_STATUS_COLOR = {
    0: ColorRGBA(r=0.55, g=0.55, b=0.55, a=1.0),   # UNCLASSIFIED - grey
    1: ColorRGBA(r=0.0, g=0.9, b=0.2, a=1.0),      # STATIC       - green
    2: ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0),      # DYNAMIC      - red
}
_LIVE_LINE_COLOR = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9)
_LIVE_ARC_COLOR = ColorRGBA(r=0.0, g=0.9, b=1.0, a=1.0)


def _recv_lines(sock):
    """Generator yielding decoded JSON objects, one per newline-delimited
    message, reconnect-tolerant at the caller's level (this just raises
    on disconnect; main loop below handles reconnecting)."""
    buf = b""
    while True:
        chunk = sock.recv(65536)
        if not chunk:
            raise ConnectionError("socket closed by Duo S")
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            if line.strip():
                yield json.loads(line)


class BridgeNode(Node):

    def __init__(self, host, port):
        super().__init__("duo_slam_bridge")
        self.host = host
        self.port = port

        self.pub_map = self.create_publisher(MarkerArray, "/map", 10)
        self.pub_lines = self.create_publisher(MarkerArray, "/lines", 10)
        self.pub_path = self.create_publisher(Path, "/trajectory", 10)
        self.pub_pose = self.create_publisher(PoseStamped, "/slam_out_pose", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self._path_msg = Path()
        self._path_msg.header.frame_id = "map"

        threading.Thread(target=self._connect_loop, daemon=True).start()
        self.get_logger().info(
            f"Connecting to Duo S at {host}:{port} ... "
            "RViz: Fixed Frame=map | add /map /lines /trajectory /slam_out_pose"
        )

    # ------------------------------------------------------------------ #
    def _connect_loop(self):
        while rclpy.ok():
            try:
                self.get_logger().info(f"dialing {self.host}:{self.port} ...")
                sock = socket.create_connection((self.host, self.port), timeout=5.0)
                sock.settimeout(None)
                self.get_logger().info("connected to Duo S")
                for msg in _recv_lines(sock):
                    self._handle_message(msg)
            except (ConnectionError, OSError) as e:
                self.get_logger().warn(f"connection lost/failed: {e}; retrying in 2s")
                import time
                time.sleep(2.0)

    # ------------------------------------------------------------------ #
    def _handle_message(self, msg):
        pose = msg["pose"]
        self._publish_tf(pose)
        self._publish_path_and_pose(pose)
        self._publish_map(msg.get("map", []))
        if "lines" in msg:
            self._publish_lines(msg["lines"])

        self.get_logger().info(
            f"scan#{msg.get('scan_idx')}  "
            f"pose=({pose['x']:+.2f},{pose['y']:+.2f},"
            f"{math.degrees(pose['theta']):+.1f}deg)  "
            f"map_entries={len(msg.get('map', []))}  "
            f"applied={msg.get('delta_applied')}",
            throttle_duration_sec=1.0,
        )

    # ------------------------------------------------------------------ #
    def _publish_tf(self, pose):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "laser"
        t.transform.translation.x = float(pose["x"])
        t.transform.translation.y = float(pose["y"])
        t.transform.translation.z = 0.0
        half = pose["theta"] / 2.0
        t.transform.rotation = Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))
        self.tf_broadcaster.sendTransform(t)

    def _publish_path_and_pose(self, pose):
        now = self.get_clock().now().to_msg()
        half = pose["theta"] / 2.0
        orientation = Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))

        ps = PoseStamped()
        ps.header.stamp = now
        ps.header.frame_id = "map"
        ps.pose.position.x = float(pose["x"])
        ps.pose.position.y = float(pose["y"])
        ps.pose.position.z = 0.0
        ps.pose.orientation = orientation
        self.pub_pose.publish(ps)

        self._path_msg.header.stamp = now
        self._path_msg.poses.append(ps)
        if len(self._path_msg.poses) > MAX_PATH_POSES:
            del self._path_msg.poses[0:len(self._path_msg.poses) - MAX_PATH_POSES]
        self.pub_path.publish(self._path_msg)

    def _publish_map(self, entries):
        now = self.get_clock().now().to_msg()
        ma = MarkerArray()
        clr = Marker()
        clr.header.frame_id = "map"
        clr.header.stamp = now
        clr.action = Marker.DELETEALL
        ma.markers.append(clr)

        lifetime = Duration(sec=1, nanosec=0)

        for i, e in enumerate(entries):
            color = _STATUS_COLOR.get(e.get("status", 0), _STATUS_COLOR[0])

            if e["type"] == "arc":
                cx, cy, r = e["mx"], e["my"], e["r"]
                if r < 0.01:
                    continue
                t_s = e.get("theta_start", 0.0)
                span = min(e.get("length", 0.1) / r, 2 * math.pi) if r > 0 else 0.0
                n = 16
                pts = [Point(x=cx + r * math.cos(t_s + span * k / n),
                              y=cy + r * math.sin(t_s + span * k / n), z=0.0)
                       for k in range(n + 1)]
            else:
                dx = math.cos(e["angle"]) * e["length"] / 2.0
                dy = math.sin(e["angle"]) * e["length"] / 2.0
                pts = [Point(x=e["mx"] - dx, y=e["my"] - dy, z=0.0),
                       Point(x=e["mx"] + dx, y=e["my"] + dy, z=0.0)]

            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = now
            m.ns, m.id = "map_entries", i
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.04
            m.color = color
            m.lifetime = lifetime
            m.pose.orientation.w = 1.0
            m.points = pts
            ma.markers.append(m)

        self.pub_map.publish(ma)

    def _publish_lines(self, lines):
        """Live per-scan features, in the SENSOR frame -- rendered under
        the 'laser' TF frame (same convention lidar_visualizer.py used),
        so RViz's map->laser transform positions them correctly without
        this bridge needing to transform them itself."""
        now = self.get_clock().now().to_msg()
        ma = MarkerArray()
        clr = Marker()
        clr.header.frame_id = "laser"
        clr.header.stamp = now
        clr.action = Marker.DELETEALL
        ma.markers.append(clr)

        lifetime = Duration(sec=0, nanosec=400_000_000)

        for i, f in enumerate(lines):
            if f["type"] == "line":
                m = Marker()
                m.header.frame_id = "laser"
                m.header.stamp = now
                m.ns, m.id = "live_lines", i
                m.type = Marker.LINE_STRIP
                m.action = Marker.ADD
                m.scale.x = 0.05
                m.color = _LIVE_LINE_COLOR
                m.lifetime = lifetime
                m.pose.orientation.w = 1.0
                m.points = [Point(x=f["x1"], y=f["y1"], z=0.0),
                             Point(x=f["x2"], y=f["y2"], z=0.0)]
                ma.markers.append(m)
            elif f["type"] == "arc":
                cx, cy, r = f["cx"], f["cy"], f["r"]
                t_s, t_e = f.get("theta_start", 0.0), f.get("theta_end", 0.0)
                dt = t_e - t_s
                if dt > math.pi: dt -= 2 * math.pi
                if dt < -math.pi: dt += 2 * math.pi
                n = 16
                pts = [Point(x=cx + r * math.cos(t_s + dt * k / n),
                              y=cy + r * math.sin(t_s + dt * k / n), z=0.0)
                       for k in range(n + 1)]
                m = Marker()
                m.header.frame_id = "laser"
                m.header.stamp = now
                m.ns, m.id = "live_arcs", i
                m.type = Marker.LINE_STRIP
                m.action = Marker.ADD
                m.scale.x = 0.04
                m.color = _LIVE_ARC_COLOR
                m.lifetime = lifetime
                m.pose.orientation.w = 1.0
                m.points = pts
                ma.markers.append(m)

        self.pub_lines.publish(ma)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True, help="Duo S IP address")
    ap.add_argument("--port", type=int, default=9191)
    args = ap.parse_args()

    rclpy.init()
    node = BridgeNode(args.host, args.port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
