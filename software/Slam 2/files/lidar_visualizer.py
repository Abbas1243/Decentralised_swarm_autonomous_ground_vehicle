#!/usr/bin/env python3
"""
lidar_visualizer.py  -  RPLIDAR A1M8 -> RViz with fit-first line + arc features

Based directly on your working lidar_plot.py (Mayuresh FYP).
Uses the same ultra_simple SDK binary and 2000-bin angle grid.
Replaces split-and-merge with fit-first algorithm (line then arc).

STM32 equivalent of this pipeline
----------------------------------
  PC (this file)          STM32 (fit_first.c)
  ---------------         ---------------------
  NUM_BINS = 2000         Array of 360 floats (1 deg bins)
  SDK binary -> regex     Hardware UART DMA -> parse
  fit-first (Python)      fit-first (C, same algorithm)
  RViz publish            UART -> Duo S -> Zenoh -> PC

Usage
-----
  source /opt/ros/humble/setup.bash
  python3 lidar_visualizer.py

  # Also show the matplotlib polar plot from your original code:
  python3 lidar_visualizer.py --matplotlib

  In RViz: Fixed Frame = laser, add /scan and /lines

Requirements
------------
  pip install pyserial numpy matplotlib
  RPLIDAR SDK must be built: ~/rplidar_sdk/output/Linux/Release/ultra_simple
"""

import argparse
import math
import re
import subprocess
import sys
import threading
import time

import numpy as np

# ── ROS2 ─────────────────────────────────────────────────────────────────────
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from visualization_msgs.msg import Marker, MarkerArray
    from geometry_msgs.msg import Point
    from std_msgs.msg import ColorRGBA
    from builtin_interfaces.msg import Duration
except ImportError:
    print("ERROR: source /opt/ros/humble/setup.bash first")
    sys.exit(1)

# ── Config ────────────────────────────────────────────────────────────────────
SDK_PATH = '/home/mayuresh/rplidar_sdk/output/Linux/Release/ultra_simple'
PORT     = '/dev/ttyUSB0'
BAUD     = 115200
MIN_MM   = 15      # A1M8 minimum spec is 15cm but we allow down to 1.5cm
MAX_MM   = 8000    # discard above this

# Fixed angle bin grid — same as your lidar_plot.py
NUM_BINS  = 2000
BIN_DEG   = 360.0 / NUM_BINS   # 0.18 deg per bin

# ── Feature extraction constants ──────────────────────────────────────────────
MIN_PTS        = 5     # minimum points to attempt any fit
MIN_LEN        = 0.15  # m — discard features shorter than this
MAX_LINES      = 80    # max total features per scan
GAP_THRESH     = 0.15  # m — cartesian jump bigger than this = new surface
LINE_TOLERANCE = 0.04  # m — max perpendicular deviation to accept a line
ARC_TOLERANCE  = 0.03  # m — max radial deviation to accept an arc

# Interpolation — fills LiDAR shadow zones on flat walls
MAX_INTERP_BINS = 8    # fill up to 8 consecutive missing bins (~1.4 deg)
MAX_INTERP_JUMP = 0.30 # m — if neighbours differ by more, don't interpolate

# ── Shared state ──────────────────────────────────────────────────────────────
_bins    = np.full(NUM_BINS, np.nan, dtype=np.float32)
_lock    = threading.Lock()
_ready   = threading.Event()
_status  = ["Connecting..."]
_n_scans = [0]


def _angle_to_bin(deg):
    """Map 0-360 degree angle to bin index."""
    return int(round(deg / 360.0 * NUM_BINS)) % NUM_BINS


# ── SDK reader ────────────────────────────────────────────────────────────────

def _sdk_reader():
    """Reads ultra_simple SDK output, fills the bin array."""
    try:
        proc = subprocess.Popen(
            [SDK_PATH, '--channel', '--serial', PORT, str(BAUD)],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True, bufsize=1,
        )
        _status[0] = f"SDK running on {PORT}"
        print(f"[lidar] SDK started: {SDK_PATH}")
        prev = None

        for line in proc.stdout:
            m = re.search(r'theta:\s*([\d.]+)\s+Dist:\s*([\d.]+)', line)
            if not m:
                continue

            deg = float(m.group(1))
            mm  = float(m.group(2))

            if prev is not None and deg < prev - 180:
                _n_scans[0] += 1
                _ready.set()
            prev = deg

            idx = _angle_to_bin(deg)
            with _lock:
                if MIN_MM < mm < MAX_MM:
                    _bins[idx] = mm / 1000.0
                else:
                    _bins[idx] = np.nan

    except FileNotFoundError:
        _status[0] = "SDK not found — build it first"
        print(f"[lidar] ERROR: {SDK_PATH} not found")
        print("[lidar] cd ~/rplidar_sdk && make")
    except Exception as e:
        _status[0] = f"ERROR: {e}"
        print(f"[lidar] {e}")


# ── Fit-first: line + arc detection ──────────────────────────────────────────
#
# Algorithm:
#   1. Group points by gaps (cartesian jump > GAP_THRESH = new surface)
#   2. For each group: try fitting a line (total least squares)
#      If max deviation < LINE_TOLERANCE -> accept as line
#   3. If line fails: try fitting a circle (3-point method)
#      If max deviation < ARC_TOLERANCE  -> accept as arc
#   4. If neither fits: split group in half and recurse
#
# This is the reference implementation of fit_first.c for STM32.
# Same logic, same constants — only syntax differs.

def _fit_line(pts, s, e):
    """
    Fit a line through pts[s..e] using total least squares
    (orthogonal / perpendicular regression).

    Returns (angle, distance, x1, y1, x2, y2, max_deviation) or None.

    angle    — Hough angle [-pi/2, pi/2]
    distance — perpendicular distance from origin
    x1,y1    — start endpoint projected onto fitted line
    x2,y2    — end endpoint projected onto fitted line
    max_dev  — worst perpendicular deviation of any point from the line
    """
    n = e - s + 1
    if n < 2:
        return None

    mx = sum(pts[i][0] for i in range(s, e+1)) / n
    my = sum(pts[i][1] for i in range(s, e+1)) / n

    Sxx = sum((pts[i][0]-mx)**2 for i in range(s, e+1))
    Syy = sum((pts[i][1]-my)**2 for i in range(s, e+1))
    Sxy = sum((pts[i][0]-mx)*(pts[i][1]-my) for i in range(s, e+1))

    diff = Sxx - Syy
    hyp  = math.hypot(diff, 2*Sxy)

    if abs(Sxy) < 1e-12:
        dx, dy = (1.0, 0.0) if Sxx >= Syy else (0.0, 1.0)
    else:
        lam_max = (Sxx + Syy + hyp) / 2.0
        dx = Sxy
        dy = lam_max - Sxx
        L  = math.hypot(dx, dy)
        dx /= L; dy /= L

    angle = math.atan2(dy, dx)
    if angle >  math.pi/2: angle -= math.pi
    if angle < -math.pi/2: angle += math.pi

    nx, ny   = -math.sin(angle), math.cos(angle)
    distance = nx * mx + ny * my

    max_dev = max(
        abs(nx * (pts[i][0] - mx) + ny * (pts[i][1] - my))
        for i in range(s, e+1)
    )

    tmin, tmax = 1e9, -1e9
    for i in range(s, e+1):
        t = (pts[i][0]-mx)*dx + (pts[i][1]-my)*dy
        if t < tmin: tmin = t
        if t > tmax: tmax = t

    x1 = mx + tmin*dx;  y1 = my + tmin*dy
    x2 = mx + tmax*dx;  y2 = my + tmax*dy

    return angle, distance, x1, y1, x2, y2, max_dev


def _fit_circle(pts, s, e):
    """
    Fit a circle using first, middle, last point (3-point method).
    Returns (cx, cy, r, max_deviation) or None if points are collinear.

    Radius sanity limits:
      r > 2.0m = basically a flat wall, treat as line instead
      r < 0.03m = noise spike, not a real physical object
    """
    ax, ay = pts[s]
    bx, by = pts[(s + e) // 2]
    cx, cy = pts[e]

    d = 2 * (ax*(by - cy) + bx*(cy - ay) + cx*(ay - by))
    if abs(d) < 1e-9:
        return None   # collinear — no circle exists

    ux = ((ax**2 + ay**2)*(by - cy) +
          (bx**2 + by**2)*(cy - ay) +
          (cx**2 + cy**2)*(ay - by)) / d

    uy = ((ax**2 + ay**2)*(cx - bx) +
          (bx**2 + by**2)*(ax - cx) +
          (cx**2 + cy**2)*(bx - ax)) / d

    r = math.hypot(ax - ux, ay - uy)

    if r > 2.0 or r < 0.03:
        return None

    max_dev = max(
        abs(math.hypot(pts[i][0] - ux, pts[i][1] - uy) - r)
        for i in range(s, e+1)
    )

    return ux, uy, r, max_dev


def _fit_first(pts, s, e, out, depth=0):
    """
    Core recursive fitter. Tries line, then arc, then splits in half.
    depth limit prevents infinite recursion on genuinely noisy data.
    """
    if e - s < MIN_PTS - 1:
        return
    if depth > 6:
        return

    # ── Try line ──────────────────────────────────────────────────────────
    line_result = _fit_line(pts, s, e)
    if line_result is not None:
        angle, distance, x1, y1, x2, y2, max_dev = line_result
        length = math.hypot(x2-x1, y2-y1)
        if max_dev < LINE_TOLERANCE and length >= MIN_LEN:
            out.append({
                "type":     "line",
                "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                "angle":    angle,
                "distance": distance,
                "length":   length,
                "n_pts":    e - s + 1,
                "pts_s":    s,
                "pts_e":    e,
            })
            return

    # ── Try arc ───────────────────────────────────────────────────────────
    if e - s >= MIN_PTS - 1:
        arc_result = _fit_circle(pts, s, e)
        if arc_result is not None:
            cx, cy, r, max_dev = arc_result
            if max_dev < ARC_TOLERANCE:
                theta_start = math.atan2(pts[s][1] - cy, pts[s][0] - cx)
                theta_end   = math.atan2(pts[e][1] - cy, pts[e][0] - cx)
                arc_len     = r * abs(theta_end - theta_start)
                if arc_len >= MIN_LEN:
                    out.append({
                        "type":        "arc",
                        "cx": cx, "cy": cy, "r": r,
                        "theta_start": theta_start,
                        "theta_end":   theta_end,
                        "length":      arc_len,
                        "n_pts":       e - s + 1,
                        "pts_s":       s,
                        "pts_e":       e,
                    })
                    return

    # ── Neither fit — split in half and recurse ───────────────────────────
    mid = (s + e) // 2
    _fit_first(pts, s,   mid, out, depth + 1)
    _fit_first(pts, mid, e,   out, depth + 1)


def split_merge(pts):
    """
    Main feature extraction entry point.
    Returns (features, curve_groups, pts).

    features     — list of dicts, each with type='line' or type='arc'
    curve_groups — list of (i, i) index pairs for arc features (for RViz)
    pts          — original cartesian point list (passed through unchanged)
    """
    if len(pts) < MIN_PTS:
        return [], [], pts

    # ── Gap detection ─────────────────────────────────────────────────────
    surfaces = []
    ss = 0
    for i in range(1, len(pts)):
        if math.hypot(pts[i][0]-pts[i-1][0], pts[i][1]-pts[i-1][1]) > GAP_THRESH:
            if i - ss >= MIN_PTS:
                surfaces.append((ss, i-1))
            ss = i
    if len(pts) - ss >= MIN_PTS:
        surfaces.append((ss, len(pts)-1))

    # ── Fit each surface ──────────────────────────────────────────────────
    all_features = []
    for s, e in surfaces:
        _fit_first(pts, s, e, all_features)
        if len(all_features) >= MAX_LINES:
            break

    # ── Tag arcs as curve_groups for RViz publisher ───────────────────────
    curve_groups = []
    for i, f in enumerate(all_features):
        if f["type"] == "arc":
            f["curve_group"] = len(curve_groups)
            curve_groups.append((i, i))

    return all_features, curve_groups, pts


# ── Bin grid -> cartesian list with intra-surface interpolation ──────────────

def bins_to_pts(dist_m):
    """
    Convert NUM_BINS distance array to ordered (x,y) cartesian list.

    Steps:
      1. Linear interpolation across small intra-surface gaps
         (fills LiDAR shadow zones on flat walls — does not invent geometry)
      2. Skip bins still NaN after step 1 (genuine gaps between surfaces)
    """
    filled = dist_m.copy()

    i = 0
    while i < NUM_BINS:
        if not np.isnan(filled[i]):
            i += 1
            continue

        run_start = i
        while i < NUM_BINS and np.isnan(filled[i]):
            i += 1
        run_end = i

        if run_end - run_start > MAX_INTERP_BINS:
            continue

        left_idx  = run_start - 1
        right_idx = run_end

        if left_idx < 0 or right_idx >= NUM_BINS:
            continue

        r_left  = filled[left_idx]
        r_right = filled[right_idx]

        if np.isnan(r_left) or np.isnan(r_right):
            continue

        if abs(r_left - r_right) > MAX_INTERP_JUMP:
            continue

        for j in range(run_start, run_end):
            t = (j - left_idx) / (right_idx - left_idx)
            filled[j] = r_left + t * (r_right - r_left)

    pts = []
    for i in range(NUM_BINS):
        r = filled[i]
        if np.isnan(r):
            continue
        a = math.radians(90.0 - i * BIN_DEG)
        pts.append((r * math.cos(a), r * math.sin(a)))
    return pts


# ── ROS2 node ─────────────────────────────────────────────────────────────────

class LidarNode(Node):

    def __init__(self):
        super().__init__("lidar_visualizer")
        self.pub_scan  = self.create_publisher(LaserScan,   "/scan",  10)
        self.pub_lines = self.create_publisher(MarkerArray, "/lines", 10)
        threading.Thread(target=self._loop, daemon=True).start()
        self.get_logger().info("Waiting for SDK data...")
        self.get_logger().info("RViz: Fixed Frame=laser | add /scan /lines")

    def _loop(self):
        while rclpy.ok():
            if not _ready.wait(timeout=2.0):
                continue
            _ready.clear()
            with _lock:
                snap = _bins.copy()
            self._pub_scan(snap)
            self._pub_lines(snap)

    def _pub_scan(self, dist_m):
        now    = self.get_clock().now().to_msg()
        a_min  = math.radians(90.0)
        a_inc  = -math.radians(BIN_DEG)
        ranges = [0.0 if np.isnan(r) else float(r) for r in dist_m]
        msg = LaserScan()
        msg.header.stamp    = now
        msg.header.frame_id = "laser"
        msg.angle_min       = a_min
        msg.angle_max       = a_min + a_inc * (NUM_BINS-1)
        msg.angle_increment = a_inc
        msg.time_increment  = 0.0
        msg.scan_time       = 0.2
        msg.range_min       = MIN_MM / 1000.0
        msg.range_max       = MAX_MM / 1000.0
        msg.ranges          = ranges
        msg.intensities     = []
        self.pub_scan.publish(msg)

    def _pub_lines(self, dist_m):
        now  = self.get_clock().now().to_msg()
        pts  = bins_to_pts(dist_m)
        lines, curve_groups, _raw_pts = split_merge(pts)

        ma  = MarkerArray()
        clr = Marker()
        clr.header.frame_id = "laser"
        clr.header.stamp    = now
        clr.action          = Marker.DELETEALL
        ma.markers.append(clr)

        lifetime = Duration(sec=0, nanosec=400_000_000)

        # ── Draw straight wall lines ──────────────────────────────────────
        for i, ln in enumerate(lines):
            # Skip arcs — drawn separately below
            if ln.get("type") == "arc":
                continue

            t     = min(ln["length"] / 2.0, 1.0)
            color = ColorRGBA(r=1.0-t, g=t, b=0.0, a=1.0)

            lm = Marker()
            lm.header.frame_id    = "laser"
            lm.header.stamp       = now
            lm.ns, lm.id          = "lines", i
            lm.type               = Marker.LINE_STRIP
            lm.action             = Marker.ADD
            lm.scale.x            = 0.05
            lm.color              = color
            lm.lifetime           = lifetime
            lm.pose.orientation.w = 1.0
            lm.points = [Point(x=ln["x1"], y=ln["y1"], z=0.0),
                         Point(x=ln["x2"], y=ln["y2"], z=0.0)]
            ma.markers.append(lm)

            # Label showing angle, distance, length
            tm = Marker()
            tm.header.frame_id    = "laser"
            tm.header.stamp       = now
            tm.ns, tm.id          = "labels", i+1000
            tm.type               = Marker.TEXT_VIEW_FACING
            tm.action             = Marker.ADD
            tm.scale.z            = 0.10
            tm.color              = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9)
            tm.lifetime           = lifetime
            tm.pose.position.x    = (ln["x1"]+ln["x2"])/2
            tm.pose.position.y    = (ln["y1"]+ln["y2"])/2
            tm.pose.position.z    = 0.15
            tm.pose.orientation.w = 1.0
            tm.text = (f"a={math.degrees(ln['angle']):.0f}  "
                       f"d={ln['distance']:.2f}m  "
                       f"L={ln['length']:.2f}m")
            ma.markers.append(tm)

        # ── Draw arcs using fitted circle geometry ────────────────────────
        # Sampled from the clean fitted equation — not raw noisy LiDAR points.
        for g_idx, (li_start, _) in enumerate(curve_groups):
            arc = lines[li_start]
            cx, cy, r = arc["cx"], arc["cy"], arc["r"]
            t_s, t_e  = arc["theta_start"], arc["theta_end"]

            # Always sweep the short way around the circle
            dt = t_e - t_s
            if dt >  math.pi: dt -= 2*math.pi
            if dt < -math.pi: dt += 2*math.pi

            n_samples = 20
            arc_pts = [
                Point(
                    x=cx + r * math.cos(t_s + dt * k / n_samples),
                    y=cy + r * math.sin(t_s + dt * k / n_samples),
                    z=0.0
                )
                for k in range(n_samples + 1)
            ]

            cm = Marker()
            cm.header.frame_id    = "laser"
            cm.header.stamp       = now
            cm.ns                 = "curves"
            cm.id                 = g_idx
            cm.type               = Marker.LINE_STRIP
            cm.action             = Marker.ADD
            cm.scale.x            = 0.04
            cm.color              = ColorRGBA(r=0.0, g=0.9, b=1.0, a=1.0)
            cm.lifetime           = lifetime
            cm.pose.orientation.w = 1.0
            cm.points             = arc_pts
            ma.markers.append(cm)

            # Label at arc midpoint
            mid_theta = t_s + dt * 0.5
            tm = Marker()
            tm.header.frame_id    = "laser"
            tm.header.stamp       = now
            tm.ns, tm.id          = "curve_labels", g_idx
            tm.type               = Marker.TEXT_VIEW_FACING
            tm.action             = Marker.ADD
            tm.scale.z            = 0.09
            tm.color              = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.9)
            tm.lifetime           = lifetime
            tm.pose.position.x    = cx + r * math.cos(mid_theta)
            tm.pose.position.y    = cy + r * math.sin(mid_theta)
            tm.pose.position.z    = 0.15
            tm.pose.orientation.w = 1.0
            tm.text               = f"arc r={r:.2f}m"
            ma.markers.append(tm)

        n_lines = sum(1 for f in lines if f["type"] == "line")
        n_arcs  = len(curve_groups)
        valid   = int(np.sum(~np.isnan(dist_m)))
        self.get_logger().info(
            f"Scan#{_n_scans[0]}  {valid}/{NUM_BINS} bins  "
            f"-> {n_lines} lines  {n_arcs} arcs",
            throttle_duration_sec=2.0)
        self.pub_lines.publish(ma)

    def destroy_node(self):
        super().destroy_node()


# ── Optional matplotlib polar plot ───────────────────────────────────────────

def run_matplotlib():
    """Same polar plot as your lidar_plot.py with line and arc overlay."""
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation

    angles = np.linspace(0, 2*math.pi, NUM_BINS, endpoint=False)

    fig = plt.figure(figsize=(10,10), facecolor='#0a0a1a')
    ax  = fig.add_subplot(111, projection='polar', facecolor='#0d0d2b')
    ax.set_theta_zero_location('N')
    ax.set_theta_direction(-1)
    ax.tick_params(colors='#6688aa', labelsize=9)
    ax.grid(color='#1a2a4a', linewidth=0.6, linestyle='--', alpha=0.7)
    ax.spines['polar'].set_color('#1a2a4a')
    ax.set_rlabel_position(45)

    sc = ax.scatter([], [], s=4, c=[], cmap='plasma',
                    vmin=0, vmax=MAX_MM/1000.0, alpha=0.9, linewidths=0)
    line_arts = []

    ax.set_title('RPLidar A1M8  +  fit-first (lines + arcs)\n',
                 color='white', fontsize=13, pad=18, fontweight='bold')
    info = fig.text(0.5, 0.015, 'Connecting...', ha='center',
                    color='#6699bb', fontsize=9)

    def update(_):
        with _lock:
            dist = _bins.copy()

        valid = ~np.isnan(dist)
        n     = valid.sum()
        if n == 0: return sc,

        sc.set_offsets(np.c_[angles[valid], dist[valid]])
        sc.set_array(dist[valid])
        ax.set_ylim(0, dist[valid].max() * 1.1)

        for a in line_arts: a.remove()
        line_arts.clear()

        pts  = bins_to_pts(dist)
        lines, curve_groups, _raw_pts = split_merge(pts)

        # Draw straight lines
        for ln in lines:
            if ln["type"] != "line":
                continue
            r1 = math.hypot(ln["x1"], ln["y1"])
            r2 = math.hypot(ln["x2"], ln["y2"])
            a1 = math.atan2(ln["y1"], ln["x1"])
            a2 = math.atan2(ln["y2"], ln["x2"])
            t  = min(ln["length"]/2.0, 1.0)
            art, = ax.plot([a1, a2], [r1, r2],
                           color=(1-t, t, 0), linewidth=2.5, alpha=0.9)
            line_arts.append(art)

        # Draw arcs sampled from fitted circle equation
        for _g_idx, (li_start, _) in enumerate(curve_groups):
            arc = lines[li_start]
            cx, cy, r = arc["cx"], arc["cy"], arc["r"]
            t_s, t_e  = arc["theta_start"], arc["theta_end"]
            dt = t_e - t_s
            if dt >  math.pi: dt -= 2*math.pi
            if dt < -math.pi: dt += 2*math.pi

            thetas = [t_s + dt * k / 20 for k in range(21)]
            xs = [cx + r * math.cos(th) for th in thetas]
            ys = [cy + r * math.sin(th) for th in thetas]
            rs_ = [math.hypot(x, y) for x, y in zip(xs, ys)]
            as_ = [math.atan2(y, x)  for x, y in zip(xs, ys)]
            art, = ax.plot(as_, rs_, color=(0, 0.9, 1.0),
                           linewidth=2.0, alpha=0.85)
            line_arts.append(art)

        n_lines = sum(1 for f in lines if f["type"] == "line")
        info.set_text(f'Bins:{n}/{NUM_BINS}  Lines:{n_lines}  '
                      f'Arcs:{len(curve_groups)}  Scans:{_n_scans[0]}')
        return sc,

    ani = animation.FuncAnimation(
        fig, update, interval=100, blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    global PORT, SDK_PATH
    ap = argparse.ArgumentParser()
    ap.add_argument("--port",       default=PORT)
    ap.add_argument("--sdk",        default=SDK_PATH)
    ap.add_argument("--matplotlib", action="store_true",
                    help="Show matplotlib polar plot (like your lidar_plot.py)")
    args = ap.parse_args()

    PORT, SDK_PATH = args.port, args.sdk

    threading.Thread(target=_sdk_reader, daemon=True).start()
    time.sleep(0.3)

    if args.matplotlib:
        rclpy.init()
        node    = LidarNode()
        ros_thr = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        ros_thr.start()
        run_matplotlib()
        node.destroy_node()
        rclpy.shutdown()
    else:
        rclpy.init()
        node = LidarNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()