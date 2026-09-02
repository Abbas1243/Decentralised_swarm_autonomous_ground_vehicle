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
from fit_first_ctypes import split_merge
from map_manager import ENTRY_UNCLASSIFIED, ENTRY_STATIC, ENTRY_DYNAMIC
from slam import SlamState
import occupancy_grid
import numpy as np

# ── ROS2 ─────────────────────────────────────────────────────────────────────
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from visualization_msgs.msg import Marker, MarkerArray
    from nav_msgs.msg import OccupancyGrid, Path
    from geometry_msgs.msg import Point, TransformStamped, Quaternion, PoseStamped
    from std_msgs.msg import ColorRGBA
    from builtin_interfaces.msg import Duration
    from tf2_ros import TransformBroadcaster
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

# /trajectory Path history — PC-only display, but still bounded so a long
# run doesn't grow this list forever.
MAX_PATH_POSES = 5000

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

MAX_RAW_LINES_LOGGED = 30   # cap so a persistently-failing SDK doesn't spam
                             # stdout forever -- enough to see the actual
                             # error text (permission denied, bad port,
                             # wrong baud, etc.) at least once.
_raw_lines_logged = [0]


def _sdk_reader():
    """Reads ultra_simple SDK output, fills the bin array.

    DIAGNOSTICS: this used to silently `continue` past any line that didn't
    match the theta/Dist regex -- which meant an SDK-side error (wrong
    port, permission denied on /dev/ttyUSB0, wrong baud, LiDAR not spinning)
    produced ZERO visible output: _ready never got set, _loop() blocked on
    _ready.wait() forever, and the whole program looked like it "failed
    silently" with nothing printed after the two startup log lines. Fixed
    by (1) printing unmatched SDK output lines instead of discarding them,
    so the SDK's own error text is visible, and (2) reporting the process's
    exit code if it terminates instead of streaming forever. See
    _sdk_watchdog() below for the "still no data after N seconds" case."""
    try:
        proc = subprocess.Popen(
            [SDK_PATH, '--channel', '--serial', PORT, str(BAUD)],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True, bufsize=1,
        )
        _status[0] = f"SDK running on {PORT}"
        print(f"[lidar] SDK started: {SDK_PATH} --channel --serial {PORT} {BAUD}")
        prev = None

        for line in proc.stdout:
            m = re.search(r'theta:\s*([\d.]+)\s+Dist:\s*([\d.]+)', line, re.IGNORECASE)
            if not m:
                if _raw_lines_logged[0] < MAX_RAW_LINES_LOGGED:
                    stripped = line.rstrip()
                    if stripped:
                        print(f"[lidar:sdk-output] {stripped}")
                        _raw_lines_logged[0] += 1
                        if _raw_lines_logged[0] == MAX_RAW_LINES_LOGGED:
                            print("[lidar:sdk-output] (further unmatched lines suppressed)")
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

        # The for-loop above only exits when proc.stdout closes, i.e. the
        # SDK process has terminated. Previously this returned silently —
        # report it, since a process that exits immediately (bad args,
        # permission denied, device busy) is a very common real cause of
        # "nothing happens after startup".
        rc = proc.poll()
        _status[0] = f"SDK process exited (code {rc})"
        print(f"[lidar] SDK process {SDK_PATH} exited with code {rc} — "
              f"no more data will arrive. Check the port/permissions/cable, "
              f"or run the SDK binary directly to see its own error output:")
        print(f"[lidar]   {SDK_PATH} --channel --serial {PORT} {BAUD}")

    except FileNotFoundError:
        _status[0] = "SDK not found — build it first"
        print(f"[lidar] ERROR: {SDK_PATH} not found")
        print("[lidar] cd ~/rplidar_sdk && make")
    except Exception as e:
        _status[0] = f"ERROR: {e}"
        print(f"[lidar] {e}")


def _sdk_watchdog(timeout_s=5.0):
    """
    Runs once, alongside _sdk_reader(). If no full scan has arrived within
    timeout_s seconds, prints a diagnostic instead of leaving the program
    silently blocked on _ready.wait() with no indication anything is wrong.
    This is what used to look like "failing silently" — the process was
    alive and rclpy was spinning, but _loop() had nothing to publish and
    said nothing about why.
    """
    time.sleep(timeout_s)
    if _n_scans[0] == 0 and not _ready.is_set():
        print(f"[lidar] WARNING: no full scan received after {timeout_s:.0f}s. "
              f"status={_status[0]!r}")
        print("[lidar] Common causes:")
        print(f"[lidar]   1. Wrong --port (currently {PORT}) — check `ls /dev/ttyUSB*`")
        print(f"[lidar]   2. Serial permission denied — try `sudo usermod -aG dialout $USER` "
              f"then re-login, or `sudo chmod 666 {PORT}` as a quick test")
        print(f"[lidar]   3. LiDAR motor not spinning / not powered")
        print(f"[lidar]   4. SDK output format differs from expected 'theta: X Dist: Y' — "
              f"see any [lidar:sdk-output] lines printed above for what it actually sent")
        print(f"[lidar]   5. Wrong baud (currently {BAUD}) for this LiDAR model")



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

# def _fit_line(pts, s, e):
#     """
#     Fit a line through pts[s..e] using total least squares
#     (orthogonal / perpendicular regression).

#     Returns (angle, distance, x1, y1, x2, y2, max_deviation) or None.

#     angle    — Hough angle [-pi/2, pi/2]
#     distance — perpendicular distance from origin
#     x1,y1    — start endpoint projected onto fitted line
#     x2,y2    — end endpoint projected onto fitted line
#     max_dev  — worst perpendicular deviation of any point from the line
#     """
#     n = e - s + 1
#     if n < 2:
#         return None

#     mx = sum(pts[i][0] for i in range(s, e+1)) / n
#     my = sum(pts[i][1] for i in range(s, e+1)) / n

#     Sxx = sum((pts[i][0]-mx)**2 for i in range(s, e+1))
#     Syy = sum((pts[i][1]-my)**2 for i in range(s, e+1))
#     Sxy = sum((pts[i][0]-mx)*(pts[i][1]-my) for i in range(s, e+1))

#     diff = Sxx - Syy
#     hyp  = math.hypot(diff, 2*Sxy)

#     if abs(Sxy) < 1e-12:
#         dx, dy = (1.0, 0.0) if Sxx >= Syy else (0.0, 1.0)
#     else:
#         lam_max = (Sxx + Syy + hyp) / 2.0
#         dx = Sxy
#         dy = lam_max - Sxx
#         L  = math.hypot(dx, dy)
#         dx /= L; dy /= L

#     angle = math.atan2(dy, dx)
#     if angle >  math.pi/2: angle -= math.pi
#     if angle < -math.pi/2: angle += math.pi

#     nx, ny   = -math.sin(angle), math.cos(angle)
#     distance = nx * mx + ny * my

#     max_dev = max(
#         abs(nx * (pts[i][0] - mx) + ny * (pts[i][1] - my))
#         for i in range(s, e+1)
#     )

#     tmin, tmax = 1e9, -1e9
#     for i in range(s, e+1):
#         t = (pts[i][0]-mx)*dx + (pts[i][1]-my)*dy
#         if t < tmin: tmin = t
#         if t > tmax: tmax = t

#     x1 = mx + tmin*dx;  y1 = my + tmin*dy
#     x2 = mx + tmax*dx;  y2 = my + tmax*dy

#     return angle, distance, x1, y1, x2, y2, max_dev


# def _fit_circle(pts, s, e):
#     """
#     Fit a circle using first, middle, last point (3-point method).
#     Returns (cx, cy, r, max_deviation) or None if points are collinear.

#     Radius sanity limits:
#       r > 2.0m = basically a flat wall, treat as line instead
#       r < 0.03m = noise spike, not a real physical object
#     """
#     ax, ay = pts[s]
#     bx, by = pts[(s + e) // 2]
#     cx, cy = pts[e]

#     d = 2 * (ax*(by - cy) + bx*(cy - ay) + cx*(ay - by))
#     if abs(d) < 1e-9:
#         return None   # collinear — no circle exists

#     ux = ((ax**2 + ay**2)*(by - cy) +
#           (bx**2 + by**2)*(cy - ay) +
#           (cx**2 + cy**2)*(ay - by)) / d

#     uy = ((ax**2 + ay**2)*(cx - bx) +
#           (bx**2 + by**2)*(ax - cx) +
#           (cx**2 + cy**2)*(bx - ax)) / d

#     r = math.hypot(ax - ux, ay - uy)

#     if r > 2.0 or r < 0.03:
#         return None

#     max_dev = max(
#         abs(math.hypot(pts[i][0] - ux, pts[i][1] - uy) - r)
#         for i in range(s, e+1)
#     )

#     return ux, uy, r, max_dev


# def _fit_first(pts, s, e, out, depth=0):
#     """
#     Core recursive fitter. Tries line, then arc, then splits in half.
#     depth limit prevents infinite recursion on genuinely noisy data.
#     """
#     if e - s < MIN_PTS - 1:
#         return
#     if depth > 6:
#         return

#     # ── Try line ──────────────────────────────────────────────────────────
#     line_result = _fit_line(pts, s, e)
#     if line_result is not None:
#         angle, distance, x1, y1, x2, y2, max_dev = line_result
#         length = math.hypot(x2-x1, y2-y1)
#         if max_dev < LINE_TOLERANCE and length >= MIN_LEN:
#             out.append({
#                 "type":     "line",
#                 "x1": x1, "y1": y1, "x2": x2, "y2": y2,
#                 "angle":    angle,
#                 "distance": distance,
#                 "length":   length,
#                 "n_pts":    e - s + 1,
#                 "pts_s":    s,
#                 "pts_e":    e,
#             })
#             return

#     # ── Try arc ───────────────────────────────────────────────────────────
#     if e - s >= MIN_PTS - 1:
#         arc_result = _fit_circle(pts, s, e)
#         if arc_result is not None:
#             cx, cy, r, max_dev = arc_result
#             if max_dev < ARC_TOLERANCE:
#                 theta_start = math.atan2(pts[s][1] - cy, pts[s][0] - cx)
#                 theta_end   = math.atan2(pts[e][1] - cy, pts[e][0] - cx)
#                 arc_len     = r * abs(theta_end - theta_start)
#                 if arc_len >= MIN_LEN:
#                     out.append({
#                         "type":        "arc",
#                         "cx": cx, "cy": cy, "r": r,
#                         "theta_start": theta_start,
#                         "theta_end":   theta_end,
#                         "length":      arc_len,
#                         "n_pts":       e - s + 1,
#                         "pts_s":       s,
#                         "pts_e":       e,
#                     })
#                     return

#     # ── Neither fit — split in half and recurse ───────────────────────────
#     mid = (s + e) // 2
#     _fit_first(pts, s,   mid, out, depth + 1)
#     _fit_first(pts, mid, e,   out, depth + 1)


# def split_merge(pts):
#     """
#     Main feature extraction entry point.
#     Returns (features, curve_groups, pts).

#     features     — list of dicts, each with type='line' or type='arc'
#     curve_groups — list of (i, i) index pairs for arc features (for RViz)
#     pts          — original cartesian point list (passed through unchanged)
#     """
#     if len(pts) < MIN_PTS:
#         return [], [], pts

#     # ── Gap detection ─────────────────────────────────────────────────────
#     surfaces = []
#     ss = 0
#     for i in range(1, len(pts)):
#         if math.hypot(pts[i][0]-pts[i-1][0], pts[i][1]-pts[i-1][1]) > GAP_THRESH:
#             if i - ss >= MIN_PTS:
#                 surfaces.append((ss, i-1))
#             ss = i
#     if len(pts) - ss >= MIN_PTS:
#         surfaces.append((ss, len(pts)-1))

#     # ── Fit each surface ──────────────────────────────────────────────────
#     all_features = []
#     for s, e in surfaces:
#         _fit_first(pts, s, e, all_features)
#         if len(all_features) >= MAX_LINES:
#             break

#     # ── Tag arcs as curve_groups for RViz publisher ───────────────────────
#     curve_groups = []
#     for i, f in enumerate(all_features):
#         if f["type"] == "arc":
#             f["curve_group"] = len(curve_groups)
#             curve_groups.append((i, i))

#     return all_features, curve_groups, pts


# # ── Bin grid -> cartesian list with intra-surface interpolation ──────────────

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
        self._scan_idx = 0
        self._slam     = SlamState()
        self._free_space = occupancy_grid.FreeSpaceAccumulator()
        self.pub_scan  = self.create_publisher(LaserScan,     "/scan",        10)
        self.pub_lines = self.create_publisher(MarkerArray,   "/lines",       10)
        self.pub_map   = self.create_publisher(OccupancyGrid, "/map",         10)
        self.pub_path  = self.create_publisher(Path,          "/trajectory",  10)
        self.pub_pose  = self.create_publisher(PoseStamped,   "/slam_out_pose", 10)
        self._path_msg = Path()
        self._path_msg.header.frame_id = "map"
        self.tf_broadcaster = TransformBroadcaster(self)
        threading.Thread(target=self._loop, daemon=True).start()
        self.get_logger().info("Waiting for SDK data...")
        self.get_logger().info(
            "RViz: Fixed Frame=map | Map topic=/map | Path topic=/trajectory | "
            "Pose topic=/slam_out_pose | add /scan /lines for live features")

    def _loop(self):
        while rclpy.ok():
            if not _ready.wait(timeout=2.0):
                continue
            _ready.clear()
            with _lock:
                snap = _bins.copy()
            self._pub_scan(snap)
            lines, curve_groups, raw_pts = self._pub_lines(snap)
            # SlamState owns matching + pose correction + the map-frame
            # transform; it re-matches internally against the corrected
            # pose, so the map stays anchored even as the LiDAR moves.
            # `lines` here are in the SENSOR frame — SlamState transforms
            # them before they ever reach MapManager.
            pose, match_result, pose_delta, delta_applied = self._slam.process_scan(
                lines, scan_idx=self._scan_idx
            )
            if self._slam.last_pose_frozen:
                # POSE FROZEN — see slam.py's MAX_UNTRUSTED_STREAK
                # docstring. The delta was magnitude-plausible but
                # withheld from current_pose entirely because too many
                # consecutive untrustworthy (off-trend/ambiguous) scans
                # have accumulated in a row — same "not enough
                # trustworthy information" treatment as too-few-matches,
                # just triggered by sustained distrust instead. Both pose
                # AND map are unchanged this scan; this is the strongest
                # of the three warning levels below (checked first) since
                # it means the system has declared itself lost, not just
                # cautious.
                self.get_logger().warn(
                    f"Scan#{self._scan_idx}  POSE FROZEN (streak="
                    f"{self._slam._untrusted_streak}): withholding delta "
                    f"dx={pose_delta.dx:+.3f} dy={pose_delta.dy:+.3f} "
                    f"dtheta={math.degrees(pose_delta.dtheta):+.1f}° — pose/map "
                    f"unchanged, waiting for a trustworthy match to recover"
                )
            elif pose_delta.valid and not delta_applied:
                # pose_estimator solved a transform for its matched pairs,
                # but the result implied an implausible per-scan jump —
                # most often caused by degraded scan data (occlusion,
                # motion blur) matching the wrong map entries while being
                # physically handled. Pose and map were NOT updated this
                # scan; see slam.py's _is_pose_delta_plausible.
                self.get_logger().warn(
                    f"Scan#{self._scan_idx}  REJECTED implausible pose delta: "
                    f"dx={pose_delta.dx:+.3f} dy={pose_delta.dy:+.3f} "
                    f"dtheta={math.degrees(pose_delta.dtheta):+.1f}°  "
                    f"(pose/map unchanged this scan)"
                )
            elif delta_applied and self._slam.last_coarse_ambiguous:
                # Pose WAS updated, but the correlative coarse search that
                # seeded it found a close competing peak (see
                # correlative_match.CoarseResult.ambiguous and slam.py's
                # coarse_ambiguous gate) — e.g. rectangular-room rotational
                # symmetry offering a second, nearly-as-good orientation.
                # Map writes were blocked this scan specifically so an
                # ambiguous coarse seed can never author the map evidence
                # that would otherwise let a wrong orientation lock itself
                # in permanently.
                self.get_logger().warn(
                    f"Scan#{self._scan_idx}  AMBIGUOUS COARSE SEED, pose delta applied but "
                    f"MAP WRITE BLOCKED: dx={pose_delta.dx:+.3f} dy={pose_delta.dy:+.3f} "
                    f"dtheta={math.degrees(pose_delta.dtheta):+.1f}°  "
                    f"(competing coarse peak nearly tied with the winner)"
                )
            elif delta_applied and not self._slam.last_delta_on_trend:
                # Pose WAS updated (it passed the single-scan magnitude
                # guard) but disagreed with the recent motion trend — see
                # slam.py's _is_consistent_with_trend. Map writes were
                # blocked this scan to prevent a wrong pose from writing
                # map evidence that would otherwise re-confirm itself on
                # the next scan (the exact mechanism behind the +10deg
                # heading lock-in seen on hardware after a slide).
                self.get_logger().warn(
                    f"Scan#{self._scan_idx}  OFF-TREND pose delta applied but MAP WRITE "
                    f"BLOCKED: dx={pose_delta.dx:+.3f} dy={pose_delta.dy:+.3f} "
                    f"dtheta={math.degrees(pose_delta.dtheta):+.1f}°  "
                    f"(disagrees with recent motion trend)"
                )
            # Refinement-loop diagnostics — see slam.py's SlamState.last_*
            # attributes. Logged every scan (throttled). coarse_* and
            # fine_total_* are now split apart (were previously only
            # visible combined) specifically so it's possible to tell
            # WHICH layer is producing a large correction: correlative_
            # match's coarse seed is bounded to +/-15cm/+/-15deg by its own
            # search window (correlative_match.SEARCH_DXY_MAX_M /
            # SEARCH_DTHETA_MAX_RAD) and cannot itself account for a true
            # per-scan motion larger than that; the fine loop's own total
            # is separately bounded per-iteration but was previously able
            # to accumulate across up to MAX_ITERATIONS steps with only a
            # single check at the very end. If coarse_dtheta is pinned
            # near its +/-15deg ceiling while fine_dtheta is ALSO large,
            # that's real motion exceeding the coarse search window, not a
            # fine-loop bug — the fix would be widening the coarse search,
            # not further tuning the refinement loop.
            self.get_logger().info(
                f"Scan#{self._scan_idx}  refine: iters={self._slam.last_iterations_run} "
                f"break={self._slam.last_break_reason} "
                f"coarse_valid={self._slam.last_coarse_valid} "
                f"coarse_ambiguous={self._slam.last_coarse_ambiguous} "
                f"coarse=({self._slam.last_coarse_dx:+.3f},{self._slam.last_coarse_dy:+.3f},"
                f"{math.degrees(self._slam.last_coarse_dtheta):+.1f}deg) "
                f"fine_total=({self._slam.last_fine_total_dx:+.3f},{self._slam.last_fine_total_dy:+.3f},"
                f"{math.degrees(self._slam.last_fine_total_dtheta):+.1f}deg) "
                f"final_weight={self._slam.last_final_total_weight:.3f} "
                f"final_eig={self._slam.last_final_min_eigenvalue:.4f} "
                f"untrusted_streak={self._slam._untrusted_streak} "
                f"pose_frozen={self._slam.last_pose_frozen}",
                throttle_duration_sec=1.0,
            )

            # Free-space accumulation for the /map OccupancyGrid (see
            # occupancy_grid.FreeSpaceAccumulator) — raycast this scan's raw
            # points from the current pose into the map frame. Skipped on
            # the same "rejected implausible delta" scans that already skip
            # the map write above (pose_delta.valid and not delta_applied):
            # self._slam.current_pose is stale relative to this scan's raw
            # data in that case, and raycasting with it would carve
            # incorrect free space at the wrong location. The off-trend-
            # but-applied case (map WRITE blocked, pose still moved) is
            # fine to raycast with — Step 4 in slam.py did apply that delta
            # to the pose, it's just not trusted enough yet to author new
            # wall entries.
            if not (pose_delta.valid and not delta_applied):
                cos_t = math.cos(pose.theta)
                sin_t = math.sin(pose.theta)
                map_frame_pts = [
                    (x * cos_t - y * sin_t + pose.x, y * cos_t + x * sin_t + pose.y)
                    for x, y in raw_pts
                ]
                self._free_space.update(pose.x, pose.y, map_frame_pts)

            self._publish_tf(pose)
            self._pub_path_and_pose(pose)
            self._pub_map()
            self._scan_idx += 1

    def _publish_tf(self, pose):
        """
        Publish the map -> laser transform from the current SLAM pose.

        This is what lets /map markers be published in the FIXED "map"
        frame while /scan and /lines stay in the moving "laser" frame —
        RViz composes them correctly through this transform instead of
        rendering everything as if it were rigidly attached to the
        sensor. Without this, moving the LiDAR makes the persistent map
        appear to shift/rebuild even when the underlying SLAM pose and
        map data are correct, because RViz has no relationship between
        "map" and "laser" to render against (this is the cause of the
        "No tf data" warning and the map jumping screenshots showed).

        pose.x, pose.y, pose.theta describe where "laser" sits in the
        "map" frame — exactly the parent(map) -> child(laser) direction
        TF expects, so no inversion is needed here.

        NOTE: _pub_scan, _pub_lines, this transform, and _pub_map each
        grab their own self.get_clock().now() rather than sharing one
        timestamp for the whole scan cycle. At 10Hz this skew is a few
        milliseconds and has not caused visible TF extrapolation issues,
        but if RViz ever complains about TF lookups in the future, unify
        these onto one timestamp captured at the top of _loop.
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "laser"
        t.transform.translation.x = float(pose.x)
        t.transform.translation.y = float(pose.y)
        t.transform.translation.z = 0.0
        # 2D rotation about Z -> quaternion (only qz, qw are non-zero)
        half = pose.theta / 2.0
        t.transform.rotation = Quaternion(
            x=0.0, y=0.0, z=math.sin(half), w=math.cos(half)
        )
        self.tf_broadcaster.sendTransform(t)

    def _pub_path_and_pose(self, pose):
        """
        Publish the running trajectory (/trajectory, nav_msgs/Path) and the
        current SLAM pose (/slam_out_pose, geometry_msgs/PoseStamped) — the
        same two topics the target RViz config (Path + Pose displays)
        expects, alongside /map now being a real OccupancyGrid (see
        _pub_map). Both are in the "map" frame, same as pose.x/y/theta.
        """
        now = self.get_clock().now().to_msg()
        half = pose.theta / 2.0
        orientation = Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))

        ps = PoseStamped()
        ps.header.stamp = now
        ps.header.frame_id = "map"
        ps.pose.position.x = float(pose.x)
        ps.pose.position.y = float(pose.y)
        ps.pose.position.z = 0.0
        ps.pose.orientation = orientation
        self.pub_pose.publish(ps)

        self._path_msg.header.stamp = now
        self._path_msg.poses.append(ps)
        if len(self._path_msg.poses) > MAX_PATH_POSES:
            # Bounded history — drop oldest, same reasoning as every other
            # fixed-capacity structure in this codebase (MapManager,
            # FeaturePacket, etc.), even though this is PC-only state.
            del self._path_msg.poses[0:len(self._path_msg.poses) - MAX_PATH_POSES]
        self.pub_path.publish(self._path_msg)

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

        # Feed this scan's features into the persistent map.
        # map_manager handles matching, weighted-average update, decay,
        # and static/dynamic classification internally.

        ma  = MarkerArray()
        clr = Marker()
        clr.header.frame_id = "laser"
        clr.header.stamp    = now
        clr.ns              = ""       
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
        ms      = self._slam.map.stats()
        p       = self._slam.current_pose
        self.get_logger().info(
            f"Scan#{_n_scans[0]}  {valid}/{NUM_BINS} bins  "
            f"-> {n_lines} lines  {n_arcs} arcs  |  "
            f"pose: x={p.x:+.3f} y={p.y:+.3f} th={math.degrees(p.theta):+.1f}°  |  "
            f"map: {ms['active']} entries "
            f"(S:{ms['static']} D:{ms['dynamic']} U:{ms['unclassified']})",
            throttle_duration_sec=2.0)
        self.pub_lines.publish(ma)
        return lines, curve_groups, _raw_pts
    
    def _pub_map(self):
        """
        Publish persistent map to /map as a real nav_msgs/OccupancyGrid
        (RViz "Map" display — black walls / white explored free space /
        gray unexplored), matching a standard occupancy-grid SLAM look.
        Walls come from the persistent line/arc MapEntry map
        (occupancy_grid.rasterize_map); free space comes from
        self._free_space, accumulated every scan in _loop via raycasting
        (see occupancy_grid.FreeSpaceAccumulator). Map is updated by
        SlamState.process_scan in _loop, which runs (and corrects the
        pose) before this is called.

        This is a PC-only rendering change — see occupancy_grid.py's
        module docstring. The underlying wall representation (MapEntry
        line/arc list) is unchanged; STATIC/DYNAMIC/UNCLASSIFIED status is
        no longer color-coded in this view (everything drawn is plain
        black/occupied) since OccupancyGrid has no per-cell color channel.
        Use /lines (still a MarkerArray) if you need the live status
        colors back.
        """
        active = self._slam.map.get_active()
        data, width, height, origin_x, origin_y = self._free_space.combine_with_walls(active)

        now = self.get_clock().now().to_msg()
        grid = OccupancyGrid()
        grid.header.stamp = now
        grid.header.frame_id = "map"
        grid.info.map_load_time = now
        grid.info.resolution = occupancy_grid.GRID_RESOLUTION_M
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = origin_x
        grid.info.origin.position.y = origin_y
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0
        grid.data = data

        self.pub_map.publish(grid)

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

    # SLAM state for matplotlib path — independent from the ROS node
    _mpl_slam = SlamState()

    # Status → matplotlib colour string
    _MPL_COLOR = {
        ENTRY_STATIC:       '#00e633',   # green
        ENTRY_DYNAMIC:      '#ff3333',   # red
        ENTRY_UNCLASSIFIED: '#888888',   # grey
    }

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

        # Pose correction + map update — features are in the SENSOR frame
        # here; SlamState transforms them into the map frame internally
        # before they reach MapManager, so the map stays anchored when the
        # LiDAR itself moves.
        _mpl_slam.process_scan(lines, scan_idx=_n_scans[0])

        # ── Draw live scan lines ──────────────────────────────────────────
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

        # ── Draw live scan arcs ───────────────────────────────────────────
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

        # ── Draw map layer ────────────────────────────────────────────────
        for entry in _mpl_slam.map.get_active():
            col = _MPL_COLOR.get(entry.status, '#888888')

            if entry.is_arc():
                cx, cy, r = entry.mx, entry.my, entry.distance
                if r < 0.01:
                    continue
                t_start = entry.theta_start
                span    = min(entry.length / r, 2 * math.pi)
                thetas  = [t_start + span * k / 20 for k in range(21)]
                xs = [cx + r * math.cos(th) for th in thetas]
                ys = [cy + r * math.sin(th) for th in thetas]
                rs_ = [math.hypot(x, y) for x, y in zip(xs, ys)]
                as_ = [math.atan2(y, x)  for x, y in zip(xs, ys)]
                art, = ax.plot(as_, rs_, color=col, linewidth=3.0,
                               alpha=0.55, linestyle='--')
                line_arts.append(art)
            else:
                # Line direction is perpendicular to Hough normal angle
                line_dir = entry.angle + math.pi / 2.0
                dx = math.cos(line_dir) * entry.length / 2.0
                dy = math.sin(line_dir) * entry.length / 2.0
                x1 = entry.mx - dx;  y1 = entry.my - dy
                x2 = entry.mx + dx;  y2 = entry.my + dy
                r1  = math.hypot(x1, y1)
                r2  = math.hypot(x2, y2)
                a1  = math.atan2(y1, x1)
                a2  = math.atan2(y2, x2)
                art, = ax.plot([a1, a2], [r1, r2], color=col,
                               linewidth=3.5, alpha=0.55, linestyle='--')
                line_arts.append(art)

        n_lines = sum(1 for f in lines if f["type"] == "line")
        ms      = _mpl_slam.map.stats()
        p       = _mpl_slam.current_pose
        info.set_text(
            f'Bins:{n}/{NUM_BINS}  Lines:{n_lines}  Arcs:{len(curve_groups)}  '
            f'Scans:{_n_scans[0]}  |  Pose: x={p.x:+.2f} y={p.y:+.2f} '
            f'th={math.degrees(p.theta):+.1f}°  |  Map:{ms["active"]} '
            f'(S:{ms["static"]} D:{ms["dynamic"]} U:{ms["unclassified"]})'
        )
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
    threading.Thread(target=_sdk_watchdog, daemon=True).start()
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