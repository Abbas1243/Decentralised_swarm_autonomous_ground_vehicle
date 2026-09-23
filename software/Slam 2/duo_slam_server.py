#!/usr/bin/env python3
"""
duo_slam_server.py
====================
Runs ON the Milk-V Duo S. Owns the whole live pipeline:

    RplidarReader (duo_lidar_reader.py)
        -> bins_to_pts()               (ported inline below, same as
                                         lidar_visualizer.py's version)
        -> fit_first_ctypes.split_merge()   (needs libfit_first.so built
                                              for aarch64 — see NOTE below)
        -> SlamState.process_scan()    (slam.py, pure Python — unchanged)
        -> compact per-scan JSON line  -> TCP socket -> laptop bridge

NO ROS2, NO ZENOH ON THE DUO S. This process only opens a plain TCP
socket server; laptop_bridge.py is the only client, and it's the one
that talks to ROS2 (on the laptop, not here). This is the direct
replacement for the STM32-feature-extraction + Zenoh-publish path in
slam_build_prompt.md's original architecture, now that both of those are
explicitly being skipped for this phase.

NOTE — libfit_first.so IS NOT YET CROSS-COMPILED FOR aarch64
-----------------------------------------------------------------
slam_progress_summary.docx lists libfit_first.so as built via
    gcc -O2 -ffast-math -shared -fPIC
which is the PC/x86 build. fit_first.c/.h were never uploaded to this
project, so I can't cross-compile the aarch64 version here. Once you
have fit_first.c/.h on the Duo S dev machine, the same recipe
slam_build_prompt.md already specifies for every other module applies:

    aarch64-linux-gnu-gcc -O2 -ffast-math -shared -fPIC \\
        -o libfit_first_arm64.so fit_first.c

...then place the resulting .so wherever fit_first_ctypes.py expects it
(check that file's own load path) on the Duo S filesystem. Until then,
this script will raise ImportError on startup with a clear message
rather than silently failing later.

NOTE — correlative_match PERFORMANCE
-----------------------------------------------------------------
Per slam_progress_compact.md, the pure-Python correlative_match.search()
measured ~2.9s/scan on real Duo S hardware — ~29x over the 100ms/10Hz
budget. A C port + arm64 .so + ctypes bridge exists per
slam_progress_correlative_match_c_port.md but those files (correlative_
match.c/.h, testing_bridge/) were not uploaded to this project either,
so slam.py is used here UNMODIFIED (pure Python correlative_match.py).
Expect scan processing to be seconds, not tens-of-milliseconds, per
scan until that C module is wired in — this script logs per-scan timing
every scan specifically so that's visible and not mistaken for a hang.

WIRE FORMAT (Duo S -> laptop, one JSON object per line, newline-delimited)
-----------------------------------------------------------------
{
  "scan_idx": int,
  "pose": {"x": float, "y": float, "theta": float},
  "delta_applied": bool,
  "map_updated": bool,
  "map": [
     {"type": "line", "angle":, "distance":, "mx":, "my":, "length":, "status": 0|1|2},
     {"type": "arc",  "r":,     "mx":, "my":, "theta_start":, "theta_end":, "length":, "status": 0|1|2},
     ...
  ],
  "lines": [ <same-shaped live scan features, SENSOR frame, for /lines display> ]
}
Kept deliberately small — no raw point cloud, no occupancy grid (that's
PC-only per occupancy_grid.py's own docstring anyway). map + lines
together are typically a few KB, trivial over WiFi at 5-10Hz.
"""

import argparse
import json
import math
import socket
import sys
import threading
import time

import numpy as np

from duo_lidar_reader import RplidarReader, NUM_BINS, BIN_DEG

try:
    from fit_first_ctypes import split_merge
except ImportError as e:
    print("[duo_slam_server] FATAL: fit_first_ctypes.split_merge unavailable "
          f"({e}). libfit_first.so almost certainly needs to be cross-"
          "compiled for aarch64 and placed where fit_first_ctypes.py "
          "expects it -- see this file's module docstring.")
    sys.exit(1)

from slam import SlamState


# ── bins -> cartesian points, same interpolation as lidar_visualizer.py ──────
MAX_INTERP_BINS = 8
MAX_INTERP_JUMP = 0.30


def bins_to_pts(dist_m):
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
        left_idx, right_idx = run_start - 1, run_end
        if left_idx < 0 or right_idx >= NUM_BINS:
            continue
        r_left, r_right = filled[left_idx], filled[right_idx]
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


def _entry_to_wire(e):
    d = {"type": "arc" if e.is_arc() else "line",
         "mx": e.mx, "my": e.my, "length": e.length, "status": e.status}
    if e.is_arc():
        d.update(r=e.distance, theta_start=e.theta_start, theta_end=e.theta_end)
    else:
        d.update(angle=e.angle, distance=e.distance)
    return d


def _feat_to_wire(f):
    if f["type"] == "line":
        return {"type": "line", "x1": f["x1"], "y1": f["y1"],
                "x2": f["x2"], "y2": f["y2"], "angle": f["angle"],
                "distance": f["distance"], "length": f["length"]}
    else:
        return {"type": "arc", "cx": f["cx"], "cy": f["cy"], "r": f["r"],
                "theta_start": f.get("theta_start", 0.0),
                "theta_end": f.get("theta_end", 0.0), "length": f["length"]}


# ── TCP socket server — single client, newline-delimited JSON, best-effort ──

class ScanStreamServer:
    """
    Plain TCP server. Accepts ONE client at a time (the laptop bridge).
    Send is best-effort: if the client is slow/gone, a send failure just
    drops that scan's message and waits for a (re)connection -- SLAM
    itself never blocks on the network, matching the "the Duo S can't run
    Zenoh + SLAM at once" constraint that got us here: no network stack
    should ever be allowed to stall the SLAM loop.
    """

    def __init__(self, host="0.0.0.0", port=9191):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((host, port))
        self._sock.listen(1)
        self._sock.settimeout(0.5)
        self._client = None
        self._client_lock = threading.Lock()
        self._stop = threading.Event()
        self._accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._accept_thread.start()
        print(f"[duo_slam_server] listening on {host}:{port}")

    def _accept_loop(self):
        while not self._stop.is_set():
            try:
                conn, addr = self._sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            print(f"[duo_slam_server] client connected: {addr}")
            with self._client_lock:
                if self._client is not None:
                    try:
                        self._client.close()
                    except Exception:
                        pass
                self._client = conn

    def send(self, obj):
        with self._client_lock:
            client = self._client
        if client is None:
            return
        try:
            payload = (json.dumps(obj) + "\n").encode("utf-8")
            client.sendall(payload)
        except OSError:
            with self._client_lock:
                if self._client is client:
                    self._client = None

    def close(self):
        self._stop.set()
        with self._client_lock:
            if self._client is not None:
                self._client.close()
        self._sock.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--lidar-port", default="/dev/ttyUSB0")
    ap.add_argument("--lidar-baud", type=int, default=115200)
    ap.add_argument("--tcp-port", type=int, default=9191)
    ap.add_argument("--send-live-lines", action="store_true", default=True,
                     help="include this scan's raw features (sensor frame) "
                          "in the wire message, for a live /lines display "
                          "alongside the persistent /map. Adds a little "
                          "bandwidth; drop with --no-send-live-lines if the "
                          "link is thin.")
    ap.add_argument("--no-send-live-lines", dest="send_live_lines", action="store_false")
    args = ap.parse_args()

    reader = RplidarReader(port=args.lidar_port, baud=args.lidar_baud)
    reader.start()
    print("[duo_slam_server] waiting for LiDAR link...")
    if not reader.ready.wait(timeout=10.0):
        print(f"[duo_slam_server] FATAL: LiDAR never came up, status={reader.status}")
        sys.exit(1)

    server = ScanStreamServer(port=args.tcp_port)
    slam = SlamState()
    scan_idx = 0

    print("[duo_slam_server] running. Ctrl-C to stop.")
    try:
        while True:
            if not reader.new_scan.wait(timeout=2.0):
                print("[duo_slam_server] no scan in 2s -- check LiDAR "
                      f"(status={reader.status})")
                continue
            reader.new_scan.clear()

            t0 = time.perf_counter()
            bins = reader.snapshot()
            pts = bins_to_pts(bins)
            lines, curve_groups, _raw_pts = split_merge(pts)

            pose, match_result, pose_delta, delta_applied = slam.process_scan(
                lines, scan_idx=scan_idx
            )
            t_total = time.perf_counter() - t0

            msg = {
                "scan_idx": scan_idx,
                "pose": {"x": pose.x, "y": pose.y, "theta": pose.theta},
                "delta_applied": bool(delta_applied),
                "map_updated": bool(slam.last_map_updated),
                "map": [_entry_to_wire(e) for e in slam.map.get_active()],
            }
            if args.send_live_lines:
                msg["lines"] = [_feat_to_wire(f) for f in lines]

            server.send(msg)

            ms = slam.map.stats()
            print(f"[duo_slam_server] scan#{scan_idx}  {t_total*1000:.0f}ms  "
                  f"pose=({pose.x:+.2f},{pose.y:+.2f},{math.degrees(pose.theta):+.1f}deg)  "
                  f"map={ms['active']}(S:{ms['static']} D:{ms['dynamic']} U:{ms['unclassified']})  "
                  f"applied={delta_applied}",
                  flush=True)

            scan_idx += 1

    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()
        server.close()


if __name__ == "__main__":
    main()
