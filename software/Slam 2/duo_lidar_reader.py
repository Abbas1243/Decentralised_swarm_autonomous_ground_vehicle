#!/usr/bin/env python3
"""
duo_lidar_reader.py
====================
Direct RPLIDAR A1 legacy-scan protocol reader, runs ON the Duo S.

WHY THIS EXISTS INSTEAD OF THE SDK SUBPROCESS
-----------------------------------------------
lidar_visualizer.py's PC path spawns the RPLIDAR SDK's `ultra_simple`
binary and parses its stdout. That binary is not cross-compiled for this
board, and this is a minimal SD-card-only Buildroot image (no eMMC, per
slam_build_prompt.md) — adding a full SDK cross-compile + build system
just to get scan data is more moving parts than needed. The RPLIDAR A1's
wire protocol (legacy scan mode) is simple and well documented; this
module talks to it directly over the USB-serial port with pyserial, no
subprocess, no SDK dependency at all.

PROTOCOL (RPLIDAR A1, legacy/normal scan mode — NOT express scan)
--------------------------------------------------------------------
Request packet:      0xA5 <cmd byte> [payload...]
  RESET       = 0x40   — soft reset, device re-boots, discard boot banner
  SCAN        = 0x20   — start continuous legacy scan, no payload
  STOP        = 0x25   — stop current scan

Response after SCAN command — one 7-byte descriptor:
  0xA5 0x5A <4-byte data-response length + send-mode, little endian>
  <1-byte data type>
followed by a continuous, unbounded stream of 5-byte measurement nodes
until STOP is sent:

  byte0            : sync_quality
                        bit0      = S  (start-of-new-scan flag)
                        bit1      = !S (complement of bit0 — sanity check)
                        bits[7:2] = quality (6 bits)
  bytes1-2 (u16 LE) : angle_q6_checkbit
                        bit0      = check bit, must always be 1
                        bits[15:1]= angle_q6  (angle in degrees * 64)
  bytes3-4 (u16 LE) : distance_q2  (distance in mm * 4; 0 = no return)

  angle_deg = angle_q6 / 64.0
  distance_mm = distance_q2 / 4.0

A new full revolution is detected the same way lidar_visualizer.py's SDK
parser already does it: when the reported angle wraps backward (this
node's angle < previous angle - 180), a full scan just completed.

USAGE
-----
    reader = RplidarReader(port="/dev/ttyUSB0", baud=115200)
    reader.start()
    reader.ready.wait()
    while True:
        reader.new_scan.wait(timeout=2.0)
        reader.new_scan.clear()
        bins = reader.snapshot()   # NUM_BINS-length np.float32 array, meters, NaN=no return
        ...
    reader.stop()

Same NUM_BINS / BIN_DEG / MIN_MM / MAX_MM convention as
lidar_visualizer.py so bins_to_pts() and everything downstream
(fit_first / split_merge) is completely unchanged.
"""

import math
import threading
import time

import numpy as np
import serial

# ── Config — matches lidar_visualizer.py's constants ─────────────────────────
NUM_BINS = 2000
BIN_DEG = 360.0 / NUM_BINS
MIN_MM = 15
MAX_MM = 8000

CMD_RESET = 0x40
CMD_SCAN = 0x20
CMD_STOP = 0x25

SYNC_BYTE = 0xA5


def _angle_to_bin(deg):
    return int(round(deg / 360.0 * NUM_BINS)) % NUM_BINS


class RplidarReader:
    """
    Background-thread RPLIDAR A1 reader. Owns the serial port, the shared
    bins array, and scan-boundary detection. Mirrors the shared-state
    pattern lidar_visualizer.py's module-level _bins/_lock/_ready already
    used, just packaged as a class instead of module globals so a single
    process (duo_slam_server.py) can own it cleanly alongside SlamState.
    """

    def __init__(self, port="/dev/ttyUSB0", baud=115200, min_mm=MIN_MM, max_mm=MAX_MM):
        self.port_name = port
        self.baud = baud
        self.min_mm = min_mm
        self.max_mm = max_mm

        self._bins = np.full(NUM_BINS, np.nan, dtype=np.float32)
        self._lock = threading.Lock()
        self.new_scan = threading.Event()   # set once per completed revolution
        self.ready = threading.Event()      # set once the serial link is up
        self.scan_count = 0
        self.status = "not started"

        self._ser = None
        self._thread = None
        self._stop_flag = threading.Event()

    # ------------------------------------------------------------------ #
    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_flag.set()
        if self._ser is not None:
            try:
                self._send_command(CMD_STOP)
            except Exception:
                pass
            try:
                self._ser.close()
            except Exception:
                pass

    def snapshot(self):
        """Copy of the current bins array — meters, NaN where no return."""
        with self._lock:
            return self._bins.copy()

    # ------------------------------------------------------------------ #
    def _send_command(self, cmd, payload=b""):
        packet = bytes([SYNC_BYTE, cmd]) + payload
        self._ser.write(packet)

    def _read_exact(self, n, timeout_s=2.0):
        deadline = time.time() + timeout_s
        buf = b""
        while len(buf) < n and time.time() < deadline:
            chunk = self._ser.read(n - len(buf))
            if chunk:
                buf += chunk
        if len(buf) < n:
            raise IOError(f"timed out reading {n} bytes, got {len(buf)}")
        return buf

    def _start_scan(self):
        # RESET first — clears any state left over from a previous run
        # (e.g. this process crashed mid-scan last time). Boot banner
        # response is variable-length ASCII text; just drain whatever
        # shows up for a short window rather than parsing it.
        self._send_command(CMD_RESET)
        time.sleep(0.5)
        self._ser.reset_input_buffer()

        self._send_command(CMD_SCAN)
        descriptor = self._read_exact(7)
        if descriptor[0] != 0xA5 or descriptor[1] != 0x5A:
            raise IOError(f"bad scan descriptor header: {descriptor[:2].hex()}")
        # descriptor[2:6] = length+mode, descriptor[6] = data type —
        # not validated further; legacy scan mode always returns 5-byte
        # nodes regardless, which is all _run() below assumes.

    # ------------------------------------------------------------------ #
    def _run(self):
        try:
            self._ser = serial.Serial(
                self.port_name, self.baud, timeout=0.2,
            )
            self.status = f"serial open on {self.port_name}"
            print(f"[duo_lidar] opened {self.port_name} @ {self.baud}")

            self._start_scan()
            self.status = "scanning"
            self.ready.set()
            print("[duo_lidar] scan started")

            prev_angle = None

            while not self._stop_flag.is_set():
                node = self._read_exact(5, timeout_s=1.0)

                sync_quality = node[0]
                s_bit = sync_quality & 0x01
                not_s_bit = (sync_quality >> 1) & 0x01
                if s_bit == not_s_bit:
                    # Sanity check failed — frame is misaligned. Resync by
                    # dropping one byte at a time until the check passes
                    # again, rather than silently trusting garbage.
                    continue

                angle_q6_checkbit = node[1] | (node[2] << 8)
                if (angle_q6_checkbit & 0x01) != 1:
                    continue   # check bit must be 1 — misaligned frame
                angle_q6 = angle_q6_checkbit >> 1
                angle_deg = angle_q6 / 64.0

                distance_q2 = node[3] | (node[4] << 8)
                dist_mm = distance_q2 / 4.0

                if prev_angle is not None and angle_deg < prev_angle - 180.0:
                    self.scan_count += 1
                    self.new_scan.set()
                prev_angle = angle_deg

                idx = _angle_to_bin(angle_deg)
                with self._lock:
                    if self.min_mm < dist_mm < self.max_mm:
                        self._bins[idx] = dist_mm / 1000.0
                    else:
                        self._bins[idx] = np.nan

        except Exception as e:
            self.status = f"ERROR: {e}"
            print(f"[duo_lidar] {e}")
            print(f"[duo_lidar] check: port exists (ls /dev/ttyUSB*), "
                  f"permissions (dialout group / udev rule), cable seated, "
                  f"motor spinning.")


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    print("duo_lidar_reader standalone test — prints scan stats, no SLAM")
    r = RplidarReader(port=args.port, baud=args.baud)
    r.start()
    if not r.ready.wait(timeout=5.0):
        print(f"never got ready, status={r.status}")
        raise SystemExit(1)

    try:
        while True:
            r.new_scan.wait(timeout=3.0)
            r.new_scan.clear()
            bins = r.snapshot()
            valid = int(np.sum(~np.isnan(bins)))
            print(f"scan #{r.scan_count}  {valid}/{NUM_BINS} valid bins  "
                  f"min={np.nanmin(bins):.2f}m max={np.nanmax(bins):.2f}m")
    except KeyboardInterrupt:
        pass
    finally:
        r.stop()
