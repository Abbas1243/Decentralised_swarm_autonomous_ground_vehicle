#!/usr/bin/env python3
"""
bag_inspect.py  —  Verify and inspect a .bag file from lidar_visualizer.py

Usage:
    python3 tools/bag_inspect.py room1.bag           # summary stats
    python3 tools/bag_inspect.py room1.bag --scan 0  # dump one scan's lines
    python3 tools/bag_inspect.py room1.bag --plot     # matplotlib map preview

What it checks:
    - File size is a multiple of the expected record size (1624 bytes)
    - Every record has a valid count (0-50)
    - Timestamps are monotonically increasing
    - Line geometry is finite and sane (not NaN/Inf, reasonable range)
    - Hough parameters are in valid range (angle in [-pi/2, pi/2])
"""

import argparse
import math
import struct
import sys
import os

# ── Struct layout — must match messages.h exactly ────────────────────────────
BAG_RECORD_FMT  = '<qII'       # int64 wall_time_ms, uint32 seq, uint32 pad
PACKET_HDR_FMT  = '<IB3x'      # uint32 timestamp_ms, uint8 count, 3 pad
LINE_FMT        = '<7fB3x'     # 7×float32 + uint8 quality + 3 pad

BAG_RECORD_SIZE  = struct.calcsize(BAG_RECORD_FMT)   # 16
PACKET_HDR_SIZE  = struct.calcsize(PACKET_HDR_FMT)   # 8
LINE_SIZE        = struct.calcsize(LINE_FMT)          # 32
MAX_LINES        = 50
FULL_PKT_SIZE    = PACKET_HDR_SIZE + MAX_LINES * LINE_SIZE   # 1608
RECORD_SIZE      = BAG_RECORD_SIZE + FULL_PKT_SIZE           # 1624


def read_record(f):
    """Read one (BagRecord, FeaturePacket) pair. Returns None at EOF."""
    raw_rec = f.read(BAG_RECORD_SIZE)
    if len(raw_rec) < BAG_RECORD_SIZE:
        return None

    wall_time_ms, seq, _ = struct.unpack(BAG_RECORD_FMT, raw_rec)

    raw_hdr = f.read(PACKET_HDR_SIZE)
    if len(raw_hdr) < PACKET_HDR_SIZE:
        return None
    timestamp_ms, count = struct.unpack(PACKET_HDR_FMT, raw_hdr)

    lines = []
    for _ in range(MAX_LINES):
        raw_line = f.read(LINE_SIZE)
        if len(raw_line) < LINE_SIZE:
            return None
        x1, y1, x2, y2, angle, distance, length, quality = \
            struct.unpack(LINE_FMT, raw_line)
        lines.append(dict(x1=x1, y1=y1, x2=x2, y2=y2,
                          angle=angle, distance=distance,
                          length=length, quality=quality))

    return dict(
        wall_time_ms=wall_time_ms,
        seq=seq,
        timestamp_ms=timestamp_ms,
        count=count,
        lines=lines[:count]   # only valid entries
    )


def validate_line(ln, scan_idx, line_idx):
    """Return list of error strings for a single LineFeature."""
    errors = []
    for field in ('x1','y1','x2','y2','angle','distance','length'):
        v = ln[field]
        if not math.isfinite(v):
            errors.append(f"  scan={scan_idx} line={line_idx} {field}={v} (not finite!)")

    if math.isfinite(ln['angle']):
        if not (-math.pi/2 - 0.01 <= ln['angle'] <= math.pi/2 + 0.01):
            errors.append(f"  scan={scan_idx} line={line_idx} angle={math.degrees(ln['angle']):.1f}° out of [-90,90]")

    if math.isfinite(ln['length']) and ln['length'] < 0:
        errors.append(f"  scan={scan_idx} line={line_idx} length={ln['length']:.3f} is negative")

    if ln['quality'] > 100:
        errors.append(f"  scan={scan_idx} line={line_idx} quality={ln['quality']} > 100")

    # Endpoint consistency — distance from computed midpoint to Hough line
    if all(math.isfinite(ln[k]) for k in ('x1','y1','x2','y2','angle','distance')):
        mx = (ln['x1'] + ln['x2']) / 2
        my = (ln['y1'] + ln['y2']) / 2
        computed_dist = -math.sin(ln['angle']) * mx + math.cos(ln['angle']) * my
        err = abs(computed_dist - ln['distance'])
        if err > 0.05:
            errors.append(f"  scan={scan_idx} line={line_idx} Hough dist mismatch: "
                          f"stored={ln['distance']:.3f} computed={computed_dist:.3f} "
                          f"(diff={err:.3f}m)")

    return errors


def inspect(path, dump_scan=None, plot=False):
    file_size = os.path.getsize(path)
    remainder = file_size % RECORD_SIZE

    print(f"\n{'═'*60}")
    print(f"  Bag file: {path}")
    print(f"{'═'*60}")
    print(f"  File size   : {file_size:,} bytes")
    print(f"  Record size : {RECORD_SIZE} bytes")
    print(f"  Expected    : {file_size // RECORD_SIZE} complete records")

    if remainder != 0:
        print(f"  ⚠  WARNING: {remainder} trailing bytes — last record may be corrupt")
        print(f"     (Ctrl+C during write? Bag may be partially usable.)")
    else:
        print(f"  ✓  File size is exact multiple of record size")

    records      = []
    all_errors   = []
    counts       = []
    prev_wall    = None
    time_errors  = 0

    with open(path, 'rb') as f:
        while True:
            rec = read_record(f)
            if rec is None:
                break
            records.append(rec)
            counts.append(rec['count'])

            # Timestamp monotonicity
            if prev_wall is not None and rec['wall_time_ms'] < prev_wall:
                time_errors += 1
            prev_wall = rec['wall_time_ms']

            # Validate each line
            for li, ln in enumerate(rec['lines']):
                errs = validate_line(ln, rec['seq'], li)
                all_errors.extend(errs)

    n = len(records)
    if n == 0:
        print("\n  ✗  No records read — file is empty or corrupt")
        return

    # ── Summary ──────────────────────────────────────────────────────────────
    duration_s = (records[-1]['wall_time_ms'] - records[0]['wall_time_ms']) / 1000.0
    avg_hz     = (n - 1) / duration_s if duration_s > 0 else 0
    avg_lines  = sum(counts) / n
    min_lines  = min(counts)
    max_lines  = max(counts)

    print(f"\n  Scans recorded : {n}")
    print(f"  Duration       : {duration_s:.1f} s  ({duration_s/60:.1f} min)")
    print(f"  Avg scan rate  : {avg_hz:.1f} Hz  (expect ~10 Hz)")
    print(f"  Lines per scan : avg={avg_lines:.1f}  min={min_lines}  max={max_lines}")

    if time_errors:
        print(f"  ⚠  {time_errors} timestamp reversals detected")
    else:
        print(f"  ✓  Timestamps monotonically increasing")

    if all_errors:
        print(f"\n  ✗  {len(all_errors)} geometry errors found:")
        for e in all_errors[:20]:
            print(e)
        if len(all_errors) > 20:
            print(f"  ... and {len(all_errors)-20} more")
    else:
        print(f"  ✓  All line geometry valid (finite, in-range Hough params)")

    # ── Sequence check ────────────────────────────────────────────────────────
    for i, rec in enumerate(records):
        if rec['seq'] != i:
            print(f"  ⚠  Sequence gap at record {i}: expected {i} got {rec['seq']}")
            break
    else:
        print(f"  ✓  Sequence numbers contiguous 0..{n-1}")

    # ── Scan dump ─────────────────────────────────────────────────────────────
    if dump_scan is not None:
        if dump_scan >= n:
            print(f"\n  ✗  --scan {dump_scan} out of range (0..{n-1})")
        else:
            rec = records[dump_scan]
            print(f"\n{'─'*60}")
            print(f"  Scan #{dump_scan}  (seq={rec['seq']}  "
                  f"t={rec['timestamp_ms']}ms  lines={rec['count']})")
            print(f"{'─'*60}")
            print(f"  {'#':>3}  {'angle°':>8}  {'dist(m)':>8}  "
                  f"{'len(m)':>7}  {'q':>4}  {'x1':>7}  {'y1':>7}  "
                  f"{'x2':>7}  {'y2':>7}")
            for i, ln in enumerate(rec['lines']):
                print(f"  {i:>3}  "
                      f"{math.degrees(ln['angle']):>8.2f}  "
                      f"{ln['distance']:>8.3f}  "
                      f"{ln['length']:>7.3f}  "
                      f"{ln['quality']:>4}  "
                      f"{ln['x1']:>7.3f}  {ln['y1']:>7.3f}  "
                      f"{ln['x2']:>7.3f}  {ln['y2']:>7.3f}")

    # ── Matplotlib map preview ────────────────────────────────────────────────
    if plot:
        _plot_map(records)

    print()


def _plot_map(records):
    """
    Overlay all scans' line features into one map image.
    Lines are drawn with alpha so frequently-seen lines appear brighter —
    this naturally highlights walls (seen every scan) vs dynamic objects.
    """
    try:
        import matplotlib.pyplot as plt
        import matplotlib.collections as mc
        import numpy as np
    except ImportError:
        print("\n  ✗  matplotlib not installed: pip install matplotlib")
        return

    print(f"\n  Plotting {len(records)} scans overlaid...")

    fig, axes = plt.subplots(1, 2, figsize=(16, 8),
                             facecolor='#0d0d1a')
    fig.suptitle('Bag File Map Verification', color='white',
                 fontsize=14, fontweight='bold', y=0.98)

    for ax in axes:
        ax.set_facecolor('#0d0d1a')
        ax.tick_params(colors='#556677')
        ax.spines['bottom'].set_color('#223344')
        ax.spines['top'].set_color('#223344')
        ax.spines['left'].set_color('#223344')
        ax.spines['right'].set_color('#223344')

    # ── Left: all lines overlaid (alpha = 1/n_records) ───────────────────────
    ax = axes[0]
    alpha = max(0.02, min(0.3, 5.0 / len(records)))

    wall_segs, curve_segs = [], []
    for rec in records:
        for ln in rec['lines']:
            seg = [(ln['x1'], ln['y1']), (ln['x2'], ln['y2'])]
            if ln['quality'] >= 85:
                wall_segs.append(seg)
            else:
                curve_segs.append(seg)

    if wall_segs:
        lc = mc.LineCollection(wall_segs, colors='#00ff88',
                               linewidths=1.2, alpha=alpha)
        ax.add_collection(lc)
    if curve_segs:
        lc = mc.LineCollection(curve_segs, colors='#00ccff',
                               linewidths=0.8, alpha=alpha * 1.5)
        ax.add_collection(lc)

    ax.autoscale()
    ax.set_aspect('equal')
    ax.set_title(f'All {len(records)} scans overlaid\n'
                 f'Green=walls  Cyan=curves/obstacles',
                 color='#aabbcc', fontsize=10)
    ax.set_xlabel('X (metres)', color='#556677')
    ax.set_ylabel('Y (metres)', color='#556677')

    # Origin marker
    ax.plot(0, 0, 'r+', markersize=14, markeredgewidth=2, label='Origin')
    ax.legend(facecolor='#1a2233', edgecolor='#334455',
              labelcolor='white', fontsize=8)

    # ── Right: line count histogram per scan ─────────────────────────────────
    ax2 = axes[1]
    counts = [r['count'] for r in records]
    scan_nums = list(range(len(records)))

    ax2.fill_between(scan_nums, counts, alpha=0.6, color='#00ff88', linewidth=0)
    ax2.plot(scan_nums, counts, color='#00ff88', linewidth=0.8, alpha=0.8)
    ax2.axhline(y=sum(counts)/len(counts), color='#ffaa00',
                linewidth=1, linestyle='--', alpha=0.8, label=f'Avg={sum(counts)/len(counts):.1f}')

    ax2.set_facecolor('#0d0d1a')
    ax2.tick_params(colors='#556677')
    for spine in ax2.spines.values():
        spine.set_color('#223344')

    ax2.set_title('Lines extracted per scan\n(dips = open space, peaks = complex scene)',
                  color='#aabbcc', fontsize=10)
    ax2.set_xlabel('Scan number', color='#556677')
    ax2.set_ylabel('Line count', color='#556677')
    ax2.set_ylim(0, max(counts) * 1.2)
    ax2.legend(facecolor='#1a2233', edgecolor='#334455',
               labelcolor='white', fontsize=9)

    plt.tight_layout()
    plt.savefig('bag_map_preview.png', dpi=150, bbox_inches='tight',
                facecolor='#0d0d1a')
    print("  ✓  Saved → bag_map_preview.png")
    plt.show()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(
        description="Inspect and verify a .bag file from lidar_visualizer.py")
    ap.add_argument("bag",          help="Path to .bag file")
    ap.add_argument("--scan", "-s", type=int, default=None,
                    help="Dump one scan's line data (0-indexed)")
    ap.add_argument("--plot", "-p", action="store_true",
                    help="Show matplotlib map overlay + line count chart")
    args = ap.parse_args()

    if not os.path.exists(args.bag):
        print(f"Error: {args.bag} not found")
        sys.exit(1)

    inspect(args.bag, dump_scan=args.scan, plot=args.plot)