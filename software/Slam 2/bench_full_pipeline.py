"""
bench_full_pipeline.py
========================
Answers the actual question raised by the hardware log: is
correlative_match.search() (coarse) the per-scan bottleneck, or is it
soft_scan_matcher's fine refinement loop (which has NO static-entry cap,
unlike the fixed correlative_match.py) -- or map_manager.update()?

Builds a synthetic room whose map size/scan feature count matches the
actual hardware log (S:43-60 static entries, ~25-30 scan features per
scan), runs it through SlamState.process_scan() many times, and reports
the per-stage timing breakdown the new last_timing_* instrumentation now
exposes -- plus how each stage scales as static map size grows, which is
the real test for "handling fast rotation" (bigger real motion -> more
STATIC entries stay in view / get matched -> if a stage is uncapped,
its cost grows with map size regardless of how fast the CPU is).
"""
import math
import random
import time
import sys

sys.path.insert(0, '.')
from slam import SlamState, Pose
from map_manager import MapEntry, ENTRY_STATIC

def make_static_map(n_entries, seed=0):
    """Directly inject mature STATIC entries into a fresh MapManager --
    bypasses the MIN_OBS_FOR_STATIC maturation wait, which is irrelevant
    to what we're timing here (steady-state per-scan cost, not map
    bootstrap time)."""
    rng = random.Random(seed)
    entries = []
    for i in range(n_entries):
        if rng.random() < 0.15:
            # occasional arc, matching the log's "5 arcs" alongside lines
            entries.append(MapEntry(
                angle=-10.0, distance=rng.uniform(0.1, 0.5),
                length=0.3, mx=rng.uniform(-5, 5), my=rng.uniform(-5, 5),
                observed=40, status=ENTRY_STATIC, active=True,
            ))
        else:
            angle = rng.choice([0.0, math.pi / 2, rng.uniform(-math.pi/2, math.pi/2)])
            entries.append(MapEntry(
                angle=angle, distance=rng.uniform(0.3, 5.0), length=1.0,
                mx=rng.uniform(-5, 5), my=rng.uniform(-5, 5),
                observed=40, status=ENTRY_STATIC, active=True,
            ))
    return entries

def make_scan(n_features=27, seed=1):
    """Scan features near a small consistent cluster of REAL walls (so the
    fine loop gets genuine weighted matches, not just noise it immediately
    breaks out of on low_weight) -- mirrors what a real LiDAR sees: a few
    nearby walls dominate, regardless of total map size elsewhere."""
    rng = random.Random(seed)
    feats = []
    base_walls = [(0.0, 1.0), (math.pi/2, 1.6), (-math.pi/3, 1.2), (math.pi/6, 0.9)]
    for i in range(n_features):
        ma, md = rng.choice(base_walls)
        a = ma + rng.uniform(-0.03, 0.03)
        d = md + rng.uniform(-0.03, 0.03)
        dirx, diry = math.cos(a), math.sin(a)
        fx, fy = -math.sin(a) * d, math.cos(a) * d
        x1 = fx - 0.4 * dirx; y1 = fy - 0.4 * diry
        x2 = fx + 0.4 * dirx; y2 = fy + 0.4 * diry
        feats.append({"type": "line", "angle": a, "distance": d,
                       "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                       "length": 0.8, "quality": 95})
    return feats

def run_bench(n_static, n_scans=40):
    slam = SlamState()
    slam.map._entries = make_static_map(n_static)
    # Also seed the SAME 4 base walls as STATIC so the scan actually
    # matches something real and the fine loop does genuine work instead
    # of breaking immediately -- appended on top of the size-n_static
    # "background" map that only exists to load the uncapped stages.
    for ma, md in [(0.0, 1.0), (math.pi/2, 1.6), (-math.pi/3, 1.2), (math.pi/6, 0.9)]:
        slam.map._entries.append(MapEntry(
            angle=ma, distance=md, length=1.0, mx=0.0, my=0.0,
            observed=40, status=ENTRY_STATIC, active=True,
        ))

    timings = {"coarse": [], "fine": [], "map_update": [], "transform": [], "total": []}
    for i in range(n_scans):
        scan = make_scan(seed=i)
        slam.process_scan(scan, scan_idx=i)
        timings["coarse"].append(slam.last_timing_coarse_s)
        timings["fine"].append(slam.last_timing_fine_s)
        timings["map_update"].append(slam.last_timing_map_update_s)
        timings["transform"].append(slam.last_timing_transform_s)
        timings["total"].append(slam.last_timing_total_s)

    return {k: (sum(v)/len(v), max(v)) for k, v in timings.items()}


print(f"{'n_static':>10} | {'coarse avg/max':>18} | {'fine avg/max':>18} | "
      f"{'map avg/max':>16} | {'TOTAL avg/max':>18} | {'%budget(100ms)':>14}")
print("-" * 108)
for n_static in (10, 40, 60, 100, 200, 400):
    r = run_bench(n_static)
    total_avg_ms = r["total"][0] * 1000
    print(f"{n_static:>10} | "
          f"{r['coarse'][0]*1000:>7.2f}/{r['coarse'][1]*1000:>7.2f} | "
          f"{r['fine'][0]*1000:>7.2f}/{r['fine'][1]*1000:>7.2f} | "
          f"{r['map_update'][0]*1000:>6.2f}/{r['map_update'][1]*1000:>6.2f} | "
          f"{total_avg_ms:>7.2f}/{r['total'][1]*1000:>7.2f} | "
          f"{total_avg_ms:>13.1f}%")