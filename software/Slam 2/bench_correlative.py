"""
bench_correlative.py
=====================
Measures search() wall-clock time for the ORIGINAL uploaded two-layer
correlative_match.py vs the restructured (cached Layer 2 + static-entry
cap) version, at map sizes representative of a maturing room map -- NOT
just the self-test's tiny 3-4 entry maps, since PERFORMANCE FIX 1 exists
specifically because cost was observed to blow up as the map GROWS.

This is the falsification test proposed earlier: does the measured
speedup come from the identified, specific causes (Layer 2's missing
rotation cache, unbounded static-entry scoring), or not.
"""
import importlib.util
import math
import random
import sys
import time
from collections import namedtuple

def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod

old = _load("old_correlative", "/home/claude/correlative_match_old_original.py")
fast = _load("fast_correlative", "/home/claude/correlative_match_old_fast.py")

_MapEntryStub = namedtuple("_MapEntryStub",
    ["angle", "distance", "mx", "my", "status", "active"])

class _ME(_MapEntryStub):
    def is_arc(self):
        return self.angle < -4.0

def _line_feat(angle, distance, x1, y1, x2, y2):
    return {"type": "line", "angle": angle, "distance": distance,
            "x1": x1, "y1": y1, "x2": x2, "y2": y2,
            "length": math.hypot(x2 - x1, y2 - y1), "quality": 100}

def make_map(n_entries, seed=0):
    """A synthetic mature map: n_entries STATIC walls scattered around a
    ~10m x 10m room, roughly what a real run accumulates over a few
    minutes (map_manager.MAX_MAP_ENTRIES=500 is the hard ceiling; 20-80
    active STATIC entries is a realistic mid-run figure for an indoor
    room with furniture)."""
    rng = random.Random(seed)
    entries = []
    for i in range(n_entries):
        angle = rng.choice([0.0, math.pi / 2, rng.uniform(-math.pi/2, math.pi/2)])
        distance = rng.uniform(0.3, 5.0)
        mx = rng.uniform(-5.0, 5.0)
        my = rng.uniform(-5.0, 5.0)
        entries.append(_ME(angle=angle, distance=distance, mx=mx, my=my,
                            status=1, active=True))
    return entries

def make_scan(n_features=15, seed=1):
    rng = random.Random(seed)
    feats = []
    for i in range(n_features):
        a = rng.uniform(-math.pi/2, math.pi/2)
        d = rng.uniform(0.3, 3.0)
        x1, y1 = rng.uniform(-1, 1), rng.uniform(-1, 1)
        x2, y2 = x1 + rng.uniform(-0.5, 0.5), y1 + rng.uniform(-0.5, 0.5)
        feats.append(_line_feat(a, d, x1, y1, x2, y2))
    return feats

def bench(module, map_entries, scan, n_runs=20):
    times = []
    for _ in range(n_runs):
        t0 = time.perf_counter()
        module.search(scan, map_entries, guess_x=0.3, guess_y=-0.2,
                       guess_theta=0.05)
        times.append(time.perf_counter() - t0)
    return sum(times) / len(times), min(times), max(times)

print(f"{'n_static':>10} | {'OLD avg (ms)':>14} | {'FAST avg (ms)':>14} | {'speedup':>8}")
print("-" * 58)
for n_static in (4, 10, 20, 40, 80, 150):
    map_entries = make_map(n_static)
    scan = make_scan(15)

    avg_old, min_old, max_old = bench(old, map_entries, scan)
    avg_fast, min_fast, max_fast = bench(fast, map_entries, scan)

    speedup = avg_old / avg_fast if avg_fast > 0 else float("inf")
    print(f"{n_static:>10} | {avg_old*1000:>14.2f} | {avg_fast*1000:>14.2f} | {speedup:>7.2f}x")

print()
print("Per-call breakdown at n_static=80 (FAST version's own instrumentation):")
map_entries = make_map(80)
scan = make_scan(15)
fast.search(scan, map_entries, guess_x=0.3, guess_y=-0.2, guess_theta=0.05)
print(f"  total   : {fast.last_search_total_s*1000:.3f} ms")
print(f"  layer1  : {fast.last_search_layer1_s*1000:.3f} ms "
      f"({fast.last_search_n_layer1_candidates} candidates)")
print(f"  layer2  : {fast.last_search_layer2_s*1000:.3f} ms "
      f"({fast.last_search_n_layer2_candidates} candidates)")
print(f"  n_static searched (post-cap): {fast.last_search_n_static}")