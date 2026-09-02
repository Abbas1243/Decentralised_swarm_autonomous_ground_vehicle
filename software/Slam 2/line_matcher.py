"""
line_matcher.py
===============
Feature matching: scan features vs map entries.

Matches both LINE and ARC features from a new scan against the current map.
Returns matched pairs with scores, and lists of unmatched indices on both sides.

This module is intentionally kept as a single pure function with no global
state.  All state lives in map_manager.py.  This makes the C port
(slam_core/line_matcher.c) a direct translation with no surprises.

MATCHING RULES (from slam_build_prompt.md Algorithm 2, extended for arcs)
--------------------------------------------------------------------------
LINE matching — Hough space distance:
    angle_diff = |scan.angle - map.angle|
    if angle_diff > PI/2: angle_diff = PI - angle_diff   (lines have PI symmetry)
    dist_diff  = |scan.distance - map.distance|
    accept if: angle_diff < ANGLE_THRESH_RAD and dist_diff < DIST_THRESH_M
    score     = angle_diff * ANGLE_WEIGHT + dist_diff    (lower is better)

ARC matching — geometric distance:
    centre_dist = hypot(scan.mx - map.mx, scan.my - map.my)
    r_diff      = |scan.r - map.r|
    accept if: centre_dist < CENTRE_THRESH_M and r_diff < R_THRESH_M
    score     = centre_dist + r_diff * R_WEIGHT          (lower is better)

Each scan feature is matched to at most ONE map entry (best score wins).
Each map entry can be matched by at most ONE scan feature (one-to-one).

OUTPUT
------
MatchResult namedtuple:
    matched        list of (scan_idx, map_idx, score)   sorted by score asc
    unmatched_scan list of scan_idx  with no map match
    unmatched_map  list of map_idx   with no scan match

UNITS
-----
All distances in metres, all angles in radians.  Same as Feature struct.

PORT PATH
---------
When porting to C (slam_core/line_matcher.c):
    match_features(scan_feats, map_entries)
    →  int line_matcher_match(const ScanFeature *scan, int n_scan,
                               const MapEntry    *map,  int n_map,
                               MatchPair *out, int *n_matched)
    Constants below become #defines in line_matcher.h.
"""

import math
from collections import namedtuple

# ---------------------------------------------------------------------------
# Tuning constants — mirror slam.conf values
# ---------------------------------------------------------------------------

ANGLE_THRESH_RAD = 0.12     # max angular difference for line match
                             # Real per-scan motion is at most ~15-30cm / a
                             # few degrees. A wide threshold (was 0.20 rad)
                             # lets a scan line match the WRONG nearby wall
                             # once the viewpoint shifts even moderately —
                             # especially dangerous in rooms with many
                             # perpendicular walls (0/90deg sets), where a
                             # rotation error approaching this threshold can
                             # make a wall start matching the WRONG wall set
                             # entirely. Tightened to reduce that window.
DIST_THRESH_M    = 0.18     # max Hough distance difference for line match
                             # Was 0.35m, more than the real per-scan
                             # translation bound — tightened so a wall can
                             # only match its true counterpart.
ANGLE_WEIGHT     = 2.0      # how much angle error counts vs distance error

CENTRE_THRESH_M  = 0.25     # max centre-to-centre distance for arc match
R_THRESH_M       = 0.08     # max radius difference for arc match
R_WEIGHT         = 1.5      # how much radius error counts vs centre error

MIN_QUALITY      = 60        # scan features below this quality are skipped
                             # LOOPHOLE FIX: was 100 (the literal maximum
                             # possible score from fit_first) -- requiring
                             # perfection meant any scan with the slightest
                             # noise, motion blur, or partial occlusion
                             # produced ZERO usable features that scan, not
                             # degraded-but-usable matching. That starved
                             # pose_estimator of matches even when the pose
                             # itself was fine, forcing far more "skip this
                             # scan" outcomes than necessary and widening
                             # the real-motion gaps that let pose diverge
                             # in the first place. Restored to the ~60-70
                             # range this module's own historical comments
                             # describe as the working value.

# ---------------------------------------------------------------------------
# Return type
# ---------------------------------------------------------------------------

MatchResult = namedtuple(
    "MatchResult",
    ["matched", "unmatched_scan", "unmatched_map"]
)
# matched        : list of (scan_idx: int, map_idx: int, score: float)
# unmatched_scan : list of int  — scan indices with no match
# unmatched_map  : list of int  — map indices with no match


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _angle_diff_line(a, b):
    """
    Angular difference between two Hough line angles.
    Lines have PI symmetry — angle and angle+PI describe the same line.
    Result is in [0, PI/2].
    """
    diff = abs(a - b) % math.pi
    if diff > math.pi / 2.0:
        diff = math.pi - diff
    return diff


def _score_line(scan_feat, map_entry):
    adiff = _angle_diff_line(scan_feat["angle"], map_entry.angle)
    if adiff >= ANGLE_THRESH_RAD:
        return None

    # Check both sign conventions — Hough distance can be positive or negative
    # for the same wall depending on eigenvector direction from fit_first
    ddiff = abs(scan_feat["distance"] - map_entry.distance)
    ddiff_flip = abs(scan_feat["distance"] + map_entry.distance)
    ddiff = min(ddiff, ddiff_flip)

    if ddiff >= DIST_THRESH_M:
        return None

    score = adiff * ANGLE_WEIGHT + ddiff
    return score, adiff, ddiff


def _score_arc(scan_feat, map_entry):
    """
    Compute match score for an ARC scan feature vs an ARC map entry.
    Returns (score, centre_dist, r_diff) or None if outside thresholds.

    scan_feat : dict with keys 'cx', 'cy', 'r'  (from fit_first_ctypes)
    map_entry : MapEntry (from map_manager, is_arc() == True)
    """
    centre_dist = math.hypot(
        scan_feat["cx"] - map_entry.mx,
        scan_feat["cy"] - map_entry.my
    )
    if centre_dist >= CENTRE_THRESH_M:
        return None

    rdiff = abs(scan_feat["r"] - map_entry.distance)  # distance field holds radius
    if rdiff >= R_THRESH_M:
        return None

    score = centre_dist + rdiff * R_WEIGHT
    return score, centre_dist, rdiff


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def match_features(scan_features, map_entries):
    """
    Match scan features against map entries.

    Parameters
    ----------
    scan_features : list of dicts
        Output of fit_first_ctypes.split_merge() — features list.
        Each dict has 'type' == 'line' or 'arc', plus type-specific fields.

    map_entries : list of MapEntry
        Current map from map_manager.  Only active entries are considered.
        map_entries[i].is_arc() distinguishes line vs arc entries.

    Returns
    -------
    MatchResult
        .matched        : list of (scan_idx, map_idx, score)
        .unmatched_scan : list of scan_idx
        .unmatched_map  : list of map_idx
    """
    # Collect active map indices split by type for fast filtering
    active_line_map_idx = [
        i for i, e in enumerate(map_entries)
        if e.active and not e.is_arc()
    ]
    active_arc_map_idx = [
        i for i, e in enumerate(map_entries)
        if e.active and e.is_arc()
    ]

    # best_for_scan[scan_idx] = (score, map_idx)
    best_for_scan = {}

    for si, sf in enumerate(scan_features):
        # Skip low-quality features — not worth matching
        if sf.get("quality", 100) < MIN_QUALITY:
            continue

        ftype = sf["type"]

        if ftype == "line":
            candidates = active_line_map_idx
            score_fn   = _score_line
        elif ftype == "arc":
            candidates = active_arc_map_idx
            score_fn   = _score_arc
        else:
            continue  # unknown type — skip

        best_score = None
        best_mi    = None

        for mi in candidates:
            result = score_fn(sf, map_entries[mi])
            if result is None:
                continue
            score = result[0]
            if best_score is None or score < best_score:
                best_score = score
                best_mi    = mi

        if best_mi is not None:
            # Keep only if this scan feature beats any previous claim on this
            # map entry — we resolve conflicts below
            existing = best_for_scan.get(si)
            if existing is None or best_score < existing[0]:
                best_for_scan[si] = (best_score, best_mi)

    # Resolve one-to-one constraint: if two scan features claim the same map
    # entry, keep only the lower-score one.
    # Build map_idx -> (score, scan_idx) tracking best claimant per map slot.
    best_for_map = {}   # map_idx -> (score, scan_idx)
    for si, (score, mi) in best_for_scan.items():
        existing = best_for_map.get(mi)
        if existing is None or score < existing[0]:
            best_for_map[mi] = (score, si)

    # Build final matched list from best_for_map (one-to-one guaranteed)
    matched = []
    matched_scan_idx = set()
    matched_map_idx  = set()

    for mi, (score, si) in best_for_map.items():
        matched.append((si, mi, score))
        matched_scan_idx.add(si)
        matched_map_idx.add(mi)

    # Sort by score ascending — pose_estimator will use the top matches first
    matched.sort(key=lambda t: t[2])

    # Unmatched scan features (skip quality-filtered ones — they're not "unmatched")
    unmatched_scan = [
        si for si, sf in enumerate(scan_features)
        if si not in matched_scan_idx
        and sf.get("quality", 100) >= MIN_QUALITY
        and sf["type"] in ("line", "arc")
    ]

    # Unmatched active map entries
    unmatched_map = [
        i for i in range(len(map_entries))
        if map_entries[i].active and i not in matched_map_idx
    ]

    return MatchResult(
        matched        = matched,
        unmatched_scan = unmatched_scan,
        unmatched_map  = unmatched_map,
    )


# ---------------------------------------------------------------------------
# Self-test — run directly: python3 line_matcher.py
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import sys

    # We need MapEntry for the test — inline a minimal stub so this file
    # can be tested without importing map_manager
    from collections import namedtuple as _nt

    _MapEntryStub = _nt("_MapEntryStub",
        ["angle", "distance", "length", "mx", "my",
         "theta_start", "observed", "last_seen", "status", "active"])

    class _ME(_MapEntryStub):
        def is_arc(self):
            return self.angle < -4.0   # ARC_ANGLE_SENTINEL == -10.0 (map_manager.py)

    print("line_matcher self-test")
    print("=" * 50)

    # ── Test 1: single line match ────────────────────────────────────────
    map_e = [
        _ME(angle=0.0, distance=1.0, length=1.0, mx=0.0, my=1.0, theta_start=0.0,
            observed=5, last_seen=0, status=1, active=True),
    ]
    scan_f = [
        {"type": "line", "angle": 0.01, "distance": 1.02,
         "length": 0.9, "quality": 100},
    ]
    r = match_features(scan_f, map_e)
    assert len(r.matched) == 1, f"T1 expected 1 match, got {len(r.matched)}"
    assert r.matched[0][0] == 0 and r.matched[0][1] == 0
    assert len(r.unmatched_scan) == 0
    assert len(r.unmatched_map)  == 0
    print(f"  T1 PASS  single line match  score={r.matched[0][2]:.4f}")

    # ── Test 2: angle too far apart — no match ───────────────────────────
    map_e2 = [
        _ME(angle=0.0, distance=1.0, length=1.0, mx=0.0, my=1.0, theta_start=0.0,
            observed=3, last_seen=0, status=1, active=True),
    ]
    scan_f2 = [
        {"type": "line", "angle": 0.5, "distance": 1.0,
         "length": 0.9, "quality": 100},
    ]
    r2 = match_features(scan_f2, map_e2)
    assert len(r2.matched) == 0, f"T2 expected no match, got {r2.matched}"
    assert len(r2.unmatched_scan) == 1
    assert len(r2.unmatched_map)  == 1
    print(f"  T2 PASS  angle mismatch → no match")

    # ── Test 3: distance too far apart — no match ────────────────────────
    map_e3 = [
        _ME(angle=0.0, distance=1.0, length=1.0, mx=0.0, my=1.0, theta_start=0.0,
            observed=3, last_seen=0, status=1, active=True),
    ]
    scan_f3 = [
        {"type": "line", "angle": 0.0, "distance": 2.5,
         "length": 0.9, "quality": 100},
    ]
    r3 = match_features(scan_f3, map_e3)
    assert len(r3.matched) == 0, f"T3 expected no match, got {r3.matched}"
    print(f"  T3 PASS  distance mismatch → no match")

    # ── Test 4: two scan lines, one map line — best wins ─────────────────
    map_e4 = [
        _ME(angle=0.0, distance=1.0, length=1.0, mx=0.0, my=1.0, theta_start=0.0,
            observed=5, last_seen=0, status=1, active=True),
    ]
    scan_f4 = [
        {"type": "line", "angle": 0.01,  "distance": 1.01, "length": 0.9, "quality": 100},
        {"type": "line", "angle": 0.10,  "distance": 1.10, "length": 0.9, "quality": 100},
    ]
    r4 = match_features(scan_f4, map_e4)
    assert len(r4.matched) == 1, f"T4 expected 1 match, got {len(r4.matched)}"
    # scan_idx 0 should win (closer in both angle and distance)
    assert r4.matched[0][0] == 0, f"T4 wrong scan_idx: {r4.matched[0][0]}"
    assert len(r4.unmatched_scan) == 1   # scan_idx 1 loses
    print(f"  T4 PASS  two-to-one conflict → best score wins  winner=scan[{r4.matched[0][0]}]")

    # ── Test 5: arc match ────────────────────────────────────────────────
    # ARC_ANGLE_SENTINEL = -10.0 flags an arc in MapEntry (map_manager.py)
    map_e5 = [
        _ME(angle=-10.0, distance=0.5, length=0.8, mx=0.1, my=0.2, theta_start=0.0,
            observed=4, last_seen=0, status=1, active=True),
    ]
    scan_f5 = [
        {"type": "arc", "cx": 0.12, "cy": 0.19, "r": 0.51,
         "length": 0.8, "quality": 100},
    ]
    r5 = match_features(scan_f5, map_e5)
    assert len(r5.matched) == 1, f"T5 expected 1 arc match, got {r5.matched}"
    print(f"  T5 PASS  arc match  score={r5.matched[0][2]:.4f}")

    # ── Test 6: arc vs line — type mismatch, no cross-type match ─────────
    map_e6 = [
        _ME(angle=0.0, distance=1.0, length=1.0, mx=0.0, my=1.0, theta_start=0.0,
            observed=3, last_seen=0, status=1, active=True),  # LINE entry
    ]
    scan_f6 = [
        {"type": "arc", "cx": 0.0, "cy": 1.0, "r": 0.5,
         "length": 0.8, "quality": 100},
    ]
    r6 = match_features(scan_f6, map_e6)
    assert len(r6.matched) == 0, f"T6 arc must not match line entry: {r6.matched}"
    print(f"  T6 PASS  arc vs line map entry → no cross-type match")

    # ── Test 7: quality filter ────────────────────────────────────────────
    map_e7 = [
        _ME(angle=0.0, distance=1.0, length=1.0, mx=0.0, my=1.0, theta_start=0.0,
            observed=5, last_seen=0, status=1, active=True),
    ]
    scan_f7 = [
        {"type": "line", "angle": 0.01, "distance": 1.01,
         "length": 0.9, "quality": 30},   # below MIN_QUALITY
    ]
    r7 = match_features(scan_f7, map_e7)
    assert len(r7.matched) == 0, f"T7 low-quality feature must be skipped"
    assert len(r7.unmatched_scan) == 0   # not in unmatched either — just ignored
    print(f"  T7 PASS  quality < {MIN_QUALITY} → feature skipped entirely")

    # ── Test 8: PI symmetry — angle=0 matches angle=PI ───────────────────
    map_e8 = [
        _ME(angle=0.0, distance=1.0, length=1.0, mx=0.0, my=1.0, theta_start=0.0,
            observed=5, last_seen=0, status=1, active=True),
    ]
    scan_f8 = [
        {"type": "line", "angle": math.pi - 0.05, "distance": 1.0,
         "length": 0.9, "quality": 100},
    ]
    r8 = match_features(scan_f8, map_e8)
    assert len(r8.matched) == 1, f"T8 PI symmetry failed: {r8.matched}"
    print(f"  T8 PASS  PI symmetry: angle≈PI matches angle=0  score={r8.matched[0][2]:.4f}")

    print()
    print("All tests passed.")
    sys.exit(0)