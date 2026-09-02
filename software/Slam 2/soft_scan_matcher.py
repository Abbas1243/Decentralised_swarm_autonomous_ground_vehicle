"""
soft_scan_matcher.py
=====================
Soft-correspondence Gauss-Newton pose refinement for the FINE refinement
loop, replacing line_matcher's hard 1:1 best-match-wins correspondence
inside that loop specifically.

WHY THIS EXISTS -- see slam_progress_update_pose_estimator.md root cause #2
-----------------------------------------------------------------------------
Compared against hector_slam's ScanMatcher (tu-darmstadt-ros-pkg/hector_slam,
hector_mapping/include/hector_slam_lib/matcher/ScanMatcher.h +
map/OccGridMapUtil.h): Hector NEVER makes a discrete "which map feature does
this point belong to" decision. OccGridMapUtil::interpMapValueWithDerivatives
bilinearly interpolates the occupancy grid to get a smooth value AND its
analytic gradient at any point; getCompleteHessianDerivs sums every point's
contribution directly into one Gauss-Newton system. There is no argmax, so
there is nothing to flip between iterations.

Your existing fine-refinement loop (slam.py's per-iteration call into
line_matcher.match_features -> build_line_matches_from_match_result ->
solve_pose_step) picks a single best-scoring HARD match per scan feature
each GN iteration. When the pose is uncertain mid-motion, that winner-take-
all pick can flip between two candidate walls from one iteration to the
next -- this is exactly the "correspondence flip-flop" failure mode logged
in slam_progress_update_pose_estimator.md (dx/dy oscillating between two
near-opposite values for 5+ sec during combined rotate+translate).

correlative_match.py already uses soft Gaussian weighting instead of hard
thresholds -- but only for the COARSE seed (brute-force grid search over
candidate poses). This module brings the same soft-weighting philosophy to
the FINE refinement step: instead of committing to one map line/arc per
scan feature, every nearby STATIC map line/arc contributes a continuously
-varying weighted term to the same Gauss-Newton normal equations
pose_estimator.py already builds -- just summed over ALL soft candidates
instead of the single hard-matched one. A wrong-wall candidate is never a
"winner" to begin with, so it can never flip; it just fades out (its
Gaussian weight drops) as the pose improves each iteration, which is the
same convergence behaviour Hector gets from its smooth interpolated cost
surface, translated into line/arc feature space instead of an occupancy
grid (keeps the RAM-budget reasoning in slam_build_prompt.md intact -- no
grid is allocated anywhere by this module).

THIS IS A DROP-IN REPLACEMENT FOR ONE STEP, NOT THE WHOLE LOOP
-------------------------------------------------------------------
solve_soft_pose_step() has the exact same contract as
pose_estimator.solve_pose_step(): given scan features already transformed
into the map frame by the current working pose (slam.py's
transform_features_to_map_frame), return ONE Gauss-Newton correction
(dx, dy, dtheta) for this iteration. slam.py's outer loop (coarse seed,
iteration count, convergence check, magnitude/residual plausibility gates,
trend/probation guards) is UNCHANGED -- only the correspondence+solve step
inside that loop changes. line_matcher.py, map_manager.py, and
correlative_match.py are untouched; this module is purely additive.

WEIGHTING
---------
Same Gaussian falloff shape and sigma values as correlative_match.py (kept
identical on purpose -- coarse and fine stages should agree on what
"close enough to plausibly correspond" means):
    LINE : exp(-(angle_diff/SIGMA_ANGLE_RAD)^2) * exp(-(dist_diff/SIGMA_DIST_M)^2)
    ARC  : exp(-(centre_dist/SIGMA_CENTRE_M)^2) * exp(-(r_diff/SIGMA_R_M)^2)
Pairs scoring below WEIGHT_FLOOR are dropped before accumulation -- this is
purely a performance cutoff (skip negligible terms), NOT a hard correspondence
decision; it never turns into a discrete winner and never causes flip-flop,
because a pair near the floor contributes almost nothing either way.

PORT PATH
---------
When porting to C (slam_core/soft_scan_matcher.c):
    solve_soft_pose_step() -> soft_scan_matcher_solve()
    Same weighted-accumulation pattern as pose_estimator.c's normal
    equations -- one extra nested loop (scan features x static map
    features) instead of relying on line_matcher's precomputed 1:1 pairs.
    O(n_scan * n_static_map), both bounded (<=50, <=500) -- worst case
    25,000 weight evaluations per GN iteration, each a handful of trig +
    2 exp() calls. Comfortably sub-millisecond on a 1GHz Cortex-A53; if
    profiling ever shows otherwise, a coarse angle-bucket prefilter (skip
    map lines whose angle differs by more than ~3*SIGMA_ANGLE_RAD before
    computing dist/exp at all) is the first optimization to reach for --
    not implemented here since it hasn't been needed at these sizes.
"""

import math

# ---------------------------------------------------------------------------
# Weighting constants -- kept identical to correlative_match.py's sigmas so
# the coarse and fine stages agree on correspondence plausibility.
# ---------------------------------------------------------------------------

SIGMA_ANGLE_RAD = 0.15
SIGMA_DIST_M    = 0.15
SIGMA_CENTRE_M  = 0.15
SIGMA_R_M       = 0.08

WEIGHT_FLOOR = 0.02   # drop negligible-weight pairs before accumulating --
                       # performance cutoff only, see module docstring.

MIN_TOTAL_WEIGHT = 0.3   # confidence gate -- mirrors
                          # correlative_match.MIN_SCORE_PER_FEATURE /
                          # pose_estimator.MIN_MATCHES in spirit: too little
                          # accumulated weight means "not enough information
                          # this iteration", caller should treat like an
                          # invalid/low-confidence step (e.g. break the GN
                          # loop, same as len(matches) < MIN_MATCHES today).

MIN_EIGENVALUE_THRESHOLD = 0.3   # conditioning gate for the 2x2 translation
                                  # system -- see solve_soft_pose_step's
                                  # returned min_eigenvalue. Starting value,
                                  # same spirit as pose_estimator's
                                  # MAX_ACCEPTABLE_RESIDUAL (deliberately
                                  # generous, tune tighter once real logged
                                  # values from a known-good run exist).


def _angle_diff_line(a, b):
    """Same PI-symmetric angular difference as line_matcher._angle_diff_line
    and correlative_match._angle_diff_line."""
    diff = abs(a - b) % math.pi
    if diff > math.pi / 2.0:
        diff = math.pi - diff
    return diff


def _normalize_pair(scan_angle, scan_distance, map_angle):
    """Identical to pose_estimator._normalize_pair -- half-plane
    normalisation exploiting line PI-symmetry, applied per-candidate since
    soft matching compares one scan line against MANY map lines, each
    potentially needing its own half-plane flip."""
    diff = scan_angle - map_angle
    if diff > math.pi / 2.0:
        return scan_angle - math.pi, -scan_distance
    elif diff < -math.pi / 2.0:
        return scan_angle + math.pi, -scan_distance
    return scan_angle, scan_distance


def _line_pair_weight(scan_angle, scan_distance, map_entry):
    """Gaussian soft-correspondence weight for one (scan line, map line)
    candidate pair. Returns (weight, norm_angle, norm_distance) -- the
    normalised scan values are returned too so the caller doesn't have to
    recompute the half-plane flip a second time."""
    norm_angle, norm_distance = _normalize_pair(scan_angle, scan_distance, map_entry.angle)
    adiff = _angle_diff_line(norm_angle, map_entry.angle)
    ddiff = abs(norm_distance - map_entry.distance)
    weight = (math.exp(-(adiff / SIGMA_ANGLE_RAD) ** 2)
              * math.exp(-(ddiff / SIGMA_DIST_M) ** 2))
    return weight, norm_angle, norm_distance


def _arc_pair_weight(scan_cx, scan_cy, scan_r, map_entry):
    """Gaussian soft-correspondence weight for one (scan arc, map arc)
    candidate pair. map_entry.distance holds the stored radius, same
    convention as everywhere else in this codebase."""
    centre_dist = math.hypot(scan_cx - map_entry.mx, scan_cy - map_entry.my)
    rdiff = abs(scan_r - map_entry.distance)
    weight = (math.exp(-(centre_dist / SIGMA_CENTRE_M) ** 2)
              * math.exp(-(rdiff / SIGMA_R_M) ** 2))
    return weight


def _min_eigenvalue_2x2(a11, a12, a22):
    """
    Smaller eigenvalue of the symmetric 2x2 matrix [[a11,a12],[a12,a22]] --
    i.e. the accumulated translation normal-equations matrix BEFORE it is
    solved. This directly measures how well-constrained translation is
    along its WORST direction: a small min eigenvalue means some direction
    (not necessarily x or y) is only weakly constrained by the currently
    weighted-in map lines -- the exact near-parallel-wall degeneracy
    described in pose_estimator.py's module docstring and
    slam_progress_update_pose_estimator.md's root cause #1.

    This replaces pose_estimator.check_angular_diversity's role for the
    soft-correspondence path: that function inferred degeneracy indirectly
    from the ANGULAR SPREAD of a discrete matched-pairs list, which is a
    proxy for the thing that actually matters (the conditioning of the
    system about to be solved). With soft correspondence there is no
    discrete matched-pairs list to measure spread over -- but the real
    system (a11, a12, a22) is already being built regardless, so reading
    its own eigenvalues is a strictly more direct measurement, not a
    workaround.

    A matched arc (which adds ARC_TRANSLATION_WEIGHT to BOTH a11 and a22
    isotropically, see solve_soft_pose_step) raises both diagonal entries
    together and therefore raises the min eigenvalue directly -- no special
    -case is needed for "an arc anchors both axes", it falls out of this
    same eigenvalue computation automatically.

    Closed-form for a symmetric 2x2: eigenvalues are
        trace/2 +/- sqrt((trace/2)^2 - det)
    """
    trace = a11 + a22
    det = a11 * a22 - a12 * a12
    disc = (trace / 2.0) ** 2 - det
    if disc < 0.0:
        disc = 0.0   # numerical guard only -- a real symmetric matrix's
                      # discriminant is never negative in exact arithmetic
    sqrt_disc = math.sqrt(disc)
    return trace / 2.0 - sqrt_disc


def _solve_2x2(a11, a12, a21, a22, b1, b2):
    """Identical to pose_estimator._solve_2x2 -- closed-form Cramer's rule,
    (0,0) on a near-singular system (e.g. every weighted-in line parallel
    and no arc contribution)."""
    det = a11 * a22 - a12 * a21
    if abs(det) < 1e-9:
        return 0.0, 0.0
    dx = (b1 * a22 - b2 * a12) / det
    dy = (a11 * b2 - a21 * b1) / det
    return dx, dy


def solve_soft_pose_step(scan_features, static_lines, static_arcs):
    """
    ONE Gauss-Newton step using SOFT correspondence -- drop-in alternative
    to pose_estimator.solve_pose_step() for use inside slam.py's per-
    iteration refinement loop. Same contract: scan_features must already be
    transformed into the map frame using the current working pose (see
    slam.py's transform_features_to_map_frame), matching how
    solve_pose_step's callers already prepare their inputs.

    Unlike solve_pose_step, this function does NOT take pre-matched
    LineMatch/ArcMatch lists -- it takes the raw scan features and the raw
    STATIC map entries directly, and considers every (scan, map) candidate
    pair itself (weighted, not hard-matched). This is what removes the
    correspondence flip-flop: there is no discrete pairing decision for the
    pose to destabilise between iterations.

    MEMORY: single pass, running scalar accumulators only (a11/a12/a22/
    b1/b2/sin_sum/cos_sum/total_weight) -- no list of surviving pairs is
    ever stored. An earlier draft of this function collected passing pairs
    into a Python list to reuse between the rotation and translation steps;
    that list is UNBOUNDED in the worst case (every scan feature paired
    with every static map entry, up to MAX_FEATURES x MAX_MAP_ENTRIES =
    50 x 500 = 25,000 entries) and would violate slam_build_prompt.md's "no
    dynamic memory allocation" constraint once ported to C. Fixed here by
    computing the rotation contribution and the translation contribution
    for each pair in the SAME loop iteration -- both only need each pair's
    weight/norm_angle/norm_distance transiently, never a stored collection
    of them. Direct 1:1 C port: fixed local float accumulators, two nested
    fixed-bound for-loops, no malloc.

    Parameters
    ----------
    scan_features : list of dicts (type='line'/'arc'), already in the map
        frame for the current working pose guess.
    static_lines : list of MapEntry, STATIC status, not is_arc()
    static_arcs  : list of MapEntry, STATIC status, is_arc()

    Returns
    -------
    (dx, dy, dtheta, total_weight, min_eigenvalue)
        dx, dy, dtheta : the GN correction for this iteration, to be
            applied to working_pose exactly like solve_pose_step's output.
        total_weight : sum of all weights that contributed (lines + arcs) --
            diagnostic / confidence signal. Caller should treat
            total_weight < MIN_TOTAL_WEIGHT the same way an invalid/too-few-
            matches result is treated today (break the refinement loop,
            don't apply a near-zero-evidence correction).
        min_eigenvalue : smaller eigenvalue of the 2x2 translation system
            actually solved -- see _min_eigenvalue_2x2's docstring. Caller
            should treat min_eigenvalue < MIN_EIGENVALUE_THRESHOLD as
            "translation poorly constrained along some direction", same
            role pose_estimator.check_angular_diversity played for the
            hard-match path.
    """
    # ---- Pass 1: rotation only needs the weighted circular mean, which
    #      does not depend on dtheta itself -- compute it first in its own
    #      cheap pass (line pairs only; arcs never contribute to rotation).
    sin_sum = 0.0
    cos_sum = 0.0

    for feat in scan_features:
        if feat.get("type") != "line":
            continue
        scan_angle = feat["angle"]
        scan_distance = feat["distance"]
        for entry in static_lines:
            weight, norm_angle, _ = _line_pair_weight(scan_angle, scan_distance, entry)
            if weight < WEIGHT_FLOOR:
                continue
            d = entry.angle - norm_angle
            sin_sum += weight * math.sin(d)
            cos_sum += weight * math.cos(d)

    dtheta = 0.0 if (sin_sum == 0.0 and cos_sum == 0.0) else math.atan2(sin_sum, cos_sum)

    # ---- Pass 2: translation normal equations + total_weight, lines
    #      (post-rotation residual) + arcs (direct isotropic centre
    #      constraint). Re-evaluates each line pair's weight rather than
    #      reusing pass 1's -- deliberate: this keeps memory at fixed
    #      scalars only (see docstring) at the cost of recomputing ~5
    #      scalar ops + 2 exp() per pair a second time, which is exactly
    #      the RAM-vs-CPU trade this whole codebase already makes
    #      everywhere else (e.g. map_manager re-deriving values instead of
    #      caching them across calls).
    a11 = a12 = a22 = 0.0
    b1 = b2 = 0.0
    total_weight = 0.0

    for feat in scan_features:
        if feat.get("type") != "line":
            continue
        scan_angle = feat["angle"]
        scan_distance = feat["distance"]
        for entry in static_lines:
            weight, norm_angle, norm_distance = _line_pair_weight(scan_angle, scan_distance, entry)
            if weight < WEIGHT_FLOOR:
                continue
            residual = entry.distance - norm_distance
            nx, ny = -math.sin(entry.angle), math.cos(entry.angle)
            a11 += weight * nx * nx
            a12 += weight * nx * ny
            a22 += weight * ny * ny
            b1  += weight * nx * residual
            b2  += weight * ny * residual
            total_weight += weight

    for feat in scan_features:
        if feat.get("type") != "arc":
            continue
        for entry in static_arcs:
            weight = _arc_pair_weight(feat["cx"], feat["cy"], feat["r"], entry)
            if weight < WEIGHT_FLOOR:
                continue
            a11 += weight
            a22 += weight
            b1  += weight * (entry.mx - feat["cx"])
            b2  += weight * (entry.my - feat["cy"])
            total_weight += weight

    dx, dy = _solve_2x2(a11, a12, a12, a22, b1, b2)
    min_eigenvalue = _min_eigenvalue_2x2(a11, a12, a22)

    return dx, dy, dtheta, total_weight, min_eigenvalue


# ---------------------------------------------------------------------------
# Self-test -- run directly: python3 soft_scan_matcher.py
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import sys
    from collections import namedtuple as _nt

    _MapEntryStub = _nt("_MapEntryStub", ["angle", "distance", "mx", "my"])

    class _ME(_MapEntryStub):
        def is_arc(self):
            return self.angle < -4.0

    def _line_feat(angle, distance):
        return {"type": "line", "angle": angle, "distance": distance}

    def _arc_feat(cx, cy, r):
        return {"type": "arc", "cx": cx, "cy": cy, "r": r}

    print("soft_scan_matcher self-test")
    print("=" * 50)

    # -- T1: single true wall, no confuser -> recovers a known small offset -
    true_dtheta = 0.03
    map_e = [_ME(angle=0.0, distance=1.0, mx=0.0, my=1.0)]
    scan_f = [_line_feat(-true_dtheta, 1.0)]   # scan sees it rotated by -dtheta
    dx, dy, dtheta, w, eig = solve_soft_pose_step(scan_f, map_e, [])
    assert w > MIN_TOTAL_WEIGHT, f"T1 expected confident weight, got {w}"
    assert abs(dtheta - true_dtheta) < 1e-3, f"T1 dtheta off: {dtheta} vs {true_dtheta}"
    print(f"  T1 PASS  single clean pair: dtheta={dtheta:.4f} (true={true_dtheta:.4f}) weight={w:.3f}")

    # -- T2: THE FLIP-FLOP CASE -- a real wall plus a nearby "trap" wall that
    #        would win a HARD best-score match at a slightly different pose,
    #        causing line_matcher to flip its pick between iterations. The
    #        soft matcher must not oscillate: check the correction direction
    #        stays consistent (same sign) across two slightly different
    #        working-pose linearisations, instead of flipping sign the way a
    #        discrete argmax-based match could.
    true_wall = _ME(angle=0.0, distance=1.00, mx=0.0, my=1.0)
    trap_wall = _ME(angle=0.05, distance=1.05, mx=0.0, my=1.05)   # close enough to
                                                                    # confuse a hard
                                                                    # 1:1 matcher
    map_e2 = [true_wall, trap_wall]

    # Two slightly different scan observations (as if two consecutive GN
    # iterations re-transformed the scan by a slightly different working
    # pose) -- both should pull dtheta toward the SAME sign, not flip.
    scan_a = [_line_feat(0.02, 1.02)]
    scan_b = [_line_feat(0.01, 1.01)]
    dxA, dyA, dthetaA, wA, eigA = solve_soft_pose_step(scan_a, map_e2, [])
    dxB, dyB, dthetaB, wB, eigB = solve_soft_pose_step(scan_b, map_e2, [])
    assert (dthetaA >= 0) == (dthetaB >= 0), \
        f"T2 FLIP-FLOP: dthetaA={dthetaA:.4f} dthetaB={dthetaB:.4f} changed sign"
    assert (dyA >= 0) == (dyB >= 0), \
        f"T2 FLIP-FLOP: dyA={dyA:.4f} dyB={dyB:.4f} changed sign"
    print(f"  T2 PASS  no sign flip across two nearby linearisations "
          f"(dthetaA={dthetaA:.4f} dthetaB={dthetaB:.4f})")

    # -- T3: parallel-wall translation ambiguity still fixed by one arc,
    #        same guarantee pose_estimator.py's T7 already provides -------
    map_e3 = [
        _ME(angle=0.0, distance=1.0, mx=0.0, my=1.0),
        _ME(angle=0.0, distance=-1.0, mx=0.0, my=-1.0),
    ]
    arc_e3 = [_ME(angle=-10.0, distance=0.5, mx=0.5, my=0.5)]
    scan_f3 = [
        _line_feat(0.0, 1.0 - 0.20),    # both walls shifted by true dx=0.20
        _line_feat(0.0, -1.0 - 0.20),
    ]
    scan_arc3 = [_arc_feat(0.5 - 0.20, 0.5, 0.5)]
    dx3, dy3, dtheta3, w3, eig3 = solve_soft_pose_step(scan_f3 + scan_arc3, map_e3, arc_e3)
    assert abs(dx3 - 0.20) < 1e-2, f"T3 dx should be recovered via arc constraint: {dx3}"
    print(f"  T3 PASS  parallel-wall ambiguity resolved by soft arc constraint: dx={dx3:.4f}")

    # -- T4: no nearby static features -> low confidence, caller should skip
    dx4, dy4, dtheta4, w4, eig4 = solve_soft_pose_step(
        [_line_feat(0.0, 1.0)], [_ME(angle=1.4, distance=5.0, mx=5.0, my=0.0)], []
    )
    assert w4 < MIN_TOTAL_WEIGHT, f"T4 expected low confidence weight, got {w4}"
    print(f"  T4 PASS  no plausible correspondence -> low weight={w4:.4f}, caller should skip")

    # -- T5: eigenvalue conditioning check -- two parallel walls alone must
    #        report a low min_eigenvalue (translation along their shared
    #        direction is unconstrained); adding one arc must raise it,
    #        mirroring pose_estimator's check_angular_diversity guarantee
    #        but read directly off the solved system instead of a discrete-
    #        match angular-spread proxy. ---------------------------------
    map_e5 = [
        _ME(angle=0.0, distance=1.0, mx=0.0, my=1.0),
        _ME(angle=0.02, distance=1.5, mx=0.0, my=1.5),   # nearly parallel to the first
    ]
    scan_f5 = [_line_feat(0.0, 1.0), _line_feat(0.02, 1.5)]
    _, _, _, _, eig5_no_arc = solve_soft_pose_step(scan_f5, map_e5, [])
    assert eig5_no_arc < MIN_EIGENVALUE_THRESHOLD, \
        f"T5 expected near-parallel walls alone to be poorly conditioned, got {eig5_no_arc}"

    arc_e5 = [_ME(angle=-10.0, distance=0.4, mx=0.3, my=0.3)]
    scan_arc5 = [_arc_feat(0.3, 0.3, 0.4)]
    _, _, _, _, eig5_with_arc = solve_soft_pose_step(scan_f5 + scan_arc5, map_e5, arc_e5)
    assert eig5_with_arc > eig5_no_arc, \
        f"T5 expected the arc to raise the min eigenvalue: {eig5_with_arc} vs {eig5_no_arc}"
    print(f"  T5 PASS  conditioning check: near-parallel walls alone eig={eig5_no_arc:.4f} "
          f"(< threshold {MIN_EIGENVALUE_THRESHOLD}), with arc eig={eig5_with_arc:.4f}")

    print()
    print("All tests passed.")
    sys.exit(0)