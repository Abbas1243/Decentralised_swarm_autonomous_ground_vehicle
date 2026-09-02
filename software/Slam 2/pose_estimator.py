"""
pose_estimator.py
==================
Pose correction from matched LINE features (slam_build_prompt.md Algorithm 3),
extended with ARC centre constraints to fix the parallel-wall translation
ambiguity (see module notes below on ArcMatch).

Given a set of matched (scan_line, map_line) pairs in Hough form
(angle, distance), estimate the rigid transform (dx, dy, dtheta) that best
aligns the scan onto the map.  This is "scan-to-map" odometry — there are
no wheel encoders in this pipeline; the correction comes entirely from line
(and now arc) geometry.

This module is a pure function over plain numbers.  It does NOT know about
MapEntry, scan-feature dicts, FeaturePacket, or any other module's types.
The caller (eventually slam.c) is responsible for extracting
(scan_angle, scan_distance, map_angle, map_distance) for each matched line
pair, and (scan_cx, scan_cy, map_cx, map_cy) for each matched arc pair,
before calling estimate_pose_delta() / solve_pose_step().  This mirrors the
boundary already used by line_matcher.py (pure function, no global state)
and keeps the C port a direct translation.

ALGORITHM (slam_build_prompt.md Algorithm 3, + arc extension)
----------------------------------------------------------------
Input  : matched line pairs, each as (scan_angle, scan_distance,
                                       map_angle,  map_distance)
         matched arc pairs,  each as (scan_cx, scan_cy, map_cx, map_cy)
Output : pose correction (dx, dy, dtheta)

Step 1 — rotation
    dtheta = circular mean of (map_angle_i - scan_angle_i) over all line
    pairs. Lines have PI symmetry (angle, dist) == (angle+PI, -dist), so
    each pair is first normalised into the same half-plane before
    differencing — identical fix to the one already applied in
    map_manager._update_line_entry. Arcs have no orientation and do not
    contribute to rotation.

Step 2 — translation given dtheta
    LINE contribution: rotate each scan line's angle by dtheta, then build
    the overdetermined linear system A * [dx, dy]^T = b from:
        dx*cos(map_angle) + dy*sin(map_angle) = scan_distance_rotated - map_distance
    A line's Hough distance only constrains translation PERPENDICULAR to
    that line — translation parallel to the line is unobservable from it.
    If every matched line happens to be parallel (or close to it — the
    common case in a rectangular room with two long facing walls), the
    normal-equations system is near-singular along that shared direction:
    _solve_2x2's |det|<1e-9 guard only catches EXACT parallelism, so a
    near-parallel set still "solves" while amplifying noise into a large,
    wrong dx/dy along the unconstrained axis. This is what produces a
    magnitude- and residual-passing-looking delta that is actually just
    noise riding an unconstrained direction — observed directly as
    metre-scale jumps during a sideways slide along a room's dominant wall
    axis, while pure rotation (always observable from any line angles)
    tracked perfectly the whole time.

    ARC contribution: a matched arc's CENTRE gives a direct, isotropic
    (dx, dy) constraint —
        map_cx = scan_cx + dx
        map_cy = scan_cy + dy
    — that is NOT direction-degenerate the way a line's perpendicular
    distance is. Even a single matched arc anchors both translation axes
    at once, which is exactly what breaks the parallel-wall ambiguity
    above. Arcs are folded into the same A^T A / A^T b normal-equations
    system as independent unit-weight rows (see ARC_TRANSLATION_WEIGHT).

Step 3 — solve
    Normal equations: (A^T A) [dx,dy]^T = A^T b
    This is a plain 2x2 system — solved with closed-form Cramer's rule.
    No Eigen, no numpy linear algebra. Direct 1:1 port to C.

Step 4 — iterate
    Repeat steps 1-3 up to MAX_ITERATIONS times, re-deriving dtheta and
    [dx,dy] against the *updated* pose guess each time. Stop early once the
    combined correction magnitude drops below CONVERGENCE_THRESHOLD.

Step 5 — apply (caller's responsibility)
    current_pose.x     += dx
    current_pose.y     += dy
    current_pose.theta += dtheta
    Then transform next scan's features into the map frame before matching.

CONSTRAINTS
-----------
- All math in plain Python floats standing in for C `float` (f-suffix on
  the C side). No numpy/Eigen-style linear algebra — only scalar ops and
  one closed-form 2x2 solve.
- No dynamic allocation equivalent — fixed-size iteration, no growing lists
  inside the hot loop (matches list is built once by the caller).
- Minimum 2 matched pairs (lines + arcs combined) required. Fewer than 2 ->
  caller must skip the update (this module returns a zero-delta PoseDelta
  and a `valid` flag instead of raising, since "not enough information this
  scan" is a normal runtime condition on embedded hardware, not an error).
- Arcs contribute to translation only, never to rotation (no orientation).

PORT PATH
---------
When porting to C (slam_core/pose_estimator.c / pose_estimator.h):
    LineMatch struct        -> pose_estimator.h
        float scan_angle, scan_distance, map_angle, map_distance;
    ArcMatch struct          -> pose_estimator.h
        float scan_cx, scan_cy, map_cx, map_cy;
    PoseDelta struct         -> pose_estimator.h
        float dx, dy, dtheta; uint8_t valid;
    estimate_pose_delta()    -> pose_estimator_estimate()
    Constants below          -> #defines in pose_estimator.h
"""

import math
from collections import namedtuple

# ---------------------------------------------------------------------------
# Tuning constants — mirror slam.conf values
# ---------------------------------------------------------------------------

MAX_ITERATIONS         = 8       # slam.conf: max_iterations
CONVERGENCE_THRESHOLD  = 0.001   # slam.conf: convergence_threshold
MIN_MATCHES            = 2       # Algorithm 3: minimum 2 matched pairs required
                                  # (lines + arcs combined)

ARC_TRANSLATION_WEIGHT = 1.0     # weight of one arc-centre constraint row in
                                  # the A^T A / A^T b normal-equations system,
                                  # relative to a line's unit-normal row.
                                  # Kept at 1.0 (same order of magnitude as a
                                  # single line constraint) — tune only if
                                  # arcs prove noisier/cleaner than lines in
                                  # practice.

# ---------------------------------------------------------------------------
# Input / output types
# ---------------------------------------------------------------------------

LineMatch = namedtuple(
    "LineMatch",
    ["scan_angle", "scan_distance", "map_angle", "map_distance"]
)
# All angles in radians (Hough angle, [-pi/2, pi/2] convention).
# All distances in metres (signed Hough perpendicular distance).

ArcMatch = namedtuple(
    "ArcMatch",
    ["scan_cx", "scan_cy", "map_cx", "map_cy"]
)
# Arc centre in the same frame convention as LineMatch's distances — scan_cx/
# scan_cy are the arc centre as seen in this scan (already rotated by any
# working pose guess upstream, same as scan_angle/scan_distance for lines),
# map_cx/map_cy are the arc centre as stored in the map. Radius is NOT
# included here — Algorithm 3 uses centre position only for translation;
# radius agreement is already enforced upstream by line_matcher's arc
# matching thresholds before a pair ever reaches this module.

PoseDelta = namedtuple(
    "PoseDelta",
    ["dx", "dy", "dtheta", "valid", "iterations"]
)
# dx, dy       : metres   — translation correction
# dtheta       : radians  — rotation correction
# valid        : bool     — False if fewer than MIN_MATCHES pairs were given;
#                           dx/dy/dtheta are 0.0 in that case, caller must
#                           skip applying this delta to the pose.
# iterations   : int      — how many Gauss-Newton iterations actually ran
#                           (for diagnostics / tuning, not required by callers)


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _normalize_pair(scan_angle, scan_distance, map_angle):
    """
    Normalise a scan line's (angle, distance) into the same half-plane as
    the map line it matched against, exploiting line PI-symmetry:
        (angle, distance) == (angle + PI, -distance)

    Without this, a scan line whose fitted eigenvector points the opposite
    way from the map line's stored eigenvector produces a ~180 degree swing
    in raw angle difference, corrupting the circular mean in Step 1 exactly
    the way it would have corrupted map_manager's running average before
    that module's own half-plane fix.

    Returns (angle, distance) — possibly with angle shifted by +-PI and
    distance negated to match.
    """
    diff = scan_angle - map_angle
    if diff > math.pi / 2.0:
        return scan_angle - math.pi, -scan_distance
    elif diff < -math.pi / 2.0:
        return scan_angle + math.pi, -scan_distance
    return scan_angle, scan_distance


def _estimate_rotation(matches, theta_guess):
    """
    Step 1 — rotation.

    dtheta = circular mean of (map_angle_i - scan_angle_i), each pair first
    normalised into the same half-plane as its map partner. Arcs have no
    orientation and are not passed to this function.

    theta_guess is the rotation already applied by previous iterations —
    passed through so callers/tests can inspect convergence, but the
    circular mean itself is computed fresh from the raw matches each call
    (Gauss-Newton re-linearises from the original data every iteration,
    it does not compound deltas onto deltas).

    Returns dtheta in radians.
    """
    sin_sum = 0.0
    cos_sum = 0.0
    for m in matches:
        _, _ = m.scan_angle, m.scan_distance  # explicit: distance unused here
        norm_angle, _ = _normalize_pair(m.scan_angle, m.scan_distance, m.map_angle)
        d = m.map_angle - norm_angle
        sin_sum += math.sin(d)
        cos_sum += math.cos(d)

    if sin_sum == 0.0 and cos_sum == 0.0:
        return 0.0
    return math.atan2(sin_sum, cos_sum)


def _solve_2x2(a11, a12, a21, a22, b1, b2):
    """
    Closed-form solve of:
        [a11 a12] [dx]   [b1]
        [a21 a22] [dy] = [b2]
    using Cramer's rule. Returns (dx, dy), or (0.0, 0.0) if the system is
    singular (determinant ~ 0 — e.g. all matched lines parallel AND no arc
    matches present, which leaves the translation along their shared
    direction unobservable).

    Plain scalar arithmetic only — direct 1:1 port to C, no library calls.
    """
    det = a11 * a22 - a12 * a21
    if abs(det) < 1e-9:
        return 0.0, 0.0
    dx = (b1 * a22 - b2 * a12) / det
    dy = (a11 * b2 - a21 * b1) / det
    return dx, dy


def _estimate_translation(matches, arc_matches, dtheta):
    """
    Step 2 + 3 — translation given dtheta, solved via normal equations.

    LINE rows: for each matched line pair, after rotating the scan line's
    angle by dtheta (folded in via _normalize_pair already applied
    upstream), the translation we are solving for satisfies:

        nx*dx + ny*dy = map_distance - scan_distance_rot

    where (nx, ny) = (-sin(map_angle), cos(map_angle)) is the line's unit
    NORMAL in Hough convention (perpendicular to the line, pointing from
    the origin toward the line) — NOT (cos, sin), which is the line's
    *direction* vector and moving along it does not change perpendicular
    distance at all.

    ARC rows: for each matched arc pair, the centre gives a direct,
    isotropic constraint:

        dx = map_cx - scan_cx
        dy = map_cy - scan_cy

    contributed as two independent unit-weight rows ([1,0]->b=map_cx-scan_cx
    and [0,1]->b=map_cy-scan_cy) in the SAME normal-equations system as the
    line rows — this is what makes a single matched arc sufficient to
    anchor both translation axes even when every line in the match set is
    parallel.

    Builds A^T A and A^T b directly (2x2 + 2x1) rather than forming A
    explicitly — avoids any matrix library, mirrors straight-line C code.
    """
    a11 = a12 = a22 = 0.0   # A^T A entries (a21 == a12, line is symmetric)
    b1 = b2 = 0.0           # A^T b entries

    for m in matches:
        norm_angle, norm_distance = _normalize_pair(
            m.scan_angle, m.scan_distance, m.map_angle
        )
        residual = m.map_distance - norm_distance

        # Hough NORMAL vector, not direction vector — see docstring above.
        ca = -math.sin(m.map_angle)
        sa =  math.cos(m.map_angle)

        a11 += ca * ca
        a12 += ca * sa
        a22 += sa * sa
        b1  += ca * residual
        b2  += sa * residual

    for am in arc_matches:
        # Two unit-weight rows: [1,0] and [0,1] — an arc centre constrains
        # dx and dy directly and independently of any line direction.
        a11 += ARC_TRANSLATION_WEIGHT
        a22 += ARC_TRANSLATION_WEIGHT
        b1  += ARC_TRANSLATION_WEIGHT * (am.map_cx - am.scan_cx)
        b2  += ARC_TRANSLATION_WEIGHT * (am.map_cy - am.scan_cy)

    dx, dy = _solve_2x2(a11, a12, a12, a22, b1, b2)
    return dx, dy


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def solve_pose_step(matches, arc_matches=()):
    """
    ONE Gauss-Newton linearisation step: rotation then translation, no
    internal iteration, no outlier rejection, no convergence check.

    slam.py's process_scan owns the OUTER loop: transform scan -> map frame
    using the current working pose, re-run line_matcher.match_features
    against that improved pose, rebuild the LineMatch/ArcMatch lists, THEN
    call this function again. Re-matching every iteration is what makes
    this real ICP instead of "refine the same wrong correspondence more
    precisely".

    pose_estimator.py still does not know about the map or about matching
    -- this function takes the same flat LineMatch/ArcMatch lists every
    other function in this module takes, and nothing else.

    Returns
    -------
    (dx, dy, dtheta) -- the correction for THIS step only. Caller is
    responsible for applying it to a working pose and re-matching before
    calling this again.
    """
    dtheta = _estimate_rotation(matches, 0.0)
    dx, dy = _estimate_translation(matches, arc_matches, dtheta)
    return dx, dy, dtheta


RESIDUAL_ANGLE_WEIGHT = 1.0   # weights the rotation term (radians) against
                               # the translation term (metres) in
                               # compute_residual's combined score -- purely
                               # a tuning knob, mirrors line_matcher's own
                               # ANGLE_WEIGHT pattern for the same reason
                               # (angle and distance are different units,
                               # this is the exchange rate between them)


def compute_residual(matches, dx, dy, dtheta, arc_matches=()):
    """
    POST-SOLVE plausibility check -- how well does (dx, dy, dtheta) actually
    explain the matched pairs (lines AND arcs) it was solved from?

    WHY THIS EXISTS: slam.py's magnitude-only guard (MAX_DELTA_TRANSLATION_M
    / MAX_DELTA_ROTATION_RAD) only asks "is this delta small enough to be
    physically plausible for one scan". It never asks "does this delta
    actually align the pairs it claims to have solved". A wrong-wall
    correspondence (or an unconstrained-axis solve — see module docstring
    on parallel walls) can produce a delta that is magnitude-plausible but
    still doesn't fit the geometry well -- residual is what catches that,
    and magnitude checks structurally cannot.

    Returns the MEAN per-pair residual (rotation term in radians scaled by
    RESIDUAL_ANGLE_WEIGHT plus translation term in metres for lines; plain
    Euclidean centre error in metres for arcs) -- NOT normalised to [0,1].
    Caller (slam.py) picks an absolute threshold (tune against real logged
    residuals from a known-good run).
    """
    if not matches and not arc_matches:
        return 0.0

    total = 0.0
    n = 0

    for m in matches:
        norm_angle, norm_distance = _normalize_pair(
            m.scan_angle, m.scan_distance, m.map_angle
        )

        angle_after = norm_angle + dtheta
        d_ang = m.map_angle - angle_after
        while d_ang > math.pi / 2.0:
            d_ang -= math.pi
        while d_ang <= -math.pi / 2.0:
            d_ang += math.pi

        nx, ny = -math.sin(m.map_angle), math.cos(m.map_angle)
        predicted_distance = norm_distance + nx * dx + ny * dy
        d_dist = m.map_distance - predicted_distance

        total += abs(d_ang) * RESIDUAL_ANGLE_WEIGHT + abs(d_dist)
        n += 1

    for am in arc_matches:
        rx = am.map_cx - (am.scan_cx + dx)
        ry = am.map_cy - (am.scan_cy + dy)
        total += math.hypot(rx, ry)
        n += 1

    return total / n


def check_angular_diversity(matches, arc_matches=(), min_spread_rad=math.radians(15.0)):
    """
    True if the matched set is safe to trust for translation -- either
    because at least one ARC match is present (an arc centre constrains
    both translation axes directly and is never direction-degenerate, so
    it unconditionally clears this check regardless of the line set), or
    because the matched LINE set spans at least min_spread_rad of distinct
    wall orientations (i.e. is NOT a near-parallel / near-singular
    configuration).

    WHY THIS EXISTS: two or three matched lines that are all nearly
    parallel (e.g. two segments of the same long wall, or two facing walls
    of a corridor) make the translation solve's A^T A matrix near-singular
    ALONG the wall-parallel axis -- translation perpendicular to those
    walls is unconstrained. _solve_2x2's |det|<1e-9 guard only catches
    EXACT parallelism; a near-parallel set still passes that guard while
    amplifying small residuals into large, wrong dx/dy. This is the root
    cause of large spurious translation jumps during a sideways slide
    along a room's dominant wall direction, while rotation (always
    observable from any line angles) tracks correctly throughout.

    Angles are compared using the same PI-symmetric wrap line_matcher and
    map_manager already use elsewhere (a wall and its PI-rotated twin are
    the same orientation, not diverse).

    Returns False (not diverse enough to trust) for fewer than 2 matches
    when no arc match is present.
    """
    if arc_matches:
        return True

    if len(matches) < 2:
        return False

    max_spread = 0.0
    for i in range(len(matches)):
        for j in range(i + 1, len(matches)):
            d = abs(matches[i].map_angle - matches[j].map_angle) % math.pi
            if d > math.pi / 2.0:
                d = math.pi - d
            if d > max_spread:
                max_spread = d

    return max_spread >= min_spread_rad


def _reject_outlier_matches(matches):
    """
    Filter out matched LINE pairs whose individual rotation residual is
    wildly inconsistent with the majority, before they ever reach the
    least-squares solve. Arc matches are not passed through this filter —
    they have no rotation residual to compare and are geometrically
    independent of the line correspondence problem this filter targets.

    WHY THIS EXISTS: line_matcher.match_features() does best-score 1:1
    matching with no cross-pair consistency check. A scan line can be paired
    with the WRONG map line (e.g. an adjacent parallel wall) whenever the
    viewpoint shifts enough that two different walls both fall inside
    line_matcher's angle/distance thresholds. That single bad pair then
    corrupts the entire least-squares solve in _estimate_rotation /
    _estimate_translation, because those functions have no robustness —
    every pair contributes equally to the circular mean and normal
    equations. One wrong correspondence can swing dx/dy/dtheta by several
    multiples of the true (small) per-scan motion.

    METHOD: use each pair's *rotation* residual (map_angle - scan_angle,
    half-plane normalised) as the outlier signal. Real per-scan rotation is
    small and consistent across all genuinely-matched pairs (they all see
    the same rigid rotation). A mismatched pair's residual is essentially
    unrelated to the true rotation and will usually sit far from the
    consensus. Rotation residual is used (rather than translation residual)
    because it does not depend on an already-estimated dtheta — it can be
    computed directly from the raw matches in one pass, making this filter
    independent of solver state and trivial to port to C as a pre-pass.

    With fewer than 4 matches there isn't enough redundancy to safely
    identify outliers (could reject a legitimate minority by chance), so
    the filter is a no-op in that regime — the existing MIN_MATCHES /
    plausibility-guard machinery downstream is the only protection then,
    same as before this function existed.

    Returns a possibly-shorter list of matches. Never returns fewer than
    MIN_MATCHES pairs if at least MIN_MATCHES were inliers; if rejection
    would drop below MIN_MATCHES, returns the original list unfiltered
    (better to attempt a possibly-noisy solve than to manufacture a
    too-few-matches case that silently skips correction entirely).
    """
    if len(matches) < 4:
        return matches

    # Per-pair rotation residual: map_angle - scan_angle, half-plane
    # normalised exactly as _estimate_rotation does internally.
    residuals = []
    for m in matches:
        norm_angle, _ = _normalize_pair(m.scan_angle, m.scan_distance, m.map_angle)
        d = m.map_angle - norm_angle
        # Wrap into (-pi/2, pi/2] — residuals are small angle differences,
        # not full angles, so this keeps the comparison metric sane even
        # near the +-pi/2 wrap boundary.
        while d > math.pi / 2.0:
            d -= math.pi
        while d <= -math.pi / 2.0:
            d += math.pi
        residuals.append(d)

    # Consensus = median residual (robust to a minority of outliers,
    # unlike the mean which an outlier can drag arbitrarily far).
    sorted_res = sorted(residuals)
    n = len(sorted_res)
    if n % 2 == 1:
        median = sorted_res[n // 2]
    else:
        median = (sorted_res[n // 2 - 1] + sorted_res[n // 2]) / 2.0

    # Reject any pair whose residual is far from the consensus. Threshold
    # is generous on purpose (this is a coarse pre-filter, not the final
    # word) — real per-scan rotation noise is a few degrees at most; a
    # mismatched wall typically produces a residual several times that.
    OUTLIER_THRESH_RAD = 0.10   # ~5.7 degrees from the median residual

    inliers = [
        m for m, r in zip(matches, residuals)
        if abs(r - median) < OUTLIER_THRESH_RAD
    ]

    if len(inliers) < MIN_MATCHES:
        # Filtering would leave too little to work with — fall back to the
        # unfiltered set rather than manufacturing an invalid result.
        return matches

    return inliers


def estimate_pose_delta(matches, arc_matches=()):
    """
    Estimate the rigid-body pose correction (dx, dy, dtheta) that best
    aligns a set of matched scan/map LINE pairs, plus any matched ARC
    centre pairs.

    Parameters
    ----------
    matches : list of LineMatch
        Each entry is (scan_angle, scan_distance, map_angle, map_distance)
        for one matched line pair, in radians / metres. Build these from
        line_matcher.match_features() output — see
        build_line_matches_from_match_result() below for the glue.
    arc_matches : list of ArcMatch
        Each entry is (scan_cx, scan_cy, map_cx, map_cy) for one matched
        arc pair, in metres. Build these from
        build_arc_matches_from_match_result() below. Optional — omit or
        pass an empty sequence for line-only behaviour (unchanged from
        before arc support was added).

    Returns
    -------
    PoseDelta(dx, dy, dtheta, valid, iterations)
        valid is False (delta all zero) if len(matches)+len(arc_matches)
        < MIN_MATCHES. Caller must check .valid before applying the delta
        to the pose -- this is a normal "not enough info this scan"
        outcome, not an error.
    """
    if len(matches) + len(arc_matches) < MIN_MATCHES:
        return PoseDelta(dx=0.0, dy=0.0, dtheta=0.0, valid=False, iterations=0)

    # Reject mismatched LINE pairs before they corrupt the least-squares
    # solve — see _reject_outlier_matches docstring. Arc matches are not
    # filtered here (see that function's docstring for why).
    matches = _reject_outlier_matches(matches)

    total_dx = 0.0
    total_dy = 0.0
    total_dtheta = 0.0
    iterations_run = 0

    # Working copy of line matches, re-expressed relative to the
    # accumulated guess each iteration so Gauss-Newton actually converges
    # instead of repeatedly solving the same linearisation. Arc centres
    # need the same treatment: their residual (map_c - scan_c) must be
    # folded by each iteration's correction too, or the same correction
    # gets re-solved and re-applied on every iteration (double-counting
    # across MAX_ITERATIONS instead of converging).
    working = list(matches)
    working_arcs = list(arc_matches)

    for _ in range(MAX_ITERATIONS):
        dtheta = _estimate_rotation(working, total_dtheta)
        dx, dy = _estimate_translation(working, working_arcs, dtheta)

        total_dx += dx
        total_dy += dy
        total_dtheta += dtheta
        iterations_run += 1

        # Re-linearise: fold this iteration's correction into the working
        # matches so the next iteration solves the *remaining* error, not
        # the same error again. Rotating scan_angle by dtheta accounts for
        # the heading update; dx/dy correction is folded into scan_distance
        # via projection onto the map normal.
        #
        # ORDER MATTERS HERE. The translation correction
        # (-sin(map_angle))*dx + cos(map_angle)*dy is expressed using
        # map_angle's normal vector, so it is only valid to apply to a
        # distance value that is ALREADY in the same sign convention as
        # map_distance. Folding in the correction first and normalising
        # afterward silently flips the effective sign of the correction
        # whenever the working match happens to still be in the opposite
        # half-plane from map_angle.
        #
        # FIX: normalise into map_angle's half-plane FIRST, then rotate by
        # dtheta and fold in the translation correction in that
        # consistent, already-aligned frame.
        new_working = []
        for m in working:
            norm_angle, norm_distance = _normalize_pair(
                m.scan_angle, m.scan_distance, m.map_angle
            )
            angle = norm_angle + dtheta
            distance = norm_distance
            distance += (-math.sin(m.map_angle)) * dx + math.cos(m.map_angle) * dy
            angle, distance = _normalize_pair(angle, distance, m.map_angle)
            new_working.append(LineMatch(
                scan_angle=angle,
                scan_distance=distance,
                map_angle=m.map_angle,
                map_distance=m.map_distance,
            ))
        working = new_working

        # Fold this iteration's translation into the working arc centres
        # the same way it was folded into the working line distances above
        # — otherwise the next iteration re-solves and re-applies the same
        # correction instead of the remaining residual.
        working_arcs = [
            ArcMatch(
                scan_cx=am.scan_cx + dx,
                scan_cy=am.scan_cy + dy,
                map_cx=am.map_cx,
                map_cy=am.map_cy,
            )
            for am in working_arcs
        ]

        correction_mag = abs(dx) + abs(dy) + abs(dtheta)
        if correction_mag < CONVERGENCE_THRESHOLD:
            break

    return PoseDelta(
        dx=total_dx, dy=total_dy, dtheta=total_dtheta,
        valid=True, iterations=iterations_run
    )


def build_line_matches_from_match_result(scan_features, map_entries, match_result,
                                          static_status=1):
    """
    Glue helper — NOT part of the core algorithm.

    This is the one place allowed to know about both scan-feature dicts
    (from fit_first_ctypes.split_merge) and MapEntry objects (from
    map_manager). It extracts plain LineMatch tuples for matched LINE pairs
    only, skipping ARC matches (those go through
    build_arc_matches_from_match_result below instead).

    STATUS FILTER — only STATIC map entries drive pose correction.
    A DYNAMIC entry (person, moved chair — see map_manager.py's own
    classification) or a fresh UNCLASSIFIED entry (not yet confirmed
    stable) can shift position for reasons that have nothing to do with
    the robot's own motion. Feeding those into the rigid-transform solver
    injects noise or outright wrong pose corrections — the robot would
    appear to "move" every time someone walks past it, even while
    physically stationary. Restricting to STATIC entries means pose
    correction simply does not run until the map has matured enough to
    have confirmed static anchors (map_manager's MIN_OBS_FOR_STATIC) —
    no correction is safer than a wrong one built from unverified data.

    pose_estimator.py deliberately does not import map_manager.py (stays
    a pure numeric module — see module docstring), so the STATIC status
    value is passed in as a parameter rather than imported, keeping this
    function decoupled from map_manager's internal enum while still
    enforcing the filter at the one boundary allowed to know both shapes.

    In the C port this logic lives in slam.c, not in pose_estimator.c —
    pose_estimator.c stays a pure numeric module exactly like its Python
    counterpart above this function.

    Parameters
    ----------
    scan_features : list of dicts (line_matcher / map_manager feature format)
    map_entries    : list of MapEntry
    match_result   : MatchResult from line_matcher.match_features()
    static_status  : value of MapEntry.status that means "STATIC"
                     (default 1, matching map_manager.ENTRY_STATIC — passed
                     explicitly by callers that import map_manager so this
                     module never has to)

    Returns
    -------
    list of LineMatch — ready to pass into estimate_pose_delta()
    """
    out = []
    for si, mi, _score in match_result.matched:
        sf = scan_features[si]
        if sf.get("type") != "line":
            continue
        entry = map_entries[mi]
        if entry.is_arc():
            continue
        if entry.status != static_status:
            continue
        out.append(LineMatch(
            scan_angle=sf["angle"],
            scan_distance=sf["distance"],
            map_angle=entry.angle,
            map_distance=entry.distance,
        ))
    return out


def build_arc_matches_from_match_result(scan_features, map_entries, match_result,
                                         static_status=1):
    """
    Glue helper — NOT part of the core algorithm. Arc-match counterpart to
    build_line_matches_from_match_result() above; same STATUS FILTER
    reasoning applies (only STATIC map entries drive pose correction).

    Extracts plain ArcMatch tuples for matched ARC pairs only, skipping
    LINE matches (those go through build_line_matches_from_match_result
    instead). Radius is intentionally not carried through — Algorithm 3's
    translation solve uses centre position only; radius agreement was
    already enforced upstream by line_matcher's arc-matching thresholds
    before a pair ever reaches this module.

    Parameters
    ----------
    scan_features : list of dicts (line_matcher / map_manager feature format)
    map_entries    : list of MapEntry
    match_result   : MatchResult from line_matcher.match_features()
    static_status  : value of MapEntry.status that means "STATIC"
                     (default 1, matching map_manager.ENTRY_STATIC)

    Returns
    -------
    list of ArcMatch — ready to pass into estimate_pose_delta() / solve_pose_step()
    """
    out = []
    for si, mi, _score in match_result.matched:
        sf = scan_features[si]
        if sf.get("type") != "arc":
            continue
        entry = map_entries[mi]
        if not entry.is_arc():
            continue
        if entry.status != static_status:
            continue
        out.append(ArcMatch(
            scan_cx=sf["cx"],
            scan_cy=sf["cy"],
            map_cx=entry.mx,
            map_cy=entry.my,
        ))
    return out


# ---------------------------------------------------------------------------
# Self-test — run directly: python3 pose_estimator.py
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import sys

    print("pose_estimator self-test")
    print("=" * 50)

    def _transform_line_to_scan_frame(map_angle, map_distance, dx, dy, dtheta):
        """
        Ground-truth helper for tests: given a map line and a known robot
        pose offset (dx, dy, dtheta) FROM the map frame TO the scan frame,
        compute what the scan would observe for that line.
        """
        scan_angle = map_angle - dtheta
        nx, ny = -math.sin(map_angle), math.cos(map_angle)
        scan_distance = map_distance - (nx * dx + ny * dy)
        while scan_angle > math.pi / 2.0:
            scan_angle -= math.pi
            scan_distance = -scan_distance
        while scan_angle < -math.pi / 2.0:
            scan_angle += math.pi
            scan_distance = -scan_distance
        return scan_angle, scan_distance

    def _transform_arc_to_scan_frame(map_cx, map_cy, dx, dy, dtheta):
        """
        Ground-truth helper: given a map arc centre and a known robot pose
        offset (dx, dy, dtheta) FROM the map frame TO the scan frame,
        compute what the scan would observe for that arc's centre.
        Rotation is included for completeness even though pure translation
        is what these tests exercise for arcs.
        """
        cos_t, sin_t = math.cos(-dtheta), math.sin(-dtheta)
        cx = map_cx - dx
        cy = map_cy - dy
        rx = cx * cos_t - cy * sin_t
        ry = cy * cos_t + cx * sin_t
        return rx, ry

    # ── T1: pure translation, two perpendicular walls ─────────────────────
    true_dx, true_dy, true_dtheta = 0.10, 0.05, 0.0
    map_lines = [(0.0, 1.0), (math.pi / 2.0, 2.0)]
    matches = []
    for ma, md in map_lines:
        sa, sd = _transform_line_to_scan_frame(ma, md, true_dx, true_dy, true_dtheta)
        matches.append(LineMatch(scan_angle=sa, scan_distance=sd, map_angle=ma, map_distance=md))

    result = estimate_pose_delta(matches)
    assert result.valid, "T1 expected valid result"
    assert abs(result.dx - true_dx) < 1e-3, f"T1 dx off: {result.dx} vs {true_dx}"
    assert abs(result.dy - true_dy) < 1e-3, f"T1 dy off: {result.dy} vs {true_dy}"
    assert abs(result.dtheta - true_dtheta) < 1e-3, f"T1 dtheta off: {result.dtheta}"
    print(f"  T1 PASS  pure translation recovered: dx={result.dx:.4f} dy={result.dy:.4f} "
          f"dtheta={result.dtheta:.4f}  ({result.iterations} iters)")

    # ── T2: pure rotation, three walls at different angles ────────────────
    true_dx, true_dy, true_dtheta = 0.0, 0.0, 0.12
    map_lines = [(0.0, 1.0), (math.pi / 2.0, 2.0), (-math.pi / 4.0, 1.5)]
    matches = []
    for ma, md in map_lines:
        sa, sd = _transform_line_to_scan_frame(ma, md, true_dx, true_dy, true_dtheta)
        matches.append(LineMatch(scan_angle=sa, scan_distance=sd, map_angle=ma, map_distance=md))

    result = estimate_pose_delta(matches)
    assert result.valid, "T2 expected valid result"
    assert abs(result.dx) < 1e-3, f"T2 dx should be ~0: {result.dx}"
    assert abs(result.dy) < 1e-3, f"T2 dy should be ~0: {result.dy}"
    assert abs(result.dtheta - true_dtheta) < 1e-3, f"T2 dtheta off: {result.dtheta} vs {true_dtheta}"
    print(f"  T2 PASS  pure rotation recovered: dtheta={result.dtheta:.4f} "
          f"(true={true_dtheta:.4f})  ({result.iterations} iters)")

    # ── T3: combined rotation + translation, four walls ────────────────────
    true_dx, true_dy, true_dtheta = 0.15, -0.08, 0.07
    map_lines = [(0.0, 1.0), (math.pi / 2.0, 2.0), (-math.pi / 3.0, 1.2), (math.pi / 6.0, 0.8)]
    matches = []
    for ma, md in map_lines:
        sa, sd = _transform_line_to_scan_frame(ma, md, true_dx, true_dy, true_dtheta)
        matches.append(LineMatch(scan_angle=sa, scan_distance=sd, map_angle=ma, map_distance=md))

    result = estimate_pose_delta(matches)
    assert result.valid, "T3 expected valid result"
    assert abs(result.dx - true_dx) < 1e-3, f"T3 dx off: {result.dx} vs {true_dx}"
    assert abs(result.dy - true_dy) < 1e-3, f"T3 dy off: {result.dy} vs {true_dy}"
    assert abs(result.dtheta - true_dtheta) < 1e-3, f"T3 dtheta off: {result.dtheta} vs {true_dtheta}"
    print(f"  T3 PASS  combined transform recovered: dx={result.dx:.4f} dy={result.dy:.4f} "
          f"dtheta={result.dtheta:.4f}  ({result.iterations} iters)")

    # ── T4: PI-symmetry — scan line reports opposite eigenvector sign ─────
    true_dx, true_dy, true_dtheta = 0.05, 0.02, 0.03
    map_lines = [(0.05, 1.0), (math.pi / 2.0 - 0.1, 1.8), (-math.pi / 2.0 + 0.2, 0.9)]
    matches = []
    for ma, md in map_lines:
        sa, sd = _transform_line_to_scan_frame(ma, md, true_dx, true_dy, true_dtheta)
        matches.append(LineMatch(scan_angle=sa, scan_distance=sd, map_angle=ma, map_distance=md))
    flipped = matches[0]
    flipped_angle = flipped.scan_angle + math.pi
    matches[0] = LineMatch(
        scan_angle=flipped_angle,
        scan_distance=-flipped.scan_distance,
        map_angle=flipped.map_angle,
        map_distance=flipped.map_distance,
    )

    result = estimate_pose_delta(matches)
    assert result.valid, "T4 expected valid result"
    assert abs(result.dx - true_dx) < 1e-3, f"T4 dx off: {result.dx} vs {true_dx}"
    assert abs(result.dy - true_dy) < 1e-3, f"T4 dy off: {result.dy} vs {true_dy}"
    assert abs(result.dtheta - true_dtheta) < 1e-3, f"T4 dtheta off: {result.dtheta} vs {true_dtheta}"
    print(f"  T4 PASS  PI-symmetry flip handled: dx={result.dx:.4f} dy={result.dy:.4f} "
          f"dtheta={result.dtheta:.4f}")

    # ── T5: too few matches -> invalid result, no crash ────────────────────
    result5 = estimate_pose_delta([
        LineMatch(scan_angle=0.0, scan_distance=1.0, map_angle=0.0, map_distance=1.0)
    ])
    assert result5.valid is False, "T5 expected invalid (< MIN_MATCHES)"
    assert result5.dx == 0.0 and result5.dy == 0.0 and result5.dtheta == 0.0
    print(f"  T5 PASS  single match -> valid=False, zero delta, no crash")

    result5b = estimate_pose_delta([])
    assert result5b.valid is False, "T5b expected invalid (zero matches)"
    print(f"  T5b PASS  zero matches -> valid=False, zero delta, no crash")

    # ── T6: all-parallel matches -> singular translation, rotation still OK ─
    true_dtheta = 0.04
    map_lines = [(0.0, 1.0), (0.0, 1.5)]
    matches = []
    for ma, md in map_lines:
        sa, sd = _transform_line_to_scan_frame(ma, md, 0.0, 0.0, true_dtheta)
        matches.append(LineMatch(scan_angle=sa, scan_distance=sd, map_angle=ma, map_distance=md))
    result6 = estimate_pose_delta(matches)
    assert result6.valid, "T6 expected valid result (degenerate but should not crash)"
    assert abs(result6.dtheta - true_dtheta) < 1e-3, f"T6 dtheta off: {result6.dtheta}"
    print(f"  T6 PASS  parallel-walls degenerate case: no crash, dtheta={result6.dtheta:.4f}")

    # ── T7: parallel-wall translation ambiguity is FIXED by one arc match ──
    # Two lines both at EXACTLY angle=0 (parallel, e.g. two facing walls of
    # a corridor) each have their Hough NORMAL along y only -- their
    # combined A^T A matrix has a11=a12=a21=0 identically, so the dense 2x2
    # Cramer solve is exactly singular (det=0) and _solve_2x2's guard
    # correctly refuses to guess, returning (0.0, 0.0) for BOTH axes -- not
    # just the direction that's truly unobservable (this row/column
    # pattern is why a merely "mostly parallel" real-world match set can
    # silently amplify noise into a large wrong answer instead of cleanly
    # zeroing out: see check_angular_diversity's docstring for that
    # near-but-not-exactly-singular case). With one matched arc centre
    # added, translation should be fully recovered on both axes.
    true_dx, true_dy, true_dtheta = 0.20, 0.05, 0.0
    map_lines7 = [(0.0, 1.0), (0.0, -1.0)]   # both parallel to x-axis
    matches7 = []
    for ma, md in map_lines7:
        sa, sd = _transform_line_to_scan_frame(ma, md, true_dx, true_dy, true_dtheta)
        matches7.append(LineMatch(scan_angle=sa, scan_distance=sd, map_angle=ma, map_distance=md))

    result7_no_arc = estimate_pose_delta(matches7)
    assert result7_no_arc.valid, "T7 expected valid result even when degenerate"
    assert abs(result7_no_arc.dx) < 1e-9 and abs(result7_no_arc.dy) < 1e-9, \
        (f"T7 setup check: exactly-parallel lines alone must leave translation "
         f"fully unobserved (0,0) via the singular-system guard, got "
         f"dx={result7_no_arc.dx} dy={result7_no_arc.dy}")

    map_cx, map_cy = 0.5, 0.5
    scan_cx, scan_cy = _transform_arc_to_scan_frame(map_cx, map_cy, true_dx, true_dy, true_dtheta)
    arc_matches7 = [ArcMatch(scan_cx=scan_cx, scan_cy=scan_cy, map_cx=map_cx, map_cy=map_cy)]

    result7 = estimate_pose_delta(matches7, arc_matches7)
    assert result7.valid, "T7 expected valid result with arc present"
    assert abs(result7.dx - true_dx) < 1e-3, \
        f"T7 dx should now be recovered via the arc constraint: {result7.dx} vs {true_dx}"
    assert abs(result7.dy - true_dy) < 1e-3, \
        f"T7 dy should now be recovered via the arc constraint: {result7.dy} vs {true_dy}"
    print(f"  T7 PASS  parallel-wall translation ambiguity (lines alone gave "
          f"dx={result7_no_arc.dx:.4f} dy={result7_no_arc.dy:.4f}) fully resolved "
          f"by one arc match: dx={result7.dx:.4f} dy={result7.dy:.4f}")

    # ── T8: check_angular_diversity — arc presence bypasses the parallel
    #         line check entirely ─────────────────────────────────────────
    parallel_matches = [
        LineMatch(scan_angle=0.0, scan_distance=1.0, map_angle=0.0, map_distance=1.0),
        LineMatch(scan_angle=0.01, scan_distance=1.5, map_angle=0.01, map_distance=1.5),
    ]
    assert check_angular_diversity(parallel_matches) is False, \
        "T8 parallel lines with no arc must fail diversity check"
    assert check_angular_diversity(parallel_matches, arc_matches=[
        ArcMatch(scan_cx=0.0, scan_cy=0.0, map_cx=0.0, map_cy=0.0)
    ]) is True, "T8 presence of an arc match must bypass the parallel-line check"
    print(f"  T8 PASS  check_angular_diversity: parallel lines rejected alone, "
          f"accepted once an arc match is present")

    print()
    print("All tests passed.")
    sys.exit(0)