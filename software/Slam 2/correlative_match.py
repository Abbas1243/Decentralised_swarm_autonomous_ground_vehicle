"""
correlative_match.py
=====================
Correlative (search-and-score) coarse pose estimation over LINE and ARC
features. Runs BEFORE line_matcher's hard-correspondence matching, to solve
the chicken-and-egg problem documented in slam.py / pose_estimator.py:
hard correspondence (line_matcher) can only be trusted once the pose guess
is already close; but the pose guess only gets better by trusting some
correspondence. Correlative search breaks that loop by never committing to
a single correspondence at all -- it scores many candidate poses against
the WHOLE feature set at once (smooth, soft scoring) and returns whichever
candidate pose makes the scan agree best with the map, globally.

This is the feature-space analogue of Hector/Cartographer's real-time
correlative scan matcher, scored on compact Hough (line) and centre+radius
(arc) parameters instead of a stored occupancy grid or point cloud --
zero extra RAM, no grid, plain scalar arithmetic only.

TWO-LAYER SEARCH -- WHY THIS EXISTS
------------------------------------------------------------------------
A single flat exhaustive sweep over one fixed-size window has two problems
that showed up directly on real hardware logs:

  1. FIXED WINDOW = FIXED REACH. A window wide enough to catch a fast
     combined rotate+translate motion is expensive to sweep at fine
     resolution every scan (cost grows ~step_count^3). Widening the window
     to chase fast motion means paying that cost on every scan, even slow
     ones.

  2. PICKING ONLY THE ARGMAX HIDES AMBIGUITY. A room with rectangular
     symmetry (two facing walls repeated on both axes) can produce a
     SECOND candidate pose -- e.g. the true pose rotated ~90 degrees --
     that scores nearly as well as the true one. A flat sweep evaluates
     both but then THROWS AWAY the comparison and returns only the
     winner. If noise ever tips the winner to the wrong peak, nothing
     downstream can tell the difference between "confidently correct"
     and "narrowly won against an equally plausible wrong answer" --
     this is exactly what produced an observed ~90 degree pose lock-in
     that then rebuilt the entire map around the wrong orientation.

FIX -- coarse-to-fine (breadth) + greedy multi-start refinement (reach):

  Layer 1 (breadth): sweep the FULL window at a COARSE, cheap step and
      keep the top-K distinct scoring candidates instead of just the
      single winner (see TOP_K_CANDIDATES, _insert_topk_distinct). This
      is what catches "there's a second, nearly-as-good peak" -- the
      thing a flat argmax sweep structurally cannot report.

  Layer 2 (reach): independently refine EACH of the K survivors with a
      small pattern-search / "three-step search"-style walk -- repeatedly
      sweep a SMALL window centred on the current best point, re-centre
      on whatever improves the score, stop when a round finds nothing
      better or MAX_WALK_ROUNDS is reached (see _greedy_refine). This
      lets a single coarse candidate reach well beyond the coarse grid's
      own step size cheaply, without paying for a permanently wide fine
      sweep every scan. Bounded by MAX_WALK_DTHETA_RAD / MAX_WALK_DXY_M
      measured from the ORIGINAL guess passed into search() -- same
      "cap the total correction, not the per-step size" philosophy as
      slam.py's MAX_DELTA_ROTATION_RAD / _clamp_iteration_step -- so an
      unlucky sequence of local improvements cannot wander arbitrarily
      far from a trustworthy region.

  After both layers, the two best refined candidates are compared. If
  they are close (see AMBIGUITY_MARGIN_RATIO), the result is flagged
  `ambiguous=True` in CoarseResult. dx/dy/dtheta/valid are still returned
  as before (this is additive, not a new rejection gate) -- callers
  (slam.py) are expected to treat an ambiguous coarse result the same
  way they already treat a low-confidence one: apply the pose update but
  do not trust it enough to author new map evidence until the ambiguity
  clears on a later scan. This is what closes the "silently locked onto
  the wrong symmetric peak" failure mode instead of just going faster.

  A single flat exhaustive sweep (K=1, no greedy refinement) is the
  special case this generalises -- setting TOP_K_CANDIDATES=1 and
  MAX_WALK_ROUNDS=0 recovers the original behaviour exactly.

WHY NOT BRANCH-AND-BOUND (Cartographer's real method)
------------------------------------------------------------------------
Branch-and-bound prunes large regions of the search space using a
provable score upper bound computed from a precomputed multi-resolution
occupancy grid. It finds the true global optimum touching a small
fraction of candidates a flat sweep would -- strictly better than the
two-layer approach here in principle. It is NOT used here because
computing that bound cheaply requires storing a grid, which is exactly
the RAM cost slam_build_prompt.md rejected line-feature SLAM to avoid on
the 512MB Duo S. The two-layer design above never stores a grid -- it is
a pure CPU-time optimisation over the same flat feature-scoring function
this module already used, so it costs zero additional RAM.

WHY NOT BINARY SEARCH
------------------------------------------------------------------------
Binary search assumes a unimodal (single-peak) cost surface so that
comparing two points tells you which half of the space to discard. The
score surface here is provably NOT always unimodal (see the rectangular-
room symmetry case above) -- discarding half the space on that
assumption would silently throw away the correct peak whenever a
competing one exists, which is worse than what this module had before,
not better.

WHY SOFT SCORING INSTEAD OF line_matcher'S HARD THRESHOLDS
-------------------------------------------------------------
line_matcher._score_line() returns None (pair does not exist) the instant
angle_diff or dist_diff exceeds its threshold. That is correct and
necessary for the FINE stage once the pose is already close -- but it is
exactly why a slightly-wrong pose guess makes true correspondences vanish
silently. Here, every (scan feature, map feature) pair always produces a
quality value in (0, 1] via a Gaussian falloff -- a near-miss scores low
but never disappears -- so a single ambiguous or borderline feature can
never derail the search; it is simply outvoted by every other feature
agreeing at the true pose.

WHY STATIC-ONLY
---------------
Exactly the same reasoning as pose_estimator.build_line_matches_from_
match_result: a DYNAMIC entry (person, moved chair) or a fresh
UNCLASSIFIED entry can shift position for reasons that have nothing to do
with the robot's own motion. Scoring against those would let something
else moving in the room drag the coarse pose search in the wrong
direction. Only confirmed-stable STATIC map entries are used here. Like
pose_estimator.py, this module does NOT import map_manager.py -- the
STATIC status value is passed in by the caller (slam.py) as a parameter,
keeping this module a pure numeric module with no knowledge of MapEntry
internals beyond the handful of attributes it reads directly (mirrors the
existing line_matcher.py / pose_estimator.py convention exactly).

CONFIDENCE GATING -- DO NOT SKIP THIS
--------------------------------------
A correlative search ALWAYS returns some best-scoring candidate out of
whatever grid it searched, even if every candidate is a bad fit (e.g. real
motion exceeded the search window, or the map has too few STATIC anchors
to be trustworthy yet). Silently returning that "best of a bad set" as if
it were a real answer would just relocate the exact failure mode this
module exists to fix -- a confidently wrong pose feeding downstream and
polluting the map. search() therefore applies gates before declaring a
result valid:
    1. n_static_lines + n_static_arcs must meet MIN_STATIC_FEATURES --
       too few anchors to search against reliably (early scans, or right
       after a DYNAMIC purge).
    2. best_score / n_scan_features_scored must meet MIN_SCORE_PER_FEATURE
       -- the winning candidate must actually explain the scan reasonably
       well, not just be the least-bad of a poor set.
Failing either gate returns valid=False with zero deltas. Callers MUST
treat that exactly like today's "too few matches" case: fall back to the
existing pose guess (do not fabricate a pose), and this scan should not be
allowed to author new map evidence (see slam.py's skip_map_update).
A THIRD, separate signal -- `ambiguous` -- does NOT force valid=False; it
tells the caller the winning candidate had a close competitor, so the
delta can still be applied but should not yet be trusted to author new
map evidence (see slam.py's delta_fully_trusted gating).

PERFORMANCE
-----------
Layer 1 (coarse sweep): for a fixed candidate dtheta, every scan feature
is rotated exactly ONCE (cos/sin computed once per dtheta step). Sweeping
the dx,dy sub-grid at that dtheta is then pure O(1) scalar arithmetic per
feature per candidate -- unchanged from the original single-layer design,
just run at a coarser step (COARSE_DTHETA_STEP_RAD / COARSE_DXY_STEP_M)
than before, so the total candidate count is LOWER than the original
flat fine sweep despite covering the same (or a wider) range.

Layer 2 (greedy refine): each of the K survivors is refined independently
with a small local window (WALK_WINDOW_DTHETA_RAD / WALK_WINDOW_DXY_M) at
a finer step, for up to MAX_WALK_ROUNDS rounds, stopping as soon as a
round finds no improvement. Total candidates evaluated across all K walks
is bounded and, in the common case (few rounds needed), well under the
cost of a single flat fine sweep over the original wide range -- while
still reaching MAX_WALK_DTHETA_RAD / MAX_WALK_DXY_M from the original
guess, which can exceed the layer-1 window. Exact costs are tunable via
the constants below; profile on-target (Duo S) before finalising them,
same as every other tuning constant in this codebase.

PORT PATH
---------
When porting to C (slam_core/correlative_match.c):
    search()  -> correlative_search(const ScanFeature *scan, int n_scan,
                                     const MapEntry *map, int n_map,
                                     float guess_x, float guess_y,
                                     float guess_theta,
                                     CoarseResult *out)
    Constants below become #defines in correlative_match.h.
    _coarse_sweep_topk()   -> fixed-size top-K array (K small, e.g. 3),
        insertion sort in place -- no dynamic allocation, same pattern
        map_manager.c's fixed MAX_MAP_ENTRIES array already uses.
    _greedy_refine()       -> fixed-bound triple for-loop per round, same
        no-malloc pattern as every other module in slam_core/.
    No dynamic allocation anywhere -- all loops are fixed-bound; the
    per-dtheta rotated-feature cache is a fixed-size MAX_FEATURES-length
    scratch array reused every dtheta step, exactly as before.
"""

import math
import time
import numpy as np
from collections import namedtuple

# ---------------------------------------------------------------------------
# PERFORMANCE FIX 1 -- static-entry cap (mirrors the fix later applied in
# the pyramid rewrite of this module, after a confirmed ~15-20x per-scan
# slowdown was measured once the map reached ~30 STATIC entries: search()
# cost scales with n_static_entries because EVERY candidate pose scores
# against EVERY static line/arc -- see _score_candidate's inner loop. Left
# uncapped, this grows with the map forever. A STATIC entry far from the
# current pose guess contributes almost nothing to the Gaussian soft-score
# anyway (SIGMA_DIST_M/SIGMA_CENTRE_M ~ 0.15m falloff), so capping to the
# nearest MAX_STATIC_ENTRIES_SEARCHED entries loses negligible real signal
# for what is, physically, always a local search. This was previously
# MISSING from this two-layer file -- ported over unchanged rather than
# reinvented, since it is a pure win with no behavioural downside at
# realistic map sizes (confirmed by T1-T7 below still passing unmodified).
# ---------------------------------------------------------------------------
MAX_STATIC_ENTRIES_SEARCHED = 70

# ---------------------------------------------------------------------------
# Layer 1 -- coarse sweep constants
# ---------------------------------------------------------------------------

SEARCH_DTHETA_MAX_RAD  = math.radians(80.0)  # full range swept by layer 1
SEARCH_DXY_MAX_M       = 0.15            # full range swept by layer 1
COARSE_DTHETA_STEP_RAD = math.radians(10.0)   # layer 1 step -- coarse, cheap
COARSE_DXY_STEP_M      = 0.05             # layer 1 step -- coarse, cheap

TOP_K_CANDIDATES = 3   # how many DISTINCT coarse peaks survive layer 1.
                        # K=1 recovers the original single-winner behaviour.

MIN_PEAK_SEPARATION_DTHETA_RAD = math.radians(20.0)
MIN_PEAK_SEPARATION_DXY_M      = 0.15
# Two coarse candidates within this angular/linear distance of each other
# are treated as samples of the SAME local peak (adjacent grid cells),
# not two distinct peaks -- see _insert_topk_distinct's docstring for why
# this matters.

# ---------------------------------------------------------------------------
# Layer 2 -- greedy local-refinement ("pattern search" / "three-step
# search" family) constants
# ---------------------------------------------------------------------------

WALK_DTHETA_STEP_RAD   = math.radians(3.0)
WALK_WINDOW_DTHETA_RAD = math.radians(15.0)   # +/- this much searched each round
WALK_DXY_STEP_M        = 0.03
WALK_WINDOW_DXY_M      = 0.06                # +/- this much searched each round
MAX_WALK_ROUNDS        = 3                   # re-centre at most this many times

MAX_WALK_DTHETA_RAD = math.radians(30.0)  # total reach cap, measured from the
MAX_WALK_DXY_M       = 0.25               # ORIGINAL guess passed into search()
                                           # (not from wherever a walk started)
                                           # -- see _greedy_refine docstring.

IMPROVEMENT_EPS = 1e-6   # a round must beat the current best by more than
                          # this to count as "improved" -- avoids infinite
                          # micro-oscillation from floating point noise.

# ---------------------------------------------------------------------------
# Ambiguity detection
# ---------------------------------------------------------------------------

AMBIGUITY_MARGIN_RATIO = 0.10
# If the second-best refined candidate scores within this fraction of the
# best candidate's score, the result is flagged ambiguous=True. Starting
# value -- tune tighter/looser once real logged (score, second_score)
# pairs from a known-good run are available, same spirit as every other
# "deliberately generous starting value" constant elsewhere in this
# codebase (e.g. pose_estimator.MAX_ACCEPTABLE_RESIDUAL).

# ---------------------------------------------------------------------------
# Soft-scoring falloff widths (NOT hard thresholds -- see module docstring)
# ---------------------------------------------------------------------------

SIGMA_ANGLE_RAD = 0.15    # line angle falloff width
SIGMA_DIST_M    = 0.15    # line Hough-distance falloff width
SIGMA_CENTRE_M  = 0.15    # arc centre-distance falloff width
SIGMA_R_M       = 0.08    # arc radius falloff width

# ---------------------------------------------------------------------------
# Confidence gates -- see module docstring "CONFIDENCE GATING"
# ---------------------------------------------------------------------------

MIN_STATIC_FEATURES    = 3      # need at least this many STATIC map anchors
MIN_SCORE_PER_FEATURE  = 0.35   # winning candidate must average at least this
                                 # per scored scan feature (max possible is 1.0)

# ---------------------------------------------------------------------------
# Return type
# ---------------------------------------------------------------------------

CoarseResult = namedtuple(
    "CoarseResult",
    ["dx", "dy", "dtheta", "score", "n_static", "valid",
     "ambiguous", "second_score"]
)
# dx, dy       : metres  -- translation correction ON TOP OF the pose_guess passed in
# dtheta       : radians -- rotation correction ON TOP OF the pose_guess passed in
# score        : float   -- winning candidate's total score (diagnostics only)
# n_static     : int      -- number of STATIC map entries searched against
# valid        : bool     -- False if either confidence gate failed; dx/dy/dtheta
#                            are 0.0 in that case, caller must NOT apply them and
#                            must treat this scan like a low-confidence scan.
# ambiguous    : bool     -- True if the second-best refined candidate scored
#                            within AMBIGUITY_MARGIN_RATIO of the winner (e.g.
#                            a rectangular room's rotational symmetry). dx/dy/
#                            dtheta are still the winner's values -- caller
#                            should apply the pose update but NOT treat it as
#                            trustworthy enough to author new map evidence
#                            until the ambiguity clears (see slam.py).
# second_score : float    -- the second-best refined candidate's score, 0.0
#                            if fewer than 2 distinct candidates survived.
#                            Diagnostic / for tuning AMBIGUITY_MARGIN_RATIO.

# ---------------------------------------------------------------------------
# Timing diagnostics -- module-level last_* attributes, same established
# pattern as slam.py's SlamState.last_iterations_run etc. Populated on
# every search() call so a caller (or a benchmark script) can see WHERE
# time actually went, instead of assuming. Not used for any control-flow
# decision -- pure instrumentation, same "diagnostics only" status as
# CoarseResult.score.
# ---------------------------------------------------------------------------
last_search_total_s   = 0.0
last_search_layer1_s  = 0.0
last_search_layer2_s  = 0.0
last_search_n_static  = 0
last_search_n_layer1_candidates = 0
last_search_n_layer2_candidates = 0


def _rotate_point(x, y, cos_t, sin_t):
    return x * cos_t - y * sin_t, y * cos_t + x * sin_t


def _wrap_line_angle(angle):
    """Wrap into [-pi/2, pi/2], matching the Hough convention used
    everywhere else (line_matcher, map_manager, pose_estimator, slam)."""
    if angle > math.pi / 2.0:
        angle -= math.pi
    elif angle < -math.pi / 2.0:
        angle += math.pi
    return angle


def _angle_diff_line(a, b):
    """Same PI-symmetric angular difference as line_matcher._angle_diff_line."""
    diff = abs(a - b) % math.pi
    if diff > math.pi / 2.0:
        diff = math.pi - diff
    return diff


def _prepare_static_maps(map_entries, static_status, guess_x=None, guess_y=None,
                          max_entries=MAX_STATIC_ENTRIES_SEARCHED):
    """
    Split active STATIC map entries into line / arc lists once per call.

    PERFORMANCE FIX 1 (see MAX_STATIC_ENTRIES_SEARCHED above): when
    guess_x/guess_y are given and the combined STATIC entry count exceeds
    max_entries, only the max_entries entries nearest (by midpoint/centre
    distance) to the current pose guess are kept. This is what keeps
    search()'s per-scan cost bounded as the map grows, instead of
    degrading linearly (in practice, worse -- see PERFORMANCE FIX 2 below,
    every kept entry is re-scored by every Layer-2 candidate too) with
    total map size forever.

    guess_x/guess_y default to None (no cap applied) so existing callers
    that don't pass a guess (e.g. direct unit tests) are unaffected --
    identical behaviour to the pre-fix version at small map sizes.
    """
    candidates = [
        e for e in map_entries if e.active and e.status == static_status
    ]

    if guess_x is not None and guess_y is not None and len(candidates) > max_entries:
        candidates.sort(key=lambda e: (e.mx - guess_x) ** 2 + (e.my - guess_y) ** 2)
        candidates = candidates[:max_entries]

    static_lines = []
    static_arcs = []
    for e in candidates:
        if e.is_arc():
            static_arcs.append(e)
        else:
            static_lines.append(e)
    return static_lines, static_arcs


def _prerotate_features(scan_features, cos_t, sin_t, guess_theta):
    """
    Rotate every scan feature by (guess_theta + dtheta) ONCE for this dtheta
    step. Returns two lists of lightweight tuples ready for O(1) dx,dy
    scoring:
        rotated_lines : list of (rmx, rmy, angle', nx, ny, base_dist)
        rotated_arcs  : list of (rcx, rcy, r)
    Non-line/arc features are skipped (mirrors line_matcher's handling of
    unknown types).
    """
    rotated_lines = []
    rotated_arcs = []

    for feat in scan_features:
        ftype = feat.get("type")

        if ftype == "line":
            x1, y1 = _rotate_point(feat["x1"], feat["y1"], cos_t, sin_t)
            x2, y2 = _rotate_point(feat["x2"], feat["y2"], cos_t, sin_t)
            rmx = (x1 + x2) / 2.0
            rmy = (y1 + y2) / 2.0
            angle = _wrap_line_angle(feat["angle"] + guess_theta)
            nx, ny = -math.sin(angle), math.cos(angle)
            base_dist = nx * rmx + ny * rmy
            rotated_lines.append((rmx, rmy, angle, nx, ny, base_dist))

        elif ftype == "arc":
            rcx, rcy = _rotate_point(feat["cx"], feat["cy"], cos_t, sin_t)
            rotated_arcs.append((rcx, rcy, feat["r"]))

    return rotated_lines, rotated_arcs


def _score_candidate(rotated_lines, rotated_arcs, dx, dy,
                      static_lines, static_arcs):
    """
    SCALAR REFERENCE implementation. Score one candidate (dx, dy) at the
    dtheta already baked into rotated_lines/rotated_arcs. Returns
    (total_score, n_features_scored). Pure O(1)-per-feature-per-map-entry
    arithmetic -- no trig here.

    PERFORMANCE NOTE (profiled -- see progress notes): this loop, called
    once per (dx,dy,dtheta) candidate, is the ACTUAL dominant cost of
    search() -- 97%+ of runtime at n_static=20, not _prerotate_features
    (under 2%). _score_grid_vectorized below batches this exact
    computation across an entire (dx,dy) grid at once via numpy instead of
    one Python-level call per candidate; kept here UNCHANGED as the
    ground-truth reference for test_vectorized_matches_scalar below, and
    as the direct 1:1 source for the eventual hand-written C port (numpy
    has no C equivalent -- the scalar loop shape here, not the vectorized
    one, is what slam_core/correlative_match.c should mirror).
    """
    total = 0.0
    n_scored = 0

    for (rmx, rmy, angle, nx, ny, base_dist) in rotated_lines:
        dist = base_dist + nx * dx + ny * dy
        best_q = 0.0
        for e in static_lines:
            adiff = _angle_diff_line(angle, e.angle)
            # sign-aware distance, same reasoning as line_matcher._score_line
            ddiff = min(abs(dist - e.distance), abs(dist + e.distance))
            q = (math.exp(-(adiff / SIGMA_ANGLE_RAD) ** 2)
                 * math.exp(-(ddiff / SIGMA_DIST_M) ** 2))
            if q > best_q:
                best_q = q
        total += best_q
        n_scored += 1

    for (rcx, rcy, r) in rotated_arcs:
        cx = rcx + dx
        cy = rcy + dy
        best_q = 0.0
        for e in static_arcs:
            centre_dist = math.hypot(cx - e.mx, cy - e.my)
            rdiff = abs(r - e.distance)   # distance field holds radius
            q = (math.exp(-(centre_dist / SIGMA_CENTRE_M) ** 2)
                 * math.exp(-(rdiff / SIGMA_R_M) ** 2))
            if q > best_q:
                best_q = q
        total += best_q
        n_scored += 1

    return total, n_scored


def _static_line_arrays(static_lines):
    """Build once-per-search() numpy arrays of static line angle/distance,
    reused across every dtheta step of both Layer 1 and Layer 2 --
    avoids rebuilding these arrays per candidate."""
    if not static_lines:
        return np.empty(0), np.empty(0)
    return (np.array([e.angle for e in static_lines], dtype=np.float64),
            np.array([e.distance for e in static_lines], dtype=np.float64))


def _static_arc_arrays(static_arcs):
    """Build once-per-search() numpy arrays of static arc centre/radius."""
    if not static_arcs:
        return np.empty(0), np.empty(0), np.empty(0)
    return (np.array([e.mx for e in static_arcs], dtype=np.float64),
            np.array([e.my for e in static_arcs], dtype=np.float64),
            np.array([e.distance for e in static_arcs], dtype=np.float64))


def _score_grid_vectorized(rotated_lines, rotated_arcs, dx_grid, dy_grid,
                            se_angle, se_dist, sa_mx, sa_my, sa_r):
    """
    Score every (dx, dy) combination in dx_grid x dy_grid, at the ONE
    dtheta already baked into rotated_lines/rotated_arcs (from
    _prerotate_features), against the given STATIC line/arc arrays --
    all in one batch of numpy operations instead of one Python-level
    _score_candidate() call per (dx, dy) pair. Same exact arithmetic as
    _score_candidate (verified by test_vectorized_matches_scalar below,
    at a NONZERO guess_x/guess_y -- see the warning below for why that
    specific check matters), just computed for a whole grid at once.

    *** dx_grid / dy_grid MUST ALREADY BE ABSOLUTE VALUES, I.E.
    guess_x + dx_offset / guess_y + dy_offset -- NEVER dx_offset alone. ***

    This is not a stylistic note -- it is a direct callback to a real,
    already-shipped regression: the pyramid rewrite of this module (see
    slam_progress_update_coarse_search_and_trend_gating.md, "Bug 2")
    dropped guess_x/guess_y from an equivalent vectorized scoring path
    during an earlier rewrite. It passed every self-test that existed at
    the time because EVERY one of them called search() with
    guess_x=guess_y=0.0, and "add 0.0" and "don't add anything" are
    indistinguishable at that specific input. The bug then produced
    confidently-wrong (not low-confidence) coarse poses on real hardware
    once the robot had moved away from the map origin -- i.e. almost
    always. To avoid repeating that exact mistake here:
        1. Callers (see _coarse_sweep_topk / _greedy_refine below) build
           dx_grid/dy_grid with the guess offset already folded in, at
           the single point where the grid is constructed -- not
           threaded through as a separate parameter that could be
           silently forgotten at a call site.
        2. test_vectorized_matches_scalar below runs its cross-check at
           a NONZERO guess_x/guess_y specifically -- a dropped-offset bug
           is invisible at guess=(0,0) by construction, so a regression
           test that only exercises guess=(0,0) provides zero protection
           against this exact failure mode.
        3. search()'s own self-test T9 (new, added alongside this
           change) additionally checks END-TO-END recovered dx/dy at a
           nonzero guess, not just the internal scoring function in
           isolation -- catching the bug even if some future refactor
           moved where the offset gets added.

    Returns (total, n_scored):
        total    : np.ndarray shape (len(dx_grid), len(dy_grid))
        n_scored : int -- same definition as _score_candidate's second
                   return value
    """
    n_dx = dx_grid.shape[0]
    n_dy = dy_grid.shape[0]
    total = np.zeros((n_dx, n_dy), dtype=np.float64)
    n_scored = 0

    dxg = dx_grid[:, None]   # (n_dx, 1) -- broadcasts against dy below
    dyg = dy_grid[None, :]   # (1, n_dy)

    for (rmx, rmy, angle, nx, ny, base_dist) in rotated_lines:
        n_scored += 1
        if se_angle.size == 0:
            continue

        dist = base_dist + nx * dxg + ny * dyg          # (n_dx, n_dy)

        adiff = np.abs(angle - se_angle) % math.pi        # (n_static,)
        adiff = np.where(adiff > math.pi / 2.0, math.pi - adiff, adiff)

        d3 = dist[:, :, None]                              # (n_dx,n_dy,1)
        ddiff = np.minimum(np.abs(d3 - se_dist), np.abs(d3 + se_dist))
        q = (np.exp(-(adiff / SIGMA_ANGLE_RAD) ** 2)
             * np.exp(-(ddiff / SIGMA_DIST_M) ** 2))       # (n_dx,n_dy,n_static)
        total += q.max(axis=2)

    for (rcx, rcy, r) in rotated_arcs:
        n_scored += 1
        if sa_mx.size == 0:
            continue

        cx = rcx + dxg                                     # (n_dx, 1)
        cy = rcy + dyg                                     # (1, n_dy)
        cx3 = np.broadcast_to(cx, (n_dx, n_dy))[:, :, None]
        cy3 = np.broadcast_to(cy, (n_dx, n_dy))[:, :, None]
        centre_dist = np.sqrt((cx3 - sa_mx) ** 2 + (cy3 - sa_my) ** 2)
        rdiff = np.abs(r - sa_r)
        q = (np.exp(-(centre_dist / SIGMA_CENTRE_M) ** 2)
             * np.exp(-(rdiff / SIGMA_R_M) ** 2))
        total += q.max(axis=2)

    return total, n_scored


def _score_pose(scan_features, guess_theta, dtheta, dx, dy, guess_x, guess_y,
                 static_lines, static_arcs):
    """
    Scalar convenience wrapper -- no longer called by search() (Layer 2
    now uses _score_grid_vectorized, see _greedy_refine), kept only as a
    single-candidate reference matching _score_candidate's calling
    convention for ad-hoc debugging/testing.
    """
    cos_t = math.cos(guess_theta + dtheta)
    sin_t = math.sin(guess_theta + dtheta)
    rotated_lines, rotated_arcs = _prerotate_features(
        scan_features, cos_t, sin_t, guess_theta + dtheta
    )
    return _score_candidate(rotated_lines, rotated_arcs, guess_x + dx, guess_y + dy,
                             static_lines, static_arcs)


def _insert_topk_distinct(top, candidate, k):
    """
    Insert `candidate` = (score, dx, dy, dtheta, n_scored) into `top`
    (mutated in place, kept sorted descending by score, capped at k
    entries) -- but ONLY as a genuinely separate peak. Two candidates
    within MIN_PEAK_SEPARATION_DTHETA_RAD / MIN_PEAK_SEPARATION_DXY_M of
    each other are treated as the SAME peak (adjacent coarse grid cells
    sampling the same local maximum), and only the higher-scoring one is
    kept.

    WHY THIS MATTERS: without this de-duplication, top-K would frequently
    fill up with 2-3 grid cells all sitting right next to the SAME true
    peak, instead of K genuinely distinct candidate poses -- silently
    defeating the entire purpose of keeping more than one candidate (which
    is to catch a competing peak elsewhere in the search space, like a
    rectangular room's +/-90 degree symmetry).
    """
    score, dx, dy, dtheta, n_scored = candidate

    for i, (s2, dx2, dy2, dtheta2, n2) in enumerate(top):
        if (abs(dtheta - dtheta2) < MIN_PEAK_SEPARATION_DTHETA_RAD
                and math.hypot(dx - dx2, dy - dy2) < MIN_PEAK_SEPARATION_DXY_M):
            if score > s2:
                top[i] = candidate
                top.sort(key=lambda c: -c[0])
            return

    if len(top) < k:
        top.append(candidate)
        top.sort(key=lambda c: -c[0])
    elif score > top[-1][0]:
        top[-1] = candidate
        top.sort(key=lambda c: -c[0])


def _coarse_sweep_topk(scan_features, guess_x, guess_y, guess_theta,
                        static_lines, static_arcs, k):
    """
    LAYER 1 -- exhaustive sweep of the FULL (SEARCH_DTHETA_MAX_RAD,
    SEARCH_DXY_MAX_M) window at the COARSE step, keeping the top-k
    DISTINCT scoring candidates (see _insert_topk_distinct) instead of
    just the single best. This is what gives visibility into a competing
    peak that a flat argmax sweep would silently discard.

    VECTORIZED: the dtheta loop stays a plain Python loop (small -- 13
    steps at default constants), but for each dtheta the entire dx,dy
    grid (9x9=81 points at default constants) is scored in ONE batch via
    _score_grid_vectorized instead of 81 separate _score_candidate calls.
    Every grid cell that used to be individually scored and inserted
    still is -- this changes only HOW the scores are computed, not the
    search structure or which candidates get considered (see
    test_vectorized_matches_scalar for a direct numeric cross-check).

    Returns a list of up to k tuples (score, dx, dy, dtheta, n_scored),
    sorted descending by score. May return fewer than k if the window
    doesn't contain k distinct local peaks.
    """
    top = []
    n_candidates = 0

    se_angle, se_dist = _static_line_arrays(static_lines)
    sa_mx, sa_my, sa_r = _static_arc_arrays(static_arcs)

    n_dtheta_steps = int(round(2 * SEARCH_DTHETA_MAX_RAD / COARSE_DTHETA_STEP_RAD)) + 1
    dtheta_vals = (-SEARCH_DTHETA_MAX_RAD
                   + COARSE_DTHETA_STEP_RAD * np.arange(n_dtheta_steps))

    n_dxy_steps = int(round(2 * SEARCH_DXY_MAX_M / COARSE_DXY_STEP_M)) + 1
    dx_offsets = -SEARCH_DXY_MAX_M + COARSE_DXY_STEP_M * np.arange(n_dxy_steps)
    dy_offsets = -SEARCH_DXY_MAX_M + COARSE_DXY_STEP_M * np.arange(n_dxy_steps)
    # ABSOLUTE grids -- guess offset folded in HERE, at construction, not
    # threaded through _score_grid_vectorized as a separate parameter that
    # a future edit could forget to add (see that function's docstring).
    dx_grid = guess_x + dx_offsets
    dy_grid = guess_y + dy_offsets

    for dtheta in dtheta_vals:
        dtheta = float(dtheta)
        cos_t = math.cos(guess_theta + dtheta)
        sin_t = math.sin(guess_theta + dtheta)
        rotated_lines, rotated_arcs = _prerotate_features(
            scan_features, cos_t, sin_t, guess_theta + dtheta
        )

        total, n_scored = _score_grid_vectorized(
            rotated_lines, rotated_arcs, dx_grid, dy_grid,
            se_angle, se_dist, sa_mx, sa_my, sa_r
        )
        n_candidates += n_dxy_steps * n_dxy_steps
        if n_scored == 0:
            continue

        n_dx_local, n_dy_local = total.shape
        for ix in range(n_dx_local):
            dx_off = float(dx_offsets[ix])
            row = total[ix]
            for iy in range(n_dy_local):
                score = float(row[iy])
                if score <= 0.0:
                    continue
                dy_off = float(dy_offsets[iy])
                _insert_topk_distinct(top, (score, dx_off, dy_off, dtheta, n_scored), k)

    global last_search_n_layer1_candidates
    last_search_n_layer1_candidates = n_candidates
    return top


def _within_walk_bounds(dx, dy, dtheta):
    """True if (dx, dy, dtheta) -- an offset from the ORIGINAL guess passed
    into search() -- is still within the total reach cap for layer 2."""
    return (abs(dtheta) <= MAX_WALK_DTHETA_RAD
            and math.hypot(dx, dy) <= MAX_WALK_DXY_M)


def _greedy_refine(scan_features, guess_x, guess_y, guess_theta,
                    static_lines, static_arcs,
                    start_dx, start_dy, start_dtheta, start_score, start_n):
    """
    LAYER 2 -- pattern-search-style local refinement (the "three-step
    search" / "diamond search" family used for motion estimation in video
    codecs is the same core idea): repeatedly sweep a SMALL window centred
    on the current best candidate, re-centre on whatever improves the
    score, and stop once a round finds no improvement or MAX_WALK_ROUNDS
    is reached.

    This lets a single layer-1 candidate reach further than the coarse
    sweep's own window/step size cheaply (each round only pays for a small
    local window, not the whole original range), which is what lets this
    module track fast motion without paying for a permanently wide fine
    sweep on every scan.

    PERFORMANCE FIX 2a (rotation cache) + 2b (vectorized scoring):
    the ORIGINAL version of this function called _score_pose for every
    single (dx_off, dy_off, dtheta_off) candidate, which both re-rotated
    every scan feature from scratch AND scored one (dx,dy) at a time via
    a Python-level loop. Profiling (see progress notes) showed the
    rotation redundancy was actually a rounding error (<2% of total
    runtime) -- the REAL cost is the per-candidate scoring loop itself
    (_score_candidate's O(n_scan x n_static) inner loop, called once per
    candidate). This revision fixes both: rotate ONCE per dtheta_off
    (outer loop, same restructuring as before), then score the ENTIRE
    dx,dy sub-grid for that dtheta_off in one numpy batch via
    _score_grid_vectorized instead of one Python call per candidate.

    Same candidates evaluated, same bounds, same MAX_WALK_ROUNDS /
    stopping rule -- only HOW each candidate's score is computed changes
    (see test_vectorized_matches_scalar for a direct numeric cross-check
    against the untouched scalar reference, _score_candidate).

    BOUNDED SEARCH, NOT UNBOUNDED HILL-CLIMBING: every candidate evaluated
    must satisfy _within_walk_bounds relative to the ORIGINAL guess passed
    into search() -- not relative to wherever this particular walk started.
    Without this cap, a sequence of small local improvements could in
    principle keep walking the pose arbitrarily far from a trustworthy
    region, one small "still slightly better" step at a time. Same
    "cap the total correction, not the per-step size" philosophy as
    slam.py's MAX_DELTA_ROTATION_RAD / _clamp_iteration_step.

    KNOWN LIMITATION (unchanged by this fix -- why this is only HALF the
    fix, see module docstring): a greedy walk run from a SINGLE starting
    point can only ever discover the local peak nearest that starting
    point; it can walk past a competing peak's basin without ever noticing
    it exists. Making each candidate cheaper to evaluate does not change
    WHICH candidates get evaluated or WHERE the walk can converge -- this
    limitation is about search coverage, not about CPU cost, and is not
    addressed by this fix. This is exactly why search() runs this function
    independently from MULTIPLE starting points (the K survivors of layer
    1) rather than greedily refining only the single best coarse candidate
    -- each walk explores its own local neighbourhood, and comparing the K
    final results is what surfaces ambiguity instead of hiding it.

    Returns (dx, dy, dtheta, score, n_scored) -- the refined candidate.
    """
    cur_dx, cur_dy, cur_dtheta = start_dx, start_dy, start_dtheta
    cur_score, cur_n = start_score, start_n
    n_candidates = 0

    se_angle, se_dist = _static_line_arrays(static_lines)
    sa_mx, sa_my, sa_r = _static_arc_arrays(static_arcs)

    n_dtheta_off_steps = int(round(2 * WALK_WINDOW_DTHETA_RAD / WALK_DTHETA_STEP_RAD)) + 1
    n_dxy_off_steps = int(round(2 * WALK_WINDOW_DXY_M / WALK_DXY_STEP_M)) + 1
    dtheta_offsets = (-WALK_WINDOW_DTHETA_RAD
                      + WALK_DTHETA_STEP_RAD * np.arange(n_dtheta_off_steps))
    dxy_offsets = -WALK_WINDOW_DXY_M + WALK_DXY_STEP_M * np.arange(n_dxy_off_steps)

    for _ in range(MAX_WALK_ROUNDS):
        best_dx, best_dy, best_dtheta = cur_dx, cur_dy, cur_dtheta
        best_score, best_n = cur_score, cur_n

        for dtheta_off in dtheta_offsets:
            dtheta_off = float(dtheta_off)
            cand_dtheta = cur_dtheta + dtheta_off

            # Rotate ONCE for this dtheta_off -- reused across every
            # (dx_off, dy_off) candidate below (Fix 2a).
            cos_t = math.cos(guess_theta + cand_dtheta)
            sin_t = math.sin(guess_theta + cand_dtheta)
            rotated_lines, rotated_arcs = _prerotate_features(
                scan_features, cos_t, sin_t, guess_theta + cand_dtheta
            )

            # ABSOLUTE grids -- guess offset AND the walk's current
            # re-centre point (cur_dx/cur_dy) folded in HERE, at
            # construction -- same discipline as _coarse_sweep_topk, see
            # _score_grid_vectorized's docstring for why this matters.
            dx_grid = guess_x + cur_dx + dxy_offsets
            dy_grid = guess_y + cur_dy + dxy_offsets

            total, n_scored = _score_grid_vectorized(
                rotated_lines, rotated_arcs, dx_grid, dy_grid,
                se_angle, se_dist, sa_mx, sa_my, sa_r
            )
            n_candidates += n_dxy_off_steps * n_dxy_off_steps
            if n_scored == 0:
                continue

            for ix in range(n_dxy_off_steps):
                cand_dx = cur_dx + float(dxy_offsets[ix])
                row = total[ix]
                for iy in range(n_dxy_off_steps):
                    cand_dy = cur_dy + float(dxy_offsets[iy])
                    if not _within_walk_bounds(cand_dx, cand_dy, cand_dtheta):
                        continue
                    score = float(row[iy])
                    if score > best_score + IMPROVEMENT_EPS:
                        best_score, best_n = score, n_scored
                        best_dx, best_dy, best_dtheta = cand_dx, cand_dy, cand_dtheta

        if best_score <= cur_score + IMPROVEMENT_EPS:
            break   # no improvement this round -- converged, stop early
        cur_dx, cur_dy, cur_dtheta = best_dx, best_dy, best_dtheta
        cur_score, cur_n = best_score, best_n

    global last_search_n_layer2_candidates
    last_search_n_layer2_candidates += n_candidates
    return cur_dx, cur_dy, cur_dtheta, cur_score, cur_n



def search(scan_features, map_entries, guess_x, guess_y, guess_theta,
           static_status=1):
    """
    Run the two-layer correlative coarse pose search (see module docstring
    for the full breadth+reach rationale).

    Parameters
    ----------
    scan_features : list of dicts (fit_first_ctypes.split_merge output,
        SENSOR frame -- same convention as line_matcher / pose_estimator
        callers already use before any transform is applied)
    map_entries : list of MapEntry (from map_manager) -- only .active and
        .status == static_status entries are used; others ignored entirely
    guess_x, guess_y, guess_theta : current pose estimate (map frame) to
        search around. Pass self.current_pose's x/y/theta.
    static_status : value of MapEntry.status meaning STATIC (default 1,
        matching map_manager.ENTRY_STATIC) -- passed explicitly so this
        module never has to import map_manager, mirroring pose_estimator.py

    Returns
    -------
    CoarseResult -- dx, dy, dtheta are corrections to ADD to
    (guess_x, guess_y, guess_theta), NOT absolute values. valid=False means
    a confidence gate failed; caller must not apply the deltas (they are
    0.0) and must treat this scan as low-confidence, same handling as an
    invalid pose_estimator delta. ambiguous=True means a close competing
    candidate was found; caller should apply the delta but not yet trust
    it to author new map evidence (see slam.py's delta_fully_trusted).
    """
    global last_search_total_s, last_search_layer1_s, last_search_layer2_s
    global last_search_n_static, last_search_n_layer1_candidates
    global last_search_n_layer2_candidates
    last_search_n_layer2_candidates = 0   # accumulated across all K walks below
    t_search_start = time.perf_counter()

    # PERFORMANCE FIX 1: pass guess_x/guess_y through so the static-entry
    # cap (MAX_STATIC_ENTRIES_SEARCHED) can select the entries nearest the
    # current pose guess instead of scoring the whole map every scan.
    static_lines, static_arcs = _prepare_static_maps(
        map_entries, static_status, guess_x, guess_y
    )
    n_static = len(static_lines) + len(static_arcs)
    last_search_n_static = n_static

    if n_static < MIN_STATIC_FEATURES:
        last_search_total_s = time.perf_counter() - t_search_start
        return CoarseResult(dx=0.0, dy=0.0, dtheta=0.0, score=0.0,
                             n_static=n_static, valid=False,
                             ambiguous=False, second_score=0.0)

    # ---- Layer 1: coarse sweep, keep top-K distinct candidates ----------
    t_layer1_start = time.perf_counter()
    top = _coarse_sweep_topk(scan_features, guess_x, guess_y, guess_theta,
                              static_lines, static_arcs, TOP_K_CANDIDATES)
    last_search_layer1_s = time.perf_counter() - t_layer1_start

    if not top:
        last_search_total_s = time.perf_counter() - t_search_start
        return CoarseResult(dx=0.0, dy=0.0, dtheta=0.0, score=0.0,
                             n_static=n_static, valid=False,
                             ambiguous=False, second_score=0.0)

    # ---- Layer 2: refine each surviving candidate independently ---------
    t_layer2_start = time.perf_counter()
    refined = []
    for score, dx, dy, dtheta, n_scored in top:
        rdx, rdy, rdtheta, rscore, rn = _greedy_refine(
            scan_features, guess_x, guess_y, guess_theta,
            static_lines, static_arcs, dx, dy, dtheta, score, n_scored,
        )
        refined.append((rscore, rdx, rdy, rdtheta, rn))
    last_search_layer2_s = time.perf_counter() - t_layer2_start

    refined.sort(key=lambda c: -c[0])
    best_score, best_dx, best_dy, best_dtheta, best_n = refined[0]

    # Two DIFFERENT layer-1 seeds can walk (layer 2) into the SAME final
    # peak if their basins overlap -- refinement is a converging process,
    # not a fixed offset, so "started distinct" does not guarantee "ended
    # distinct". Comparing best_score against a near-duplicate of itself
    # would report ambiguity that isn't real. Only count a refined
    # candidate as the "second" one for ambiguity purposes if it is STILL
    # a genuinely separate peak from the winner after refinement, using
    # the same separation test _insert_topk_distinct used before
    # refinement.
    second_score = 0.0
    for (s, dx, dy, dtheta, n) in refined[1:]:
        if (abs(dtheta - best_dtheta) >= MIN_PEAK_SEPARATION_DTHETA_RAD
                or math.hypot(dx - best_dx, dy - best_dy) >= MIN_PEAK_SEPARATION_DXY_M):
            second_score = s
            break   # refined is sorted descending -- first distinct entry
                     # found is the best genuinely-competing candidate

    if best_n == 0:
        last_search_total_s = time.perf_counter() - t_search_start
        return CoarseResult(dx=0.0, dy=0.0, dtheta=0.0, score=0.0,
                             n_static=n_static, valid=False,
                             ambiguous=False, second_score=0.0)

    normalized = best_score / best_n
    if normalized < MIN_SCORE_PER_FEATURE:
        last_search_total_s = time.perf_counter() - t_search_start
        return CoarseResult(dx=0.0, dy=0.0, dtheta=0.0, score=best_score,
                             n_static=n_static, valid=False,
                             ambiguous=False, second_score=second_score)

    ambiguous = (len(refined) > 1
                 and second_score >= best_score * (1.0 - AMBIGUITY_MARGIN_RATIO))

    last_search_total_s = time.perf_counter() - t_search_start
    return CoarseResult(dx=best_dx, dy=best_dy, dtheta=best_dtheta,
                         score=best_score, n_static=n_static, valid=True,
                         ambiguous=ambiguous, second_score=second_score)


# ---------------------------------------------------------------------------
# Self-test -- run directly: python3 correlative_match.py
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import sys
    from collections import namedtuple as _nt

    _MapEntryStub = _nt("_MapEntryStub",
        ["angle", "distance", "mx", "my", "status", "active"])

    class _ME(_MapEntryStub):
        def is_arc(self):
            return self.angle < -4.0

    def _line_feat(angle, distance, x1, y1, x2, y2):
        return {"type": "line", "angle": angle, "distance": distance,
                "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                "length": math.hypot(x2 - x1, y2 - y1), "quality": 100}

    print("correlative_match self-test")
    print("=" * 50)

    # ── T1: map built at origin, scan taken after a real 8cm/6deg motion ──
    # (well outside line_matcher's hard thresholds at zero-guess, but well
    # inside this module's search window)
    true_dx, true_dy, true_dtheta = 0.08, -0.05, math.radians(6.0)

    def _shift(feat, dx, dy, dtheta):
        cos_t, sin_t = math.cos(-dtheta), math.sin(-dtheta)
        x1 = feat["x1"] - dx; y1 = feat["y1"] - dy
        x2 = feat["x2"] - dx; y2 = feat["y2"] - dy
        x1r, y1r = _rotate_point(x1, y1, cos_t, sin_t)
        x2r, y2r = _rotate_point(x2, y2, cos_t, sin_t)
        return _line_feat(_wrap_line_angle(feat["angle"] - dtheta), 0.0,
                           x1r, y1r, x2r, y2r)

    map_e = [
        _ME(angle=0.0, distance=1.0, mx=0.0, my=1.0, status=1, active=True),
        _ME(angle=math.pi/2, distance=2.0, mx=2.0, my=0.0, status=1, active=True),
        _ME(angle=-math.pi/3, distance=1.4, mx=-0.6, my=0.8, status=1, active=True),
    ]
    base_scan = [
        _line_feat(0.0, 1.0, -0.5, 1.0, 0.5, 1.0),
        _line_feat(math.pi/2, 2.0, 2.0, -0.5, 2.0, 0.5),
        _line_feat(-math.pi/3, 1.4, -1.0, 0.2, -0.3, 1.0),
    ]
    moved_scan = [_shift(f, true_dx, true_dy, true_dtheta) for f in base_scan]

    result = search(moved_scan, map_e, guess_x=0.0, guess_y=0.0, guess_theta=0.0)
    assert result.valid, f"T1 expected valid result, got {result}"
    assert abs(result.dx - true_dx) < WALK_DXY_STEP_M, f"T1 dx off: {result}"
    assert abs(result.dy - true_dy) < WALK_DXY_STEP_M, f"T1 dy off: {result}"
    assert abs(result.dtheta - true_dtheta) < WALK_DTHETA_STEP_RAD, f"T1 dtheta off: {result}"
    print(f"  T1 PASS  recovered pose within refined resolution "
          f"(dx={result.dx:.4f} dy={result.dy:.4f} dtheta={math.degrees(result.dtheta):.2f}deg) "
          f"ambiguous={result.ambiguous}")

    # ── T2: too few STATIC entries -> invalid, no crash ────────────────────
    sparse_map = map_e[:2]  # only 2 STATIC entries < MIN_STATIC_FEATURES
    result2 = search(base_scan, sparse_map, 0.0, 0.0, 0.0)
    assert result2.valid is False, "T2 expected invalid (too few STATIC anchors)"
    assert result2.dx == 0.0 and result2.dy == 0.0 and result2.dtheta == 0.0
    print(f"  T2 PASS  too few STATIC anchors -> valid=False, zero delta")

    # ── T3: DYNAMIC/UNCLASSIFIED entries must not influence the search ─────
    map_with_mover = list(map_e) + [
        _ME(angle=0.3, distance=5.0, mx=5.0, my=5.0, status=2, active=True),  # DYNAMIC, far off
        _ME(angle=-0.9, distance=-3.0, mx=-3.0, my=3.0, status=0, active=True),  # UNCLASSIFIED
    ]
    result3 = search(moved_scan, map_with_mover, guess_x=0.0, guess_y=0.0, guess_theta=0.0)
    assert result3.valid, f"T3 expected valid result, got {result3}"
    assert abs(result3.dx - true_dx) < WALK_DXY_STEP_M
    assert abs(result3.dy - true_dy) < WALK_DXY_STEP_M
    assert result3.n_static == 3, f"T3 expected n_static=3 (movers excluded), got {result3.n_static}"
    print(f"  T3 PASS  DYNAMIC/UNCLASSIFIED entries excluded from search: {result3}")

    # ── T4: motion far outside the total reach cap -> low score -> invalid ─
    far_scan = [_shift(f, 1.0, 1.0, 0.0) for f in base_scan]  # 1m, way outside grid
    result4 = search(far_scan, map_e, guess_x=0.0, guess_y=0.0, guess_theta=0.0)
    assert result4.valid is False, f"T4 expected invalid (out of search window): {result4}"
    print(f"  T4 PASS  out-of-window motion rejected by score gate: score={result4.score:.3f}")

    # ── T5: PARALLEL-WALL TRANSLATION AMBIGUITY -- the same real, well-
    #         documented degeneracy pose_estimator.py's own module
    #         docstring describes (two facing walls constrain translation
    #         PERPENDICULAR to them but leave translation PARALLEL to them
    #         totally unconstrained). Two candidate x-offsets on either
    #         side of the true position score almost identically because
    #         nothing in the scan disambiguates them. ambiguous MUST fire.
    #         (A 90-degree-rotational-symmetry case is a real ambiguity
    #         too, but at typical separations it sits OUTSIDE this
    #         search's own coarse window and so isn't a same-scan
    #         ambiguity this function could ever be expected to catch in
    #         one shot -- that class of drift is what the cross-scan
    #         trend/probation guard in slam.py exists for instead.)
    # Three walls, all STILL parallel (all angle=0) so x stays completely
    # unconstrained -- three instead of two only to clear
    # MIN_STATIC_FEATURES, not to break the ambiguity being tested.
    map_parallel = [
        _ME(angle=0.0, distance=1.0, mx=0.0, my=1.0, status=1, active=True),
        _ME(angle=0.0, distance=-1.0, mx=0.0, my=-1.0, status=1, active=True),
        _ME(angle=0.0, distance=2.0, mx=0.0, my=2.0, status=1, active=True),
    ]
    scan_parallel = [
        _line_feat(0.0, 1.0, -0.8, 1.0, 0.8, 1.0),
        _line_feat(0.0, -1.0, -0.8, -1.0, 0.8, -1.0),
        _line_feat(0.0, 2.0, -0.8, 2.0, 0.8, 2.0),
    ]
    result5 = search(scan_parallel, map_parallel, guess_x=0.0, guess_y=0.0, guess_theta=0.0)
    assert result5.valid, f"T5 expected valid result, got {result5}"
    assert result5.ambiguous, (
        f"T5 expected parallel-wall translation ambiguity to be flagged "
        f"(no scan feature constrains x, so many x-offsets should tie for "
        f"best), got best={result5.score:.3f} second={result5.second_score:.3f}"
    )
    print(f"  T5 PASS  parallel-wall translation ambiguity correctly flagged "
          f"(best={result5.score:.3f} second={result5.second_score:.3f})")

    # ── T6: same walls, but one arc anchors x -- ambiguity resolved ────────
    # Same fix pose_estimator.py's own T7 uses (an arc centre constraint is
    # never direction-degenerate) -- here it should also collapse the
    # NUMBER of near-tied candidates correlative_match finds, not just fix
    # the later least-squares solve.
    map_asym = list(map_parallel) + [
        _ME(angle=-10.0, distance=0.3, mx=0.5, my=0.5, status=1, active=True),
    ]
    scan_asym = list(scan_parallel) + [
        {"type": "arc", "cx": 0.5, "cy": 0.5, "r": 0.3,
         "length": 0.3 * math.pi, "quality": 100},
    ]
    result6 = search(scan_asym, map_asym, guess_x=0.0, guess_y=0.0, guess_theta=0.0)
    assert result6.valid, f"T6 expected valid result, got {result6}"
    assert not result6.ambiguous, (
        f"T6 expected the arc to anchor x and resolve the ambiguity, "
        f"got best={result6.score:.3f} second={result6.second_score:.3f}"
    )
    print(f"  T6 PASS  arc anchors translation -- ambiguity resolved "
          f"(best={result6.score:.3f} second={result6.second_score:.3f})")

    # ── T7: a fast combined motion beyond the LAYER-1 window, but within
    #         the layer-2 total reach cap, must still be recovered -- this
    #         is the whole point of the greedy-walk "reach" extension. ────
    true_dx7, true_dy7, true_dtheta7 = 0.22, 0.05, math.radians(25.0)
    assert math.hypot(true_dx7, true_dy7) > SEARCH_DXY_MAX_M or abs(true_dtheta7) > SEARCH_DTHETA_MAX_RAD, \
        "T7 setup: motion must exceed layer-1's own window to test layer-2 reach"
    assert math.hypot(true_dx7, true_dy7) <= MAX_WALK_DXY_M and abs(true_dtheta7) <= MAX_WALK_DTHETA_RAD, \
        "T7 setup: motion must still be within layer-2's total reach cap"

    map_e7 = [
        _ME(angle=0.0, distance=1.0, mx=0.0, my=1.0, status=1, active=True),
        _ME(angle=math.pi/2, distance=2.0, mx=2.0, my=0.0, status=1, active=True),
        _ME(angle=-math.pi/3, distance=1.4, mx=-0.6, my=0.8, status=1, active=True),
        _ME(angle=-10.0, distance=0.4, mx=0.3, my=1.6, status=1, active=True),
    ]
    scan_e7_base = base_scan + [
        {"type": "arc", "cx": 0.3, "cy": 1.6, "r": 0.4, "length": 0.4 * math.pi, "quality": 100},
    ]
    fast_scan7 = [_shift(f, true_dx7, true_dy7, true_dtheta7) if f["type"] == "line" else f
                  for f in scan_e7_base]
    # also shift the arc centre for consistency
    cos_t7, sin_t7 = math.cos(-true_dtheta7), math.sin(-true_dtheta7)
    for f in fast_scan7:
        if f["type"] == "arc":
            cx = f["cx"] - true_dx7; cy = f["cy"] - true_dy7
            f["cx"], f["cy"] = _rotate_point(cx, cy, cos_t7, sin_t7)

    result7 = search(fast_scan7, map_e7, guess_x=0.0, guess_y=0.0, guess_theta=0.0)
    assert result7.valid, f"T7 expected valid result (reach cap should cover this motion), got {result7}"
    assert abs(result7.dx - true_dx7) < 0.04, f"T7 dx off: {result7}"
    assert abs(result7.dy - true_dy7) < 0.04, f"T7 dy off: {result7}"
    assert abs(result7.dtheta - true_dtheta7) < math.radians(4.0), f"T7 dtheta off: {result7}"
    print(f"  T7 PASS  fast motion beyond layer-1's own window recovered via "
          f"layer-2 reach (dx={result7.dx:.3f} dy={result7.dy:.3f} "
          f"dtheta={math.degrees(result7.dtheta):.1f}deg)")

    # ── T8: test_vectorized_matches_scalar -- direct numeric cross-check
    #         between _score_grid_vectorized and the untouched scalar
    #         reference (_score_candidate), at a NONZERO guess_x/guess_y.
    #         See _score_grid_vectorized's docstring: an earlier rewrite of
    #         the pyramid version of this module dropped guess_x/guess_y
    #         from an equivalent vectorized path and passed every existing
    #         test, because every existing test used guess=(0,0) -- a
    #         dropped offset is invisible there by construction. This test
    #         is written specifically so a repeat of that exact mistake
    #         CANNOT pass silently. ─────────────────────────────────────
    rng_map = [
        _ME(angle=0.1, distance=1.2, mx=0.3, my=1.1, status=1, active=True),
        _ME(angle=math.pi / 2 - 0.05, distance=1.8, mx=1.9, my=0.2, status=1, active=True),
        _ME(angle=-math.pi / 3, distance=0.9, mx=0.6, my=0.3, status=1, active=True),
        _ME(angle=-10.0, distance=0.35, mx=0.7, my=0.6, status=1, active=True),  # ARC
    ]
    rng_scan = [
        _line_feat(0.08, 1.15, -0.5, 1.1, 0.5, 1.2),
        _line_feat(math.pi / 2 - 0.02, 1.75, 1.8, -0.3, 2.0, 0.6),
        _line_feat(-math.pi / 3 + 0.03, 0.92, 0.3, -0.1, 0.9, 0.6),
        {"type": "arc", "cx": 0.68, "cy": 0.58, "r": 0.35,
         "length": 0.35 * math.pi, "quality": 100},
    ]
    static_lines_t8 = [e for e in rng_map if not e.is_arc()]
    static_arcs_t8  = [e for e in rng_map if e.is_arc()]

    # Deliberately NONZERO guess -- this is the one condition that made
    # the historical bug invisible. dx_grid/dy_grid built exactly the way
    # _coarse_sweep_topk / _greedy_refine build them: guess + offsets.
    guess_x_t8, guess_y_t8, guess_theta_t8 = 1.35, -0.72, math.radians(9.0)
    dtheta_probe = math.radians(3.0)
    cos_t8 = math.cos(guess_theta_t8 + dtheta_probe)
    sin_t8 = math.sin(guess_theta_t8 + dtheta_probe)
    rotated_lines_t8, rotated_arcs_t8 = _prerotate_features(
        rng_scan, cos_t8, sin_t8, guess_theta_t8 + dtheta_probe
    )

    dx_offsets_t8 = np.array([-0.06, -0.03, 0.0, 0.03, 0.06])
    dy_offsets_t8 = np.array([-0.06, -0.03, 0.0, 0.03, 0.06])
    dx_grid_t8 = guess_x_t8 + dx_offsets_t8
    dy_grid_t8 = guess_y_t8 + dy_offsets_t8

    se_angle_t8, se_dist_t8 = _static_line_arrays(static_lines_t8)
    sa_mx_t8, sa_my_t8, sa_r_t8 = _static_arc_arrays(static_arcs_t8)
    total_grid, n_scored_grid = _score_grid_vectorized(
        rotated_lines_t8, rotated_arcs_t8, dx_grid_t8, dy_grid_t8,
        se_angle_t8, se_dist_t8, sa_mx_t8, sa_my_t8, sa_r_t8
    )

    max_abs_diff = 0.0
    for ix, dxo in enumerate(dx_offsets_t8):
        for iy, dyo in enumerate(dy_offsets_t8):
            scalar_total, scalar_n = _score_candidate(
                rotated_lines_t8, rotated_arcs_t8,
                guess_x_t8 + float(dxo), guess_y_t8 + float(dyo),
                static_lines_t8, static_arcs_t8,
            )
            diff = abs(scalar_total - float(total_grid[ix, iy]))
            max_abs_diff = max(max_abs_diff, diff)
            assert scalar_n == n_scored_grid, \
                f"T8 n_scored mismatch: scalar={scalar_n} vectorized={n_scored_grid}"

    assert max_abs_diff < 1e-9, \
        f"T8 vectorized scoring must match the scalar reference exactly " \
        f"(max abs diff={max_abs_diff}) -- if this fails, check that " \
        f"guess_x/guess_y is being folded into dx_grid/dy_grid BEFORE " \
        f"calling _score_grid_vectorized, not passed separately or dropped"
    print(f"  T8 PASS  test_vectorized_matches_scalar: vectorized grid scoring "
          f"matches the scalar reference exactly (max abs diff={max_abs_diff:.2e}) "
          f"at a NONZERO guess (x={guess_x_t8}, y={guess_y_t8}, "
          f"theta={math.degrees(guess_theta_t8):.1f}deg)")

    # ── T9: END-TO-END nonzero-guess regression -- catches the bug even if
    #         some future refactor moved WHERE the guess offset gets added
    #         (T8 only checks the scoring function in isolation). Same
    #         small-motion recovery as T1, just centred on a pose far from
    #         the map origin instead of at it -- this is exactly the
    #         real-hardware condition ("almost always, once the robot has
    #         moved") that let the historical bug through undetected. ────
    map_e9 = [
        _ME(angle=0.0, distance=1.0, mx=0.0, my=1.0, status=1, active=True),
        _ME(angle=math.pi / 2, distance=2.0, mx=2.0, my=0.0, status=1, active=True),
        _ME(angle=-math.pi / 3, distance=1.4, mx=1.212, my=0.7, status=1, active=True),
    ]
    base_scan9 = [
        _line_feat(0.0, 1.0, -0.5, 1.0, 0.5, 1.0),
        _line_feat(math.pi / 2, 2.0, 2.0, -0.5, 2.0, 0.5),
        _line_feat(-math.pi / 3, 1.4, 0.962, 1.133, 1.462, 0.267),
    ]
    true_dx9, true_dy9, true_dtheta9 = 0.06, -0.04, math.radians(5.0)
    moved_scan9 = [_shift(f, true_dx9, true_dy9, true_dtheta9) for f in base_scan9]

    # Guess is close to (not equal to) the true correction, mirroring how
    # search() is actually called mid-run (working_pose already close from
    # a previous iteration) -- and, critically, NONZERO, which is the one
    # condition needed to exercise the dropped-offset bug class. It must
    # NOT be so far from the true pose that the residual exceeds this
    # module's own SEARCH_DXY_MAX_M/SEARCH_DTHETA_MAX_RAD window, or the
    # test would be checking window coverage instead of offset correctness.
    guess_x9, guess_y9, guess_theta9 = 0.05, -0.03, math.radians(4.0)
    result9 = search(moved_scan9, map_e9,
                      guess_x=guess_x9, guess_y=guess_y9, guess_theta=guess_theta9)
    assert result9.valid, f"T9 expected valid result at nonzero guess, got {result9}"
    assert abs((guess_x9 + result9.dx) - true_dx9) < 0.03, \
        f"T9 dx off at nonzero guess (regression check for the dropped-offset " \
        f"bug class): {result9}"
    assert abs((guess_y9 + result9.dy) - true_dy9) < 0.03, \
        f"T9 dy off at nonzero guess: {result9}"
    assert abs((guess_theta9 + result9.dtheta) - true_dtheta9) < math.radians(3.0), \
        f"T9 dtheta off at nonzero guess: {result9}"
    print(f"  T9 PASS  end-to-end recovery correct at a NONZERO guess "
          f"(guess=({guess_x9},{guess_y9},{math.degrees(guess_theta9):.1f}deg) "
          f"+ delta -> absolute ({guess_x9+result9.dx:.4f},"
          f"{guess_y9+result9.dy:.4f},"
          f"{math.degrees(guess_theta9+result9.dtheta):.2f}deg) "
          f"vs true ({true_dx9},{true_dy9},{math.degrees(true_dtheta9):.1f}deg))")

    print()
    print("All tests passed.")
    sys.exit(0)