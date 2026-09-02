"""
correlative_match.py
=====================
Multi-resolution correlative (search-and-score) coarse pose estimation over
LINE and ARC features. Runs BEFORE line_matcher's hard-correspondence
matching, to solve the chicken-and-egg problem documented in slam.py /
pose_estimator.py: hard correspondence (line_matcher) can only be trusted
once the pose guess is already close; but the pose guess only gets better
by trusting some correspondence. Correlative search breaks that loop by
never committing to a single correspondence at all -- it scores many
candidate poses against the WHOLE feature set at once (smooth, soft
scoring) and returns whichever candidate pose makes the scan agree best
with the map, globally.

This is the feature-space analogue of Hector/Cartographer's real-time
correlative scan matcher, scored on compact Hough (line) and centre+radius
(arc) parameters instead of a stored occupancy grid or point cloud --
zero extra RAM, no grid, plain scalar arithmetic only.

GUESS TRANSLATION BUG (FIXED) -- READ THIS IF TOUCHING SCORING CODE
------------------------------------------------------------------------
Every candidate pose search() scores is (guess_x + dx, guess_y + dy,
guess_theta + dtheta) -- search()'s own docstring says so explicitly. But
the line-scoring code in _score_grid_vectorized (and the dead scalar
reference path, _score_candidate) computed each candidate line's Hough
distance as `base_dist + nx*dx + ny*dy` -- guess_x/guess_y were NEVER
added in. Same omission for arc centres (`rcx + dx` instead of
`rcx + guess_x + dx`). This is silently correct only when guess_x==
guess_y==0.0 -- exactly the only case every self-test in this file
exercised (T1-T7 below all call search() with guess_x=0.0, guess_y=0.0),
which is why nothing caught it.

CONSEQUENCE ON HARDWARE: as soon as the robot's real pose moved away from
the map origin, every candidate's line/arc distance was scored against
the wrong absolute reference frame by an amount equal to however far
guess_x/guess_y actually were from zero. Once that error approached
SIGMA_DIST_M/SIGMA_CENTRE_M (0.15m), scores collapsed map-wide
(final_weight/final_eig -> ~0, "poor_conditioning" breaks) -- and because
the missing offset is roughly CONSTANT for a given guess, the search
could also lock onto a candidate (dx, dy, dtheta) that happened to
partially cancel it out: a confidently wrong answer, not just a
low-confidence one (large coarse_valid=True rotation swings of 30-70
degrees observed on real logs, correlated with pose.x/pose.y being
non-trivially far from the map origin, not with how large the robot's
actual per-scan motion was). Fixed by adding guess_x/guess_y into the
distance/centre calculations in both _score_grid_vectorized and
_score_candidate; see T8/T9 below for regression coverage with a nonzero
guess, which the original test suite never had.

MULTI-RESOLUTION PYRAMID -- WHY THIS EXISTS
------------------------------------------------------------------------
hector_slam's real ScanMatcher (tu-darmstadt-ros-pkg/hector_slam,
hector_mapping/include/hector_slam_lib/matcher/ScanMatcher.h +
OccGridMapUtil.h) gets its robustness from two things this module cannot
literally copy (see PORT PATH / WHY NOT A REAL GRID below), but CAN copy
the *principle* of: (1) matching against a smooth, continuous cost surface
instead of committing to discrete correspondences, and (2) searching that
surface at MULTIPLE RESOLUTIONS -- coarse grid levels first, each one
refining the next -- rather than a single flat sweep. (1) is already
covered by this module's Gaussian soft-scoring (see WHY SOFT SCORING
below); this revision adds (2).

The previous design here was a two-layer scheme: one coarse exhaustive
sweep (keeping the top-K distinct peaks), then an independent GREEDY
pattern-search walk per surviving candidate to reach further / refine
finer. That greedy walk had two structural weaknesses a true pyramid does
not have:
  - It could get stuck in a local uphill direction and stop before finding
    a better nearby point one step further out (greedy hill-climbing has
    no guarantee of finding the local optimum within its window, unlike
    an exhaustive sweep of that window).
  - Its "reach" (how far it could travel from its coarse starting point)
    was governed by a SEPARATE set of constants (MAX_WALK_DXY_M /
    MAX_WALK_DTHETA_RAD) from the coarse sweep's own window
    (SEARCH_DXY_MAX_M / SEARCH_DTHETA_MAX_RAD) -- two different levers
    controlling what is conceptually one thing (how far the pose can be
    corrected this scan), which made total reach harder to reason about
    and tune, and needed its own dedicated self-test to cover the
    "coarse window too small, greedy reach saves it" case specifically.

FIX -- true coarse-to-fine PYRAMID (see LEVELS below): the FIRST level
exhaustively sweeps the full SEARCH_DTHETA_MAX_RAD / SEARCH_DXY_MAX_M
window at a coarse step, keeping the top-K distinct peaks (unchanged from
before -- this is what catches "there's a second, nearly-as-good peak"
that a flat argmax sweep cannot report). EVERY SUBSEQUENT level then
exhaustively re-sweeps a NARROWER window, at a FINER step, RE-CENTRED on
each surviving candidate from the level above -- not a greedy walk, a full
grid sweep of that (now much smaller) window, so nothing is missed within
it. Each level's window is chosen to comfortably exceed half the previous
level's step size, guaranteeing no true peak can fall in the gap between
coarse grid samples and land outside the next level's refinement window
(see LEVELS' inline comments for the actual margins used). Total reach
from the original guess is now governed by ONE number -- level 0's own
window -- since every later level only refines a point level 0 already
found, never travels further from the original guess than level 0's own
range plus a small quantization margin. This removes the old two-lever
reach split entirely.

WHY NOT A REAL GRID (staying feature-based, not switching to occupancy
grid SLAM)
------------------------------------------------------------------------
hector's actual precision and robustness come from matching every scan
POINT (typically hundreds) against a dense, bilinearly-interpolated
occupancy grid with true multi-resolution grid levels, solved via Eigen.
slam_build_prompt.md rejected exactly that architecture for this project
on Day 0 -- Hector SLAM, SLAM Toolbox, and Cartographer were all named as
"too heavy for 512MB RISC-V" specifically because of the occupancy-grid
RAM cost (a 400x400 cell grid at 5cm resolution is 160KB per level times
however many resolution levels; a vector/line-feature map is ~5KB). Line
and arc features were chosen specifically to avoid that cost, at the
acknowledged expense of matching against a much SPARSER, noisier signal
(a handful of STATIC map entries per scan, not hundreds of points). This
module borrows hector's coarse-to-fine SEARCH STRATEGY -- which is free,
it costs zero extra RAM regardless of how many levels are used, since
every level scores against the same small STATIC line/arc list -- without
reopening the RAM-driven decision to go feature-based in the first place.

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
map evidence (see slam.py's delta_fully_trusted gating). Ambiguity is
checked using AMBIGUITY_MIN_SEP_DTHETA_RAD / AMBIGUITY_MIN_SEP_DXY_M -- a
separation scale tied to the score surface's own physical breadth
(SIGMA_ANGLE_RAD / SIGMA_DIST_M), NOT to the finest pyramid level's grid
step. Two refined candidates that both sit on the same broad score-surface
hilltop, a few cm/degrees apart, are convergence to one peak, not two
competing peaks -- see AMBIGUITY_MIN_SEP_DTHETA_RAD's docstring for the
false-positive this distinction fixes.

PERFORMANCE
-----------
Every level's cost follows the same pattern as before: for a fixed
candidate dtheta, every scan feature is rotated exactly ONCE (cos/sin
computed once per dtheta step), then the dx,dy sub-sweep at that dtheta is
pure O(1) scalar arithmetic per feature per static map entry. Level 0
evaluates the same number of candidates as the old single coarse sweep
did. Each subsequent level evaluates a NARROWER window at a FINER step,
once per surviving candidate from the level above (bounded by
TOP_K_CANDIDATES) -- total candidates across all levels is a small
constant multiple of level 0's own cost, not an explosion, since window
size shrinks roughly as fast as step count grows per level (see LEVELS'
inline sizing comments). Exact costs are tunable via LEVELS below; profile
on-target (Duo S) before finalising them, same as every other tuning
constant in this codebase.

PORT PATH
---------
When porting to C (slam_core/correlative_match.c):
    search()  -> correlative_search(const ScanFeature *scan, int n_scan,
                                     const MapEntry *map, int n_map,
                                     float guess_x, float guess_y,
                                     float guess_theta,
                                     CoarseResult *out)
    LEVELS below become a fixed-size array of #defined struct literals
    (N_LEVELS is small and known at compile time -- 3 in this revision).
    _sweep_and_insert()    -> a single fixed-bound triple for-loop
        (dtheta, dx, dy) reused for EVERY level via an outer
        `for (level = 0; level < N_LEVELS; level++)` loop -- there is only
        ONE sweep routine now (the old design needed two: one for the
        coarse exhaustive sweep, one for the greedy walk). This is
        actually a SIMPLER C port than the design it replaces.
    _insert_topk_distinct() -> fixed-size top-K array (K small, e.g. 3),
        insertion sort in place -- no dynamic allocation, same pattern
        map_manager.c's fixed MAX_MAP_ENTRIES array already uses.
    No dynamic allocation anywhere -- all loops are fixed-bound; the
    per-dtheta rotated-feature cache is a fixed-size MAX_FEATURES-length
    scratch array reused every dtheta step, exactly as before.
"""

import math
from collections import namedtuple
import numpy as np

# ---------------------------------------------------------------------------
# Search window -- level 0's own range IS the total reach of this module.
# Every later pyramid level only refines a point level 0 already found, so
# nothing can end up further from the original guess than this (plus a
# small quantization margin -- see LEVELS' inline comments). This replaces
# the old design's SEPARATE greedy-walk reach cap (MAX_WALK_DXY_M /
# MAX_WALK_DTHETA_RAD) -- there is now exactly one number that controls how
# far this module can correct the pose in one scan, not two.
# ---------------------------------------------------------------------------

SEARCH_DTHETA_MAX_RAD = math.radians(80.0)   # total reach: rotation
SEARCH_DXY_MAX_M      = 0.30               # total reach: translation

TOP_K_CANDIDATES = 3   # how many DISTINCT peaks survive at EVERY level,
                        # including the finest. K=1 recovers a plain
                        # single-winner coarse-to-fine search (no ambiguity
                        # detection). Cost scales linearly with K at every
                        # level after the first, since each level refines
                        # K survivors from the level above.

# ---------------------------------------------------------------------------
# Multi-resolution pyramid -- each level narrows the window and sharpens
# the step by roughly the same factor (~5x) going into the next level.
# SIZING RULE (why nothing gets lost between levels): a level's `dtheta_range`
# / `dxy_range` must be >= half of the PREVIOUS level's step -- that half-step
# is the worst-case distance between a true peak and the nearest coarse grid
# sample that found it, so the next level's window must reach at least that
# far past its own centre to be guaranteed to still contain the true peak.
# Every level below keeps comfortable margin above that minimum.
# ---------------------------------------------------------------------------

_Level = namedtuple(
    "_Level",
    ["dtheta_step", "dxy_step", "dtheta_range", "dxy_range",
     "min_sep_dtheta", "min_sep_dxy"]
)
# dtheta_step, dxy_step   : grid resolution swept AT this level
# dtheta_range, dxy_range : +/- window swept at this level, RE-CENTRED on
#                           each surviving candidate from the level above
#                           (level 0 is centred on the original guess, i.e.
#                           offset (0,0,0))
# min_sep_dtheta/dxy      : how close two candidates must be to count as
#                           the SAME peak at this level (see
#                           _insert_topk_distinct) -- shrinks at finer
#                           levels since the window itself has shrunk;
#                           using a coarse-level separation at a fine
#                           level would wrongly merge genuinely distinct
#                           nearby peaks the finer resolution can now tell
#                           apart.

LEVELS = (
    # Level 0 -- coarse, full reach. Same step/window/separation as the
    # original single-layer design, so a real motion this module already
    # handled well continues to be found the same way.
    _Level(dtheta_step=math.radians(10.0), dxy_step=0.05,
           dtheta_range=SEARCH_DTHETA_MAX_RAD, dxy_range=SEARCH_DXY_MAX_M,
           min_sep_dtheta=math.radians(20.0), min_sep_dxy=0.15),
    # Level 1 -- refine. Window (+/-12deg, +/-6cm) comfortably exceeds half
    # of level 0's step (+/-5deg, +/-2.5cm), so no level-0 peak can have
    # landed outside this window relative to its own coarse grid sample.
    _Level(dtheta_step=math.radians(2.0), dxy_step=0.015,
           dtheta_range=math.radians(12.0), dxy_range=0.06,
           min_sep_dtheta=math.radians(6.0), min_sep_dxy=0.05),
    # Level 2 -- fine. Window (+/-2.5deg, +/-1.5cm) comfortably exceeds
    # half of level 1's step (+/-1deg, +/-0.75cm). Final achievable
    # resolution: 0.4deg / 0.4cm -- an order of magnitude finer than the
    # old greedy walk's finest step (3deg / 3cm), with no risk of a
    # greedy hill-climb stalling short of the true local peak, since this
    # is an exhaustive sweep of the window, not a hill-climb.
    _Level(dtheta_step=math.radians(0.4), dxy_step=0.004,
           dtheta_range=math.radians(2.5), dxy_range=0.015,
           min_sep_dtheta=math.radians(1.2), min_sep_dxy=0.01),
)

# ---------------------------------------------------------------------------
# Ambiguity detection
# ---------------------------------------------------------------------------

AMBIGUITY_MARGIN_RATIO = 0.10
# If the second-best FINAL (finest-level) candidate scores within this
# fraction of the best candidate's score, the result is flagged
# ambiguous=True. Starting value -- tune tighter/looser once real logged
# (score, second_score) pairs from a known-good run are available, same
# spirit as every other "deliberately generous starting value" constant
# elsewhere in this codebase (e.g. pose_estimator.MAX_ACCEPTABLE_RESIDUAL).

AMBIGUITY_MIN_SEP_DTHETA_RAD = math.radians(20.0)
AMBIGUITY_MIN_SEP_DXY_M = 0.15
# Separation used ONLY for the final post-refinement "is this second
# candidate a genuinely different peak" check in search() -- deliberately
# NOT the same value as any single level's own min_sep_dtheta/min_sep_dxy
# (see LEVELS above), which exist purely to de-duplicate near-identical
# GRID samples during that level's own sweep and shrink at finer levels
# along with the grid step.
#
# THE BUG THIS FIXES: an earlier version of this ambiguity check reused
# LEVELS[-1] (the finest level)'s own min_sep, which is tiny (~1deg/1cm)
# because it only needs to be big enough to de-duplicate adjacent
# 0.4deg/0.4cm grid samples. But the actual SCORE SURFACE (Gaussian
# falloff with SIGMA_ANGLE_RAD/SIGMA_DIST_M ~ 0.15 rad / 0.15 m) is far
# BROADER than that -- two samples several cm/degrees apart on the same
# true peak's flat hilltop can score within a fraction of a percent of
# each other, which is expected and correct (it is one basin, sampled at
# two nearby points), not evidence of two competing physical peaks. Using
# the finest level's tiny grid-dedup separation to judge "distinctness"
# here produced false ambiguity flags on ordinary, well-determined,
# non-degenerate scans (caught by this module's own self-test: a 3-line,
# non-parallel, single-true-peak scenario was incorrectly flagged
# ambiguous because two samples ~1.5deg/1.5cm apart on the SAME peak's
# broad hilltop both scored ~3.0). Ambiguity is a claim about PHYSICAL
# space (a second, meaningfully different candidate pose), so it must be
# judged at a separation scale tied to the score surface's own physical
# breadth (SIGMA_*), not to whatever grid step the finest pyramid level
# happens to use.

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
# PERFORMANCE CAP (Option 1) -- search() cost scales with n_static_entries
# (every candidate pose scores against every STATIC line/arc, see
# _score_candidate_grid). Left unbounded, this grows with the map forever
# (confirmed directly: lowering MIN_OBS_FOR_STATIC in map_manager.py sped up
# STATIC promotion and immediately produced a ~15-20x per-scan slowdown once
# the map reached ~30 STATIC entries, measured via consecutive Scan# refine:
# log timestamps going from ~0.14s apart to ~2s apart). A STATIC entry far
# from the current pose guess contributes almost nothing to the Gaussian
# soft-score anyway (SIGMA_DIST_M/SIGMA_CENTRE_M ~ 0.15m falloff), so capping
# to the nearest MAX_STATIC_ENTRIES_SEARCHED entries loses negligible real
# signal for what is, physically, always a local search.
# ---------------------------------------------------------------------------

MAX_STATIC_ENTRIES_SEARCHED = 40   # generous starting cap -- tune down only
                                    # after confirming search quality (T1-T7
                                    # below) is unaffected at your real map
                                    # density.

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

    PERFORMANCE CAP (Option 1 -- see MAX_STATIC_ENTRIES_SEARCHED above): when
    guess_x/guess_y are given and the combined STATIC entry count exceeds
    max_entries, only the max_entries entries nearest (by midpoint/centre
    distance) to the current pose guess are kept. This is what keeps
    search()'s per-scan cost bounded as the map grows, instead of degrading
    linearly with total map size forever.

    guess_x/guess_y default to None (no cap applied) so existing callers
    that don't pass a guess (e.g. direct unit tests) are unaffected.
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

    NOTE: base_dist here is the Hough distance of the ROTATED-ONLY scan
    midpoint -- it deliberately does NOT include guess_x/guess_y (those
    aren't known at rotation time in the original dtheta-outer-loop
    structure this was written for). _score_candidate below is responsible
    for adding guess_x/guess_y in on top of (dx, dy) -- see the "GUESS
    TRANSLATION BUG" fix there; this function itself does not need to
    change, only its caller's contract needs to be honoured correctly.
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


def _score_candidate(rotated_lines, rotated_arcs, dx, dy, guess_x, guess_y,
                      static_lines, static_arcs):
    """
    SCALAR reference implementation -- kept for cross-checking the
    vectorized grid path below (see test_vectorized_matches_scalar) and as
    the direct 1:1 C port source (see PORT PATH in the module docstring).
    Not used by search() itself anymore -- _score_grid_vectorized replaces
    it in the hot path. Score one candidate (dx, dy) at the dtheta already
    baked into rotated_lines/rotated_arcs. Returns (total_score,
    n_features_scored). Pure O(1)-per-feature-per-map-entry arithmetic --
    no trig here.

    GUESS TRANSLATION BUG (fixed): the candidate pose being scored is
    (guess_x + dx, guess_y + dy, guess_theta + dtheta) -- see search()'s
    own docstring. rotated_lines/rotated_arcs only carry the ROTATED scan
    geometry (no absolute translation baked in -- see _prerotate_features),
    so guess_x/guess_y MUST be added here, on top of dx/dy, before
    comparing against the map's absolute coordinates. The previous version
    of this function took only (dx, dy) and silently scored every
    candidate as if guess_x/guess_y were always zero -- correct only at
    the map origin, silently wrong (and, worse, sometimes confidently
    wrong -- see module docstring's GUESS TRANSLATION BUG note) everywhere
    else. Every self-test in this file called search() with
    guess_x=guess_y=0.0, so this was invisible until real hardware moved
    the robot away from the origin.
    """
    total = 0.0
    n_scored = 0

    for (rmx, rmy, angle, nx, ny, base_dist) in rotated_lines:
        dist = base_dist + nx * (guess_x + dx) + ny * (guess_y + dy)
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
        cx = rcx + guess_x + dx
        cy = rcy + guess_y + dy
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


# ---------------------------------------------------------------------------
# Vectorized grid scoring (Option 3) -- numpy replacement for the dx,dy
# sub-sweep inside _sweep_and_insert. Same math as _score_candidate /
# _prerotate_features, computed for the WHOLE dx,dy grid (at one dtheta) in
# one batch of array operations instead of one Python-level function call
# per (dx, dy) pair. This is a PC-visualization/prototyping speed change
# only -- see module docstring's PORT PATH: the scalar functions above stay
# the source of truth for the eventual hand-written C port, this is not a
# drop-in translation of this file's structure.
# ---------------------------------------------------------------------------

def _static_line_arrays(static_lines):
    if not static_lines:
        return np.empty(0), np.empty(0)
    return (np.array([e.angle for e in static_lines], dtype=np.float64),
            np.array([e.distance for e in static_lines], dtype=np.float64))


def _static_arc_arrays(static_arcs):
    if not static_arcs:
        return np.empty(0), np.empty(0), np.empty(0)
    return (np.array([e.mx for e in static_arcs], dtype=np.float64),
            np.array([e.my for e in static_arcs], dtype=np.float64),
            np.array([e.distance for e in static_arcs], dtype=np.float64))


def _score_grid_vectorized(scan_features, guess_x, guess_y, guess_theta, dtheta,
                            dx_grid, dy_grid,
                            se_angle, se_dist, sa_mx, sa_my, sa_r):
    """
    Score every (dx, dy) combination in dx_grid x dy_grid, at one fixed
    dtheta, against all given STATIC line/arc arrays at once.

    Returns (total, n_scored):
        total    : np.ndarray shape (len(dx_grid), len(dy_grid)) -- summed
                   soft score across every scan feature, matching what
                   repeated _score_candidate(dx, dy) calls would produce.
        n_scored : int -- number of scan features that contributed (lines
                   + arcs), same definition _score_candidate returns.

    Each scan feature contributes a (n_dx, n_dy) array to `total` via numpy
    broadcasting against the static arrays -- the O(n_static) "best match"
    reduction (max over candidates) that _score_candidate does with a
    Python for-loop + running best_q becomes a single .max(axis=-1) call.
    """
    cos_t = math.cos(guess_theta + dtheta)
    sin_t = math.sin(guess_theta + dtheta)

    n_dx = dx_grid.shape[0]
    n_dy = dy_grid.shape[0]
    total = np.zeros((n_dx, n_dy), dtype=np.float64)
    n_scored = 0

    dxg = dx_grid[:, None]   # (n_dx, 1) -- broadcasts against dy below
    dyg = dy_grid[None, :]   # (1, n_dy)

    for feat in scan_features:
        ftype = feat.get("type")

        if ftype == "line":
            x1, y1 = _rotate_point(feat["x1"], feat["y1"], cos_t, sin_t)
            x2, y2 = _rotate_point(feat["x2"], feat["y2"], cos_t, sin_t)
            rmx = (x1 + x2) / 2.0
            rmy = (y1 + y2) / 2.0
            angle = _wrap_line_angle(feat["angle"] + guess_theta + dtheta)
            nx, ny = -math.sin(angle), math.cos(angle)
            base_dist = nx * rmx + ny * rmy
            n_scored += 1

            if se_angle.size == 0:
                continue

            # BUG FIX: every candidate pose being scored is
            # (guess_x + dx, guess_y + dy, guess_theta + dtheta) -- see
            # search()'s own docstring ("dx, dy, dtheta are corrections to
            # ADD to guess_x, guess_y, guess_theta"). base_dist above only
            # has the ROTATED scan midpoint in it; the guess's own absolute
            # translation was never added before this comparison against
            # the map's absolute Hough distance. That made every candidate's
            # score silently wrong by an amount that grows with how far the
            # robot has actually moved from the map origin -- exactly zero
            # in every existing self-test (all called with guess_x=guess_y
            # =0.0), which is why this went undetected. See the module-level
            # "GUESS TRANSLATION BUG" note for the full failure chain this
            # caused on hardware (coarse search converging on a confidently
            # wrong pose once guess_x/guess_y left the ~15cm SIGMA_DIST_M
            # neighbourhood of the origin).
            dist = base_dist + nx * (guess_x + dxg) + ny * (guess_y + dyg)   # (n_dx, n_dy)

            adiff = np.abs(angle - se_angle) % math.pi        # (n_static,)
            adiff = np.where(adiff > math.pi / 2.0, math.pi - adiff, adiff)

            d3 = dist[:, :, None]                              # (n_dx,n_dy,1)
            ddiff = np.minimum(np.abs(d3 - se_dist), np.abs(d3 + se_dist))
            q = (np.exp(-(adiff / SIGMA_ANGLE_RAD) ** 2)
                 * np.exp(-(ddiff / SIGMA_DIST_M) ** 2))       # (n_dx,n_dy,n_static)
            total += q.max(axis=2)

        elif ftype == "arc":
            rcx, rcy = _rotate_point(feat["cx"], feat["cy"], cos_t, sin_t)
            n_scored += 1

            if sa_mx.size == 0:
                continue

            # Same fix as the line case above -- the candidate arc centre
            # in the MAP frame is (guess_x + dx, guess_y + dy) applied on
            # top of the rotated scan-frame centre (rcx, rcy), not just
            # (dx, dy) alone.
            cx = rcx + guess_x + dxg                           # (n_dx, 1)
            cy = rcy + guess_y + dyg                           # (1, n_dy)
            cx3 = np.broadcast_to(cx, (n_dx, n_dy))[:, :, None]
            cy3 = np.broadcast_to(cy, (n_dx, n_dy))[:, :, None]
            centre_dist = np.sqrt((cx3 - sa_mx) ** 2 + (cy3 - sa_my) ** 2)
            rdiff = np.abs(feat["r"] - sa_r)
            q = (np.exp(-(centre_dist / SIGMA_CENTRE_M) ** 2)
                 * np.exp(-(rdiff / SIGMA_R_M) ** 2))
            total += q.max(axis=2)

    return total, n_scored


def _insert_topk_distinct(top, candidate, k, min_sep_dtheta, min_sep_dxy):
    """
    Insert `candidate` = (score, dx, dy, dtheta, n_scored) into `top`
    (mutated in place, kept sorted descending by score, capped at k
    entries) -- but ONLY as a genuinely separate peak. Two candidates
    within `min_sep_dtheta` / `min_sep_dxy` of each other are treated as
    the SAME peak (adjacent grid cells sampling the same local maximum, or
    -- at finer pyramid levels -- two coarse-level survivors that have
    refined into the same true peak), and only the higher-scoring one is
    kept.

    WHY THIS MATTERS: without this de-duplication, top-K would frequently
    fill up with 2-3 grid cells all sitting right next to the SAME true
    peak, instead of K genuinely distinct candidate poses -- silently
    defeating the entire purpose of keeping more than one candidate (which
    is to catch a competing peak elsewhere in the search space, like a
    rectangular room's +/-90 degree symmetry). The separation thresholds
    are passed in per-call (not module constants) because they shrink at
    finer pyramid levels -- see LEVELS' min_sep_dtheta/min_sep_dxy fields.
    """
    score, dx, dy, dtheta, n_scored = candidate

    for i, (s2, dx2, dy2, dtheta2, n2) in enumerate(top):
        if (abs(dtheta - dtheta2) < min_sep_dtheta
                and math.hypot(dx - dx2, dy - dy2) < min_sep_dxy):
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


def _sweep_and_insert(scan_features, guess_x, guess_y, guess_theta,
                       center_dx, center_dy, center_dtheta,
                       level, static_lines, static_arcs, top, k):
    """
    Exhaustively sample the window (level.dtheta_range, level.dxy_range)
    around (center_dtheta, center_dx, center_dy) at (level.dtheta_step,
    level.dxy_step) resolution, inserting every scorable sample into `top`
    (mutated in place) via _insert_topk_distinct using level's own
    separation thresholds.

    VECTORIZED (Option 3): the dtheta loop stays a plain Python loop (small
    -- at most a few dozen steps even at level 0), but for each dtheta the
    entire dx,dy grid is now scored in one batch via
    _score_grid_vectorized/numpy instead of one Python function call per
    (dx, dy) pair (see that function's docstring). This changes only how
    the scores are COMPUTED, not the search structure itself -- every grid
    cell that used to be individually scored and inserted still is;
    _insert_topk_distinct's de-duplication logic is untouched. See
    test_vectorized_matches_scalar() in the self-test block for a direct
    numeric cross-check against the original scalar path.
    """
    static_line_arrays = _static_line_arrays(static_lines)
    static_arc_arrays = _static_arc_arrays(static_arcs)

    n_dtheta_steps = int(round(2 * level.dtheta_range / level.dtheta_step)) + 1
    dtheta_offsets = (center_dtheta - level.dtheta_range
                       + level.dtheta_step * np.arange(n_dtheta_steps))

    n_dxy_steps = int(round(2 * level.dxy_range / level.dxy_step)) + 1
    dx_grid = center_dx - level.dxy_range + level.dxy_step * np.arange(n_dxy_steps)
    dy_grid = center_dy - level.dxy_range + level.dxy_step * np.arange(n_dxy_steps)

    for dtheta in dtheta_offsets:
        dtheta = float(dtheta)
        total, n_scored = _score_grid_vectorized(
            scan_features, guess_x, guess_y, guess_theta, dtheta,
            dx_grid, dy_grid, *static_line_arrays, *static_arc_arrays
        )
        if n_scored == 0:
            continue

        # Insert EVERY grid cell, same as the original scalar sweep --
        # correctness (in particular, not silently missing a genuine
        # second peak that shares a dtheta with the winner) matters more
        # here than shaving off insert-bookkeeping calls, and
        # _insert_topk_distinct's own work per call is O(k)=O(3), trivial
        # compared to the score computation this replaces. The speedup
        # comes entirely from _score_grid_vectorized above, not from
        # skipping any candidates.
        n_dx_local, n_dy_local = total.shape
        for ix in range(n_dx_local):
            dx = float(dx_grid[ix])
            row = total[ix]
            for iy in range(n_dy_local):
                score = float(row[iy])
                if score <= 0.0:
                    continue
                dy = float(dy_grid[iy])
                _insert_topk_distinct(
                    top, (score, dx, dy, dtheta, n_scored), k,
                    level.min_sep_dtheta, level.min_sep_dxy
                )


def search(scan_features, map_entries, guess_x, guess_y, guess_theta,
           static_status=1):
    """
    Run the multi-resolution correlative coarse pose search (see module
    docstring for the full pyramid rationale).

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
    static_lines, static_arcs = _prepare_static_maps(
        map_entries, static_status, guess_x, guess_y
    )
    n_static = len(static_lines) + len(static_arcs)

    if n_static < MIN_STATIC_FEATURES:
        return CoarseResult(dx=0.0, dy=0.0, dtheta=0.0, score=0.0,
                             n_static=n_static, valid=False,
                             ambiguous=False, second_score=0.0)

    # ---- Level 0: full-window coarse sweep, centred on the original guess
    top = []
    _sweep_and_insert(scan_features, guess_x, guess_y, guess_theta,
                       0.0, 0.0, 0.0, LEVELS[0],
                       static_lines, static_arcs, top, TOP_K_CANDIDATES)

    if not top:
        return CoarseResult(dx=0.0, dy=0.0, dtheta=0.0, score=0.0,
                             n_static=n_static, valid=False,
                             ambiguous=False, second_score=0.0)

    # ---- Levels 1..N: refine EVERY surviving candidate at progressively
    # finer resolution, each level's sweep re-centred on the candidate it
    # refines. Each survivor is refined INDEPENDENTLY into its own local
    # top-1 (not into one top-k list shared across all survivors) -- this
    # matters when candidates tie or nearly tie (e.g. a genuinely flat
    # ridge, like the parallel-wall translation ambiguity where dx is
    # completely unconstrained): sharing one top-k budget across survivors
    # let the FIRST survivor's own local neighborhood fill the entire
    # budget before the other survivors were even refined, silently
    # discarding genuinely distinct peaks the level above had already
    # found and correctly kept separate. Refining independently then
    # merging guarantees every surviving level-(L-1) candidate contributes
    # at least one representative going into the next level, unless two
    # independent survivors' refinements land on the same point (in which
    # case merging them is correct).
    for level in LEVELS[1:]:
        next_top = []
        for (_score, dx, dy, dtheta, _n) in top:
            local_top = []
            _sweep_and_insert(scan_features, guess_x, guess_y, guess_theta,
                               dx, dy, dtheta, level,
                               static_lines, static_arcs, local_top, 1)
            if local_top:
                _insert_topk_distinct(
                    next_top, local_top[0], TOP_K_CANDIDATES,
                    level.min_sep_dtheta, level.min_sep_dxy
                )
        if next_top:
            top = next_top
        # else: refinement found nothing scorable at this level -- keep the
        # coarser level's top as a fallback rather than losing everything
        # (defensive; the candidate's own centre was already scorable, so
        # this should not normally trigger).

    top.sort(key=lambda c: -c[0])
    best_score, best_dx, best_dy, best_dtheta, best_n = top[0]

    # Only count a second candidate as genuinely competing if it is STILL
    # a separate peak from the winner at a separation scale tied to the
    # score surface's own physical breadth (AMBIGUITY_MIN_SEP_*, NOT the
    # finest level's tiny grid-dedup threshold -- see that constant's
    # docstring for why conflating the two produced false ambiguity flags
    # on ordinary, well-determined scans).
    second_score = 0.0
    for (s, dx, dy, dtheta, n) in top[1:]:
        if (abs(dtheta - best_dtheta) >= AMBIGUITY_MIN_SEP_DTHETA_RAD
                or math.hypot(dx - best_dx, dy - best_dy) >= AMBIGUITY_MIN_SEP_DXY_M):
            second_score = s
            break   # top is sorted descending -- first distinct entry
                     # found is the best genuinely-competing candidate

    if best_n == 0:
        return CoarseResult(dx=0.0, dy=0.0, dtheta=0.0, score=0.0,
                             n_static=n_static, valid=False,
                             ambiguous=False, second_score=0.0)

    normalized = best_score / best_n
    if normalized < MIN_SCORE_PER_FEATURE:
        return CoarseResult(dx=0.0, dy=0.0, dtheta=0.0, score=best_score,
                             n_static=n_static, valid=False,
                             ambiguous=False, second_score=second_score)

    ambiguous = (len(top) > 1
                 and second_score >= best_score * (1.0 - AMBIGUITY_MARGIN_RATIO))

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
    # inside this module's search window) -- and now recovered to FINE
    # pyramid precision, not just the old coarse+greedy tolerance.
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
        _ME(angle=-math.pi/3, distance=1.4, mx=1.212, my=0.7, status=1, active=True),
    ]
    base_scan = [
        _line_feat(0.0, 1.0, -0.5, 1.0, 0.5, 1.0),
        _line_feat(math.pi/2, 2.0, 2.0, -0.5, 2.0, 0.5),
        _line_feat(-math.pi/3, 1.4, 0.962, 1.133, 1.462, 0.267),
    ]
    moved_scan = [_shift(f, true_dx, true_dy, true_dtheta) for f in base_scan]

    result = search(moved_scan, map_e, guess_x=0.0, guess_y=0.0, guess_theta=0.0)
    assert result.valid, f"T1 expected valid result, got {result}"
    finest = LEVELS[-1]
    assert abs(result.dx - true_dx) < finest.dxy_step * 2, f"T1 dx off: {result}"
    assert abs(result.dy - true_dy) < finest.dxy_step * 2, f"T1 dy off: {result}"
    assert abs(result.dtheta - true_dtheta) < finest.dtheta_step * 2, f"T1 dtheta off: {result}"
    print(f"  T1 PASS  recovered pose to fine pyramid precision "
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
    assert abs(result3.dx - true_dx) < finest.dxy_step * 2
    assert abs(result3.dy - true_dy) < finest.dxy_step * 2
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
    #         nothing in the scan disambiguates them. ambiguous MUST fire,
    #         even after multi-resolution refinement collapses each side's
    #         many near-tied x samples down to one representative peak per
    #         side.
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

    # ── T7: PRECISION -- this is the actual capability gain from switching
    #         to a real multi-resolution pyramid instead of coarse-sweep +
    #         greedy-walk: a real motion well within the search window
    #         should now be recovered to sub-degree / sub-centimetre
    #         accuracy (old finest greedy step was 3deg/3cm; new finest
    #         pyramid level is 0.4deg/0.4cm), and WITHOUT any risk of a
    #         greedy hill-climb stalling short of the true local peak
    #         (this sweeps the finest window exhaustively, it does not
    #         hill-climb). ─────────────────────────────────────────────
    true_dx7, true_dy7, true_dtheta7 = 0.041, 0.017, math.radians(4.3)
    moved_scan7 = [_shift(f, true_dx7, true_dy7, true_dtheta7) for f in base_scan]
    result7 = search(moved_scan7, map_e, guess_x=0.0, guess_y=0.0, guess_theta=0.0)
    assert result7.valid, f"T7 expected valid result, got {result7}"
    assert abs(result7.dx - true_dx7) < 0.004, f"T7 dx not fine-precision: {result7}"
    assert abs(result7.dy - true_dy7) < 0.004, f"T7 dy not fine-precision: {result7}"
    assert abs(result7.dtheta - true_dtheta7) < math.radians(0.6), \
        f"T7 dtheta not fine-precision: {result7}"
    print(f"  T7 PASS  fine-pyramid precision recovered a sub-5cm/5deg motion to "
          f"sub-cm/sub-degree accuracy (dx={result7.dx:.4f} dy={result7.dy:.4f} "
          f"dtheta={math.degrees(result7.dtheta):.2f}deg) -- old greedy-walk finest "
          f"step (3deg/3cm) could not have matched this without hill-climb risk")

    # ── T8: GUESS TRANSLATION BUG regression -- guess_x/guess_y are
    #         nonzero (the robot has genuinely moved from the map origin),
    #         which every earlier test in this file (T1-T7) never
    #         exercised -- all called search() with guess_x=guess_y=0.0.
    #         Before the fix, search() silently dropped guess_x/guess_y
    #         from every candidate's scored distance, so any guess away
    #         from the origin produced systematically -- and sometimes
    #         confidently -- wrong results. This recovers a real small
    #         motion FROM a pose that is already well away from the
    #         origin, exactly the situation a real moving robot is in
    #         almost the entire time it runs. ─────────────────────────
    guess_x8, guess_y8, guess_theta8 = 1.20, -0.70, math.radians(20.0)
    true_dx8, true_dy8, true_dtheta8 = 0.05, -0.03, math.radians(4.0)
    total_dx8 = guess_x8 + true_dx8
    total_dy8 = guess_y8 + true_dy8
    total_dtheta8 = guess_theta8 + true_dtheta8
    moved_scan8 = [_shift(f, total_dx8, total_dy8, total_dtheta8) for f in base_scan]

    result8 = search(moved_scan8, map_e,
                      guess_x=guess_x8, guess_y=guess_y8, guess_theta=guess_theta8)
    assert result8.valid, f"T8 expected valid result, got {result8}"
    assert abs(result8.dx - true_dx8) < finest.dxy_step * 2, \
        f"T8 dx off -- GUESS TRANSLATION BUG regression: {result8}"
    assert abs(result8.dy - true_dy8) < finest.dxy_step * 2, \
        f"T8 dy off -- GUESS TRANSLATION BUG regression: {result8}"
    assert abs(result8.dtheta - true_dtheta8) < finest.dtheta_step * 2, \
        f"T8 dtheta off -- GUESS TRANSLATION BUG regression: {result8}"
    print(f"  T8 PASS  recovered a small motion from a NONZERO guess "
          f"(guess=({guess_x8:.2f},{guess_y8:.2f},{math.degrees(guess_theta8):.0f}deg)): "
          f"dx={result8.dx:.4f} dy={result8.dy:.4f} dtheta={math.degrees(result8.dtheta):.2f}deg "
          f"-- this exact case was silently wrong before the guess-translation fix")

    # ── T9: direct cross-check between the vectorized hot path
    #         (_score_grid_vectorized) and the scalar reference path
    #         (_prerotate_features/_score_candidate) at a NONZERO guess --
    #         this is the "test_vectorized_matches_scalar" cross-check the
    #         "Vectorized grid scoring" section's own comment has promised
    #         since it was written, but which never actually existed.
    #         Confirms both paths agree AND both correctly include
    #         guess_x/guess_y (the fix above touched both paths). ───────
    guess_x9, guess_y9, guess_theta9 = 0.8, 0.6, math.radians(-10.0)
    dtheta9_probe = math.radians(2.0)
    dx9_probe, dy9_probe = 0.03, -0.02

    static_lines9, static_arcs9 = _prepare_static_maps(map_e, static_status=1)
    se_angle9, se_dist9 = _static_line_arrays(static_lines9)
    sa_mx9, sa_my9, sa_r9 = _static_arc_arrays(static_arcs9)

    vec_total9, vec_n9 = _score_grid_vectorized(
        base_scan, guess_x9, guess_y9, guess_theta9, dtheta9_probe,
        np.array([dx9_probe]), np.array([dy9_probe]),
        se_angle9, se_dist9, sa_mx9, sa_my9, sa_r9,
    )

    cos_t9 = math.cos(guess_theta9 + dtheta9_probe)
    sin_t9 = math.sin(guess_theta9 + dtheta9_probe)
    rotated_lines9, rotated_arcs9 = _prerotate_features(
        base_scan, cos_t9, sin_t9, guess_theta9 + dtheta9_probe
    )
    scalar_total9, scalar_n9 = _score_candidate(
        rotated_lines9, rotated_arcs9, dx9_probe, dy9_probe, guess_x9, guess_y9,
        static_lines9, static_arcs9,
    )

    assert vec_n9 == scalar_n9, f"T9 feature count mismatch: {vec_n9} vs {scalar_n9}"
    assert abs(float(vec_total9[0, 0]) - scalar_total9) < 1e-6, (
        f"T9 vectorized/scalar score mismatch at nonzero guess: "
        f"vec={float(vec_total9[0, 0]):.6f} scalar={scalar_total9:.6f}"
    )
    print(f"  T9 PASS  vectorized and scalar scoring paths agree at a nonzero "
          f"guess (score={scalar_total9:.4f}) -- the cross-check this file's "
          f"own comments promised but never implemented")

    print()
    print("All tests passed.")
    sys.exit(0)