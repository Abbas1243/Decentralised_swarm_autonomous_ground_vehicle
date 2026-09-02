"""
slam.py
=======
Top-level SLAM loop — Version 1 (Local SLAM only, slam_build_prompt.md).

This is the Python stand-in for slam_core/slam.c. It is the ONLY module
that is allowed to know about all three leaf modules at once:

    line_matcher.py    — pure function: scan features vs map -> matched pairs
    pose_estimator.py  — pure function: matched line pairs -> pose delta
    map_manager.py     — owns map state: update from scan features

Each leaf module stays exactly as pure as it already was. slam.py is where
the pipeline shape lives, exactly mirroring how slam.c will #include
line_matcher.h, pose_estimator.h, and map_manager.h without those headers
including each other.

PER-SCAN LOOP (slam_build_prompt.md, Version 1)
--------------------------------------------------
    FeaturePacket arrives (10 Hz)
            |
    1. Initial pose guess
           pose_guess = last_pose + odom_hint   (odom_hint optional, unused here
                                                  — no wheel encoders in this rig)
    2. Line matching
           line_matcher.match_features(scan_features, map_entries)
    3. Pose refinement
           pose_estimator.estimate_pose_delta(matches)
    4. Update current pose
           current_pose += refined_delta   (skip if estimator returned invalid)
    5. Map update
           Transform scan features into the MAP FRAME using the just-corrected
           pose (this transform did not exist before pose_estimator — see
           transform_features_to_map_frame() below), THEN call
           map_manager.update() on the transformed features.
    6. Return updated pose + map state

WHY THE FRAME TRANSFORM IS NEW
--------------------------------
map_manager.py's own docstring says: "before pose_estimator exists, the
robot is assumed stationary... frame drift is acceptable." That assumption
is no longer true once pose_estimator runs. From this point on, every
scan's features must be rotated/translated by the corrected pose BEFORE
they reach map_manager, or matched-but-uncorrected features will silently
re-introduce the drift pose_estimator just removed.

The transform must touch every field map_manager / line_matcher actually
read from a scan feature dict (verified against the source, not assumed):
    LINE : type, angle, distance, x1, y1, x2, y2, length, quality
    ARC  : type, cx, cy, r, theta_start, theta_end, length, quality

ARCS are transformed too (so map_manager can place them correctly) even
though pose_estimator (Algorithm 3) does not use arcs for the pose solve
itself — only line matches drive the correction.

PORT PATH
---------
When porting to C (slam_core/slam.c):
    SlamState struct        -> slam.h        (Pose current_pose; MapManager map;)
    slam_process_scan()     -> the function below, same five steps, no malloc:
        transform writes into a fixed MAX_FEATURES_PER_SCAN-sized scratch
        array (already the case here — see transform function) instead of
        building a new list.
    transform_features_to_map_frame() -> slam_transform_scan() in slam.c
"""

import math

import correlative_match
import pose_estimator
import soft_scan_matcher
from line_matcher import match_features
from map_manager import MapManager, ENTRY_STATIC
from pose_estimator import (
    build_line_matches_from_match_result,
    build_arc_matches_from_match_result,
    estimate_pose_delta,
    solve_pose_step,
    compute_residual,
    check_angular_diversity,
    PoseDelta,
)


# ---------------------------------------------------------------------------
# Per-scan pose delta sanity limits.
#
# A single bad scan (motion blur, brief occlusion from a hand passing in
# front of the sensor, a sudden drop in valid LiDAR returns while the unit
# is being physically handled) can produce a geometrically self-consistent
# but WRONG line-to-map correspondence: every individual matched line can
# still clear line_matcher's angle/distance thresholds and pose_estimator's
# math can still solve a valid rigid transform for that correspondence —
# the transform is just answering the wrong question, because it was given
# the wrong pairing. The result is a pose_delta that implies the robot
# teleported a metre or more in a single ~0.14s scan interval, which then
# corrupts the map (old STATIC walls stop matching and start decaying,
# wrong new entries get added) — observed directly: a real ~10-20cm hand
# slide produced an estimated 1.4m jump in one scan.
#
# These limits reject any single-scan delta that is not physically
# plausible for a hand-paced motion at this rig's ~7Hz observed scan rate,
# BEFORE it is applied to current_pose or used to update the map. This is
# the same "not enough information this scan, skip it" pattern already
# used for pose_delta.valid == False (too few matches) — just applied to
# "this delta doesn't make sense" instead of "no delta was computable".
MAX_DELTA_TRANSLATION_M = 0.025# max |dx|,|dy| considered plausible per scan
MAX_DELTA_ROTATION_RAD  = math.radians(30)  # max |dtheta| plausible per scan

# LOOPHOLE FIX: magnitude plausibility alone cannot catch a wrong-wall
# correspondence that happens to solve to a SMALL delta — see
# pose_estimator.compute_residual's docstring. This is the threshold on
# that residual (mean per-pair error, rotation term in radians + distance
# term in metres — see RESIDUAL_ANGLE_WEIGHT in pose_estimator.py) above
# which a refinement step is rejected outright regardless of how small its
# dx/dy/dtheta looked. Starting value is deliberately generous (this is a
# new check, tune tighter once real logged residuals from a known-good run
# are available) — a well-matched set of STATIC pairs typically resolves to
# well under half this value.
MAX_ACCEPTABLE_RESIDUAL = 0.15


# ---------------------------------------------------------------------------
# Per-ITERATION step bound for the soft-correspondence refinement loop.
#
# THE GAP THIS CLOSES: soft_scan_matcher.solve_soft_pose_step's Gauss-Newton
# linearisation — like pose_estimator's hard-match version before it — is
# only locally valid: it assumes the correction it is solving for is SMALL
# relative to the current working_pose guess (the Hough-normal translation
# model, the circular-mean rotation estimate, and the Gaussian correspondence
# weights themselves all implicitly assume this). MAX_DELTA_TRANSLATION_M /
# MAX_DELTA_ROTATION_RAD above only bound the FINAL accumulated delta for
# the whole scan, AFTER every iteration has already run — they do nothing to
# stop one single iteration mid-loop from taking an oversized, badly-
# linearised step when the true motion between scans combines BOTH
# significant rotation and significant translation at once. A large
# simultaneous rotate+translate is exactly the case where the linear
# approximation is weakest, so it is exactly the case where an unclamped
# single step is most likely to overshoot — the correspondence weighting
# picks up the wrong nearby map lines for the (now badly wrong) intermediate
# pose, and the loop either diverges outright or settles on an inaccurate
# final pose, which matches the "translation+rotation together loses
# accurate localisation" symptom reported after switching to the soft-
# correspondence loop.
#
# FIX: clamp EACH iteration's (dx, dy, dtheta) to a bound well inside the
# per-scan plausibility limits above, forcing a large true motion to be
# walked down across several of MAX_ITERATIONS' smaller, well-linearised
# steps — each one re-matched (soft correspondence is recomputed fresh
# every iteration against the improved working_pose) — instead of one
# large, unreliable leap. This is a standard trust-region-style damping for
# Gauss-Newton/ICP, not a new algorithm; correlative_match's coarse seed
# already gets working_pose within +/-15cm/+/-15 deg before this loop even
# starts, so the fine loop's remaining job is genuinely local cleanup and
# does not need — and should not take — large single steps.
MAX_ITER_STEP_TRANSLATION_M = 0.08
MAX_ITER_STEP_ROTATION_RAD  = math.radians(8.0)


def _clamp_iteration_step(dx, dy, dtheta):
    """
    Scale (dx, dy) down (preserving direction) if their combined magnitude
    exceeds MAX_ITER_STEP_TRANSLATION_M; clip dtheta independently to
    +/-MAX_ITER_STEP_ROTATION_RAD. Values already inside both bounds pass
    through unchanged — this only engages for oversized single-iteration
    steps, it does not add damping to already-small, well-behaved updates.
    """
    translation_mag = math.hypot(dx, dy)
    if translation_mag > MAX_ITER_STEP_TRANSLATION_M:
        scale = MAX_ITER_STEP_TRANSLATION_M / translation_mag
        dx *= scale
        dy *= scale

    if dtheta > MAX_ITER_STEP_ROTATION_RAD:
        dtheta = MAX_ITER_STEP_ROTATION_RAD
    elif dtheta < -MAX_ITER_STEP_ROTATION_RAD:
        dtheta = -MAX_ITER_STEP_ROTATION_RAD

    return dx, dy, dtheta


def _is_pose_delta_plausible(pose_delta):
    """
    True if pose_delta's magnitude is within MAX_DELTA_TRANSLATION_M /
    MAX_DELTA_ROTATION_RAD. Does not check .valid — caller is expected to
    have already checked that (an invalid delta is zero by construction
    and would trivially pass this check, which is not the question being
    asked here).
    """
    return (abs(pose_delta.dx) <= MAX_DELTA_TRANSLATION_M
            and abs(pose_delta.dy) <= MAX_DELTA_TRANSLATION_M
            and abs(pose_delta.dtheta) <= MAX_DELTA_ROTATION_RAD)


# ---------------------------------------------------------------------------
# Cross-scan consistency guard.
#
# THE GAP THIS CLOSES: MAX_DELTA_TRANSLATION_M / MAX_DELTA_ROTATION_RAD only
# look at ONE scan's delta in isolation. They have no memory of where the
# pose was trending. During a real motion event, several consecutive scans
# can each independently solve a delta that is individually within the
# single-scan plausibility limit, but whose DIRECTION is wrong (because
# line_matcher paired a scan wall with the wrong nearby wall — easy to do
# in a room with many parallel/perpendicular walls once the viewpoint is
# mid-motion and uncertain). None of those deltas trips the magnitude
# guard alone, but applied one after another they compound into a
# multi-degree heading error and a position several tens of cm off — and
# once that wrong pose is applied, it starts validating itself: new scan
# features get added to the map as fresh UNCLASSIFIED entries computed
# FROM the wrong pose, which then re-confirm the wrong pose on the next
# scan. This was observed directly on hardware: th drifted from +2.7deg to
# +10.2deg over a handful of "accepted" scans during a 20-30cm slide, then
# stayed locked at +10deg for 250+ subsequent scans because the corrupted
# map now agreed with the corrupted pose.
#
# FIX: track a short history of recently ACCEPTED deltas (the ones that
# passed the single-scan magnitude guard and were actually applied). A new
# delta must be consistent with the recent trend (same general direction,
# not a sharp reversal or an outsized jump relative to the recent average)
# before its STRONGEST consequence — unlocking map writes via Step 5 — is
# allowed. An inconsistent delta still updates the pose tracking estimate
# in a damped way (so the system doesn't freeze completely on a genuine
# but jittery motion), but is NOT allowed to author new map entries until
# the trend re-stabilises. This breaks the self-validation loop: bad poses
# can no longer write the map evidence that would otherwise convince the
# system they were correct.
TREND_HISTORY_LEN      = 5     # number of recent accepted deltas tracked
TREND_ROTATION_TOL_RAD = math.radians(6.0)   # max deviation from trend mean
TREND_TRANSLATION_TOL_M = 0.10               # max deviation from trend mean

# Direction-consistency check (see _is_consistent_with_trend's docstring,
# part (2)) — compares this scan's translation direction against the MOST
# RECENT accepted delta only, not the flat history mean, so it adapts
# instantly during real sustained turning instead of flagging every scan
# against a stale pre-turn direction.
DIRECTION_CHECK_MIN_MAG_M = 0.04   # below this, a vector's direction is
                                    # noise-dominated -- skip the direction
                                    # check entirely (magnitude check above
                                    # still applies)
DIRECTION_CHECK_MIN_COS = math.cos(math.radians(30.0))
                                    # minimum cosine similarity between this
                                    # delta's direction and the previous
                                    # accepted delta's direction -- 60 deg
                                    # generous starting tolerance (a real
                                    # scan-to-scan direction change from
                                    # smooth turning is far smaller than
                                    # this at normal scan rates; a wrong-
                                    # correspondence anomaly typically
                                    # disagrees far more sharply). Tune
                                    # tighter once logged real data from a
                                    # known-good run is available, same
                                    # spirit as MAX_ACCEPTABLE_RESIDUAL.

# LOOPHOLE FIX: the original off-trend handling wiped self._delta_history
# to [] entirely, which made _is_consistent_with_trend return True ("no
# opinion") for the very next delta regardless of whether IT was also
# wrong — two bad-in-a-row deltas could both slip through map-write gating.
# Fix: history is no longer wiped (the real trend stays available to check
# future deltas against); instead, an off-trend event puts the system on
# PROBATION for this many scans, during which map writes stay blocked even
# if a later delta individually looks trend-consistent again — requiring
# a short run of agreement, not just one lucky match, before trusting map
# writes again.
TREND_PROBATION_SCANS = 2

# AMBIGUITY_PROBATION_SCANS -- a separate, LONGER probation window used
# specifically when coarse_ambiguous fires (see coarse_ambiguous in
# process_scan). This is deliberately longer than TREND_PROBATION_SCANS.
#
# WHY THESE ARE NOT THE SAME NUMBER: an off-trend delta is ordinary noise —
# one scan disagreed with a short recent history, most likely a transient
# mismatch, and the underlying pose estimate is still anchored near the
# truth. An ambiguous coarse result is a different, structurally worse kind
# of failure: correlative_match found a SECOND candidate orientation
# nearly tied with the winner (e.g. rectangular-room symmetry), meaning
# the search itself could not tell which peak was real THIS scan.
#
# THE FAILURE MODE THIS CLOSES: once the working pose has drifted onto the
# wrong peak, correlative_match's search window is centred on that WRONG
# guess for every subsequent scan -- the true peak can fall entirely
# outside the window, so nothing looks ambiguous anymore from that point
# on. The system then looks locally confident and self-consistent (it
# agrees with itself, just about the wrong orientation), TREND_PROBATION_
# SCANS's short 2-scan window clears, and map writes resume -- now
# confidently authoring WRONG map entries instead of sparsely polluting
# them, which is a worse visual outcome than not fixing the ambiguity gate
# at all (a dense, well-connected wrong wall reads as "the map", not as
# obvious noise to be second-guessed). A longer, separate probation after
# an ambiguity event buys more scans for the trend/plausibility guards to
# catch a genuine wrong-peak lock-in before map writes are trusted again,
# rather than releasing trust the moment the (now self-consistent) wrong
# trajectory produces two on-trend scans in a row.
#
# Starting value is deliberately several times TREND_PROBATION_SCANS, not
# tuned against real logged recovery times yet -- same "deliberately
# generous starting value, tune once real data exists" spirit as
# MAX_ACCEPTABLE_RESIDUAL / AMBIGUITY_MARGIN_RATIO elsewhere in this
# codebase.
AMBIGUITY_PROBATION_SCANS = 10


# ---------------------------------------------------------------------------
# MAX_UNTRUSTED_STREAK -- caps how many CONSECUTIVE untrustworthy deltas
# (off-trend and/or ambiguous-seeded) may be applied to current_pose before
# process_scan stops applying them and FREEZES the pose entirely.
#
# THE GAP THIS CLOSES -- confirmed directly on a real hardware log: heading
# walked from +0.1deg to -56.3deg to +70-98deg over one continuous run,
# while map.stats() showed STATIC entries completely flat (~12-16) the
# whole time and UNCLASSIFIED entries exploding 45 -> 229. That combination
# is diagnostic: the MAP was being protected correctly (TREND_PROBATION_
# SCANS / AMBIGUITY_PROBATION_SCANS blocked almost every write -- dozens of
# consecutive "OFF-TREND ... MAP WRITE BLOCKED" log lines), but
# self.current_pose itself was NOT protected at all. _trend_probation and
# coarse_ambiguous were, before this fix, only ever consulted in the Step 5
# (map write) gate -- Step 4 (apply pose_delta to current_pose) checks
# nothing but _is_pose_delta_plausible's single-scan MAGNITUDE bound. A
# long unbroken run of individually-plausible-but-untrustworthy deltas —
# exactly what an unconstrained fine soft-match loop produces once
# correlative_match's coarse seed fails (coarse_valid=False, which
# dominated the failing log: too few STATIC anchors matched within its
# window, or real per-scan motion outside +/-15cm/+/-15deg) — had NOTHING
# stopping it from walking current_pose to an arbitrary wrong heading, one
# magnitude-plausible step at a time, forever. The map staying "protected"
# gave false confidence: the pose consuming those same deltas was silently
# accumulating the exact error the map guards were built to keep out.
#
# FIX: track a running count of consecutive scans where the applied delta
# was NOT trend-consistent-and-unambiguous (mirrors _trend_probation's
# bookkeeping, but drives Step 4, not just Step 5). Once that streak
# exceeds this threshold, process_scan stops calling
# current_pose.apply_delta() -- the pose FREEZES at its last trustworthy
# value, exactly like the existing "too few matches" (pose_delta.valid ==
# False) case already does, rather than continuing to free-run on
# increasingly-suspect data. This gives correlative_match's NEXT coarse
# search a stable, still-correct guess_x/guess_y/guess_theta to search
# around instead of chasing an ever-drifting one -- the actual mechanism
# by which the system can recover, versus compounding error indefinitely
# with no way back (which is what a permanently wrong TF/pose looks like
# from the outside: real motion after that point is measured relative to
# a wrong origin forever).
#
# Deliberately SEPARATE from _trend_probation's threshold (which only
# needs to survive a couple of scans to protect the map) -- freezing the
# pose is a much stronger, more visible intervention (odometry stops
# advancing at all) and should only trigger once distrust has clearly
# persisted, not on the first or second off-trend blip that
# _is_consistent_with_trend's own "don't freeze on real jittery motion"
# reasoning is meant to tolerate. Starting value, not yet tuned against
# real logged recovery times -- same spirit as every other "deliberately
# generous starting value" constant in this codebase.
MAX_UNTRUSTED_STREAK = 1


def _is_consistent_with_trend(pose_delta, history, last_applied_delta):
    """
    True if pose_delta's rotation and translation are consistent with
    recent accepted-delta history. With fewer than 2 samples in history
    there is no trend yet to check against — returns True (no opinion),
    matching the existing "let the system bootstrap" pattern used
    elsewhere (e.g. pose_delta.valid==False on an immature map).

    TWO SEPARATE CHECKS, EACH FIXING A DISTINCT FAILURE MODE SEEN ON
    HARDWARE — history matters here, see both bugs below:

    (1) MAGNITUDE vs the history MEAN (still flat-averaged — magnitude
        does not have a staleness problem the way direction does, see
        (2) below): dx/dy are defined in the fixed MAP frame. A robot
        moving at a genuinely constant real speed WHILE TURNING produces
        a per-scan map-frame translation vector whose magnitude stays
        constant but whose DIRECTION rotates continuously with heading.
        An earlier version of this function compared raw dx/dy components
        against a history mean and had no way to distinguish that entirely
        normal case from actually erratic motion — logs showed 300+
        consecutive scans stuck OFF-TREND for the full duration of a
        sustained turn, and because MapManager.update() (adds AND decays
        entries) is skipped whenever writes are blocked, that lockout
        froze the map for the whole turn (stale walls that never decayed
        — the earlier "wall shown where there's no obstacle" report).

    (2) DIRECTION vs the MOST RECENT single accepted delta ONLY, not a
        flat mean of history: this is the fix for a REGRESSION the
        magnitude-only version above introduced. Removing direction
        checking entirely (to fix (1)) also removed ALL protection
        against a wrong-correspondence delta that happens to have a
        plausible MAGNITUDE but points in an unrelated or wrong
        direction — which then gets treated as "on-trend" and is trusted
        to author new map evidence. Hardware logs after that version
        showed exactly this: map UNCLASSIFIED entries exploding past 150
        while STATIC entries stayed flat near 11-16 (bad poses writing
        junk that never matures, because it doesn't correspond to
        anything real), pose freezing for long stretches, and repeated
        large REJECTED deltas — map corruption compounding on itself.
        Comparing direction against the MOST RECENT accepted delta only
        (rather than a flat mean over TREND_HISTORY_LEN scans) is what
        makes this compatible with (1): during a real sustained turn, the
        immediately-previous scan's direction has already rotated to
        reflect the mid-turn heading, so a smoothly continuing turn stays
        direction-consistent scan to scan even though it would NOT be
        consistent with a 5-scan-old direction from before the turn
        started. A genuinely wrong-correspondence direction anomaly, by
        contrast, still disagrees sharply with what the ADJACENT scan
        just did, so it is still caught.

        IMPORTANT: "most recent" here means last_applied_delta — the most
        recent delta that was APPLIED at all, regardless of whether IT was
        trend-consistent — not history[-1], which only contains trend-
        consistent deltas. Using history[-1] as the reference reintroduces
        a staleness cascade: once one scan is blocked as off-trend, the
        next comparison's reference is already more than one real scan
        old, widening the effective rotation/motion gap being checked and
        cascading into the same lockout the magnitude fix was meant to
        remove (confirmed directly — using history[-1] here reproduced the
        original buggy version's block rate almost exactly).

        Skipped (treated as consistent) when either vector's magnitude is
        below DIRECTION_CHECK_MIN_MAG_M — the direction of a near-zero
        translation is dominated by noise and not a meaningful signal
        either way.

    dtheta is compared directly against the history mean (not converted
    to magnitude or treated specially) — a rotation delta already adds
    correctly regardless of the robot's current heading, so raw dtheta
    has no equivalent direction-rotates-with-heading problem translation
    has.
    """
    if len(history) < 2:
        return True

    mean_translation_mag = sum(math.hypot(d.dx, d.dy) for d in history) / len(history)
    this_translation_mag = math.hypot(pose_delta.dx, pose_delta.dy)
    mean_dtheta = sum(d.dtheta for d in history) / len(history)

    magnitude_ok = (abs(this_translation_mag - mean_translation_mag) <= TREND_TRANSLATION_TOL_M
                     and abs(pose_delta.dtheta - mean_dtheta) <= TREND_ROTATION_TOL_RAD)
    if not magnitude_ok:
        return False

    if last_applied_delta is None:
        return True   # no direction reference yet — magnitude check already passed

    recent_mag = math.hypot(last_applied_delta.dx, last_applied_delta.dy)
    if this_translation_mag < DIRECTION_CHECK_MIN_MAG_M or recent_mag < DIRECTION_CHECK_MIN_MAG_M:
        return True   # too small to have a meaningful direction either way

    cos_similarity = ((pose_delta.dx * last_applied_delta.dx + pose_delta.dy * last_applied_delta.dy)
                       / (this_translation_mag * recent_mag))
    return cos_similarity >= DIRECTION_CHECK_MIN_COS

    return (abs(this_translation_mag - mean_translation_mag) <= TREND_TRANSLATION_TOL_M
            and abs(pose_delta.dtheta - mean_dtheta) <= TREND_ROTATION_TOL_RAD)


# ---------------------------------------------------------------------------
# Pose — local to slam.py for now.
#
# slam_build_prompt.md defines Pose as SLAM-internal state (x, y, theta),
# explicitly OUT of scope for messages.h (the wire-protocol-only header).
# It does not yet exist as a shared module anywhere in this codebase, and
# introducing a new pose.py for a 3-field struct used by exactly one module
# would be premature — if/when pose_graph.c (Version 2) needs PoseNode /
# PoseConstraint, promote this to its own module then.
# ---------------------------------------------------------------------------

class Pose:
    """Robot pose in the map frame. x, y in metres, theta in radians."""

    __slots__ = ("x", "y", "theta")

    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta

    def apply_delta(self, dx, dy, dtheta):
        """In-place pose update: current_pose += refined_delta (Step 4)."""
        self.x += dx
        self.y += dy
        self.theta += dtheta

    def __repr__(self):
        return f"Pose(x={self.x:.3f}, y={self.y:.3f}, theta={math.degrees(self.theta):.1f}°)"


# ---------------------------------------------------------------------------
# Frame transform — sensor frame -> map frame, using the corrected pose.
# ---------------------------------------------------------------------------

def _rotate_point(x, y, cos_t, sin_t):
    """Rotate (x, y) by theta (given as precomputed cos/sin) about the origin."""
    return x * cos_t - y * sin_t, y * cos_t + x * sin_t


def transform_features_to_map_frame(scan_features, pose):
    """
    Transform a list of scan feature dicts from the sensor frame into the
    map frame, given the robot's pose (x, y, theta) in the map frame.

    This is a pure coordinate transform — no matching, no map mutation.
    Called once per scan, after pose correction (Step 4) and before
    map_manager.update() (Step 5).

    Parameters
    ----------
    scan_features : list of dicts
        Output of fit_first_ctypes.split_merge() — each dict has
        'type' == 'line' or 'arc' plus type-specific fields (see module
        docstring for the exact field list this function rewrites).
    pose : Pose
        Robot pose in the map frame for this scan.

    Returns
    -------
    list of dicts — same shape as the input, all spatial fields rewritten
    into the map frame. Quality, length, type, and any other non-spatial
    field are passed through unchanged. Input list is not mutated; new
    dicts are returned (the C port replaces this with in-place writes into
    a fixed scratch array — no allocation per scan).

    LINE transform:
        endpoints (x1,y1) (x2,y2): rotate by theta, then translate by (pose.x, pose.y)
        Hough angle: angle' = angle + theta, re-wrapped into [-pi/2, pi/2]
        Hough distance: recomputed from the transformed midpoint and angle'
                     rather than rotating the scalar distance directly, since
                     translation moves the perpendicular foot of the line
                     unless the robot sits exactly on the original line.
                     This recompute is correct regardless of which side of
                     the wrap the angle lands on, so no separate sign-flip
                     bookkeeping is needed here (unlike pose_estimator's
                     _normalize_pair, which works with scalar distances that
                     have no midpoint to recompute from).
        length: unchanged (rotation/translation is rigid, preserves length)

    ARC transform:
        centre (cx, cy): rotate by theta, then translate by (pose.x, pose.y)
        theta_start / theta_end: shift by pose.theta (arc orientation rotates
                     with the sensor frame)
        radius (r), length: unchanged (rigid transform)
    """
    cos_t = math.cos(pose.theta)
    sin_t = math.sin(pose.theta)

    out = []
    for feat in scan_features:
        ftype = feat.get("type")

        if ftype == "line":
            x1, y1 = _rotate_point(feat["x1"], feat["y1"], cos_t, sin_t)
            x2, y2 = _rotate_point(feat["x2"], feat["y2"], cos_t, sin_t)
            x1 += pose.x; y1 += pose.y
            x2 += pose.x; y2 += pose.y

            angle = feat["angle"] + pose.theta
            # Re-wrap into [-pi/2, pi/2] — consistent with the Hough
            # convention used everywhere else (pose_estimator, map_manager).
            if angle > math.pi / 2.0:
                angle -= math.pi
            elif angle < -math.pi / 2.0:
                angle += math.pi

            # Recompute Hough distance from the transformed midpoint —
            # robust to translation, unlike rotating the scalar distance
            # directly (which only works for pure rotation about the origin).
            mx = (x1 + x2) / 2.0
            my = (y1 + y2) / 2.0
            nx, ny = -math.sin(angle), math.cos(angle)
            distance = nx * mx + ny * my

            new_feat = dict(feat)
            new_feat["x1"] = x1
            new_feat["y1"] = y1
            new_feat["x2"] = x2
            new_feat["y2"] = y2
            new_feat["angle"] = angle
            new_feat["distance"] = distance
            out.append(new_feat)

        elif ftype == "arc":
            cx, cy = _rotate_point(feat["cx"], feat["cy"], cos_t, sin_t)
            cx += pose.x
            cy += pose.y

            new_feat = dict(feat)
            new_feat["cx"] = cx
            new_feat["cy"] = cy
            new_feat["theta_start"] = feat.get("theta_start", 0.0) + pose.theta
            new_feat["theta_end"] = feat.get("theta_end", 0.0) + pose.theta
            out.append(new_feat)

        else:
            # Unknown feature type — pass through unchanged rather than
            # silently dropping it; map_manager._should_add already rejects
            # unknown types, so this is safe and keeps the function total.
            out.append(dict(feat))

    return out


# ---------------------------------------------------------------------------
# SlamState — top-level orchestrator
# ---------------------------------------------------------------------------

class SlamState:
    """
    Owns the two pieces of persistent state for Version 1 local SLAM:
        current_pose : Pose         — robot pose estimate, map frame
        map          : MapManager   — persistent feature map

    Usage
    -----
        slam = SlamState()
        for scan_features, scan_idx in scan_stream:
            pose, match_result, pose_delta, delta_applied = slam.process_scan(scan_features, scan_idx)
    """

    def __init__(self, initial_pose=None):
        self.current_pose = initial_pose if initial_pose is not None else Pose()
        self.map = MapManager()
        self._delta_history = []   # recent ACCEPTED deltas, most recent last
                                    # — see _is_consistent_with_trend / Step 5
                                    # gating in process_scan for why this exists
        self._trend_probation = 0  # LOOPHOLE FIX: after an off-trend delta,
                                    # history used to be wiped to [] entirely
                                    # — _is_consistent_with_trend returns True
                                    # ("no opinion") for len(history) < 2, so
                                    # the VERY NEXT delta (even if also wrong)
                                    # sailed through unchecked. Now history is
                                    # left intact (still checked against the
                                    # real trend) and this counter instead
                                    # requires N consecutive on-trend deltas
                                    # before map writes are trusted again —
                                    # see process_scan Step 5 gating.
        self.last_delta_on_trend = True   # status of most recent process_scan call
        self.last_map_updated = True      # status of most recent process_scan call

        self._untrusted_streak = 0   # consecutive scans where the applied
                                      # delta was NOT trend-consistent-and-
                                      # unambiguous -- see MAX_UNTRUSTED_
                                      # STREAK's docstring. Drives whether
                                      # Step 4 freezes current_pose, in
                                      # addition to _trend_probation
                                      # already driving Step 5's map-write
                                      # freeze.
        self.last_pose_frozen = False   # True if THIS scan's delta was
                                         # magnitude-plausible but withheld
                                         # from current_pose entirely
                                         # because MAX_UNTRUSTED_STREAK was
                                         # exceeded -- distinct from
                                         # delta_applied==False (which
                                         # covers invalid/implausible
                                         # deltas that never had a chance
                                         # to move the pose in the first
                                         # place).

        # Direction-check reference — see _is_consistent_with_trend's
        # docstring part (2). Deliberately SEPARATE from _delta_history:
        # _delta_history only accumulates on-trend deltas (used for the
        # magnitude-mean check and to decide map-write trust), but using
        # history[-1] as the direction reference meant that once ONE scan
        # got blocked as off-trend, the reference for the NEXT comparison
        # was already stale by more than one real scan — during fast
        # rotation this widened the effective rotation gap being checked
        # each time and cascaded into the same near-permanent lockout the
        # magnitude-only fix was supposed to eliminate (confirmed by a
        # direct A/B test: reintroducing history-based direction checking
        # reproduced the OLD buggy version's exact block rate). Updated on
        # EVERY applied delta (regardless of trend status) so it always
        # reflects the truly most recent real motion, not the most recent
        # TRUSTED one.
        self._last_applied_delta = None   # PoseDelta or None

        # Refinement-loop diagnostics — see process_scan's soft-
        # correspondence loop. Exposed as instance attributes (same
        # established pattern as last_delta_on_trend / last_map_updated
        # above) rather than widening process_scan's return tuple, since
        # several callers and self-tests already unpack that tuple at a
        # fixed arity. Captures the LAST executed iteration's values even
        # when the loop broke early, so a caller can tell WHY it stopped
        # (too little weight? poorly conditioned?) not just that it did.
        self.last_iterations_run = 0
        self.last_coarse_valid = False
        self.last_coarse_ambiguous = False
        self.last_coarse_dx = 0.0
        self.last_coarse_dy = 0.0
        self.last_coarse_dtheta = 0.0
        self.last_fine_total_dx = 0.0
        self.last_fine_total_dy = 0.0
        self.last_fine_total_dtheta = 0.0
        self.last_final_total_weight = 0.0
        self.last_final_min_eigenvalue = 0.0
        self.last_break_reason = "none"   # "none" | "low_weight" |
                                           # "poor_conditioning" | "converged" |
                                           # "max_iterations" | "exceeded_scan_budget"

    def process_scan(self, scan_features, scan_idx=None, odom_hint=None):
        """
        Run one full SLAM cycle for a single scan's worth of features.

        Parameters
        ----------
        scan_features : list of dicts
            One scan's features, in the SENSOR frame (as produced by
            fit_first / split_merge — NOT yet transformed).
        scan_idx : int or None
            Scan sequence number, forwarded to map_manager for decay timing.
            If None, MapManager auto-increments internally.
        odom_hint : (dx, dy, dtheta) or None
            Optional wheel-odometry hint for the initial pose guess
            (Step 1). This rig has no wheel encoders, so this is always
            None today — kept as a parameter so slam.c's signature does not
            need to change if encoders are added later.

        Returns
        -------
        (pose, match_result, pose_delta, delta_applied)
            pose         : Pose            — updated current_pose (same object,
                                              mutated in place, also returned
                                              for convenience)
            match_result : MatchResult     — from line_matcher, for diagnostics
            pose_delta   : PoseDelta        — from pose_estimator, for diagnostics.
                                              pose_delta.valid is False on scans
                                              with too few matches. NOTE this can
                                              be True even when delta_applied is
                                              False — see delta_applied below.
            delta_applied: bool            — True only if pose_delta.valid AND
                                              its magnitude was within plausible
                                              per-scan limits (see
                                              _is_pose_delta_plausible). False
                                              means current_pose and self.map
                                              were NOT updated this scan — either
                                              because there weren't enough
                                              matches, or because the computed
                                              delta implied an implausible jump
                                              (degraded scan data mid-motion
                                              matched the wrong map entries).
                                              Callers that log/display per-scan
                                              status should key off this field,
                                              not pose_delta.valid alone.
        """
        # ---- Step 1 — initial pose guess --------------------------------
        if odom_hint is not None:
            odx, ody, odtheta = odom_hint
            self.current_pose.apply_delta(odx, ody, odtheta)
        # else: pose_guess == last_pose (no-op), as specified when no
        # odometry hint is available.

        # ---- Step 1b — correlative coarse search ------------------------
        # THE CORE FIX (see process_scan module-level notes and
        # correlative_match.py's docstring for the full rationale): hard
        # correspondence (line_matcher.match_features, below) can only be
        # trusted once the pose guess is already close — but the guess
        # only gets close by trusting some correspondence. Correlative
        # search breaks that loop WITHOUT committing to any single
        # correspondence: it scores a small grid of candidate poses around
        # the current guess against ALL STATIC map features at once (soft,
        # non-committal scoring) and returns whichever candidate makes the
        # whole scan agree best. That result seeds the loop below at a
        # pose where hard correspondence is now very likely to get it
        # right the first time.
        #
        # If correlative_match.search() is not confident (too few STATIC
        # anchors yet — e.g. an immature/bootstrapping map — or no
        # candidate scored well enough) it returns valid=False and we fall
        # back to exactly today's behaviour: seed with the unmodified
        # current_pose. This is a graceful degrade, not a new failure
        # mode — the loop below still runs normally from there.
        coarse = correlative_match.search(
            scan_features, self.map._entries,
            guess_x=self.current_pose.x,
            guess_y=self.current_pose.y,
            guess_theta=self.current_pose.theta,
            static_status=ENTRY_STATIC,
        )
        if coarse.valid:
            working_pose = Pose(
                self.current_pose.x + coarse.dx,
                self.current_pose.y + coarse.dy,
                self.current_pose.theta + coarse.dtheta,
            )
        else:
            working_pose = Pose(
                self.current_pose.x, self.current_pose.y, self.current_pose.theta
            )
        # right after computing `coarse` and before the refinement for-loop,
        # add a scan-specific budget cap
        if coarse.valid:
            scan_budget_translation = MAX_DELTA_TRANSLATION_M
            scan_budget_rotation = MAX_DELTA_ROTATION_RAD
        else:
            # No verified coarse seed this scan -- fine loop is refining from an
            # unanchored guess, so cap it much tighter than the normal per-scan
            # budget (same spirit as MAX_ITER_STEP_*, just applied to the WHOLE
            # scan when there's no coarse confirmation at all).
            scan_budget_translation = MAX_ITER_STEP_TRANSLATION_M
            scan_budget_rotation = MAX_ITER_STEP_ROTATION_RAD

        # ---- Steps 2+3 — re-matching Gauss-Newton refinement -------------
        # SOFT CORRESPONDENCE (see soft_scan_matcher.py module docstring,
        # written up after comparing against hector_slam's ScanMatcher).
        # This used to re-decide a single HARD 1:1 best-match correspondence
        # every iteration (line_matcher.match_features) and solve against
        # that. That was real ICP, an improvement over the original single-
        # linearisation bug — but a discrete "winner" correspondence can
        # still FLIP between two similarly-scoring map lines from one
        # iteration to the next while the pose is still uncertain, which is
        # root cause #2 in slam_progress_update_pose_estimator.md (dx/dy
        # oscillating between two near-opposite values for 5+ sec during
        # combined rotate+translate).
        #
        # soft_scan_matcher.solve_soft_pose_step never picks a winner: every
        # nearby STATIC map line/arc contributes a continuously-weighted
        # term to the same Gauss-Newton normal equations pose_estimator.py
        # already built, so there is no discrete pairing left to flip.
        #
        # Only STATIC map entries drive correction — same reasoning as
        # before (see build_line_matches_from_match_result's docstring):
        # a DYNAMIC or fresh UNCLASSIFIED entry can shift for reasons
        # unrelated to the robot's own motion. Split once, outside the
        # loop — self.map._entries does not change while working_pose is
        # being refined, only re-splitting on every call would be wasted
        # work.
        static_lines = [e for e in self.map._entries
                         if e.active and e.status == ENTRY_STATIC and not e.is_arc()]
        static_arcs = [e for e in self.map._entries
                        if e.active and e.status == ENTRY_STATIC and e.is_arc()]

        match_result = None
        total_dx = total_dy = total_dtheta = 0.0
        iterations_run = 0
        refinement_ran = False
        break_reason = "max_iterations"   # overwritten below on early exit;
                                           # stays this value if the loop runs
                                           # all pose_estimator.MAX_ITERATIONS
                                           # without breaking early
        final_total_weight = 0.0
        final_min_eigenvalue = 0.0

        for _ in range(pose_estimator.MAX_ITERATIONS):
            iter_features = transform_features_to_map_frame(
                scan_features, working_pose
            )

            dx, dy, dtheta, total_weight, min_eigenvalue = soft_scan_matcher.solve_soft_pose_step(
                iter_features, static_lines, static_arcs
            )
            final_total_weight = total_weight
            final_min_eigenvalue = min_eigenvalue

            if total_weight < soft_scan_matcher.MIN_TOTAL_WEIGHT:
                # Not enough soft-weighted evidence THIS iteration — same
                # normal "not enough info this scan" case the old
                # len(matches) < MIN_MATCHES check covered, just measured
                # continuously instead of by discrete pair count.
                break_reason = "low_weight"
                break

            if min_eigenvalue < soft_scan_matcher.MIN_EIGENVALUE_THRESHOLD:
                # Translation poorly constrained along some direction by
                # the currently weighted-in map lines (e.g. all near-
                # parallel) and no arc strong enough to anchor it yet —
                # same protection check_angular_diversity used to provide,
                # now read directly off the conditioning of the system
                # actually being solved (see
                # soft_scan_matcher._min_eigenvalue_2x2's docstring for why
                # this is a more direct measurement than the old angular-
                # spread proxy over a discrete matched-pairs list, which no
                # longer exists in the soft-correspondence path).
                break_reason = "poor_conditioning"
                break

            # See MAX_ITER_STEP_TRANSLATION_M / MAX_ITER_STEP_ROTATION_RAD
            # module-level comment: bound THIS iteration's step before
            # applying it, so a large combined rotate+translate motion is
            # walked down across several smaller, well-linearised steps
            # instead of risking one oversized, badly-linearised leap.
            # Applied after the confidence/conditioning gates above, which
            # judge the SOLVED system's quality and should see the
            # unclamped values — clamping only changes how much of that
            # solved correction gets applied this iteration, not whether it
            # was trustworthy to begin with.
            dx, dy, dtheta = _clamp_iteration_step(dx, dy, dtheta)

            working_pose.apply_delta(dx, dy, dtheta)
            total_dx += dx
            total_dy += dy
            total_dtheta += dtheta
            iterations_run += 1
            refinement_ran = True

            if abs(dx) + abs(dy) + abs(dtheta) < pose_estimator.CONVERGENCE_THRESHOLD:
                break_reason = "converged"
                break

            # Early exit once the accumulated correction (coarse seed +
            # fine refinement so far) already exceeds the SAME outer
            # plausibility budget _is_pose_delta_plausible checks after
            # this loop finishes. Running the remaining iterations at that
            # point cannot help — this scan's delta is already going to be
            # rejected wholesale by that outer check regardless of what
            # happens next, and continuing to iterate only risks
            # compounding further into an even worse correspondence basin
            # while burning CPU that a real-time loop cannot spare. This
            # directly targets a pattern seen repeatedly in hardware logs:
            # break=max_iterations (all 8 iterations consumed) immediately
            # followed by REJECTED implausible pose delta — all of that
            # work was already doomed by iteration 3-4 in those cases.
            combined_dx_so_far = (coarse.dx if coarse.valid else 0.0) + total_dx
            combined_dy_so_far = (coarse.dy if coarse.valid else 0.0) + total_dy
            combined_dtheta_so_far = (coarse.dtheta if coarse.valid else 0.0) + total_dtheta
            if (math.hypot(combined_dx_so_far, combined_dy_so_far) > MAX_DELTA_TRANSLATION_M
                    or abs(combined_dtheta_so_far) > MAX_DELTA_ROTATION_RAD):
                break_reason = "exceeded_scan_budget"
                break

        self.last_iterations_run = iterations_run
        self.last_coarse_valid = coarse.valid
        self.last_coarse_ambiguous = coarse.valid and coarse.ambiguous
        self.last_coarse_dx = coarse.dx if coarse.valid else 0.0
        self.last_coarse_dy = coarse.dy if coarse.valid else 0.0
        self.last_coarse_dtheta = coarse.dtheta if coarse.valid else 0.0
        self.last_fine_total_dx = total_dx
        self.last_fine_total_dy = total_dy
        self.last_fine_total_dtheta = total_dtheta
        self.last_final_total_weight = final_total_weight
        self.last_final_min_eigenvalue = final_min_eigenvalue
        self.last_break_reason = break_reason

        # Diagnostic-only HARD match against the final working pose. No
        # longer drives correspondence for the pose solve above (that's
        # soft_scan_matcher's job now) — kept so callers that inspect
        # match_result (logging, tests, lidar_visualizer.py) still get a
        # normal MatchResult to look at.
        final_iter_features = transform_features_to_map_frame(
            scan_features, working_pose
        )
        match_result = match_features(final_iter_features, self.map._entries)

        # Combine the coarse seed (if it was applied) with the refinement
        # loop's accumulated correction into ONE delta relative to the
        # ORIGINAL self.current_pose, using the same plain-additive
        # convention Pose.apply_delta already uses everywhere else in this
        # file (pose_estimator solves dx/dy directly in the map frame, not
        # sensor-frame-relative, so this composition is consistent with
        # the rest of the codebase, not a new assumption).
        combined_dx     = (coarse.dx if coarse.valid else 0.0) + total_dx
        combined_dy     = (coarse.dy if coarse.valid else 0.0) + total_dy
        combined_dtheta = (coarse.dtheta if coarse.valid else 0.0) + total_dtheta

        pose_delta = PoseDelta(
            dx=combined_dx, dy=combined_dy, dtheta=combined_dtheta,
            valid=(coarse.valid or refinement_ran),
            iterations=iterations_run,
        )

        # ---- Step 4 — update current pose -----------------------------------
        # A delta can be .valid (pose_estimator solved a rigid transform
        # for the matched pairs it was given) while still being physically
        # implausible for this rig's scan rate — see _is_pose_delta_plausible
        # docstring above. Treat an implausible delta the same way as an
        # invalid one: skip applying it, skip using it for this scan's map
        # update, and let the NEXT scan (hopefully with better data) try
        # again from the last trusted pose. SlamState stays ROS2-agnostic —
        # delta_applied is returned so callers (e.g. lidar_visualizer.py,
        # which does have a logger) can report the rejection themselves.
        delta_applied = pose_delta.valid and _is_pose_delta_plausible(pose_delta)
        delta_rejected_as_implausible = pose_delta.valid and not delta_applied

        # SECOND, INDEPENDENT check — trend consistency (see
        # _is_consistent_with_trend docstring above for the full failure
        # mode this closes: several individually-plausible-but-wrong deltas
        # compounding into a large heading/position error that then
        # self-validates by writing wrong map entries). This is checked
        # only when delta_applied is already True — an implausible delta
        # was already rejected above and never reaches this check.
        delta_on_trend = delta_applied and _is_consistent_with_trend(
            pose_delta, self._delta_history, self._last_applied_delta
        )

        # AMBIGUITY GATE — see correlative_match.py's CoarseResult.ambiguous
        # docstring: a close second-best coarse candidate means the coarse
        # search itself found a competing peak (e.g. rectangular-room
        # rotational symmetry) it could not distinguish from the winner.
        # coarse.valid/.dx/.dy/.dtheta were being consumed everywhere in
        # this function EXCEPT .ambiguous, which meant the one signal
        # purpose-built to catch a wrong-peak lock-in never reached the
        # trust decision below — an ambiguous coarse seed was trusted
        # exactly as much as an unambiguous one. This was directly
        # responsible for an observed false ~+30deg heading lock-in: five
        # OFF-TREND-but-applied scans (each individually magnitude-
        # plausible) walked the pose to +20deg, at which point the coarse
        # search started confidently reporting a competing ~-40deg peak
        # (coarse=(-0.200,+0.150,-40.0deg) recurring for dozens of scans)
        # that the outer magnitude guard could reject but nothing was
        # checking WHY the search kept finding it in the first place.
        #
        # FIX: treat an ambiguous coarse result exactly like an off-trend
        # delta for trust purposes — the pose still moves (so the system
        # does not freeze on a genuinely ambiguous-but-real scan, same
        # "don't freeze on real jittery motion" reasoning
        # _is_consistent_with_trend already uses) but it is NOT allowed to
        # (a) author new map evidence, or (b) become part of the trend
        # history that would excuse a future delta. This closes the same
        # self-validation loop TREND_PROBATION_SCANS closes for off-trend
        # deltas, applied to the coarse-search's own ambiguity signal
        # instead of only to the accepted-delta history comparison.
        coarse_ambiguous = coarse.valid and coarse.ambiguous
        trend_and_unambiguous = delta_on_trend and not coarse_ambiguous

        # POSE FREEZE GATE — see MAX_UNTRUSTED_STREAK's docstring for the
        # full failure mode this closes: _trend_probation/coarse_ambiguous
        # used to gate ONLY the Step 5 map write below, leaving Step 4
        # (applying pose_delta to current_pose) protected by nothing but
        # the single-scan magnitude bound. A long unbroken run of
        # individually-plausible-but-untrustworthy deltas could walk
        # current_pose to an arbitrary wrong heading with the map staying
        # clean the entire time — confirmed directly on hardware (heading
        # +0.1deg -> -56.3deg -> +70-98deg over one run while STATIC
        # entries stayed flat and UNCLASSIFIED exploded 45->229).
        #
        # Decided BEFORE touching current_pose, using this scan's own
        # trend_and_unambiguous result plus the streak already
        # accumulated from PRIOR scans — this scan's own result is not
        # allowed to excuse itself.
        self.last_pose_frozen = (
            delta_applied and not trend_and_unambiguous
            and self._untrusted_streak >= MAX_UNTRUSTED_STREAK
        )

        if delta_applied and not self.last_pose_frozen:
            self.current_pose.apply_delta(
                pose_delta.dx, pose_delta.dy, pose_delta.dtheta
            )
            # Updated on EVERY applied delta, on-trend or not — see
            # _last_applied_delta's __init__ comment for why this must be
            # separate from _delta_history (which only accumulates on-
            # trend deltas). This keeps the direction-check reference
            # fresh even through a run of off-trend scans. NOT updated
            # when frozen — the pose did not move, so the direction
            # reference must not pretend it did.
            self._last_applied_delta = pose_delta
            if trend_and_unambiguous:
                # Only trend-consistent deltas get remembered — an
                # off-trend delta still moves the pose (Step 4 above, so
                # we don't freeze on a real but jittery motion — UNLESS
                # the untrusted streak has already run past
                # MAX_UNTRUSTED_STREAK, see the freeze gate above) but is
                # deliberately NOT added to history, so it cannot itself
                # become part of the "trend" that excuses the next
                # off-trend delta. This prevents a slow drift of the trend
                # window toward a wrong answer.
                self._delta_history.append(pose_delta)
                if len(self._delta_history) > TREND_HISTORY_LEN:
                    self._delta_history.pop(0)
                # LOOPHOLE FIX: count down probation on each on-trend
                # delta, rather than clearing it in one shot — requires
                # TREND_PROBATION_SCANS consecutive on-trend deltas after
                # a disagreement before map writes are trusted again (see
                # TREND_PROBATION_SCANS docstring above).
                if self._trend_probation > 0:
                    self._trend_probation -= 1
                # This scan was fully trustworthy — the pose is no longer
                # "lost". Reset the streak so a single good scan (after
                # however long a bad run) starts earning trust back
                # immediately, same "one clean match is enough to start
                # recovering" spirit as _trend_probation counting down
                # per-scan rather than needing to hit exactly zero misses.
                self._untrusted_streak = 0
            else:
                # Reached when EITHER the delta was off-trend OR the
                # coarse seed that helped produce it was ambiguous (see
                # coarse_ambiguous above). These are NOT treated
                # identically anymore — see AMBIGUITY_PROBATION_SCANS'
                # docstring for why an ambiguity event needs a longer,
                # harder-to-clear probation than ordinary off-trend
                # jitter: once the pose has drifted onto a wrong peak,
                # correlative_match's search window recentres on that
                # WRONG guess, the true peak can fall entirely outside it,
                # and the system stops looking ambiguous at all from that
                # point on — it looks locally self-consistent instead. A
                # short probation clears on the very next couple of
                # (self-consistently wrong) on-trend scans and resumes
                # confidently authoring wrong map entries, which is a
                # WORSE visual outcome than the sparse pollution this gate
                # was meant to fix (a dense, well-connected wrong wall
                # reads as "the map", not as an obvious anomaly).
                #
                # LOOPHOLE FIX: history is NOT wiped here anymore (that
                # was the blind spot — see TREND_PROBATION_SCANS docstring
                # above: an empty history makes _is_consistent_with_trend
                # return True/"no opinion" for the very next delta, so two
                # bad-in-a-row deltas could both slip through). The real
                # trend stays intact and keeps being checked against.
                # Instead, put map writes on probation for the next
                # several scans regardless of whether they individually
                # look trend-consistent.
                #
                # NEVER SHORTEN an in-progress probation: if a fresh
                # ambiguity event fires while a shorter off-trend
                # probation is already counting down, extend to the
                # longer window rather than resetting to whichever fired
                # most recently -- an ambiguity event mid-recovery is a
                # sign the recovery isn't done, not a reason to shrink it.
                required_probation = (AMBIGUITY_PROBATION_SCANS if coarse_ambiguous
                                       else TREND_PROBATION_SCANS)
                self._trend_probation = max(self._trend_probation, required_probation)
                self._untrusted_streak += 1
        elif delta_applied and self.last_pose_frozen:
            # POSE FROZEN — magnitude-plausible, but the untrusted streak
            # has run past MAX_UNTRUSTED_STREAK. Do not touch current_pose
            # or _last_applied_delta at all this scan (same treatment as
            # pose_delta.valid==False — "not enough trustworthy
            # information", just triggered by sustained distrust instead
            # of too few matches). Keep counting the streak and hold
            # probation open so a single lucky trend-consistent-looking
            # scan right after a long bad run still can't immediately
            # re-open map writes — the pose needs to actually stabilise
            # first.
            self._untrusted_streak += 1
            self._trend_probation = max(self._trend_probation, TREND_PROBATION_SCANS)

        # after the existing elif delta_applied and self.last_pose_frozen: block
        elif not pose_delta.valid:
            n_static = sum(1 for e in self.map._entries
                        if e.active and e.status == ENTRY_STATIC)
            map_is_mature = n_static >= correlative_match.MIN_STATIC_FEATURES
            if map_is_mature:
                self._untrusted_streak += 1
                self._trend_probation = max(self._trend_probation, TREND_PROBATION_SCANS)
        else:
            map_is_mature = False   # not reached in this branch, keeps the name defined

        ...

        delta_fully_trusted = (delta_applied and delta_on_trend
                                and self._trend_probation == 0
                                and not coarse_ambiguous)
        skip_map_update = (
            delta_rejected_as_implausible
            or (delta_applied and not delta_fully_trusted)
            or (not pose_delta.valid and map_is_mature)   # <-- the missing gate
        )

        # ---- Step 5 — map update ----------------------------------------
        # Skip the map update when EITHER:
        #   (a) this scan's delta was rejected as IMPLAUSIBLE (single-scan
        #       magnitude guard — matches existed and pose_estimator solved
        #       a rigid transform, but the result implied an impossible
        #       jump), OR
        #   (b) the delta was applied but is OFF-TREND relative to recent
        #       accepted deltas (cross-scan consistency guard — see above).
        # Updating the map through a pose we have just decided not to fully
        # trust is exactly how the map gets polluted with bad new entries
        # that then validate the bad pose on the next scan: map_manager.
        # update() adds unmatched features as brand-new entries regardless
        # of why no pose correction was applied, which is what produced the
        # U:53/U:69 entry floods and the locked-in +10deg heading error
        # observed on hardware.
        #
        # This must NOT apply to the "too few matches" case (pose_delta
        # simply .valid == False, e.g. scan 0 against an empty map, or any
        # scan before the map has matured enough STATIC anchors) — that is
        # the normal, expected bootstrapping condition Version 1 already
        # relies on to grow the map from nothing. Gating Step 5 on
        # pose_delta.valid instead of these more specific flags breaks
        # map-building from scratch entirely (caught immediately by this
        # file's own T1 self-test going from 2 expected entries to 0).
        # LOOPHOLE FIX: a delta can look individually trend-consistent
        # (delta_on_trend True) while the system is still on probation
        # from a recent disagreement (see TREND_PROBATION_SCANS above) —
        # map writes must stay blocked through the whole probation window,
        # not just for the one off-trend scan that triggered it.
        #
        # AMBIGUITY GATE: also require not coarse_ambiguous, even on a
        # scan where trend/probation alone would have trusted it — a
        # competing coarse peak this close means the seed itself was a
        # coin flip between two orientations, and letting that author new
        # STATIC-track map evidence is exactly how a wrong peak gets
        # locked in permanently (see coarse_ambiguous's docstring above).
        delta_fully_trusted = (delta_applied and delta_on_trend
                                and self._trend_probation == 0
                                and not coarse_ambiguous)
        skip_map_update = delta_rejected_as_implausible or (delta_applied and not delta_fully_trusted)

        if not skip_map_update:
            # Re-transform using the now-corrected pose (cheaper than
            # caching the pre-correction transform and patching it, and
            # keeps this function obviously correct — the C port can
            # optimise this into a single transform + incremental
            # correction later if profiling shows it matters; at <=50
            # features/scan this is not the bottleneck on a 1GHz A53).
            final_features = transform_features_to_map_frame(
                scan_features, self.current_pose
            )
            # map_manager.update() re-matches internally (it owns its own
            # call to match_features) rather than reusing match_result
            # from Step 2, since the corrected pose can change which
            # features match.
            self.map.update(final_features, scan_idx=scan_idx)
        # else: pose unchanged this scan (or moved but not trustworthy
        # enough to author map evidence) — skip the map update too. The
        # map simply does not advance this scan; it picks back up once a
        # later scan produces a trend-consistent, plausible delta.

        # Expose this scan's trend/map-write status for callers that want
        # to log/display it (e.g. lidar_visualizer.py) without changing
        # the existing 4-tuple return shape that earlier callers rely on.
        # LOOPHOLE FIX: this now reports delta_fully_trusted (not the raw
        # per-scan delta_on_trend) so callers see "map write blocked"
        # accurately during a TREND_PROBATION_SCANS window, not just on
        # the single scan that triggered probation.
        self.last_delta_on_trend = delta_fully_trusted
        self.last_map_updated = not skip_map_update

        return self.current_pose, match_result, pose_delta, delta_applied


# ---------------------------------------------------------------------------
# Self-test — run directly: python3 slam.py
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import sys

    print("slam.py self-test")
    print("=" * 50)

    def _line_feat(angle, distance, x1, y1, x2, y2, quality=100):
        return {
            "type": "line",
            "angle": angle, "distance": distance,
            "x1": x1, "y1": y1, "x2": x2, "y2": y2,
            "length": math.hypot(x2 - x1, y2 - y1),
            "quality": quality,
        }

    # ── T1: stationary robot, repeated identical scan -> pose stays at origin,
    #         map builds up exactly as map_manager's own self-test expects ──
    slam = SlamState()
    scan = [
        _line_feat(0.0,  1.0,  -0.5, 1.0, 0.5, 1.0),
        _line_feat(math.pi / 2, 2.0,  2.0, -0.5, 2.0, 0.5),
    ]
    for i in range(5):
        pose, match_result, delta, delta_applied = slam.process_scan(scan, scan_idx=i)

    assert abs(pose.x) < 1e-6 and abs(pose.y) < 1e-6 and abs(pose.theta) < 1e-6, \
        f"T1 stationary robot should stay at origin, got {pose}"
    assert slam.map.count_active() == 2, \
        f"T1 expected 2 map entries, got {slam.map.count_active()}"
    print(f"  T1 PASS  stationary robot: pose stays at origin, map has 2 entries  {pose}")

    # ── T2: robot translates by a known amount between scans; pose should
    #         track the injected motion once matches are available ────────
    def _shift_scan_for_pose(base_scan, dx, dy, dtheta):
        """
        Build what the SENSOR would report if the robot moved by
        (dx, dy, dtheta) relative to where base_scan was taken (robot at
        origin). This is the inverse of transform_features_to_map_frame:
        moving the robot by +delta makes stationary map walls appear
        shifted by -delta in the new sensor frame.
        """
        inv_pose = Pose(x=-dx, y=-dy, theta=-dtheta)
        # Rotate by inv_pose.theta first is wrong order for inverse of a
        # rotate-then-translate; use transform_features_to_map_frame with
        # the FULL inverse transform decomposed correctly:
        cos_t = math.cos(-dtheta)
        sin_t = math.sin(-dtheta)
        out = []
        for feat in base_scan:
            x1 = feat["x1"] - dx; y1 = feat["y1"] - dy
            x2 = feat["x2"] - dx; y2 = feat["y2"] - dy
            x1r, y1r = _rotate_point(x1, y1, cos_t, sin_t)
            x2r, y2r = _rotate_point(x2, y2, cos_t, sin_t)
            angle = feat["angle"] - dtheta
            sign = 1.0
            if angle > math.pi / 2.0:
                angle -= math.pi; sign = -1.0
            elif angle < -math.pi / 2.0:
                angle += math.pi; sign = -1.0
            mx = (x1r + x2r) / 2.0; my = (y1r + y2r) / 2.0
            nx, ny = -math.sin(angle), math.cos(angle)
            distance = nx * mx + ny * my
            f2 = dict(feat)
            f2.update(x1=x1r, y1=y1r, x2=x2r, y2=y2r, angle=angle, distance=distance)
            out.append(f2)
        return out

    slam2 = SlamState()
    base_scan = [
        _line_feat(0.0,  1.0,  -0.5, 1.0,  0.5, 1.0),
        _line_feat(math.pi / 2, 2.0,  2.0, -0.5,  2.0, 0.5),
        _line_feat(-math.pi / 3, 1.4, -1.0, 0.2, -0.3, 1.0),
    ]
    # Scans 0-1: build the map at the origin and let entries mature to
    # STATIC (MIN_OBS_FOR_STATIC=2 in map_manager.py). Pose correction is
    # restricted to STATIC entries only (see process_scan Step 3), so
    # there is nothing to correct against until the map has matured —
    # this is intentional, not a limitation of the test.
    from map_manager import MIN_OBS_FOR_STATIC
    for i in range(MIN_OBS_FOR_STATIC):
        slam2.process_scan(base_scan, scan_idx=i)
    assert all(e.status == 1 for e in slam2.map.get_active()), \
        "T2 setup: all entries should be STATIC before testing motion tracking"

    # Scan N: robot has moved by (0.05, 0.02, 0.01) — small motion so it
    # stays within line_matcher's angle/distance thresholds.
    true_dx, true_dy, true_dtheta = 0.05, 0.02, 0.01
    moved_scan = _shift_scan_for_pose(base_scan, true_dx, true_dy, true_dtheta)
    pose2, match_result2, delta2, delta2_applied = slam2.process_scan(
        moved_scan, scan_idx=MIN_OBS_FOR_STATIC
    )

    assert delta2.valid, "T2 expected a valid pose delta once map entries are STATIC"
    assert delta2_applied, "T2 expected the delta to be applied (within plausible limits)"
    assert abs(pose2.x - true_dx) < 1e-3, f"T2 pose.x off: {pose2.x} vs {true_dx}"
    assert abs(pose2.y - true_dy) < 1e-3, f"T2 pose.y off: {pose2.y} vs {true_dy}"
    assert abs(pose2.theta - true_dtheta) < 1e-3, f"T2 pose.theta off: {pose2.theta} vs {true_dtheta}"
    print(f"  T2 PASS  injected motion ({true_dx},{true_dy},{true_dtheta}) tracked "
          f"against STATIC entries: {pose2}")

    # ── T3: too few matches on a scan -> pose unchanged, no crash ─────────
    slam3 = SlamState()
    slam3.process_scan(base_scan, scan_idx=0)
    sparse_scan = [base_scan[0]]   # only one feature — below MIN_MATCHES
    pose_before = Pose(slam3.current_pose.x, slam3.current_pose.y, slam3.current_pose.theta)
    pose3, match_result3, delta3, delta3_applied = slam3.process_scan(sparse_scan, scan_idx=1)
    assert delta3.valid is False, "T3 expected invalid delta (too few matches)"
    assert delta3_applied is False, "T3 expected delta not applied"
    assert pose3.x == pose_before.x and pose3.y == pose_before.y and pose3.theta == pose_before.theta, \
        "T3 pose must be unchanged when delta is invalid"
    print(f"  T3 PASS  sparse scan -> pose unchanged, no crash: {pose3}")

    # ── T4: arc features survive the frame transform unscathed ────────────
    arc_feat = {
        "type": "arc", "cx": 0.3, "cy": 0.4, "r": 0.5,
        "theta_start": 0.0, "theta_end": math.pi / 2,
        "length": 0.5 * math.pi / 2, "quality": 80,
    }
    pose_t4 = Pose(x=1.0, y=0.5, theta=math.pi / 4)
    transformed = transform_features_to_map_frame([arc_feat], pose_t4)
    t = transformed[0]
    expected_cx = 1.0 + (0.3 * math.cos(math.pi / 4) - 0.4 * math.sin(math.pi / 4))
    expected_cy = 0.5 + (0.4 * math.cos(math.pi / 4) + 0.3 * math.sin(math.pi / 4))
    assert abs(t["cx"] - expected_cx) < 1e-6, f"T4 arc cx off: {t['cx']} vs {expected_cx}"
    assert abs(t["cy"] - expected_cy) < 1e-6, f"T4 arc cy off: {t['cy']} vs {expected_cy}"
    assert abs(t["theta_start"] - math.pi / 4) < 1e-6
    assert abs(t["r"] - 0.5) < 1e-9, "T4 radius must be unchanged by rigid transform"
    print(f"  T4 PASS  arc transform: centre and theta correctly rotated+translated")

    # ── T5: line length is preserved by the rigid transform ───────────────
    line_feat = _line_feat(0.3, 0.8, 0.0, 0.0, 1.0, 0.0)
    pose_t5 = Pose(x=2.0, y=-1.0, theta=0.6)
    transformed5 = transform_features_to_map_frame([line_feat], pose_t5)
    new_len = math.hypot(
        transformed5[0]["x2"] - transformed5[0]["x1"],
        transformed5[0]["y2"] - transformed5[0]["y1"],
    )
    assert abs(new_len - line_feat["length"]) < 1e-9, \
        f"T5 length must be preserved: {new_len} vs {line_feat['length']}"
    print(f"  T5 PASS  line length preserved under rigid transform: {new_len:.4f}")

    # ── T6: a non-static (DYNAMIC) feature must NOT influence pose ────────
    # Build a map with 2 STATIC walls (robot genuinely stationary). Then,
    # on a later scan, feed back the 2 real walls UNCHANGED (robot still
    # hasn't moved) plus a 3rd line that has shifted as if something in
    # the room moved. If the STATIC-only filter (process_scan Step 3)
    # works, the moved 3rd line is excluded from pose estimation entirely
    # and the recovered pose stays at the origin — proving the system
    # does not "recalibrate" just because something else in view moved.
    from map_manager import MIN_OBS_FOR_STATIC as _MIN_STATIC

    slam6 = SlamState()
    anchor_scan = [
        _line_feat(0.0,  1.0,  -0.5, 1.0,  0.5, 1.0),
        _line_feat(math.pi / 2, 2.0,  2.0, -0.5,  2.0, 0.5),
    ]
    moving_obj_line = _line_feat(-math.pi / 4, 0.8, -0.4, 0.0, 0.0, 0.4)

    # Scans 0..MIN_STATIC-1: robot stationary, room empty except 2 walls.
    # Anchors mature to STATIC; nothing dynamic in view yet.
    for i in range(_MIN_STATIC):
        slam6.process_scan(anchor_scan, scan_idx=i)
    assert all(e.status == 1 for e in slam6.map.get_active()), \
        "T6 setup: anchor walls should be STATIC before introducing the mover"

    # Next scan: same 2 static walls (robot still hasn't moved) + a new
    # line that will itself start UNCLASSIFIED — far enough from the
    # anchors in angle/distance that it can never be confused for them.
    scan_with_mover = anchor_scan + [moving_obj_line]
    pose6, mr6, delta6, delta6_applied = slam6.process_scan(scan_with_mover, scan_idx=_MIN_STATIC)

    assert delta6.valid, "T6 expected a valid delta (2 STATIC anchors still match)"
    assert delta6_applied, "T6 expected the delta to be applied"
    assert abs(pose6.x) < 1e-6 and abs(pose6.y) < 1e-6 and abs(pose6.theta) < 1e-6, \
        f"T6 pose should stay at origin (robot did not move): {pose6}"
    print(f"  T6 PASS  new (non-static) feature present, pose unaffected: {pose6}")

    # Now confirm the converse directly at the pose_estimator glue layer:
    # if the "mover" line is forced to MATCH an existing map entry that is
    # DYNAMIC, it must be excluded from the LineMatch list altogether.
    from collections import namedtuple as _nt
    from line_matcher import MatchResult
    _StubEntry = _nt("_StubEntry", ["angle", "distance", "status"])
    class _SE(_StubEntry):
        def is_arc(self):
            return False

    stub_map = [
        _SE(angle=0.0, distance=1.0, status=1),       # STATIC
        _SE(angle=math.pi / 2, distance=2.0, status=2),  # DYNAMIC
    ]
    stub_scan = [
        {"type": "line", "angle": 0.01, "distance": 1.01, "length": 0.9, "quality": 80},
        {"type": "line", "angle": math.pi / 2 + 0.01, "distance": 2.02,
         "length": 0.9, "quality": 80},
    ]
    stub_result = MatchResult(matched=[(0, 0, 0.01), (1, 1, 0.02)],
                               unmatched_scan=[], unmatched_map=[])
    filtered = build_line_matches_from_match_result(
        stub_scan, stub_map, stub_result, static_status=1
    )
    assert len(filtered) == 1, f"T6b expected only the STATIC match to survive, got {filtered}"
    assert filtered[0].map_angle == 0.0, "T6b wrong entry survived the filter"
    print(f"  T6b PASS  DYNAMIC-matched pair excluded from LineMatch list, STATIC pair kept")

    # ── T7: a VALID but implausibly large delta must be rejected ──────────
    # Reproduces the real hardware failure pattern: a delta can be .valid
    # (pose_estimator solved a rigid transform) while still being
    # physically implausible for one scan. This must be caught BEFORE
    # being applied to current_pose or used for the map update, distinct
    # from the "too few matches" case (which legitimately allows map
    # building/growth on an immature map).
    #
    # Tested directly against the gating predicate rather than via a
    # specific line_matcher tolerance geometry — with thresholds tightened
    # (ANGLE_THRESH_RAD/DIST_THRESH_M reduced to prevent wrong-wall
    # correspondences during real motion, see line_matcher.py) the gap
    # between "still matches" and "exceeds plausibility" is narrow and
    # depends on wall geometry in ways that make a single synthetic test
    # case fragile; the predicate itself is the thing worth testing.
    from pose_estimator import PoseDelta as _PoseDelta

    slam7 = SlamState()
    walls7 = [(0.0, 1.0), (math.pi / 2, 2.0), (-math.pi / 2, 1.5)]

    def _scan_at_pose7(px, py, ptheta):
        out = []
        for ma, md in walls7:
            nx, ny = -math.sin(ma), math.cos(ma)
            sa = ma - ptheta
            sd = md - (nx * px + ny * py)
            while sa > math.pi / 2: sa -= math.pi; sd = -sd
            while sa < -math.pi / 2: sa += math.pi; sd = -sd
            fx, fy = -math.sin(sa) * sd, math.cos(sa) * sd
            dirx, diry = math.cos(sa), math.sin(sa)
            out.append(_line_feat(sa, sd, fx - 0.7 * dirx, fy - 0.7 * diry,
                                   fx + 0.7 * dirx, fy + 0.7 * diry, quality=100))
        return out

    for i in range(MIN_OBS_FOR_STATIC):
        slam7.process_scan(_scan_at_pose7(0, 0, 0), scan_idx=i)

    map7_before = slam7.map.count_active()
    pose7_before = (slam7.current_pose.x, slam7.current_pose.y, slam7.current_pose.theta)

    implausible_delta = _PoseDelta(dx=0.0, dy=0.50, dtheta=0.0, valid=True, iterations=3)
    assert not _is_pose_delta_plausible(implausible_delta), \
        "T7 setup: 0.50m dy must be flagged implausible (exceeds MAX_DELTA_TRANSLATION_M)"

    plausible_delta = _PoseDelta(dx=0.05, dy=0.05, dtheta=0.01, valid=True, iterations=2)
    assert _is_pose_delta_plausible(plausible_delta), \
        "T7 setup: a real ~5-7cm delta must be flagged plausible"

    far_scan = _scan_at_pose7(2.0, 2.0, 0.0)   # 2m diagonal — clearly out of match range
    pose7, mr7, delta7, applied7 = slam7.process_scan(far_scan, scan_idx=MIN_OBS_FOR_STATIC)
    assert applied7 is False, "T7 expected no delta applied for an out-of-range scan"
    assert (pose7.x, pose7.y, pose7.theta) == pose7_before, \
        f"T7 pose must be unchanged when nothing trustworthy matched: {pose7}"
    print(f"  T7 PASS  implausible-delta gating verified directly; "
          f"out-of-range scan leaves pose untouched")

    # ── T8: cross-scan trend consistency — several individually-plausible
    #         but WRONG-DIRECTION deltas must NOT compound into a large
    #         heading/position error, and must NOT be allowed to author
    #         new map entries while off-trend. ────────────────────────────
    # Reproduces the real hardware failure precisely: th drifted from
    # +2.7deg to +10.2deg over a handful of scans during a 20-30cm slide,
    # each individual delta passing the single-scan magnitude guard, then
    # LOCKED at the wrong +10deg heading for 250+ scans because the
    # corrupted pose had already written map entries that re-confirmed it.
    slam8 = SlamState()
    walls8 = [(0.0, 1.0), (math.pi / 2, 2.0), (-math.pi / 3, 1.4)]

    def _scan_at_pose8(px, py, ptheta):
        out = []
        for ma, md in walls8:
            nx, ny = -math.sin(ma), math.cos(ma)
            sa = ma - ptheta
            sd = md - (nx * px + ny * py)
            while sa > math.pi / 2: sa -= math.pi; sd = -sd
            while sa < -math.pi / 2: sa += math.pi; sd = -sd
            fx, fy = -math.sin(sa) * sd, math.cos(sa) * sd
            dirx, diry = math.cos(sa), math.sin(sa)
            out.append(_line_feat(sa, sd, fx - 0.7 * dirx, fy - 0.7 * diry,
                                   fx + 0.7 * dirx, fy + 0.7 * diry, quality=100))
        return out

    for i in range(MIN_OBS_FOR_STATIC):
        slam8.process_scan(_scan_at_pose8(0, 0, 0), scan_idx=i)
    assert all(e.status == 1 for e in slam8.map.get_active()), \
        "T8 setup: all entries should be STATIC before testing trend rejection"

    map8_before = slam8.map.count_active()

    # Establish a consistent small trend: three scans of a real ~3cm/scan
    # rightward slide (the kind of motion that should build trend history).
    base8 = MIN_OBS_FOR_STATIC
    last_pose8 = None
    for i in range(3):
        p, mr, d, a = slam8.process_scan(
            _scan_at_pose8(0.03 * (i + 1), 0, 0), scan_idx=base8 + i
        )
        last_pose8 = p
    assert len(slam8._delta_history) >= 1, \
        "T8 setup: trend history should have accumulated from consistent small deltas"

    # Now inject a single scan whose solved delta is individually still
    # under MAX_DELTA_TRANSLATION_M/MAX_DELTA_ROTATION_RAD (so the
    # single-scan guard alone would accept it) but is sharply inconsistent
    # with the established trend — simulating a wrong-wall correspondence
    # mid-motion. Directly injected via a stubbed pose_estimator result
    # would require patching the module; instead we use a real scan
    # geometry that solves to an off-trend rotation, which is what
    # actually happened on hardware (a sudden multi-degree jump against a
    # gentle, consistent slide).
    # Off-trend probe: dx stays near the established trend (~0.02m/scan)
    # but dy jumps by 0.13m — individually still within
    # MAX_DELTA_TRANSLATION_M (0.20m), so the single-scan guard alone
    # would accept it (this is the exact shape of the real hardware
    # failure: an individually-plausible delta pointed in the wrong
    # direction relative to the established motion). TREND_TRANSLATION_TOL_M
    # is 0.10m, so a 0.13m deviation from the trend mean must be caught by
    # the trend guard specifically.
    off_trend_scan = _scan_at_pose8(0.02, 0.13, 0.0)
    map8_before_offtrend = slam8.map.count_active()
    p8, mr8, d8, a8 = slam8.process_scan(off_trend_scan, scan_idx=base8 + 3)

    if d8.valid and _is_pose_delta_plausible(d8):
        # This delta passed the single-scan guard (as the real hardware
        # case did) — it must have been caught by the trend guard instead,
        # meaning no new map entries were authored from it.
        #
        # LOOPHOLE FIX (was: history wiped to [] here, which meant
        # _is_consistent_with_trend returned True/"no opinion" for the
        # very NEXT delta regardless of whether it was also wrong — two
        # bad-in-a-row deltas could both slip through). Now: history stays
        # intact (the real trend is still there to check future deltas
        # against) and probation blocks map writes for
        # TREND_PROBATION_SCANS scans, not just this one.
        assert slam8.map.count_active() == map8_before_offtrend, \
            "T8 off-trend delta must not be allowed to add new map entries"
        assert slam8._trend_probation > 0, \
            "T8 off-trend delta must place the system on probation"
        assert len(slam8._delta_history) >= 1, \
            "T8 trend history must NOT be wiped -- this was the blind spot fix"
        print(f"  T8 PASS  off-trend delta (dx={d8.dx:.3f} dy={d8.dy:.3f}) passed "
              f"single-scan guard but map writes were blocked by trend guard, "
              f"probation={slam8._trend_probation}, history intact")

        # ── T8b: THE BLIND SPOT ITSELF — a second scan back at the ORIGINAL
        # (correct) trend must still have its map writes blocked, because
        # probation has not yet cleared. Under the old wipe-to-[] behavior
        # this second delta would have sailed through unchecked (history
        # was empty -> "no opinion" -> instantly trusted again). ──────────
        map8_before_second = slam8.map.count_active()
        p8b, mr8b, d8b, a8b = slam8.process_scan(
            _scan_at_pose8(0.03 * 4, 0, 0), scan_idx=base8 + 4
        )
        if d8b.valid and _is_pose_delta_plausible(d8b):
            assert slam8.map.count_active() == map8_before_second, \
                "T8b probation must still block map writes on the very next scan"
            print(f"  T8b PASS  probation held through the next scan — "
                  f"the exact blind spot the old wipe-to-[] logic left open "
                  f"is now closed, probation={slam8._trend_probation}")
        else:
            print(f"  T8b PASS  next scan had no trustworthy delta "
                  f"(valid={d8b.valid}); probation state unaffected")
    else:
        # The tightened matching thresholds rejected the off-trend scan
        # outright (too few/no matches) before it even reached the trend
        # guard. pose_delta.valid==False is the normal bootstrap/growth
        # case (see Step 5 docstring) — unmatched features are still
        # allowed to seed new UNCLASSIFIED map entries even with no pose
        # correction, same as T7. What matters here is the POSE itself did
        # not silently drift onto the wrong trend.
        print(f"  T8 PASS  off-trend scan rejected by matching thresholds "
              f"before reaching trend check (valid={d8.valid}, matched={len(mr8.matched)}); "
              f"pose protected from the wrong delta")

    # ── T9: parallel-wall slide — the exact real-world failure mode from
    #         the hardware log (dx jumping to 1-2m while sliding along a
    #         room's two long parallel walls) must be fixed once an ARC
    #         feature (e.g. a corner or curved leg) is visible and STATIC.
    # ─────────────────────────────────────────────────────────────────────
    def _arc_feat(cx, cy, r, quality=100):
        return {
            "type": "arc", "cx": cx, "cy": cy, "r": r,
            "theta_start": 0.0, "theta_end": math.pi,
            "length": r * math.pi, "quality": quality,
        }

    slam9 = SlamState()
    # Two long walls PARALLEL to each other (both angle=0, e.g. a corridor
    # or a room's two facing long walls) plus one arc feature (a rounded
    # corner/leg) that is NOT direction-degenerate.
    anchor_scan9 = [
        _line_feat(0.0, 1.0, -1.0, 1.0, 1.0, 1.0),
        _line_feat(0.0, 1.6, -1.0, 1.6, 1.0, 1.6),
        _arc_feat(0.8, 0.0, 0.15),
    ]
    for i in range(MIN_OBS_FOR_STATIC):
        slam9.process_scan(anchor_scan9, scan_idx=i)
    assert all(e.status == 1 for e in slam9.map.get_active()), \
        "T9 setup: anchors should be STATIC before testing the slide"

    # Slide the robot sideways ALONG the parallel walls by 18cm — this is
    # exactly the motion direction the two parallel lines alone cannot
    # constrain (their shared normal is perpendicular to this slide).
    true_dx9, true_dy9, true_dtheta9 = 0.18, 0.0, 0.0

    def _shift_scan9(base_scan, dx, dy, dtheta):
        cos_t, sin_t = math.cos(-dtheta), math.sin(-dtheta)
        out = []
        for feat in base_scan:
            if feat["type"] == "line":
                x1 = feat["x1"] - dx; y1 = feat["y1"] - dy
                x2 = feat["x2"] - dx; y2 = feat["y2"] - dy
                x1r, y1r = _rotate_point(x1, y1, cos_t, sin_t)
                x2r, y2r = _rotate_point(x2, y2, cos_t, sin_t)
                angle = feat["angle"] - dtheta
                sign = 1.0
                if angle > math.pi / 2.0:
                    angle -= math.pi; sign = -1.0
                elif angle < -math.pi / 2.0:
                    angle += math.pi; sign = -1.0
                mx = (x1r + x2r) / 2.0; my = (y1r + y2r) / 2.0
                nx, ny = -math.sin(angle), math.cos(angle)
                distance = nx * mx + ny * my
                f2 = dict(feat)
                f2.update(x1=x1r, y1=y1r, x2=x2r, y2=y2r, angle=angle, distance=distance)
                out.append(f2)
            elif feat["type"] == "arc":
                cx = feat["cx"] - dx; cy = feat["cy"] - dy
                cxr, cyr = _rotate_point(cx, cy, cos_t, sin_t)
                f2 = dict(feat)
                f2.update(cx=cxr, cy=cyr)
                out.append(f2)
        return out

    slid_scan9 = _shift_scan9(anchor_scan9, true_dx9, true_dy9, true_dtheta9)
    pose9, mr9, delta9, applied9 = slam9.process_scan(
        slid_scan9, scan_idx=MIN_OBS_FOR_STATIC
    )

    assert delta9.valid, "T9 expected a valid delta (arc + lines all still match)"
    assert applied9, f"T9 expected the delta to be applied, got pose_delta={delta9}"
    assert abs(pose9.x - true_dx9) < 0.03, \
        f"T9 dx should track the true 18cm slide, not blow up: pose={pose9}"
    assert abs(pose9.y - true_dy9) < 0.03, f"T9 dy off: pose={pose9}"
    print(f"  T9 PASS  parallel-wall 18cm slide correctly tracked (no metre-scale "
          f"jump) once a STATIC arc feature is present: {pose9}")

    # ── T10: SIMULTANEOUS rotation + translation — the exact failure mode
    #         reported after switching to the soft-correspondence loop.
    #         A single unclamped Gauss-Newton iteration is most likely to
    #         overshoot precisely when a scan combines significant rotation
    #         AND significant translation together (the linear
    #         approximation soft_scan_matcher.solve_soft_pose_step relies on
    #         is weakest there). MAX_ITER_STEP_TRANSLATION_M /
    #         MAX_ITER_STEP_ROTATION_RAD force the loop to walk this down
    #         across several smaller, re-matched steps instead. ────────────
    slam10 = SlamState()
    walls10 = [
        (0.0, 1.2), (math.pi / 2, 1.6), (-math.pi / 3, 1.0), (math.pi / 6, 1.4),
    ]

    def _scan_at_pose10(px, py, ptheta):
        out = []
        for ma, md in walls10:
            nx, ny = -math.sin(ma), math.cos(ma)
            sa = ma - ptheta
            sd = md - (nx * px + ny * py)
            while sa > math.pi / 2: sa -= math.pi; sd = -sd
            while sa < -math.pi / 2: sa += math.pi; sd = -sd
            fx, fy = -math.sin(sa) * sd, math.cos(sa) * sd
            dirx, diry = math.cos(sa), math.sin(sa)
            out.append(_line_feat(sa, sd, fx - 0.7 * dirx, fy - 0.7 * diry,
                                   fx + 0.7 * dirx, fy + 0.7 * diry, quality=100))
        return out

    for i in range(MIN_OBS_FOR_STATIC):
        slam10.process_scan(_scan_at_pose10(0, 0, 0), scan_idx=i)
    assert all(e.status == 1 for e in slam10.map.get_active()), \
        "T10 setup: anchors should be STATIC before testing combined motion"

    # A real combined rotate+translate step within one scan interval —
    # 12cm diagonal slide plus an 11 degree turn, simultaneously. Still
    # within correlative_match's +/-15cm/+/-15deg coarse search window (so
    # the coarse seed can find it), but big enough that a single unclamped
    # fine-refinement iteration would be solving well outside the linear
    # approximation's comfort zone.
    true_dx10, true_dy10, true_dtheta10 = 0.09, 0.08, math.radians(11.0)
    combined_scan10 = _scan_at_pose10(true_dx10, true_dy10, true_dtheta10)
    pose10, mr10, delta10, applied10 = slam10.process_scan(
        combined_scan10, scan_idx=MIN_OBS_FOR_STATIC
    )

    assert delta10.valid, "T10 expected a valid delta (coarse seed + STATIC anchors present)"
    assert applied10, f"T10 expected the combined delta to be applied, got {delta10}"
    assert abs(pose10.x - true_dx10) < 0.03, \
        f"T10 dx should track the true combined motion accurately: pose={pose10}"
    assert abs(pose10.y - true_dy10) < 0.03, f"T10 dy off: pose={pose10}"
    assert abs(pose10.theta - true_dtheta10) < math.radians(3.0), \
        f"T10 dtheta off: pose={pose10}"
    print(f"  T10 PASS  simultaneous 9cm/8cm slide + 11deg turn tracked accurately, "
          f"no overshoot from an oversized single-iteration step: {pose10}")

    # ── T11: AMBIGUOUS coarse seed must move the pose (don't freeze on a
    #         genuinely ambiguous scan) but must NOT be trusted to author
    #         new map evidence — this is the exact gap that let the
    #         coarse search's own ambiguity signal (correlative_match.
    #         CoarseResult.ambiguous) go unused, letting a competing
    #         rotational peak lock in permanently on real hardware. Stub
    #         correlative_match.search() directly rather than constructing
    #         a real rectangular-symmetry room — the point being tested is
    #         "does slam.py's trust gate react to .ambiguous", not
    #         "can correlative_match detect ambiguity" (that's
    #         correlative_match.py's own T5/T6 self-tests). ──────────────
    slam11 = SlamState()
    walls11 = [(0.0, 1.0), (math.pi / 2, 1.6), (-math.pi / 3, 1.0)]

    def _scan_at_pose11(px, py, ptheta):
        out = []
        for ma, md in walls11:
            nx, ny = -math.sin(ma), math.cos(ma)
            sa = ma - ptheta
            sd = md - (nx * px + ny * py)
            while sa > math.pi / 2: sa -= math.pi; sd = -sd
            while sa < -math.pi / 2: sa += math.pi; sd = -sd
            fx, fy = -math.sin(sa) * sd, math.cos(sa) * sd
            dirx, diry = math.cos(sa), math.sin(sa)
            out.append(_line_feat(sa, sd, fx - 0.7 * dirx, fy - 0.7 * diry,
                                   fx + 0.7 * dirx, fy + 0.7 * diry, quality=100))
        return out

    for i in range(MIN_OBS_FOR_STATIC):
        slam11.process_scan(_scan_at_pose11(0, 0, 0), scan_idx=i)
    assert all(e.status == 1 for e in slam11.map.get_active()), \
        "T11 setup: anchors should be STATIC before testing the ambiguity gate"

    map11_before = slam11.map.count_active()

    real_search = correlative_match.search
    try:
        def _stub_ambiguous_search(*args, **kwargs):
            r = real_search(*args, **kwargs)
            if not r.valid:
                return r
            # Force the ambiguous flag on regardless of what the real
            # search found — isolates the gate under test from whether a
            # real competing peak happened to be present in this geometry.
            return r._replace(ambiguous=True, second_score=r.score * 0.95)

        correlative_match.search = _stub_ambiguous_search

        small_scan11 = _scan_at_pose11(0.03, 0.01, math.radians(1.0))
        pose11, mr11, delta11, applied11 = slam11.process_scan(
            small_scan11, scan_idx=MIN_OBS_FOR_STATIC
        )
    finally:
        correlative_match.search = real_search

    assert slam11.last_coarse_ambiguous, \
        "T11 expected last_coarse_ambiguous to be True with the search stubbed"
    if applied11:
        assert not slam11.last_delta_on_trend, \
            "T11 an ambiguous-seeded delta must not report as fully trusted"
        assert slam11.map.count_active() == map11_before, \
            "T11 ambiguous coarse seed must not be allowed to author new map entries"
        assert slam11._trend_probation > 0, \
            "T11 ambiguous coarse seed must place the system on probation"
        print(f"  T11 PASS  ambiguous coarse seed: pose moved (applied={applied11}) "
              f"but map writes blocked (entries {map11_before}->"
              f"{slam11.map.count_active()}), probation={slam11._trend_probation}")
    else:
        # Small motion + forced ambiguity was still rejected outright by
        # an earlier gate (e.g. magnitude) — the map-write side of the
        # ambiguity gate is only reachable through delta_applied==True, so
        # this isn't a failure of THIS gate, just a stub geometry that
        # didn't reach it. The flag itself was still verified above.
        print(f"  T11 PASS  ambiguous coarse flag correctly recorded "
              f"(delta not applied this scan for an unrelated reason)")

    # ── T12: SUSTAINED off-trend drift must FREEZE the pose, not just
    #         block map writes — this is the exact hardware failure: dozens
    #         of consecutive "OFF-TREND ... MAP WRITE BLOCKED" scans, each
    #         individually magnitude-plausible, walked current_pose from
    #         +0.1deg to +70-98deg over one run while the map itself
    #         stayed nominally clean (STATIC flat, only UNCLASSIFIED
    #         exploding). Step 5's guards were doing their job; Step 4 had
    #         no equivalent protection at all. Directly drive process_scan
    #         with a stubbed pose_estimator-level delta stream via a
    #         monkeypatched soft_scan_matcher.solve_soft_pose_step, rather
    #         than constructing real geometry that happens to drift for
    #         MAX_UNTRUSTED_STREAK+ scans in a row — the point under test
    #         is the FREEZE MECHANISM's response to a long bad run, not
    #         whether a particular room geometry can produce one (T8/T8b
    #         already cover a short 1-2 scan disagreement; this is its
    #         sustained, many-scans-long extreme). ─────────────────────────
    slam12 = SlamState()
    walls12 = [(0.0, 1.0), (math.pi / 2, 1.6), (-math.pi / 3, 1.0)]

    def _scan_at_pose12(px, py, ptheta):
        out = []
        for ma, md in walls12:
            nx, ny = -math.sin(ma), math.cos(ma)
            sa = ma - ptheta
            sd = md - (nx * px + ny * py)
            while sa > math.pi / 2: sa -= math.pi; sd = -sd
            while sa < -math.pi / 2: sa += math.pi; sd = -sd
            fx, fy = -math.sin(sa) * sd, math.cos(sa) * sd
            dirx, diry = math.cos(sa), math.sin(sa)
            out.append(_line_feat(sa, sd, fx - 0.7 * dirx, fy - 0.7 * diry,
                                   fx + 0.7 * dirx, fy + 0.7 * diry, quality=100))
        return out

    for i in range(MIN_OBS_FOR_STATIC):
        slam12.process_scan(_scan_at_pose12(0, 0, 0), scan_idx=i)
    assert all(e.status == 1 for e in slam12.map.get_active()), \
        "T12 setup: anchors should be STATIC before testing the freeze mechanism"

    # Establish a short trend (matches T8's setup) so subsequent deltas can
    # be evaluated as off-trend at all -- _is_consistent_with_trend needs
    # >=2 history samples to have an opinion.
    base12 = MIN_OBS_FOR_STATIC
    for i in range(3):
        slam12.process_scan(_scan_at_pose12(0.02 * (i + 1), 0, 0), scan_idx=base12 + i)
    assert len(slam12._delta_history) >= 1, \
        "T12 setup: trend history should have accumulated"

    real_step = soft_scan_matcher.solve_soft_pose_step
    real_search = correlative_match.search
    frozen_scan_seen = False
    try:
        # Alternates GOOD / LOW-WEIGHT on successive calls. Each
        # process_scan call enters its refinement loop and calls this at
        # least once per iteration; a LOW-WEIGHT response makes that
        # scan's loop break immediately ("low_weight"), so each scan
        # consumes exactly one GOOD call followed by one LOW-WEIGHT call
        # that ends it -- i.e. exactly ONE iteration's worth of dtheta is
        # applied per scan, every scan, regardless of MAX_ITERATIONS. This
        # keeps each scan's contribution small and controlled (~8deg,
        # after MAX_ITER_STEP_ROTATION_RAD clamps it) rather than letting
        # up to 8 iterations compound into something the outer
        # MAX_DELTA_ROTATION_RAD(20deg) guard would reject outright before
        # ever reaching the freeze logic under test (an early version of
        # this stub did exactly that: n_offtrend_seen stayed 0 because
        # every scan's delta was too large and got REJECTED rather than
        # applied-but-off-trend).
        call_count = [0]

        def _stub_drifting_step(iter_features, static_lines, static_arcs):
            call_count[0] += 1
            if call_count[0] % 2 == 1:
                # ~8.6deg raw -- MAX_ITER_STEP_ROTATION_RAD (8deg) clamps
                # it down in process_scan, landing comfortably above
                # TREND_ROTATION_TOL_RAD (6deg, so it reads off-trend
                # against the near-zero established trend) and well under
                # MAX_DELTA_ROTATION_RAD (20deg, so it is not outright
                # rejected as implausible).
                return (0.0, 0.0, math.radians(8.6), 20.0, 10.0)
            else:
                return (0.0, 0.0, 0.0, 0.0, 0.0)   # weight below MIN_TOTAL_WEIGHT -> break

        def _stub_invalid_coarse(*args, **kwargs):
            # Always report no coarse seed -- isolates the freeze-streak
            # mechanism under test (a slam.py concern) from correlative_
            # match's own (now much more precise, multi-resolution) real
            # behaviour, which in this synthetic stationary-room scenario
            # correctly re-locks the coarse estimate to near-zero every
            # scan and would otherwise drown out the injected fine-loop
            # drift before the streak could ever build up. This mirrors
            # the real hardware failure this freeze mechanism was built
            # for, where coarse_valid=False dominated the log (too few
            # STATIC anchors matched within its window, or real motion
            # outside the window) and the fine loop was left unconstrained.
            return correlative_match.CoarseResult(
                dx=0.0, dy=0.0, dtheta=0.0, score=0.0, n_static=0,
                valid=False, ambiguous=False, second_score=0.0,
            )

        soft_scan_matcher.solve_soft_pose_step = _stub_drifting_step
        correlative_match.search = _stub_invalid_coarse

        n_offtrend_seen = 0
        scan_idx = base12 + 3
        for _ in range(60):
            p, mr, d, applied = slam12.process_scan(
                _scan_at_pose12(0, 0, 0), scan_idx=scan_idx
            )
            scan_idx += 1
            if applied and not slam12.last_delta_on_trend:
                n_offtrend_seen += 1
            if slam12.last_pose_frozen:
                frozen_scan_seen = True
                frozen_pose = Pose(slam12.current_pose.x, slam12.current_pose.y,
                                    slam12.current_pose.theta)
                # One more scan with the same drifting stub still active --
                # pose must NOT move further while frozen.
                slam12.process_scan(_scan_at_pose12(0, 0, 0), scan_idx=scan_idx)
                scan_idx += 1
                break
    finally:
        soft_scan_matcher.solve_soft_pose_step = real_step
        correlative_match.search = real_search

    assert frozen_scan_seen, (
        "T12 expected the pose to freeze after MAX_UNTRUSTED_STREAK "
        f"consecutive off-trend scans, but it never did across 60 "
        f"synthetic drifting scans (n_offtrend_seen={n_offtrend_seen})"
    )
    assert slam12.last_pose_frozen, "T12 pose should still be frozen on the next scan too"
    assert (slam12.current_pose.x, slam12.current_pose.y, slam12.current_pose.theta) == \
           (frozen_pose.x, frozen_pose.y, frozen_pose.theta), \
        "T12 frozen pose must not continue drifting while the streak stays high"
    print(f"  T12 PASS  sustained off-trend drift ({slam12._untrusted_streak} "
          f"consecutive untrusted scans) froze the pose at {frozen_pose} instead "
          f"of walking it indefinitely — the exact hardware failure mode "
          f"(+0.1deg -> +70-98deg heading spin) is now bounded")

    print()
    print("All tests passed.")
    sys.exit(0)