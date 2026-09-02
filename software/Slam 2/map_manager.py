"""
map_manager.py
==============
Persistent feature map for the line-feature SLAM system.

Manages a fixed-capacity array of MapEntry objects — the in-memory map that
accumulates across scans.  Handles:

    - Adding new features from unmatched scan features
    - Updating existing entries when matched (weighted average)
    - Decay and removal of entries not seen recently
    - Static / dynamic classification per entry
    - Capacity enforcement (MAX_MAP_ENTRIES cap)

This module owns all map state.  It calls line_matcher.match_features()
internally and exposes one primary entry point: update(scan_features, scan_idx).

STATIC / DYNAMIC CLASSIFICATION
--------------------------------
Every MapEntry carries a status field:

    UNCLASSIFIED  — newly added, not yet enough observations
    STATIC        — seen consistently, position stable → wall, furniture edge
    DYNAMIC       — was static/unclassified, then disappeared for DECAY_SCANS
                    consecutive scans, or reappeared at a different position
                    → person, moved chair, transient obstacle

Classification rules (applied during update):
    After a match:
        if observed >= MIN_OBS_FOR_STATIC and entry was UNCLASSIFIED:
            → promote to STATIC
        if entry was DYNAMIC and observed increases:
            → it reappeared; reset to UNCLASSIFIED and re-evaluate

    During decay (no match this scan):
        miss_count = scan_idx - last_seen
        if miss_count >= DECAY_SCANS:
            if status == STATIC:
                → mark DYNAMIC  (was reliably there, now gone → moved)
            else:
                → remove (UNCLASSIFIED feature that vanished = noise)

This is intentionally simple.  No Kalman filter, no probability grid.
Sufficient for an indoor cleaning robot distinguishing fixed walls from
moving people.

COORDINATE FRAME
----------------
All entries are stored in the sensor frame of the first scan (scan_idx == 0).
Pose correction (from pose_estimator) will transform scan features into the
map frame before calling update().  For now (before pose_estimator exists),
the robot is assumed stationary or slow — frame drift is acceptable for
initial visual testing.

PORT PATH
---------
When porting to C (slam_core/map_manager.c / map_manager.h):
    MapEntry struct     → map_manager.h
    MapManager struct   → map_manager.h  (holds the fixed array + scan counter)
    update()            → map_manager_update()
    get_active()        → iterate MapManager.entries[] checking .active flag
    Constants below     → #defines in map_manager.h

MapEntry field layout for C:
    float    angle        line: Hough angle | arc: ARC_ANGLE_SENTINEL (-10.0f)
    float    distance     line: Hough dist  | arc: radius
    float    length
    float    mx, my       line: midpoint    | arc: centre
    int32_t  observed
    int32_t  last_seen
    uint8_t  status       ENTRY_UNCLASSIFIED / ENTRY_STATIC / ENTRY_DYNAMIC
    uint8_t  active
    uint8_t  _pad[2]
    → 32 bytes, fits in MapLine from messages.h exactly if status reuses _pad[0]
       (but that's map_manager.h's problem, not messages.h)
"""

import math
from line_matcher import match_features, MatchResult

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

MAX_MAP_ENTRIES     = 500     # hard cap — matches slam.conf max_map_lines
MIN_QUALITY_TO_ADD  = 70      # scan feature must exceed this to enter map
                               # LOOPHOLE FIX: was 100 (literal maximum) --
                               # same reasoning as line_matcher.MIN_QUALITY:
                               # requiring perfection to even ENTER the map
                               # starved the map of new entries under any
                               # real-world noise/occlusion, compounding
                               # the "too few matches" problem this whole
                               # change set is fixing. Kept slightly higher
                               # than MIN_QUALITY (60) since adding a new
                               # permanent map entry should be a bit more
                               # conservative than matching against one
                               # that's already there.
MIN_LENGTH_TO_ADD   = 0.15    # metres — short fragments are noise
DECAY_SCANS         = 60      # miss this many consecutive scans → act on it
                               # At ~7 Hz this is ~8.5 seconds. Previously 10
                               # (~1.4 s), which meant STATIC walls decayed
                               # and were removed during the ~35-scan rejection
                               # storm that accompanies a physical motion event.
                               # DYNAMIC entries decay after 2*DECAY_SCANS=120
                               # scans (~17 s) — long enough to survive any
                               # realistic motion event before the map rebuilds.
MIN_OBS_FOR_STATIC  = 30      # observations to promote to STATIC (~2 s at 7 Hz)
                               # Previously 80 (~11 s). At 80, entries spent so
                               # long as UNCLASSIFIED that normal scan-to-scan
                               # jitter (one missed match in 10 scans) caused
                               # decay+removal before promotion, producing the
                               # visible map "vibration" and preventing DYNAMIC
                               # classification of genuinely moving objects
                               # (which reset to UNCLASSIFIED when re-seen and
                               # were deleted before re-reaching 80 obs).

ARC_ANGLE_SENTINEL  = -10.0   # matches messages.h / map_manager.h definition
# Must lie strictly outside the valid Hough line-angle range [-pi/2, pi/2]
# (~[-1.5708, 1.5708]) with comfortable margin. -1.0 was used previously and
# is INSIDE that range — any line with angle < -0.5 rad (< -28.6 deg, a
# perfectly normal steep wall angle) was silently misclassified as an arc.
# -10.0 cannot collide with any legitimate Hough angle.


# ---------------------------------------------------------------------------
# Status enum — mirrors C enum in map_manager.h
# ---------------------------------------------------------------------------

ENTRY_UNCLASSIFIED = 0
ENTRY_STATIC       = 1
ENTRY_DYNAMIC      = 2

_STATUS_NAMES = {
    ENTRY_UNCLASSIFIED: "UNCLASSIFIED",
    ENTRY_STATIC:       "STATIC",
    ENTRY_DYNAMIC:      "DYNAMIC",
}


# ---------------------------------------------------------------------------
# MapEntry
# ---------------------------------------------------------------------------

class MapEntry:
    """
    One entry in the persistent map.  Represents either a line or arc feature.

    LINE entry:
        angle    — Hough angle [-pi/2, pi/2] radians
        distance — perpendicular distance from map origin (metres)
        mx, my   — segment midpoint (metres)

    ARC entry:
        angle    — ARC_ANGLE_SENTINEL (-10.0)   ← IS the type flag
        distance — arc radius (metres)
        mx, my   — arc centre (metres)

    Common:
        length   — chord / arc length (metres)
        observed — how many scans this entry has been matched in
        last_seen— scan_idx of last match
        status   — ENTRY_UNCLASSIFIED / ENTRY_STATIC / ENTRY_DYNAMIC
        active   — False = slot is free (logically deleted)
    """

    # AFTER:
    __slots__ = (
        "angle", "distance", "length",
        "mx", "my",
        "theta_start", "theta_end",    # ← add theta_end
        "observed", "last_seen",
        "status", "active",
    )

    def __init__(self, angle, distance, length, mx, my,
                theta_start=0.0, theta_end=0.0,
                 observed=1, last_seen=0,
                 status=ENTRY_UNCLASSIFIED, active=True):
        self.angle       = angle
        self.distance    = distance
        self.length      = length
        self.mx          = mx
        self.my          = my
        self.theta_start = theta_start
        self.theta_end   = theta_end
        self.observed    = observed
        self.last_seen   = last_seen
        self.status      = status
        self.active      = active

    def is_arc(self):
        """True if this entry represents an arc feature."""
        return self.angle < -4.0   # ARC_ANGLE_SENTINEL == -10.0; threshold sits
                                    # comfortably below the line-angle floor of
                                    # -pi/2 (~-1.5708) with margin to spare

    def status_name(self):
        return _STATUS_NAMES.get(self.status, "?")

    def __repr__(self):
        if self.is_arc():
            return (f"MapEntry(ARC  cx={self.mx:.2f} cy={self.my:.2f} "
                    f"r={self.distance:.2f} ts={math.degrees(self.theta_start):.0f}° "
                    f"len={self.length:.2f} obs={self.observed} {self.status_name()})")
        else:
            return (f"MapEntry(LINE a={math.degrees(self.angle):.1f}° "
                    f"d={self.distance:.2f} mx={self.mx:.2f} my={self.my:.2f} "
                    f"len={self.length:.2f} obs={self.observed} "
                    f"{self.status_name()})")


# ---------------------------------------------------------------------------
# MapEntry construction from scan feature dicts
# ---------------------------------------------------------------------------

def _entry_from_line_feat(feat, scan_idx):
    """
    Create a new MapEntry from a line scan feature dict.
    Midpoint is computed from the endpoints stored in the Feature.
    """
    mx = (feat["x1"] + feat["x2"]) / 2.0
    my = (feat["y1"] + feat["y2"]) / 2.0
    return MapEntry(
        angle    = feat["angle"],
        distance = feat["distance"],
        length   = feat["length"],
        mx       = mx,
        my       = my,
        observed = 1,
        last_seen= scan_idx,
        status   = ENTRY_UNCLASSIFIED,
        active   = True,
    )


def _entry_from_arc_feat(feat, scan_idx):
    """
    Create a new MapEntry from an arc scan feature dict.
    """
    return MapEntry(
        angle       = ARC_ANGLE_SENTINEL,
        distance    = feat["r"],
        length      = feat["length"],
        mx          = feat["cx"],
        my          = feat["cy"],
        theta_start = feat.get("theta_start", 0.0),
        theta_end   = feat.get("theta_end",   0.0),
        observed    = 1,
        last_seen   = scan_idx,
        status      = ENTRY_UNCLASSIFIED,
        active      = True,
    )


# ---------------------------------------------------------------------------
# Weighted average update helpers
# ---------------------------------------------------------------------------

def _update_line_entry(entry, feat, scan_idx):
    n = entry.observed

    # Normalize incoming feature to same half-plane as entry.
    # Lines have pi-symmetry: (angle, dist) ≡ (angle+pi, -dist).
    # Without this, alternating eigenvector signs cause angle to drift to 0°
    # (a vertical wall averaged with its pi-equivalent becomes horizontal).
    feat_angle = feat["angle"]
    feat_dist  = feat["distance"]
    diff = feat_angle - entry.angle
    if diff >  math.pi / 2:
        feat_angle -= math.pi
        feat_dist   = -feat_dist
    elif diff < -math.pi / 2:
        feat_angle += math.pi
        feat_dist   = -feat_dist

    # Circular mean for angle
    ca = math.cos(entry.angle) * n + math.cos(feat_angle)
    sa = math.sin(entry.angle) * n + math.sin(feat_angle)
    entry.angle = math.atan2(sa, ca)

    # Keep entry.angle in [-pi/2, pi/2]; flip dist BEFORE weighted average
    if entry.angle > math.pi / 2:
        entry.angle -= math.pi
        feat_dist    = -feat_dist      # flip the incoming value, not entry.distance
    elif entry.angle < -math.pi / 2:
        entry.angle += math.pi
        feat_dist    = -feat_dist      # flip the incoming value, not entry.distance

    # Weighted average for distance, midpoint, length
    entry.distance = (entry.distance * n + feat_dist) / (n + 1)

    # Weighted average for distance, midpoint, length
    entry.distance = (entry.distance * n + feat_dist)  / (n + 1)
    new_mx = (feat["x1"] + feat["x2"]) / 2.0
    new_my = (feat["y1"] + feat["y2"]) / 2.0
    entry.mx     = (entry.mx * n + new_mx) / (n + 1)
    entry.my     = (entry.my * n + new_my) / (n + 1)
    entry.length = max(entry.length, feat["length"])

    entry.observed += 1
    entry.last_seen = scan_idx


def _update_arc_entry(entry, feat, scan_idx):
    n = entry.observed

    # Circular mean for theta_start
    ca = math.cos(entry.theta_start) * n + math.cos(feat.get("theta_start", 0.0))
    sa = math.sin(entry.theta_start) * n + math.sin(feat.get("theta_start", 0.0))
    entry.theta_start = math.atan2(sa, ca)

    # Circular mean for theta_end
    ca2 = math.cos(entry.theta_end) * n + math.cos(feat.get("theta_end", 0.0))
    sa2 = math.sin(entry.theta_end) * n + math.sin(feat.get("theta_end", 0.0))
    entry.theta_end = math.atan2(sa2, ca2)

    entry.distance = (entry.distance * n + feat["r"])    / (n + 1)
    entry.mx       = (entry.mx * n + feat["cx"])         / (n + 1)
    entry.my       = (entry.my * n + feat["cy"])         / (n + 1)
    entry.length   = max(entry.length, feat["length"])   # keep max, not average

    entry.observed  += 1
    entry.last_seen  = scan_idx


# ---------------------------------------------------------------------------
# Status promotion logic
# ---------------------------------------------------------------------------

def _promote_status(entry):
    """
    Called after a successful match (observed count just incremented).
    Promotes UNCLASSIFIED → STATIC when enough observations accumulated.
    Re-evaluates DYNAMIC entries that are being seen again.

    DYNAMIC re-entry logic (changed from original):
    Previously a DYNAMIC entry was immediately reset to UNCLASSIFIED on
    first re-sighting, meaning a person who walked past and returned would
    spend another MIN_OBS_FOR_STATIC scans as UNCLASSIFIED before being
    re-promoted — and if they moved again in that window they'd just be
    silently deleted (UNCLASSIFIED decays to removed, not DYNAMIC).

    Now a DYNAMIC entry stays DYNAMIC while it accumulates re-observations.
    Only once it has been re-seen MIN_OBS_FOR_STATIC times consecutively
    does it get re-promoted to STATIC. This means:
      - a re-appearing wall (moved furniture put back) → re-STATIC quickly
      - a person who briefly stops then moves again → stays DYNAMIC, no
        false promotion, correctly removed when they leave the room
    """
    if entry.status == ENTRY_DYNAMIC:
        # Accumulate re-observations while staying DYNAMIC.
        # Promote to STATIC only once stability is confirmed.
        if entry.observed >= MIN_OBS_FOR_STATIC:
            entry.status = ENTRY_STATIC
        # else: stays DYNAMIC — correct for transient re-appearances
        return

    if entry.status == ENTRY_UNCLASSIFIED:
        if entry.observed >= MIN_OBS_FOR_STATIC:
            entry.status = ENTRY_STATIC


# ---------------------------------------------------------------------------
# MapManager
# ---------------------------------------------------------------------------

class MapManager:
    """
    Holds the full map state and drives one update cycle per scan.

    Usage
    -----
        mm = MapManager()
        for scan_features, scan_idx in scan_stream:
            result = mm.update(scan_features, scan_idx)
            # result.matched / .unmatched_scan / .unmatched_map available
            active = mm.get_active()   # list of MapEntry for visualisation

    Thread safety: not thread-safe.  One scan at a time.
    """

    def __init__(self):
        self._entries = []          # list of MapEntry, includes inactive slots
        self._scan_count = 0        # total scans processed

    # ------------------------------------------------------------------ #
    # Primary entry point                                                  #
    # ------------------------------------------------------------------ #

    def update(self, scan_features, scan_idx=None):
        """
        Process one scan's worth of features and update the map.

        Parameters
        ----------
        scan_features : list of dicts
            Output of fit_first_ctypes.split_merge() — the features list.
        scan_idx : int or None
            Scan sequence number.  If None, auto-increments from internal
            counter.  Pass explicitly when replaying bag files.

        Returns
        -------
        MatchResult — same object returned by line_matcher.match_features().
        Callers can inspect .matched / .unmatched_scan / .unmatched_map.
        """
        if scan_idx is None:
            scan_idx = self._scan_count
        self._scan_count += 1

        # 1. Match scan features against current map
        result = match_features(scan_features, self._entries)

        # 2. Update matched entries
        for si, mi, score in result.matched:
            sf    = scan_features[si]
            entry = self._entries[mi]
            if entry.is_arc():
                _update_arc_entry(entry, sf, scan_idx)
            else:
                _update_line_entry(entry, sf, scan_idx)
            _promote_status(entry)

        # 3. Add new entries from unmatched scan features (if capacity allows)
        # Collect midpoints of already-matched map entries this scan
        matched_map_midpoints = [
            (self._entries[mi].mx, self._entries[mi].my)
            for _, mi, _ in result.matched
        ]

        for si in result.unmatched_scan:
            sf = scan_features[si]
            if not self._should_add(sf):
                continue
            # Skip if spatially near a map entry that was already matched this scan
            if sf["type"] == "line":
                feat_mx = (sf["x1"] + sf["x2"]) / 2.0
                feat_my = (sf["y1"] + sf["y2"]) / 2.0
                too_close = any(
                    math.hypot(feat_mx - mx, feat_my - my) < 0.30
                    for mx, my in matched_map_midpoints
                )
                if too_close:
                    continue
            if self._count_active() >= MAX_MAP_ENTRIES:
                break
            new_entry = self._make_entry(sf, scan_idx)
            if new_entry is not None:
                placed = False
                for i, e in enumerate(self._entries):
                    if not e.active:
                        self._entries[i] = new_entry
                        placed = True
                        break
                if not placed:
                    self._entries.append(new_entry)

        # 4. Decay unmatched map entries
        self._decay(result.unmatched_map, scan_idx)

        return result

    # ------------------------------------------------------------------ #
    # Query                                                                #
    # ------------------------------------------------------------------ #

    def get_active(self):
        """Return list of all active MapEntry objects (for visualisation)."""
        return [e for e in self._entries if e.active]

    def get_active_by_status(self, status):
        """Return active entries filtered by status constant."""
        return [e for e in self._entries if e.active and e.status == status]

    def count_active(self):
        return self._count_active()

    def stats(self):
        """Return dict of map statistics."""
        active = self.get_active()
        return {
            "total_slots":    len(self._entries),
            "active":         len(active),
            "lines":          sum(1 for e in active if not e.is_arc()),
            "arcs":           sum(1 for e in active if e.is_arc()),
            "unclassified":   sum(1 for e in active if e.status == ENTRY_UNCLASSIFIED),
            "static":         sum(1 for e in active if e.status == ENTRY_STATIC),
            "dynamic":        sum(1 for e in active if e.status == ENTRY_DYNAMIC),
            "scans_processed": self._scan_count,
        }

    # ------------------------------------------------------------------ #
    # Internal helpers                                                     #
    # ------------------------------------------------------------------ #

    def _count_active(self):
        return sum(1 for e in self._entries if e.active)

    def _should_add(self, feat):
        """True if this scan feature is worth adding to the map."""
        if feat.get("quality", 100) < MIN_QUALITY_TO_ADD:
            return False
        if feat.get("length", 0.0) < MIN_LENGTH_TO_ADD:
            return False
        if feat["type"] not in ("line", "arc"):
            return False
        # Duplicate suppression — reject if a very similar entry already exists.
        # Uses tighter thresholds than the matcher to catch near-misses that
        # slipped through matching but are not genuinely new features.
        for e in self._entries:
            if not e.active:
                continue
            if feat["type"] == "line" and not e.is_arc():
                adiff = abs(feat["angle"] - e.angle) % math.pi
                if adiff > math.pi / 2: adiff = math.pi - adiff
                if adiff >= 0.15:
                    continue
                # Use midpoint proximity, not Hough distance — avoids sign issues
                feat_mx = (feat["x1"] + feat["x2"]) / 2.0
                feat_my = (feat["y1"] + feat["y2"]) / 2.0
                midpoint_dist = math.hypot(feat_mx - e.mx, feat_my - e.my)
                if midpoint_dist < 0.25:   # same physical location = duplicate
                    return False
            elif feat["type"] == "arc" and e.is_arc():
                cdist = math.hypot(feat["cx"] - e.mx, feat["cy"] - e.my)
                rdiff = abs(feat["r"] - e.distance)
                if cdist < 0.30 and rdiff < 0.10:
                    return False
        return True
    def _make_entry(self, feat, scan_idx):
        """Create a MapEntry from a scan feature dict.  Returns None on error."""
        try:
            if feat["type"] == "line":
                return _entry_from_line_feat(feat, scan_idx)
            elif feat["type"] == "arc":
                return _entry_from_arc_feat(feat, scan_idx)
        except KeyError:
            return None
        return None

    def _decay(self, unmatched_map_idx, scan_idx):
        """
        Apply decay to map entries that were not matched this scan.

        miss_count = scan_idx - last_seen

        UNCLASSIFIED entries:
            miss_count >= DECAY_SCANS → remove (was noise, never confirmed)

        STATIC entries:
            miss_count >= DECAY_SCANS → mark DYNAMIC
            (reliable feature vanished → something moved)

        DYNAMIC entries:
            miss_count >= DECAY_SCANS * 2 → remove
            (dynamic object gone long enough to forget about)
        """
        for mi in unmatched_map_idx:
            entry = self._entries[mi]
            if not entry.active:
                continue
            miss = scan_idx - entry.last_seen
            if entry.status == ENTRY_STATIC:
                if miss >= DECAY_SCANS:
                    entry.status = ENTRY_DYNAMIC
            elif entry.status == ENTRY_DYNAMIC:
                if miss >= DECAY_SCANS * 2:
                    entry.active = False
            else:  # UNCLASSIFIED
                if miss >= DECAY_SCANS:
                    entry.active = False


# ---------------------------------------------------------------------------
# Self-test — run directly: python3 map_manager.py
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import sys

    print("map_manager self-test")
    print("=" * 50)

    def _line_feat(angle, distance, x1, y1, x2, y2, quality=100):
        return {
            "type": "line",
            "angle": angle, "distance": distance,
            "x1": x1, "y1": y1, "x2": x2, "y2": y2,
            "length": math.hypot(x2 - x1, y2 - y1),
            "quality": quality,
        }

    def _arc_feat(cx, cy, r, quality=100):
        return {
            "type": "arc",
            "cx": cx, "cy": cy, "r": r,
            "theta_start": 0.0, "theta_end": math.pi,
            "length": r * math.pi,
            "quality": quality,
        }

    mm = MapManager()

    # ── T1: First scan — map is empty, all features become new entries ───
    scan0 = [
        _line_feat(0.0,  1.0,  -0.5, 1.0,  0.5, 1.0),
        _line_feat(1.57, 2.0,   2.0, -0.5, 2.0, 0.5),
        _arc_feat(0.0, 0.0, 0.5),
    ]
    r0 = mm.update(scan0, scan_idx=0)
    assert len(r0.matched) == 0,        f"T1 no matches expected, got {r0.matched}"
    assert mm.count_active() == 3,      f"T1 expected 3 entries, got {mm.count_active()}"
    print(f"  T1 PASS  first scan: 3 features → 3 new map entries")

    # ── T2: Second scan — same features, should all match ────────────────
    scan1 = [
        _line_feat(0.01,  1.01,  -0.5, 1.0, 0.5, 1.0),
        _line_feat(1.57,  2.01,   2.0, -0.5, 2.0, 0.5),
        _arc_feat(0.01, 0.01, 0.51),
    ]
    r1 = mm.update(scan1, scan_idx=1)
    assert len(r1.matched) == 3,        f"T2 expected 3 matches, got {len(r1.matched)}"
    assert mm.count_active() == 3,      f"T2 map should still be 3 entries"
    # Check observed counts incremented
    active = mm.get_active()
    for e in active:
        assert e.observed == 2,         f"T2 expected observed=2, got {e.observed}"
    print(f"  T2 PASS  second scan: all 3 features matched, observed→2")

    # ── T3: Promote to STATIC after MIN_OBS_FOR_STATIC scans ─────────────
    for i in range(2, MIN_OBS_FOR_STATIC + 1):
        mm.update(scan1, scan_idx=i)
    static = mm.get_active_by_status(ENTRY_STATIC)
    assert len(static) == 3, f"T3 expected 3 STATIC entries, got {len(static)}"
    print(f"  T3 PASS  after {MIN_OBS_FOR_STATIC} observations → promoted to STATIC")

    # ── T4: Missing feature for DECAY_SCANS → STATIC becomes DYNAMIC ─────
    scan_no_line0 = [
        _line_feat(1.57, 2.01, 2.0, -0.5, 2.0, 0.5),  # only 2nd line
        _arc_feat(0.01, 0.01, 0.51),
    ]
    base_idx = MIN_OBS_FOR_STATIC + 1
    for i in range(DECAY_SCANS):
        mm.update(scan_no_line0, scan_idx=base_idx + i)

    dynamic = mm.get_active_by_status(ENTRY_DYNAMIC)
    assert len(dynamic) == 1, f"T4 expected 1 DYNAMIC entry, got {len(dynamic)}"
    assert dynamic[0].is_arc() == False, "T4 dynamic entry should be the missing line"
    print(f"  T4 PASS  STATIC entry missing {DECAY_SCANS} scans → DYNAMIC")

    # ── T5: DYNAMIC entry disappears for 2*DECAY_SCANS → removed ─────────
    base_idx2 = base_idx + DECAY_SCANS
    for i in range(DECAY_SCANS * 2 + 1):
        mm.update(scan_no_line0, scan_idx=base_idx2 + i)

    active_now = mm.get_active()
    # Original line at distance=1.0 (never matched after T4) should be gone.
    # Only the line at distance≈2.0 (always matched) and the arc should remain.
    lines = [e for e in active_now if not e.is_arc()]
    assert len(lines) == 1, f"T5 expected 1 line in map, got {len(lines)}"
    assert abs(lines[0].distance - 2.0) < 0.1, \
        f"T5 wrong line survived: distance={lines[0].distance:.3f}"
    print(f"  T5 PASS  DYNAMIC entry absent 2×DECAY_SCANS → removed from map")

    # ── T6: Low quality feature not added ────────────────────────────────
    mm2 = MapManager()
    r_lq = mm2.update([_line_feat(0.0, 1.0, -0.5, 1.0, 0.5, 1.0, quality=30)])
    assert mm2.count_active() == 0, f"T6 low quality must not enter map"
    print(f"  T6 PASS  quality < {MIN_QUALITY_TO_ADD} → not added to map")

    # ── T7: Stats output ──────────────────────────────────────────────────
    s = mm.stats()
    assert s["scans_processed"] > 0
    print(f"  T7 PASS  stats: {s}")

    print()
    print("All tests passed.")
    sys.exit(0)