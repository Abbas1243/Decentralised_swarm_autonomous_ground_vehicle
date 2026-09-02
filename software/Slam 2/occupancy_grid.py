"""
occupancy_grid.py
==================
Rasterizes the persistent line/arc feature map (map_manager.MapEntry list)
into a nav_msgs/OccupancyGrid-compatible grid, for PC-side RViz
visualization ONLY.

This is NOT part of the embedded SLAM pipeline (see slam_build_prompt.md —
line features were chosen specifically to avoid occupancy-grid RAM costs on
the Duo S). It exists purely so lidar_visualizer.py can publish a normal
gray/white/black RViz "Map" display (like Hector SLAM / Cartographer),
instead of a MarkerArray of colored line segments — same underlying
line/arc map, different PC-only rendering.

RASTERIZATION -- WALLS ONLY, NO FREE-SPACE RAYCASTING
--------------------------------------------------------
This module draws each active MapEntry (line or arc) onto the grid as
OCCUPIED (value 100) using Bresenham's algorithm. It does NOT raycast free
space from the robot pose through each LiDAR return the way Hector-style
occupancy-grid SLAM does — that would mean maintaining a second,
independent mapping representation (log-odds free/occupied per cell from
raw points) alongside the line-feature map, which is a materially bigger
module and a different, heavier design decision. Every cell not touched by
a wall stays UNKNOWN (-1, renders gray in RViz). This gives the same "Map"
display type and black/gray color convention as a normal occupancy-grid
SLAM map, at the cost of no visible free-space carve-out (no white area).

GRID
----
Fixed-size grid centered on the map origin (0,0), sized generously for an
indoor room. All in metres / cells:
    GRID_RESOLUTION_M = 0.05    (5cm/cell, matches typical Hector/RViz maps)
    GRID_WIDTH_M       = 20.0
    GRID_HEIGHT_M      = 20.0
    -> 400 x 400 cells = 160,000 int8 values. Trivial for a PC; this module
       is never intended to run on the Duo S.

nav_msgs/OccupancyGrid VALUE CONVENTION (unchanged, standard ROS)
--------------------------------------------------------------------
    -1   unknown       (RViz: gray)
     0   free          (RViz: white)      -- unused here, no cell is ever
                                              marked 0 by this module
     100 occupied      (RViz: black)

PORT PATH
---------
None. This module is PC-visualization-only (see module docstring above) —
it deliberately has no C port path, unlike every other slam_core module.
"""

import math

GRID_RESOLUTION_M = 0.05
GRID_WIDTH_M = 20.0
GRID_HEIGHT_M = 20.0

OCC_UNKNOWN = -1
OCC_FREE = 0
OCC_OCCUPIED = 100

RAY_STRIDE = 4   # subsample raw scan points before raycasting -- once a
                  # cell is marked free it stays free (see
                  # FreeSpaceAccumulator), so a persistent accumulator does
                  # not need every ray from every scan, just enough
                  # coverage accumulated over time. A full NUM_BINS=2000
                  # point scan raycast every ~7-10Hz frame is unnecessary
                  # density and a needless PC CPU cost; every 4th point
                  # still fully carves out real rooms within a few seconds.

LINE_THICKNESS_CELLS = 0   # how many EXTRA cells wide each wall is drawn
                            # beyond the single-pixel Bresenham line itself.
                            # 0 = true single-cell (5cm at default
                            # resolution) width, matching a real wall's
                            # footprint at this grid resolution. Was 1 (a
                            # 3x3 block per line point, i.e. 15cm) — walls
                            # are rebuilt from many separate short MapEntry
                            # segments (see slam_progress_summary's "long
                            # walls may appear as shorter segments" note),
                            # so overlapping 15cm-wide blocks from adjacent
                            # segments compounded into a visibly blobby,
                            # much-thicker-than-real-walls look. If 0 looks
                            # too thin/gappy on diagonal walls (Bresenham
                            # single-pixel lines can look slightly
                            # staircased), 1 is the next step up — resist
                            # going higher, since that's what caused this.


def grid_dims(resolution=GRID_RESOLUTION_M, width_m=GRID_WIDTH_M, height_m=GRID_HEIGHT_M):
    """Return (width_cells, height_cells, origin_x, origin_y).
    Origin is the map-frame (x, y) of grid cell (0, 0) -- i.e. the
    BOTTOM-LEFT corner of the grid, centered on the map origin."""
    width_cells = int(round(width_m / resolution))
    height_cells = int(round(height_m / resolution))
    origin_x = -width_m / 2.0
    origin_y = -height_m / 2.0
    return width_cells, height_cells, origin_x, origin_y


def _world_to_cell(x, y, origin_x, origin_y, resolution):
    cx = int(math.floor((x - origin_x) / resolution))
    cy = int(math.floor((y - origin_y) / resolution))
    return cx, cy


def _set_cell(grid, width, height, cx, cy, value=OCC_OCCUPIED, thickness=LINE_THICKNESS_CELLS):
    """Set one cell (and a small square neighborhood for visibility, since
    a single-pixel wall at 5cm/cell is easy to miss in RViz) to `value`,
    clipped to grid bounds."""
    for dx in range(-thickness, thickness + 1):
        for dy in range(-thickness, thickness + 1):
            x, y = cx + dx, cy + dy
            if 0 <= x < width and 0 <= y < height:
                grid[y * width + x] = value


def _bresenham(x0, y0, x1, y1):
    """Standard integer Bresenham line -- yields (x, y) cell coords from
    (x0,y0) to (x1,y1) inclusive. Pure integer arithmetic."""
    points = []
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    x, y = x0, y0
    while True:
        points.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy
    return points


def _draw_line_entry(grid, width, height, entry, origin_x, origin_y, resolution):
    """Draw one LINE MapEntry as a Bresenham segment between its two
    endpoints, reconstructed from (mx, my, angle, length) -- same
    reconstruction lidar_visualizer._pub_map already used for markers."""
    dx = math.cos(entry.angle) * entry.length / 2.0
    dy = math.sin(entry.angle) * entry.length / 2.0
    x1, y1 = entry.mx - dx, entry.my - dy
    x2, y2 = entry.mx + dx, entry.my + dy

    cx1, cy1 = _world_to_cell(x1, y1, origin_x, origin_y, resolution)
    cx2, cy2 = _world_to_cell(x2, y2, origin_x, origin_y, resolution)

    for cx, cy in _bresenham(cx1, cy1, cx2, cy2):
        _set_cell(grid, width, height, cx, cy)


def _draw_arc_entry(grid, width, height, entry, origin_x, origin_y, resolution, n_samples=32):
    """Draw one ARC MapEntry by sampling points along its sweep and
    connecting consecutive samples with Bresenham segments -- same sweep
    logic (dt fallback for missing theta_start/theta_end) already used in
    lidar_visualizer._pub_map for arc markers."""
    r = entry.distance
    if r < 0.01:
        return
    cx0, cy0 = entry.mx, entry.my
    t_s, t_e = entry.theta_start, entry.theta_end
    dt = t_e - t_s
    if dt > math.pi:
        dt -= 2 * math.pi
    if dt < -math.pi:
        dt += 2 * math.pi
    if abs(t_e) < 0.001 and abs(t_s) < 0.001:
        dt = min(entry.length / r, math.pi)
    elif abs(dt) < 0.01:
        dt = min(entry.length / r, math.pi)
    dt = max(min(dt, math.pi), -math.pi)

    prev_cell = None
    for k in range(n_samples + 1):
        theta = t_s + dt * k / n_samples
        x = cx0 + r * math.cos(theta)
        y = cy0 + r * math.sin(theta)
        cell = _world_to_cell(x, y, origin_x, origin_y, resolution)
        if prev_cell is not None:
            for cx, cy in _bresenham(prev_cell[0], prev_cell[1], cell[0], cell[1]):
                _set_cell(grid, width, height, cx, cy)
        prev_cell = cell


def rasterize_map(active_entries, resolution=GRID_RESOLUTION_M,
                   width_m=GRID_WIDTH_M, height_m=GRID_HEIGHT_M):
    """
    Rasterize a list of active MapEntry objects (map_manager.MapManager.
    get_active()) into a flat OccupancyGrid-compatible int8 list.

    Parameters
    ----------
    active_entries : list of MapEntry
        Typically MapManager.get_active() -- both STATIC and DYNAMIC
        entries are drawn (this is a visualization convenience, not a
        pose-correction input, so the STATIC-only filter used elsewhere in
        this codebase does not apply here). UNCLASSIFIED entries with too
        few observations are skipped, matching the existing MarkerArray
        behaviour in lidar_visualizer._pub_map (obs >= 3).
    resolution, width_m, height_m : grid parameters -- see grid_dims()

    Returns
    -------
    (data, width_cells, height_cells, origin_x, origin_y)
        data : list of int, length width_cells * height_cells, row-major
               (y-major then x), values in {-1, 100} -- ready to assign
               directly to nav_msgs.msg.OccupancyGrid.data
        width_cells, height_cells : grid dimensions in cells
        origin_x, origin_y : map-frame (x, y) of grid cell (0,0) -- ready
               to assign to OccupancyGrid.info.origin.position
    """
    width_cells, height_cells, origin_x, origin_y = grid_dims(resolution, width_m, height_m)
    grid = [OCC_UNKNOWN] * (width_cells * height_cells)

    for entry in active_entries:
        if entry.status == 0 and entry.observed < 3:   # ENTRY_UNCLASSIFIED, immature
            continue
        if entry.is_arc():
            _draw_arc_entry(grid, width_cells, height_cells, entry, origin_x, origin_y, resolution)
        else:
            _draw_line_entry(grid, width_cells, height_cells, entry, origin_x, origin_y, resolution)

    return grid, width_cells, height_cells, origin_x, origin_y


class FreeSpaceAccumulator:
    """
    Persistent free-space grid, built by raycasting raw LiDAR points (in the
    MAP frame, using the corrected SLAM pose) from the robot's position
    every scan. This is what produces the classic occupancy-grid look --
    white/explored where the robot has actually looked and seen nothing,
    gray/unexplored everywhere else -- that rasterize_map() alone cannot
    produce (rasterize_map only knows where WALLS are, never where the
    robot has looked and seen nothing).

    Deliberately kept SEPARATE from the wall representation: walls always
    come from rasterize_map()'s MapEntry rasterization (the persistent,
    matched, STATIC/DYNAMIC-classified line/arc map -- the single source of
    truth for "where are the walls" everywhere else in this codebase). Free
    space here is a purely visual accumulation with no bearing on SLAM
    correction whatsoever -- same "PC-visualization only, no C port path"
    boundary as the rest of this module (see module docstring). combine_
    with_walls() always lets a wall pixel win over a free pixel at the same
    cell, so a wall's presence is never visually erased by a later raycast
    grazing past it (e.g. from a slightly different pose/angle).

    ACCUMULATION IS MONOTONIC -- a cell that has ever been seen as free
    stays marked free permanently, even if a later scan's ray doesn't pass
    through it again. This mirrors the intuition of an explored map (once
    you've seen that patch of floor is clear, it stays "known" on the map)
    and avoids needing any log-odds/probabilistic decay machinery, which
    would be true overkill for a PC-only visualization aid.
    """

    def __init__(self, resolution=GRID_RESOLUTION_M, width_m=GRID_WIDTH_M, height_m=GRID_HEIGHT_M):
        self.resolution = resolution
        self.width_m = width_m
        self.height_m = height_m
        self.width, self.height, self.origin_x, self.origin_y = grid_dims(resolution, width_m, height_m)
        self._free = [False] * (self.width * self.height)

    def update(self, pose_x, pose_y, map_frame_points, stride=RAY_STRIDE):
        """
        Raycast from (pose_x, pose_y) -- the robot's current MAP-frame
        position -- to each point in map_frame_points (also MAP frame,
        already transformed by the caller using the corrected SLAM pose,
        same convention as transform_features_to_map_frame in slam.py).

        Every cell ALONG each ray is marked free, EXCLUDING the ray's own
        endpoint -- the endpoint is a single scan's noisy return, and
        whether it represents a real wall is already decided far more
        reliably by the persistent, multi-scan-confirmed MapEntry map (see
        rasterize_map). Marking it free here too would just get overridden
        by combine_with_walls() wherever a MapEntry line/arc agrees anyway,
        so excluding it only avoids briefly flashing "free" at a genuine
        wall cell before the map entry matures.

        `stride` subsamples map_frame_points for performance -- see
        RAY_STRIDE's module-level comment.
        """
        px, py = _world_to_cell(pose_x, pose_y, self.origin_x, self.origin_y, self.resolution)
        for i in range(0, len(map_frame_points), stride):
            x, y = map_frame_points[i]
            cx, cy = _world_to_cell(x, y, self.origin_x, self.origin_y, self.resolution)
            ray = _bresenham(px, py, cx, cy)
            for rx, ry in ray[:-1]:   # exclude the endpoint itself
                if 0 <= rx < self.width and 0 <= ry < self.height:
                    self._free[ry * self.width + rx] = True

    def combine_with_walls(self, active_entries):
        """
        Return (data, width, height, origin_x, origin_y) -- same shape and
        convention as rasterize_map()'s return, ready to assign directly to
        a nav_msgs/OccupancyGrid -- combining this accumulator's persistent
        free space with a fresh wall rasterization. Walls always win over
        free space at the same cell.
        """
        wall_data, width, height, ox, oy = rasterize_map(
            active_entries, self.resolution, self.width_m, self.height_m
        )
        out = list(wall_data)
        for i in range(len(out)):
            if out[i] == OCC_OCCUPIED:
                continue
            if self._free[i]:
                out[i] = OCC_FREE
        return out, width, height, ox, oy


# ---------------------------------------------------------------------------
# Self-test -- run directly: python3 occupancy_grid.py
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import sys
    from collections import namedtuple as _nt

    _MapEntryStub = _nt("_MapEntryStub",
        ["angle", "distance", "length", "mx", "my",
         "theta_start", "theta_end", "observed", "status"])

    class _ME(_MapEntryStub):
        def is_arc(self):
            return self.angle < -4.0

    print("occupancy_grid self-test")
    print("=" * 50)

    # -- T1: single horizontal line at y=0 from x=-1 to x=1 -> cells along
    #        that row should be OCCUPIED, cells elsewhere UNKNOWN --------
    entries = [
        _ME(angle=0.0, distance=0.0, length=2.0, mx=0.0, my=0.0,
            theta_start=0.0, theta_end=0.0, observed=5, status=1),
    ]
    data, w, h, ox, oy = rasterize_map(entries, resolution=0.1, width_m=4.0, height_m=4.0)
    assert w == 40 and h == 40, f"T1 grid dims wrong: {w}x{h}"

    cx, cy = _world_to_cell(0.0, 0.0, ox, oy, 0.1)
    assert data[cy * w + cx] == OCC_OCCUPIED, "T1 expected occupied at line midpoint"

    cx2, cy2 = _world_to_cell(1.9, 1.9, ox, oy, 0.1)
    assert data[cy2 * w + cx2] == OCC_UNKNOWN, "T1 expected unknown far from any wall"
    print("  T1 PASS  line rasterized: occupied along the wall, unknown elsewhere")

    # -- T2: immature UNCLASSIFIED entry (status=0, observed<3) is skipped --
    entries2 = [
        _ME(angle=0.0, distance=0.0, length=2.0, mx=0.0, my=0.0,
            theta_start=0.0, theta_end=0.0, observed=1, status=0),
    ]
    data2, w2, h2, ox2, oy2 = rasterize_map(entries2, resolution=0.1, width_m=4.0, height_m=4.0)
    assert all(v == OCC_UNKNOWN for v in data2), "T2 immature entry should not be drawn"
    print("  T2 PASS  immature UNCLASSIFIED entry skipped")

    # -- T3: arc entry draws a curved wall -----------------------------------
    entries3 = [
        _ME(angle=-10.0, distance=0.5, length=0.5 * math.pi, mx=0.0, my=0.0,
            theta_start=0.0, theta_end=math.pi, observed=5, status=1),
    ]
    data3, w3, h3, ox3, oy3 = rasterize_map(entries3, resolution=0.05, width_m=4.0, height_m=4.0)
    cx3, cy3 = _world_to_cell(0.5, 0.0, ox3, oy3, 0.05)   # theta=0 point on the arc
    assert data3[cy3 * w3 + cx3] == OCC_OCCUPIED, "T3 expected occupied at arc theta=0 point"
    print("  T3 PASS  arc rasterized onto the grid")

    # -- T4: grid_dims returns expected centering ----------------------------
    wc, hc, ox4, oy4 = grid_dims(resolution=0.05, width_m=20.0, height_m=20.0)
    assert wc == 400 and hc == 400, f"T4 dims wrong: {wc}x{hc}"
    assert abs(ox4 - (-10.0)) < 1e-9 and abs(oy4 - (-10.0)) < 1e-9, \
        f"T4 origin wrong: ({ox4},{oy4})"
    print("  T4 PASS  default grid is 400x400 cells, centered on (0,0)")

    # -- T5: FreeSpaceAccumulator -- raycast from origin marks cells along
    #        the ray free, but NOT the endpoint itself -----------------------
    acc = FreeSpaceAccumulator(resolution=0.1, width_m=4.0, height_m=4.0)
    acc.update(pose_x=0.0, pose_y=0.0, map_frame_points=[(1.0, 0.0)], stride=1)
    cx_mid, cy_mid = _world_to_cell(0.5, 0.0, acc.origin_x, acc.origin_y, acc.resolution)
    cx_end, cy_end = _world_to_cell(1.0, 0.0, acc.origin_x, acc.origin_y, acc.resolution)
    assert acc._free[cy_mid * acc.width + cx_mid] is True, "T5 expected midpoint marked free"
    assert acc._free[cy_end * acc.width + cx_end] is False, \
        "T5 endpoint must NOT be marked free (left to wall rasterization)"
    print("  T5 PASS  raycast marks the path free, leaves the endpoint alone")

    # -- T6: combine_with_walls -- a wall MapEntry at the same cell a ray
    #        passed through must win over the free marking -----------------
    acc6 = FreeSpaceAccumulator(resolution=0.1, width_m=4.0, height_m=4.0)
    # Raycast straight through where a wall will be placed (simulates a
    # stale free reading from a scan taken before the wall was confirmed).
    acc6.update(pose_x=0.0, pose_y=0.0, map_frame_points=[(1.5, 0.0)], stride=1)
    wall_entry = _ME(angle=math.pi / 2, distance=1.0, length=1.0, mx=1.0, my=0.0,
                      theta_start=0.0, theta_end=0.0, observed=5, status=1)
    data6, w6, h6, ox6, oy6 = acc6.combine_with_walls([wall_entry])
    cx6, cy6 = _world_to_cell(1.0, 0.0, ox6, oy6, acc6.resolution)
    assert data6[cy6 * w6 + cx6] == OCC_OCCUPIED, "T6 wall must win over a stale free marking"
    # A cell on the free path that is NOT a wall should still read free.
    cx6b, cy6b = _world_to_cell(0.5, 0.0, ox6, oy6, acc6.resolution)
    assert data6[cy6b * w6 + cx6b] == OCC_FREE, "T6 non-wall free cell should stay free"
    print("  T6 PASS  combine_with_walls: wall overrides free, elsewhere free is kept")

    # -- T7: a cell never raycast through and never a wall stays UNKNOWN ----
    cx7, cy7 = _world_to_cell(-1.8, -1.8, ox6, oy6, acc6.resolution)
    assert data6[cy7 * w6 + cx7] == OCC_UNKNOWN, "T7 untouched cell must remain unknown"
    print("  T7 PASS  never-visited, never-walled cell stays unexplored (gray)")

    print()
    print("All tests passed.")
    sys.exit(0)