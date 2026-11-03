"""
Simplifies a raw A* grid path (one waypoint per cell, zig-zaggy) into a
small number of long straight legs, by greedily extending line-of-sight as
far as possible before keeping a waypoint. This matters beyond cosmetics:
a turn-rate-limited, non-instant-turning robot has a minimum turning
radius, and tightly-spaced grid waypoints can require sharper turns than
that radius allows -- which causes the robot to orbit a waypoint forever
instead of ever reaching it. Fewer, straighter waypoints avoids that.
"""
from grid import cell_of, in_bounds


def _line_cells(a_cell, b_cell):
    """Bresenham line between two grid cells (inclusive)."""
    r0, c0 = a_cell
    r1, c1 = b_cell
    dr = r1 - r0
    dc = c1 - c0
    steps = max(abs(dr), abs(dc), 1)
    cells = []
    for i in range(steps + 1):
        r = round(r0 + dr * i / steps)
        c = round(c0 + dc * i / steps)
        cells.append((r, c))
    return cells


def _line_of_sight(a_px, b_px, blocked):
    a_cell = cell_of(a_px)
    b_cell = cell_of(b_px)
    for cell in _line_cells(a_cell, b_cell):
        if not in_bounds(cell) or cell in blocked:
            return False
    return True


def simplify_waypoints_px(waypoints_px, start_px, blocked):
    """
    waypoints_px: list of pixel points (the raw path, already excluding the
    robot's current position). Returns a shorter list with unnecessary
    intermediate points removed wherever a straight line stays clear.
    """
    if len(waypoints_px) <= 1:
        return list(waypoints_px)

    simplified = []
    anchor = start_px
    i = 0
    n = len(waypoints_px)
    while i < n:
        # Extend as far as possible while line-of-sight from anchor holds.
        farthest = i
        for j in range(i, n):
            if _line_of_sight(anchor, waypoints_px[j], blocked):
                farthest = j
            else:
                break
        simplified.append(waypoints_px[farthest])
        anchor = waypoints_px[farthest]
        i = farthest + 1
    return simplified
