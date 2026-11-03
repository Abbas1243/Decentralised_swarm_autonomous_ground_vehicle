"""
Generates a random obstacle layout every time the map is (re)loaded.

Reliability guarantee: after scattering random obstacle blobs, we flood-fill
from the robot spawn point and carve a straight corridor to any region
center that ended up unreachable. This means A* is *always* guaranteed to
find a path from spawn to every region, and between any two free cells that
were reachable from spawn -- there is no "the demo got unlucky and a region
is sealed off" failure mode.
"""
import random
from collections import deque

import config
from grid import in_bounds, neighbors8


def _random_blob(rows, cols, rng):
    r0 = rng.randint(0, rows - 1)
    c0 = rng.randint(0, cols - 1)
    size = rng.randint(config.OBSTACLE_BLOB_MIN, config.OBSTACLE_BLOB_MAX)
    cells = set()
    frontier = [(r0, c0)]
    while frontier and len(cells) < size:
        r, c = frontier.pop(rng.randrange(len(frontier)))
        if not (0 <= r < rows and 0 <= c < cols):
            continue
        if (r, c) in cells:
            continue
        cells.add((r, c))
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            frontier.append((r + dr, c + dc))
    return cells


def _clear_spawn_area(blocked, spawn_cell, radius):
    sr, sc = spawn_cell
    for r in range(sr - radius, sr + radius + 1):
        for c in range(sc - radius, sc + radius + 1):
            blocked.discard((r, c))


def _reachable_set(blocked, start, rows, cols):
    seen = {start}
    q = deque([start])
    while q:
        cur = q.popleft()
        for n, _, _ in neighbors8(cur):
            if n in seen or n in blocked:
                continue
            seen.add(n)
            q.append(n)
    return seen


def _carve_line(blocked, a, b):
    """Bresenham-ish straight-line clear between two cells, plus margin.

    Margin is 2 cells (a 5-wide corridor) rather than 1 -- a single-file
    1-wide gap forces every robot heading to the same region through an
    identical narrow path at once, which causes crowding-induced steering
    oscillation (multiple robots' separation forces fighting for the same
    slot). A wider corridor lets a crowd actually pass side by side.
    """
    r0, c0 = a
    r1, c1 = b
    dr = r1 - r0
    dc = c1 - c0
    steps = max(abs(dr), abs(dc), 1)
    for i in range(steps + 1):
        r = round(r0 + dr * i / steps)
        c = round(c0 + dc * i / steps)
        for rr in (r - 2, r - 1, r, r + 1, r + 2):
            for cc in (c - 2, c - 1, c, c + 1, c + 2):
                blocked.discard((rr, cc))


def generate_obstacle_grid(spawn_cell, region_center_cells, rng=None):
    """
    Returns a set of blocked (row, col) cells.
    Guarantees spawn_cell and every cell in region_center_cells are mutually
    reachable (ignoring other robots -- this is a static map property).
    """
    rng = rng or random.Random()
    rows, cols = config.GRID_ROWS, config.GRID_COLS

    blocked = set()
    for _ in range(config.NUM_OBSTACLE_BLOBS):
        blocked |= _random_blob(rows, cols, rng)

    _clear_spawn_area(blocked, spawn_cell, config.SPAWN_CLEAR_RADIUS_CELLS)
    for rc in region_center_cells:
        blocked.discard(rc)

    # Reliability fixup: carve corridors to any region not yet reachable.
    reachable = _reachable_set(blocked, spawn_cell, rows, cols)
    for target in region_center_cells:
        if target not in reachable:
            _carve_line(blocked, spawn_cell, target)
            reachable = _reachable_set(blocked, spawn_cell, rows, cols)

    return blocked


def nearest_free_cell(blocked, cell, max_radius=15):
    if cell not in blocked and in_bounds(cell):
        return cell
    r0, c0 = cell
    for radius in range(1, max_radius + 1):
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                c = (r0 + dr, c0 + dc)
                if in_bounds(c) and c not in blocked:
                    return c
    return cell
