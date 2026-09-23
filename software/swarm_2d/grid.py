"""
Grid <-> pixel conversions and basic cell queries.
The obstacle map is stored as a set of blocked (row, col) cells.
"""
import config


def cell_of(pos):
    """pixel (x, y) -> (row, col)"""
    col = int(pos[0] // config.CELL_SIZE)
    row = int(pos[1] // config.CELL_SIZE)
    return (row, col)


def cell_center_px(cell):
    """(row, col) -> pixel center (x, y)"""
    row, col = cell
    x = col * config.CELL_SIZE + config.CELL_SIZE / 2
    y = row * config.CELL_SIZE + config.CELL_SIZE / 2
    return (x, y)


def in_bounds(cell):
    row, col = cell
    return 0 <= row < config.GRID_ROWS and 0 <= col < config.GRID_COLS


def neighbors8(cell):
    row, col = cell
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr == 0 and dc == 0:
                continue
            n = (row + dr, col + dc)
            if in_bounds(n):
                yield n, dr, dc


def clamp_px(x, y):
    max_x = config.GRID_COLS * config.CELL_SIZE - 1
    max_y = config.GRID_ROWS * config.CELL_SIZE - 1
    return max(0.0, min(x, max_x)), max(0.0, min(y, max_y))
