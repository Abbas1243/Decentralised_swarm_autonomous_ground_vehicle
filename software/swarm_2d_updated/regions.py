"""
Six fixed surveillance regions laid out as a REGION_ROWS x REGION_COLS grid
over the map. Regions are fixed geometry every run; only which ones are
"active" (selected by the user) changes.
"""
import random

import config
from grid import cell_of, in_bounds


class Region:
    def __init__(self, region_id, rect_px, label):
        self.id = region_id
        self.rect_px = rect_px  # (x, y, w, h) in pixels
        self.label = label
        self.active = False

    def center_px(self):
        x, y, w, h = self.rect_px
        return (x + w / 2, y + h / 2)

    def center_cell(self):
        return cell_of(self.center_px())

    def contains_px(self, pos):
        x, y, w, h = self.rect_px
        return x <= pos[0] <= x + w and y <= pos[1] <= y + h

    def random_free_point_px(self, blocked, rng=None, attempts=25):
        rng = rng or random
        x, y, w, h = self.rect_px
        for _ in range(attempts):
            px = rng.uniform(x + 6, x + w - 6)
            py = rng.uniform(y + 6, y + h - 6)
            if cell_of((px, py)) not in blocked and in_bounds(cell_of((px, py))):
                return (px, py)
        return self.center_px()


def build_regions():
    map_h = config.GRID_ROWS * config.CELL_SIZE
    map_w = config.GRID_COLS * config.CELL_SIZE
    cell_w = map_w / config.REGION_COLS
    cell_h = map_h / config.REGION_ROWS

    regions = []
    region_id = 0
    for r in range(config.REGION_ROWS):
        for c in range(config.REGION_COLS):
            rect = (c * cell_w, r * cell_h, cell_w, cell_h)
            regions.append(Region(region_id, rect, label=str(region_id + 1)))
            region_id += 1
    return regions
