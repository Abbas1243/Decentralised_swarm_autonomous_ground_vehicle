"""
Standard 8-directional A* over the obstacle grid.
Returns a list of (row, col) cells from start to goal inclusive, or None.
"""
import heapq
import math

from grid import neighbors8


def _heuristic(a, b):
    dr = abs(a[0] - b[0])
    dc = abs(a[1] - b[1])
    return max(dr, dc) + (math.sqrt(2) - 1) * min(dr, dc)  # octile distance


def find_path(start, goal, blocked):
    if start == goal:
        return [start]
    if goal in blocked:
        return None

    open_heap = [(_heuristic(start, goal), 0.0, start)]
    came_from = {}
    g_score = {start: 0.0}
    visited = set()

    while open_heap:
        _, g, current = heapq.heappop(open_heap)
        if current in visited:
            continue
        visited.add(current)

        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        for neighbor, dr, dc in neighbors8(current):
            if neighbor in blocked or neighbor in visited:
                continue
            step_cost = math.sqrt(2) if dr != 0 and dc != 0 else 1.0
            tentative_g = g + step_cost
            if tentative_g < g_score.get(neighbor, math.inf):
                g_score[neighbor] = tentative_g
                came_from[neighbor] = current
                f = tentative_g + _heuristic(neighbor, goal)
                heapq.heappush(open_heap, (f, tentative_g, neighbor))

    return None
