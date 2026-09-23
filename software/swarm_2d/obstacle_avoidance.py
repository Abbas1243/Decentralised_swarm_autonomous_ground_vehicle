"""
Obstacle-awareness for the motion controller.

Provides two layers of obstacle protection:

1. Soft obstacle repulsion
   - Treats each blocked grid cell as a real square, not just its centre.
   - Repulsion becomes significantly stronger as the robot approaches
     the obstacle.
   - Considers the robot radius so the robot starts turning before its
     centre reaches the wall.

2. Hard emergency check
   - Detects the actual distance between the robot and obstacle surfaces.
   - Includes the robot's physical radius.
   - Used by simulation.py to force the robot into emergency creep mode.

The goal is to prevent:
    - wall ramming
    - corner clipping
    - scraping obstacle edges
    - weak avoidance when moving quickly toward a wall
"""

import math

import config
from grid import cell_of


# ================================================================
# BASIC GEOMETRY
# ================================================================

def _obstacle_cell_rect(cell):
    """
    Return the obstacle cell as:

        (left, top, right, bottom)
    """

    row, col = cell

    left = col * config.CELL_SIZE
    top = row * config.CELL_SIZE

    right = left + config.CELL_SIZE
    bottom = top + config.CELL_SIZE

    return left, top, right, bottom


def _closest_point_on_rect(pos, rect):
    """
    Find the closest point on a rectangle to pos.
    """

    x, y = pos

    left, top, right, bottom = rect

    closest_x = max(
        left,
        min(x, right)
    )

    closest_y = max(
        top,
        min(y, bottom)
    )

    return closest_x, closest_y


def _distance_to_rect(pos, rect):
    """
    Return the shortest distance from a point to a rectangle.

    If the point is inside the rectangle, distance is zero.
    """

    closest = _closest_point_on_rect(
        pos,
        rect
    )

    return math.hypot(
        pos[0] - closest[0],
        pos[1] - closest[1]
    )


# ================================================================
# NEARBY OBSTACLES
# ================================================================

def _nearby_obstacle_cells(
    pos,
    blocked,
    radius_pixels
):
    """
    Find obstacle cells within radius_pixels of the robot.

    The old implementation converted the radius directly to an
    integer number of cells. This version deliberately adds one
    extra cell because the robot can be physically close to an
    obstacle even when its centre is in a neighbouring cell.
    """

    row, col = cell_of(pos)

    radius_cells = (
        int(
            math.ceil(
                radius_pixels
                / config.CELL_SIZE
            )
        )
        + 1
    )

    found = []

    for dr in range(
        -radius_cells,
        radius_cells + 1
    ):
        for dc in range(
            -radius_cells,
            radius_cells + 1
        ):

            cell = (
                row + dr,
                col + dc
            )

            if cell in blocked:
                found.append(cell)

    return found


# ================================================================
# SOFT REPULSION
# ================================================================

def repulsion_vector(pos, blocked):
    """
    Calculate a steering vector pointing away from nearby
    obstacles.

    Unlike the previous implementation, this uses the closest
    point on the actual obstacle square rather than the obstacle
    cell centre.

    The force increases rapidly as the robot approaches an
    obstacle:

        far away
            ↓
        weak steering

        closer
            ↓
        stronger steering

        very close
            ↓
        very strong steering

    The returned vector is generally normalized, but its direction
    is based on distance-weighted obstacle contributions.
    """

    # Include the robot radius in the sensing distance.
    sensing_radius = (
        config.OBSTACLE_AVOID_RADIUS
        + config.ROBOT_RADIUS
    )

    nearby = _nearby_obstacle_cells(
        pos,
        blocked,
        sensing_radius
    )

    steer_x = 0.0
    steer_y = 0.0

    for cell in nearby:

        rect = _obstacle_cell_rect(cell)

        closest_x, closest_y = (
            _closest_point_on_rect(
                pos,
                rect
            )
        )

        dx = pos[0] - closest_x
        dy = pos[1] - closest_y

        distance = math.hypot(
            dx,
            dy
        )

        # --------------------------------------------------------
        # Robot centre is inside the obstacle.
        #
        # This should almost never happen, but if it does we need
        # a reliable escape direction.
        # --------------------------------------------------------

        if distance < 1e-6:

            left, top, right, bottom = rect

            distances = [
                (pos[0] - left, 1.0, 0.0),
                (right - pos[0], -1.0, 0.0),
                (pos[1] - top, 0.0, 1.0),
                (bottom - pos[1], 0.0, -1.0),
            ]

            escape = min(
                distances,
                key=lambda item: item[0]
            )

            dx = escape[1]
            dy = escape[2]

            distance = 0.0

        else:

            dx /= distance
            dy /= distance

        # --------------------------------------------------------
        # Effective distance from the robot body to the obstacle.
        #
        # The robot is not a point. Its radius must be considered.
        # --------------------------------------------------------

        surface_distance = max(
            0.0,
            distance - config.ROBOT_RADIUS
        )

        if surface_distance >= config.OBSTACLE_AVOID_RADIUS:
            continue

        # --------------------------------------------------------
        # Normalized proximity:
        #
        # 1.0 -> touching / extremely close
        # 0.0 -> edge of avoidance radius
        # --------------------------------------------------------

        proximity = (
            config.OBSTACLE_AVOID_RADIUS
            - surface_distance
        ) / config.OBSTACLE_AVOID_RADIUS

        proximity = max(
            0.0,
            min(1.0, proximity)
        )

        # Cubic falloff makes the avoidance response relatively
        # gentle when approaching an obstacle, but very strong
        # close to the wall.
        weight = proximity ** 3

        steer_x += dx * weight
        steer_y += dy * weight

    magnitude = math.hypot(
        steer_x,
        steer_y
    )

    if magnitude < 1e-6:
        return (0.0, 0.0)

    return (
        steer_x / magnitude,
        steer_y / magnitude
    )


# ================================================================
# HARD OBSTACLE PROTECTION
# ================================================================

def too_close_to_obstacle(
    pos,
    blocked
):
    """
    Check whether the robot body is dangerously close to an
    obstacle.

    The previous version measured:

        robot centre -> obstacle centre

    which is not the correct collision geometry.

    This version measures:

        robot centre -> obstacle surface

    and then accounts for ROBOT_RADIUS.
    """

    critical_distance = (
        config.OBSTACLE_CRITICAL_DISTANCE
        + config.ROBOT_RADIUS
    )

    nearby = _nearby_obstacle_cells(
        pos,
        blocked,
        critical_distance
    )

    for cell in nearby:

        rect = _obstacle_cell_rect(cell)

        surface_distance = _distance_to_rect(
            pos,
            rect
        )

        # Distance from the robot's BODY to the obstacle.
        body_clearance = (
            surface_distance
            - config.ROBOT_RADIUS
        )

        if body_clearance < config.OBSTACLE_CRITICAL_DISTANCE:
            return True

    return False