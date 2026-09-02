import math


def compute_formation_positions(human_x, human_y, human_theta, num_robots, min_radius=2.0, spread_factor=0.6):
    """
    Compute target (x, y, theta) for each robot slot around the human.

    - Robots are arranged in an expanding arc/ring around the human.
    - More robots → smaller angular spacing (tighter) but larger radius to cover more ground.
    - Fewer robots → wider spacing, same coverage intent.

    Args:
        human_x, human_y : human position in world frame
        human_theta       : human heading (radians)
        num_robots        : total number of robots in the swarm
        min_radius        : minimum orbit radius from human
        spread_factor     : how much radius grows with robot count

    Returns:
        List of (x, y, theta) tuples, one per robot slot (0-indexed)
    """

    # Radius grows slightly with more robots so they spread outward and cover more ground
    radius = min_radius + spread_factor * math.log1p(num_robots)

    # Distribute robots evenly around a full circle
    angular_step = (2 * math.pi) / num_robots

    positions = []
    for i in range(num_robots):
        # Angle for this slot, offset by human heading so formation rotates with human
        angle = human_theta + i * angular_step

        target_x = human_x + radius * math.cos(angle)
        target_y = human_y + radius * math.sin(angle)

        # Robot faces the human
        face_human = math.atan2(human_y - target_y, human_x - target_x)

        positions.append((target_x, target_y, face_human))

    return positions


def angle_diff(a, b):
    """Shortest signed difference between two angles (radians)."""
    diff = a - b
    while diff > math.pi:
        diff -= 2 * math.pi
    while diff < -math.pi:
        diff += 2 * math.pi
    return diff


def euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)