"""Very small decentralized task-allocation layer, e.g. "at least N robots
inside zone A". Each robot decides FOR ITSELF whether to head toward an
under-served zone, using only the same radius-limited knowledge the boids
behavior already has (its own position + its current neighbor table) --
there is no central allocator. This uses a standard swarm-robotics
"response threshold" model: the stronger the unmet need and the closer the
robot is, the more likely it is to respond.

Because knowledge is local, allocation can be temporarily wrong (e.g. two
robots on opposite sides of the swarm both head to the same understaffed
zone without knowing about each other) -- that's a realistic property of
decentralized systems, not a bug, and worth observing during testing.
"""

import numpy as np


class Zone:
    def __init__(self, zone_id, center, radius, required_count):
        self.zone_id = zone_id
        self.center = np.array(center, dtype=float)
        self.radius = radius
        self.required_count = required_count


def count_occupants(zone, known_positions):
    """known_positions: list of np.array positions (self + known neighbors)."""
    return sum(1 for p in known_positions if np.linalg.norm(p - zone.center) <= zone.radius)


def response_probability(stimulus, threshold, n=2.0):
    """Standard response-threshold curve: P = s^n / (s^n + theta^n).
    Low stimulus (need already met, or far away) -> low probability of
    responding. High stimulus -> near-certain response."""
    if stimulus <= 0:
        return 0.0
    s_n = stimulus ** n
    return s_n / (s_n + threshold ** n)


def compute_task_bias(self_position, zones, known_positions, rng,
                       response_threshold=0.5, max_bias_speed=0.15):
    """Returns (bias_vector, assigned_zone_id_or_None).
    known_positions must include the robot's own position.
    Picks the single most understaffed *known* zone, then probabilistically
    decides (via the response-threshold model) whether THIS robot responds
    to it this tick -- this naturally spreads responders across zones and
    across robots without any negotiation message being sent."""

    best_zone, best_deficit = None, 0
    for zone in zones:
        deficit = zone.required_count - count_occupants(zone, known_positions)
        if deficit > best_deficit:
            best_deficit, best_zone = deficit, zone

    if best_zone is None:
        return np.zeros(2), None  # every zone this robot knows about is satisfied

    dist = np.linalg.norm(self_position - best_zone.center)
    stimulus = best_deficit / (1.0 + dist)  # more unmet need, closer -> stronger pull
    p = response_probability(stimulus, response_threshold)

    if rng.random() >= p:
        return np.zeros(2), None  # chose not to respond this tick, keeps flocking freely

    direction = best_zone.center - self_position
    norm = np.linalg.norm(direction)
    if norm < 1e-6:
        return np.zeros(2), best_zone.zone_id  # already there
    bias = (direction / norm) * min(max_bias_speed, norm)
    return bias, best_zone.zone_id
