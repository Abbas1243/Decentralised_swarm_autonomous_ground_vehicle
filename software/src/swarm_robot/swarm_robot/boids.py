"""Pure Boids rules. Everything here takes numpy arrays in and returns a
2D acceleration/velocity-bias vector out -- no ROS, no state, easy to unit test.

Each neighbor is a dict: {"position": np.array([x, y]), "velocity": np.array([vx, vy])}
"""

import numpy as np


def separation(self_pos, neighbors, desired_spacing):
    """Push away from neighbors closer than desired_spacing. Strength grows
    as 1/distance so close robots are pushed apart harder (collision avoidance)."""
    steer = np.zeros(2)
    for n in neighbors:
        offset = self_pos - n["position"]
        dist = np.linalg.norm(offset)
        if 1e-6 < dist < desired_spacing:
            steer += (offset / dist) * (desired_spacing - dist) / desired_spacing
    return steer


def alignment(neighbors):
    """Steer toward the average heading/velocity of neighbors."""
    if not neighbors:
        return np.zeros(2)
    avg_vel = np.mean([n["velocity"] for n in neighbors], axis=0)
    return avg_vel


def cohesion(self_pos, neighbors):
    """Steer toward the centroid of neighbors, to keep the group together."""
    if not neighbors:
        return np.zeros(2)
    centroid = np.mean([n["position"] for n in neighbors], axis=0)
    return centroid - self_pos


def blended_boids(self_pos, neighbors, weights, desired_spacing):
    """Combine the three rules with configurable weights.
    weights: dict with keys 'separation', 'alignment', 'cohesion'
    Returns a single 2D vector (not yet a velocity command -- the motion
    controller decides how to turn this into wheel commands)."""
    sep = separation(self_pos, neighbors, desired_spacing)
    ali = alignment(neighbors)
    coh = cohesion(self_pos, neighbors)

    return (
        weights.get("separation", 1.5) * sep
        + weights.get("alignment", 1.0) * ali
        + weights.get("cohesion", 1.0) * coh
    )
