"""
Pure boids functions -- no pygame, no simulation state.
Mirrors the structure of the original swarm_robot/boids.py: takes a robot's
position/velocity and a list of neighbor dicts, returns a 2D vector.

neighbor dict shape: {"position": (x, y), "velocity": (vx, vy)}
"""
import math

import config


def _len(v):
    return math.hypot(v[0], v[1])


def _norm(v):
    length = _len(v)
    if length < 1e-6:
        return (0.0, 0.0)
    return (v[0] / length, v[1] / length)


def separation(position, neighbors):
    steer = [0.0, 0.0]
    count = 0
    for n in neighbors:
        dx = position[0] - n["position"][0]
        dy = position[1] - n["position"][1]
        dist = math.hypot(dx, dy)
        if 0 < dist < config.SEPARATION_RADIUS:
            weight = (config.SEPARATION_RADIUS - dist) / config.SEPARATION_RADIUS
            steer[0] += (dx / dist) * weight
            steer[1] += (dy / dist) * weight
            count += 1
    if count == 0:
        return (0.0, 0.0)
    return _norm(steer)


def alignment(velocity, neighbors):
    if not neighbors:
        return (0.0, 0.0)
    avg_vx = sum(n["velocity"][0] for n in neighbors) / len(neighbors)
    avg_vy = sum(n["velocity"][1] for n in neighbors) / len(neighbors)
    return _norm((avg_vx - velocity[0], avg_vy - velocity[1]))


def cohesion(position, neighbors):
    if not neighbors:
        return (0.0, 0.0)
    avg_x = sum(n["position"][0] for n in neighbors) / len(neighbors)
    avg_y = sum(n["position"][1] for n in neighbors) / len(neighbors)
    return _norm((avg_x - position[0], avg_y - position[1]))


def blended_boids(position, velocity, neighbors):
    sep = separation(position, neighbors)
    align = alignment(velocity, neighbors)
    coh = cohesion(position, neighbors)
    vx = (sep[0] * config.WEIGHT_SEPARATION
          + align[0] * config.WEIGHT_ALIGNMENT
          + coh[0] * config.WEIGHT_COHESION)
    vy = (sep[1] * config.WEIGHT_SEPARATION
          + align[1] * config.WEIGHT_ALIGNMENT
          + coh[1] * config.WEIGHT_COHESION)
    return (vx, vy)
