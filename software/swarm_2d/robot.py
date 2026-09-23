"""
Robot: holds per-robot state and draws itself as an oriented triangle.

Movement decisions themselves live in simulation.py. The robot keeps track
of its current behavioural state so the simulation can distinguish between:

    IDLE    -> no task assigned
    MOVING  -> travelling toward an assigned target
    HOLDING -> reached its target and should remain there

The actual transition between these states is controlled by simulation.py,
except that finishing the current path automatically changes MOVING -> HOLDING.
"""
import math
import random

import pygame

import config
from grid import cell_of, cell_center_px


class Robot:
    _next_id = 0

    # Behaviour states
    IDLE = "idle"
    MOVING = "moving"
    HOLDING = "holding"

    def __init__(self, pos, rng=None):
        self.id = Robot._next_id
        Robot._next_id += 1
        rng = rng or random

        self.pos = [pos[0], pos[1]]
        self.heading = rng.uniform(0, 2 * math.pi)
        self.speed = 0.0

        # Task / assignment state
        self.assigned_region = None
        self.state = Robot.IDLE

        # A* path
        self.path = []

        # Kept for now because simulation.py still uses it.
        # We will remove/rework the patrol system in the next step.
        self.patrol_target_px = None
        self.time_since_retarget = 0.0

        # Smoothed steering vector
        self.smoothed_steer = (0.0, 0.0)

    @property
    def velocity(self):
        return (
            math.cos(self.heading) * self.speed,
            math.sin(self.heading) * self.speed
        )

    def cell(self):
        return cell_of(self.pos)

    def state_dict(self):
        """Shape matches the neighbor dicts boids.py expects."""
        return {
            "position": tuple(self.pos),
            "velocity": self.velocity
        }

    def next_waypoint(self):
        return self.path[0] if self.path else None

    def advance_path_if_reached(self):
        """
        Remove waypoints that have been reached.

        Once the final waypoint is reached, the robot enters HOLDING state.
        It is then the responsibility of simulation.py to decide whether the
        robot should remain there or later be reassigned.
        """
        while self.path:
            wx, wy = self.path[0]

            if math.hypot(wx - self.pos[0], wy - self.pos[1]) < config.WAYPOINT_REACHED_DIST:
                self.path.pop(0)
            else:
                break

        # The robot has completed its current route.
        if not self.path and self.state == Robot.MOVING:
            self.state = Robot.HOLDING
            self.speed = 0.0

    def draw(self, surface):
        size = config.ROBOT_RADIUS * 2.0

        tip = (
            self.pos[0] + math.cos(self.heading) * size,
            self.pos[1] + math.sin(self.heading) * size
        )

        left_angle = self.heading + math.radians(150)
        right_angle = self.heading - math.radians(150)

        left = (
            self.pos[0] + math.cos(left_angle) * size * 0.75,
            self.pos[1] + math.sin(left_angle) * size * 0.75
        )

        right = (
            self.pos[0] + math.cos(right_angle) * size * 0.75,
            self.pos[1] + math.sin(right_angle) * size * 0.75
        )

        color = config.COLOR_ROBOT

        if self.state == Robot.IDLE:
            color = config.COLOR_ROBOT_IDLE
        elif self.assigned_region is not None:
            if self.assigned_region < len(config.REGION_COLORS):
                color = config.REGION_COLORS[self.assigned_region]

        pygame.draw.polygon(surface, color, [tip, left, right])
        pygame.draw.circle(
            surface,
            (255, 255, 255),
            (int(self.pos[0]), int(self.pos[1])),
            1
        )