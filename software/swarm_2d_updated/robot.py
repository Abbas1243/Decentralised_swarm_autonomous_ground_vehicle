"""Per-robot state for the decentralized swarm simulation."""
import math
import random
import pygame
import config
from grid import cell_of


class Robot:
    _next_id = 0

    IDLE = "idle"
    MOVING = "moving"
    HOLDING = "holding"

    def __init__(self, pos, rng=None):
        self.id = Robot._next_id
        Robot._next_id += 1
        rng = rng or random

        self.pos = [float(pos[0]), float(pos[1])]
        self.heading = rng.uniform(0, 2 * math.pi)
        self.speed = 0.0

        self.assigned_region = None
        self.state = Robot.IDLE

        # Stable compact-cluster slot. This is a local claim carried by the
        # robot; it is never globally rewritten every frame.
        self.formation_slot_index = None
        self.flock_anchor_px = None

        self.path = []
        self.patrol_target_px = None
        self.time_since_retarget = 0.0
        self.smoothed_steer = (0.0, 0.0)

        # Distributed membership gossip. Entries are:
        # robot_id -> (assignment_version, region_id)
        self.assignment_version = 0
        self.known_assignments = {
            self.id: (self.assignment_version, None)
        }
        self.known_bids = {}

        # Prevent one robot from migrating repeatedly during one election.
        self.migrated_epoch = -1
        self.migration_cooldown_until = 0.0
        self.migration_claim = None

    @property
    def velocity(self):
        return (
            math.cos(self.heading) * self.speed,
            math.sin(self.heading) * self.speed,
        )

    def cell(self):
        return cell_of(self.pos)

    def state_dict(self):
        return {
            "id": self.id,
            "position": tuple(self.pos),
            "velocity": self.velocity,
            "heading": self.heading,
            "state": self.state,
            "assigned_region": self.assigned_region,
            "slot_index": self.formation_slot_index,
            "known_assignments": dict(self.known_assignments),
            "known_bids": {k: dict(v) for k, v in self.known_bids.items()},
            "migration_claim": self.migration_claim,
        }

    def next_waypoint(self):
        return self.path[0] if self.path else None

    def advance_path_if_reached(self):
        while self.path:
            wx, wy = self.path[0]
            final = len(self.path) == 1
            threshold = (
                config.FINAL_TARGET_REACHED_DIST
                if final
                else config.WAYPOINT_REACHED_DIST
            )
            if math.hypot(wx - self.pos[0], wy - self.pos[1]) <= threshold:
                self.path.pop(0)
            else:
                break

        if not self.path and self.state == Robot.MOVING:
            self.state = Robot.HOLDING
            self.speed = 0.0
            self.smoothed_steer = (0.0, 0.0)

    def draw(self, surface):
        size = config.ROBOT_RADIUS * 2.0
        tip = (
            self.pos[0] + math.cos(self.heading) * size,
            self.pos[1] + math.sin(self.heading) * size,
        )
        left_angle = self.heading + math.radians(150)
        right_angle = self.heading - math.radians(150)
        left = (
            self.pos[0] + math.cos(left_angle) * size * 0.75,
            self.pos[1] + math.sin(left_angle) * size * 0.75,
        )
        right = (
            self.pos[0] + math.cos(right_angle) * size * 0.75,
            self.pos[1] + math.sin(right_angle) * size * 0.75,
        )

        if self.state == Robot.IDLE:
            color = config.COLOR_ROBOT_IDLE
        elif self.assigned_region is not None and self.assigned_region < len(config.REGION_COLORS):
            color = config.REGION_COLORS[self.assigned_region]
        else:
            color = config.COLOR_ROBOT

        pygame.draw.polygon(surface, color, [tip, left, right])
