"""Keeps a rolling table of every other robot's last-known state, filters it
down to 'neighbors within my comm radius', and drops entries nobody has
heard from in a while -- which is how a robot leaving the swarm is detected
even without an explicit SwarmEvent (e.g. it crashed, or went out of range).
"""

import time
import numpy as np


class NeighborManager:
    def __init__(self, self_id, neighbor_radius, stale_timeout_sec=2.0):
        self.self_id = self_id
        self.neighbor_radius = neighbor_radius
        self.stale_timeout_sec = stale_timeout_sec
        self._table = {}  # robot_id -> {"position":..,"velocity":..,"heading":..,"last_seen":..}

    def update_state(self, robot_id, position, velocity, heading):
        """Call this from the /swarm/states subscription callback for every
        message received, including (harmlessly) our own -- we filter that below."""
        if robot_id == self.self_id:
            return
        self._table[robot_id] = {
            "position": np.array(position, dtype=float),
            "velocity": np.array(velocity, dtype=float),
            "heading": float(heading),
            "last_seen": time.monotonic(),
        }

    def prune_stale(self):
        """Remove robots we haven't heard from recently -- treats them as
        having left the swarm. Returns the list of robot_ids that were dropped."""
        now = time.monotonic()
        stale = [
            rid for rid, s in self._table.items()
            if now - s["last_seen"] > self.stale_timeout_sec
        ]
        for rid in stale:
            del self._table[rid]
        return stale

    def remove(self, robot_id):
        """Explicit removal, e.g. triggered by a SwarmEvent.LEAVE message."""
        self._table.pop(robot_id, None)

    def get_neighbors(self, self_position):
        """Returns the list of neighbor dicts within neighbor_radius, in the
        shape boids.py expects: [{"position": np.array, "velocity": np.array}, ...]"""
        self.prune_stale()
        neighbors = []
        for state in self._table.values():
            dist = np.linalg.norm(state["position"] - self_position)
            if dist <= self.neighbor_radius:
                neighbors.append({"position": state["position"], "velocity": state["velocity"]})
        return neighbors

    def get_neighbor_headings(self, self_position):
        self.prune_stale()
        headings = []
        for state in self._table.values():
            dist = np.linalg.norm(state["position"] - self_position)
            if dist <= self.neighbor_radius:
                headings.append(state["heading"])
        return headings

    def neighbor_count(self, self_position):
        return len(self.get_neighbors(self_position))
