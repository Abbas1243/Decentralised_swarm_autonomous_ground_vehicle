"""Turns the blended-boids steering vector and the consensus velocity/heading
into a single (linear_x, angular_z) command for a differential-drive robot.

Also owns the last line of collision-avoidance defense: a hard speed clamp
and an emergency slow-down if any neighbor is dangerously close, independent
of what the boids separation term already computed (belt-and-braces).
"""

import numpy as np


class MotionController:
    def __init__(self, max_linear_speed, max_angular_speed, critical_distance,
                 boids_weight=0.6, consensus_weight=0.4):
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.critical_distance = critical_distance
        self.boids_weight = boids_weight
        self.consensus_weight = consensus_weight

    def compute_cmd_vel(self, self_position, self_heading, boids_vector, consensus_velocity, neighbors):
        """self_position: np.array(2,) -- needed only for the emergency-stop distance check
        boids_vector: np.array(2,) from boids.blended_boids
        consensus_velocity: np.array(2,) from consensus.consensus_step
        neighbors: list of neighbor dicts with ABSOLUTE "position" (as returned by
            neighbor_manager.get_neighbors), used for the emergency-stop check
        Returns (linear_x, angular_z)."""

        desired = (
            self.boids_weight * boids_vector
            + self.consensus_weight * consensus_velocity
        )

        # No signal at all (alone in the swarm) -> hold position rather than drift
        if np.linalg.norm(desired) < 1e-6:
            return 0.0, 0.0

        desired_heading = float(np.arctan2(desired[1], desired[0]))
        heading_error = self._wrap(desired_heading - self_heading)

        linear = min(np.linalg.norm(desired), self.max_linear_speed)
        # slow down while turning sharply, like a real diff-drive robot would
        linear *= max(0.0, 1.0 - abs(heading_error) / np.pi)

        angular = np.clip(2.0 * heading_error, -self.max_angular_speed, self.max_angular_speed)

        linear, angular = self._apply_emergency_stop(linear, angular, self_position, neighbors)
        return float(linear), float(angular)

    def _apply_emergency_stop(self, linear, angular, self_position, neighbors):
        for n in neighbors:
            dist = np.linalg.norm(n["position"] - self_position)
            if dist < self.critical_distance:
                return 0.0, angular * 0.5  # stop forward motion, still allowed to turn away
        return linear, angular

    @staticmethod
    def _wrap(angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi
