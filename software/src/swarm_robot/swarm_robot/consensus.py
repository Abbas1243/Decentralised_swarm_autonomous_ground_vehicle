"""Distributed consensus on velocity and heading.

Standard first-order consensus update:
    x_i(t+1) = x_i(t) + eps * sum_j a_ij * (x_j(t) - x_i(t))

With a_ij = 1 for every current neighbor j (unweighted, symmetric enough for
a proximity graph) and eps a small gain that keeps the update stable
(eps < 1 / max_degree is the classic stability bound -- we clamp it).
"""

import numpy as np


def consensus_step(self_value, neighbor_values, gain=0.1):
    """self_value / each of neighbor_values: np.array (velocity) or float (heading).
    Returns the updated value after one consensus iteration.
    Safe to call every control tick -- it converges gradually rather than snapping."""
    if not neighbor_values:
        return self_value

    n = len(neighbor_values)
    safe_gain = min(gain, 1.0 / (n + 1))  # stability bound

    correction = sum(v - self_value for v in neighbor_values)
    return self_value + safe_gain * correction


def wrap_angle(angle):
    """Keep an angle in (-pi, pi] -- needed before/after averaging headings,
    since naive averaging breaks across the +-pi wraparound."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def consensus_heading_step(self_heading, neighbor_headings, gain=0.1):
    """Same as consensus_step but for angles: converts to unit vectors first
    so the average across the wraparound (e.g. 179 deg and -179 deg) is correct."""
    if not neighbor_headings:
        return self_heading

    self_vec = np.array([np.cos(self_heading), np.sin(self_heading)])
    neighbor_vecs = [np.array([np.cos(h), np.sin(h)]) for h in neighbor_headings]

    updated_vec = consensus_step(self_vec, neighbor_vecs, gain)
    return float(np.arctan2(updated_vec[1], updated_vec[0]))
