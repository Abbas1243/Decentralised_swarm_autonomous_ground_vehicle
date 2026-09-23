"""
Consensus-based region allocation for the swarm_2d simulation.

This module is intentionally independent of pygame and Simulation.

The current simulator still runs all robots in one Python process, so this
is a *distributed-decision prototype*: every robot forms its own proposal
from its local state and nearby robot information, then a small consensus
round resolves conflicting proposals.

It does NOT control movement, A*, formation slots, or obstacle avoidance.

Future ROS2 version:
    RobotState -> local neighbours -> consensus.py -> local assignment
"""

import math


def _distance(a, b):
    return math.hypot(
        a[0] - b[0],
        a[1] - b[1],
    )


def _region_center(region):
    return region.center_px()


def build_local_neighbor_states(robot, robots, neighbor_radius):
    """
    Return only the information a robot is allowed to use for consensus.

    This deliberately avoids exposing the full Robot objects to the
    consensus layer. That makes the later ROS2 transition much cleaner.
    """
    result = []

    for other in robots:
        if other is robot:
            continue

        distance = _distance(robot.pos, other.pos)

        if distance <= neighbor_radius:
            result.append(
                {
                    "id": other.id,
                    "position": tuple(other.pos),
                    "assigned_region": other.assigned_region,
                    "state": other.state,
                }
            )

    return result


def propose_region(
    robot,
    active_regions,
    regions,
    neighbor_states,
    desired_load,
):
    """
    Produce one robot's local region proposal.

    The robot considers:
        1. distance to each active region
        2. how many nearby robots already claim that region
        3. the desired average population for that region

    A lower score is better.

    This is a local opinion, not a final global assignment.
    """
    if not active_regions:
        return None

    proposals = {}

    for region_id in active_regions:
        center = _region_center(regions[region_id])

        distance = _distance(robot.pos, center)

        nearby_claims = sum(
            1
            for neighbor in neighbor_states
            if neighbor.get("assigned_region") == region_id
        )

        # Distance is the primary factor.
        # Local demand provides a smaller balancing pressure.
        load_penalty = max(
            0.0,
            nearby_claims - desired_load
        ) * 80.0

        proposals[region_id] = distance + load_penalty

    return min(
        proposals,
        key=lambda region_id: (
            proposals[region_id],
            region_id,
        ),
    )


def _votes_for_region(proposals, region_id):
    return sum(
        1
        for proposed in proposals.values()
        if proposed == region_id
    )


def consensus_round(
    robots,
    active_regions,
    regions,
    neighbor_map,
    desired_load,
):
    """
    Run one synchronous consensus round.

    Every robot independently produces an opinion. We then publish those
    opinions to the simulated neighbourhood for the next round.

    The function returns:
        proposals: {robot: region_id}

    No robot is moved or assigned a path here.
    """
    proposals = {}

    for robot in robots:
        neighbors = neighbor_map.get(robot, [])

        proposals[robot] = propose_region(
            robot=robot,
            active_regions=active_regions,
            regions=regions,
            neighbor_states=neighbors,
            desired_load=desired_load,
        )

    return proposals


def resolve_conflicts(
    robots,
    active_regions,
    regions,
    proposals,
    neighbor_map,
    desired_counts,
):
    """
    Resolve competing proposals while preserving the requested region
    population balance.

    Robots that proposed an overloaded region reconsider it using their
    local information. The nearest robot to a region keeps priority when
    several robots are competing for the final slot.

    This is intentionally deterministic so simulation runs are reproducible.
    """
    assignments = {}
    remaining = dict(desired_counts)

    # Robots with fewer attractive choices get priority.
    ordered = sorted(
        robots,
        key=lambda robot: _proposal_difficulty(
            robot,
            active_regions,
            regions,
            proposals,
        ),
        reverse=True,
    )

    for robot in ordered:
        proposed = proposals.get(robot)

        candidates = [
            region_id
            for region_id in active_regions
            if remaining.get(region_id, 0) > 0
        ]

        if not candidates:
            candidates = list(active_regions)

        if proposed in candidates:
            # Prefer the local consensus proposal when capacity exists.
            chosen = proposed
        else:
            # The proposal lost its slot. Reconsider using local distance.
            neighbors = neighbor_map.get(robot, [])

            def score(region_id):
                center = _region_center(regions[region_id])

                distance = _distance(robot.pos, center)

                local_claims = sum(
                    1
                    for neighbor in neighbors
                    if neighbor.get("assigned_region") == region_id
                )

                return distance + local_claims * 80.0

            chosen = min(
                candidates,
                key=lambda region_id: (
                    score(region_id),
                    region_id,
                ),
            )

        assignments[robot] = chosen

        if remaining.get(chosen, 0) > 0:
            remaining[chosen] -= 1

    return assignments


def _proposal_difficulty(
    robot,
    active_regions,
    regions,
    proposals,
):
    """
    How strongly the robot has a reason to prefer one region over another.

    A larger spread means the robot has a clearer preference and should be
    resolved earlier during a conflict.
    """
    distances = []

    for region_id in active_regions:
        center = _region_center(regions[region_id])
        distances.append(_distance(robot.pos, center))

    if not distances:
        return 0.0

    return max(distances) - min(distances)


def allocate_regions(
    robots,
    active_regions,
    regions,
    neighbor_radius,
    rounds=3,
):
    """
    Complete consensus allocation.

    The public API is deliberately small so Simulation and a future ROS2
    node can both use it.

    The returned mapping is:
        {robot_object: region_id}
    """
    if not robots or not active_regions:
        return {}

    active_regions = sorted(active_regions)

    robot_count = len(robots)
    region_count = len(active_regions)

    base = robot_count // region_count
    remainder = robot_count % region_count

    desired_counts = {}

    for index, region_id in enumerate(active_regions):
        desired_counts[region_id] = (
            base + (1 if index < remainder else 0)
        )

    # Build the simulated communication graph once per allocation event.
    neighbor_map = {
        robot: build_local_neighbor_states(
            robot,
            robots,
            neighbor_radius,
        )
        for robot in robots
    }

    desired_load = robot_count / float(region_count)

    proposals = {}

    # Synchronous rounds: all robots decide from the same previous state.
    for _ in range(max(1, rounds)):
        proposals = consensus_round(
            robots=robots,
            active_regions=active_regions,
            regions=regions,
            neighbor_map=neighbor_map,
            desired_load=desired_load,
        )

        # Feed the current proposals into each robot's local view.
        for robot in robots:
            for neighbor in neighbor_map[robot]:
                neighbor_robot_id = neighbor["id"]

                for other_robot, proposal in proposals.items():
                    if other_robot.id == neighbor_robot_id:
                        neighbor["assigned_region"] = proposal
                        break

    return resolve_conflicts(
        robots=robots,
        active_regions=active_regions,
        regions=regions,
        proposals=proposals,
        neighbor_map=neighbor_map,
        desired_counts=desired_counts,
    )