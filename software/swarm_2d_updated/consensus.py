"""Decentralized equal-share election using peer-to-peer top-K bid gossip.

There is no central assignment controller.

Every robot:
    1. gossips its current assignment knowledge with neighbours;
    2. computes whether it is in a surplus region;
    3. computes a local migration bid toward an underfilled region;
    4. gossips that bid with neighbours;
    5. during the decision rounds, migrates only if its bid is among the best
       K bids known for that target, where K is the target deficit.

The protocol is deliberately split into warm-up and decision rounds. This
lets bids propagate through the connected communication graph before anyone
moves, which prevents the "everyone locally thinks they won" stampede.
"""
import math


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def region_center(region):
    return region.center_px()


def _normalise_assignment(value):
    try:
        version, region_id = value
        return int(version), region_id
    except (TypeError, ValueError):
        return 0, None


def merge_assignment_knowledge(robot, neighbour_states):
    """Gossip newest assignment versions from nearby robots."""
    merged = dict(robot.known_assignments)
    merged[robot.id] = (
        robot.assignment_version,
        robot.assigned_region,
    )

    for neighbour in neighbour_states:
        for rid, raw in neighbour.get("known_assignments", {}).items():
            version, region_id = _normalise_assignment(raw)
            old = merged.get(rid)
            if old is None or version >= old[0]:
                merged[rid] = (version, region_id)

    robot.known_assignments = merged
    return merged


def counts_from_knowledge(robot, active_regions):
    counts = {rid: 0 for rid in active_regions}
    for _, raw in robot.known_assignments.items():
        _, region_id = _normalise_assignment(raw)
        if region_id in counts:
            counts[region_id] += 1
    return counts


def known_population(robot):
    return len(robot.known_assignments)


def compute_target_counts(robot, active_regions, preferred_extra_regions=()):
    """Return equal-share target counts.

    For N robots and R active regions:
        base = N // R
        remainder = N % R

    The remainder is assigned first to newly activated/preferred regions. For
    example, 10 robots and 3 regions becomes 3/3/4 when the third region is
    the newly activated one.
    """
    active_regions = sorted(active_regions)
    if not active_regions:
        return {}

    n = max(known_population(robot), 1)
    base, remainder = divmod(n, len(active_regions))
    targets = {rid: base for rid in active_regions}

    ordering = []
    for rid in preferred_extra_regions:
        if rid in targets and rid not in ordering:
            ordering.append(rid)
    for rid in active_regions:
        if rid not in ordering:
            ordering.append(rid)

    for rid in ordering[:remainder]:
        targets[rid] += 1

    return targets


def _best_destination(robot, counts, targets, active_regions, regions):
    candidates = []
    for rid in active_regions:
        deficit = targets[rid] - counts.get(rid, 0)
        if deficit <= 0 or rid == robot.assigned_region:
            continue
        d = distance(robot.pos, region_center(regions[rid]))
        # Distance dominates. Deficit is a bounded priority boost so an empty
        # newly activated task is preferred without overwhelming proximity.
        score = d - deficit * 45.0
        candidates.append((score, d, rid, deficit))

    if not candidates:
        return None
    return min(candidates, key=lambda item: (item[0], item[1], item[2]))


def _target_has_bid_capacity(robot, target, own_score, deficit, epoch):
    """Return whether this robot could currently fit in target's top-K bids."""
    if deficit <= 0:
        return False

    others = [
        bid for rid, bid in robot.known_bids.items()
        if rid != robot.id
        and isinstance(bid, dict)
        and bid.get("epoch") == epoch
        and bid.get("target") == target
    ]
    others.sort(key=lambda b: (b.get("score", math.inf), b.get("robot_id", 10**9)))

    if len(others) < deficit:
        return True

    kth = others[deficit - 1]
    own_key = (own_score, robot.id)
    kth_key = (kth.get("score", math.inf), kth.get("robot_id", 10**9))
    return own_key <= kth_key


def build_local_bid(
    robot,
    active_regions,
    regions,
    preferred_extra_regions,
    election_epoch,
    now,
    bid_round,
):
    """Return this robot's current bid, or None.

    Destination choice is a distributed auction: once the best-K positions
    for a nearby underfilled region are already occupied by better bids, a
    robot moves on to another deficit instead of having everyone pile into the
    first destination they noticed.
    """
    if robot.assigned_region not in active_regions:
        return None
    if robot.migrated_epoch == election_epoch:
        return None
    if robot.migration_cooldown_until > now:
        return None

    counts = counts_from_knowledge(robot, active_regions)
    targets = compute_target_counts(robot, active_regions, preferred_extra_regions)
    current = robot.assigned_region

    if counts.get(current, 0) <= targets.get(current, 0):
        return None

    # Do not let the robot's stale previous bid block its own new choice.
    robot.known_bids.pop(robot.id, None)

    candidates = []
    for target in active_regions:
        if target == current:
            continue
        deficit = targets[target] - counts.get(target, 0)
        if deficit <= 0:
            continue

        d = distance(robot.pos, region_center(regions[target]))

        # Closer destinations are preferred. A bounded deficit term favours
        # genuinely empty regions without overpowering proximity.
        score = d - deficit * 45.0

        if _target_has_bid_capacity(
            robot,
            target,
            score,
            deficit,
            election_epoch,
        ):
            candidates.append((score, d, target, deficit))

    if not candidates:
        return None

    score, d, target, deficit = min(
        candidates,
        key=lambda item: (item[0], item[1], item[2]),
    )

    return {
        "robot_id": robot.id,
        "epoch": election_epoch,
        "round": bid_round,
        "target": target,
        "score": score,
        "distance": d,
        "deficit": deficit,
        "source_region": current,
    }

def merge_bid_knowledge(robot, neighbour_states, epoch):
    """Merge the newest bid from each robot in the current election epoch."""
    merged = dict(robot.known_bids)

    for neighbour in neighbour_states:
        table = neighbour.get("known_bids", {})
        for rid, raw in table.items():
            if not isinstance(raw, dict):
                continue
            if raw.get("epoch") != epoch:
                continue

            old = merged.get(rid)
            if old is None:
                merged[rid] = dict(raw)
                continue

            old_round = old.get("round", -1)
            new_round = raw.get("round", -1)
            if new_round > old_round:
                merged[rid] = dict(raw)
            elif new_round == old_round:
                # Same robot/round cannot legitimately have two different
                # proposals. Prefer the non-tombstone proposal if encountered.
                if old.get("target") is None and raw.get("target") is not None:
                    merged[rid] = dict(raw)

    robot.known_bids = merged
    return merged

def top_k_bids_for_target(robot, target, k, epoch, bid_round):
    if k <= 0:
        return []

    bids = [
        bid for bid in robot.known_bids.values()
        if bid.get("epoch") == epoch
        and bid.get("round") == bid_round
        and bid.get("target") == target
    ]
    bids.sort(key=lambda b: (b.get("score", math.inf), b.get("robot_id", 10**9)))
    return bids[:k]

def local_voting_round(
    robots,
    neighbour_map,
    active_regions,
    regions,
    preferred_extra_regions,
    election_epoch,
    now,
    decision_round,
    warmup_rounds,
):
    """One synchronous peer-to-peer auction round.

    During warm-up, robots only exchange bids and recompute them from the
    previous round's information. Once the warm-up is complete, the swarm
    elects the top-K bids already propagated through the communication graph.
    """
    active_regions = sorted(active_regions)

    # Receive current assignment/bid messages first.
    for robot in robots:
        neighbours = neighbour_map.get(robot, [])
        merge_assignment_knowledge(robot, neighbours)
        merge_bid_knowledge(robot, neighbours, election_epoch)

    # Warm-up: publish a fresh bid for the current round. A tombstone prevents
    # stale bids from surviving after a robot is no longer surplus.
    if decision_round < warmup_rounds:
        for robot in robots:
            robot.known_bids.pop(robot.id, None)
            bid = build_local_bid(
                robot,
                active_regions,
                regions,
                preferred_extra_regions,
                election_epoch,
                now,
                decision_round,
            )
            if bid is None:
                robot.known_bids[robot.id] = {
                    "robot_id": robot.id,
                    "epoch": election_epoch,
                    "round": decision_round,
                    "target": None,
                    "score": math.inf,
                }
            else:
                robot.known_bids[robot.id] = bid
        return {}

    # Decision uses the most recently propagated complete-ish round.
    bid_round = warmup_rounds - 1
    winners = {}

    for robot in robots:
        own = robot.known_bids.get(robot.id)
        if own is None:
            continue
        if own.get("epoch") != election_epoch:
            continue
        if own.get("round") != bid_round:
            continue
        if own.get("target") is None:
            continue
        if robot.migrated_epoch == election_epoch:
            continue

        counts = counts_from_knowledge(robot, active_regions)
        targets = compute_target_counts(
            robot,
            active_regions,
            preferred_extra_regions,
        )
        target = own["target"]
        deficit = targets.get(target, 0) - counts.get(target, 0)
        if deficit <= 0:
            continue

        top = top_k_bids_for_target(
            robot,
            target,
            deficit,
            election_epoch,
            bid_round,
        )
        top_ids = {bid.get("robot_id") for bid in top}
        if robot.id in top_ids:
            winners[robot] = target

    return winners

def apply_migrations(robots, migrations, election_epoch, now):
    """Apply elected robot-local decisions only."""
    for robot, target in migrations.items():
        old_region = robot.assigned_region
        robot.assigned_region = target
        robot.assignment_version += 1
        robot.known_assignments[robot.id] = (
            robot.assignment_version,
            target,
        )
        robot.migrated_epoch = election_epoch
        robot.migration_cooldown_until = now + 5.0
        robot.migration_claim = None
        robot.known_bids.pop(robot.id, None)

        robot._migration_previous_region = old_region
        robot.formation_slot_index = None
        robot.flock_anchor_px = None
        robot.patrol_target_px = None
        robot.path = []
        robot.state = "idle"
        robot.speed = 0.0
        robot.smoothed_steer = (0.0, 0.0)


def mark_local_assignment(robot, region_id):
    robot.assigned_region = region_id
    robot.assignment_version += 1
    robot.known_assignments[robot.id] = (
        robot.assignment_version,
        region_id,
    )


def expire_claims(robots, now):
    for robot in robots:
        claim = robot.migration_claim
        if claim is not None and claim.get("expires", 0.0) <= now:
            robot.migration_claim = None
