# swarm_equal_share_v3

This version changes the swarm from region-wide static formation planning to a compact flock/cluster model with finite local election epochs.

## Behaviour

- 1 active region, 10 robots -> 10 robots in that region.
- 2 active regions, 10 robots -> 5 / 5.
- 3 active regions, 10 robots -> 3 / 3 / 4, with the newest region receiving the remainder first.
- 4 active regions, 10 robots -> 2 / 2 / 3 / 3, with the newest region favoured for the remainder.

When a new region is activated:

1. Existing assignments are frozen.
2. A finite 5-round election begins.
3. Only surplus robots can volunteer.
4. Each surplus robot locally chooses an underfilled destination, preferring the closest one.
5. Nearby candidates for the same destination compare bids.
6. A winning robot migrates once during that election.
7. Existing robots keep their current cluster slot.
8. After the election rounds end, the swarm settles.

The shared assignment information is represented as a distributed gossip table carried by robots. There is no central robot/controller choosing individual assignments.

## Important

The repository's existing `regions.py`, `grid.py`, `astar.py`, `path_utils.py`, `obstacle_avoidance.py`, `boids.py`, and application entry point are not replaced by this package. Copy the four files in this folder over the matching files in the existing project.
