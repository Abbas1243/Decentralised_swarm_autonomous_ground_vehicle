A pure-Python 2D multi-robot surveillance simulation built with
Pygame.

The project is a lightweight alternative to the original Webots/ROS2
swarm setup. It focuses on swarm behaviour, task allocation, navigation,
formation and collision avoidance without requiring ROS2, Webots, or
external robot controllers.

Current Behaviour

The simulation contains six fixed surveillance regions arranged as a 2x3
grid.

Current flow:

Activate region(s)
      ↓
Balance robots across active regions
      ↓
Assign each robot a formation slot
      ↓
Plan an A* route
      ↓
Navigate while avoiding obstacles/robots
      ↓
Reach formation position
      ↓
Hold position
      ↓
Align heading with robots in the same region

Robots with no active task remain stationary. They do not randomly
wander or continuously patrol.

Project Structure

swarm_2d/
├── main.py
├── simulation.py
├── robot.py
├── boids.py
├── config.py
├── grid.py
├── obstacles.py
├── obstacle_avoidance.py
├── astar.py
├── path_utils.py
├── regions.py
└── README.md

main.py

Application entry point and Pygame event loop.

simulation.py

Main swarm controller. Handles:

robot lifecycle

region activation

balanced region allocation

formation slot generation

slot assignment

A* path planning

movement

heading alignment

collision handling

rendering

robot.py

Stores individual robot state.

States:

IDLE → MOVING → HOLDING

IDLE means no task is assigned.

MOVING means the robot is travelling to its assigned formation slot.

HOLDING means the robot has reached its slot and should remain there.

boids.py

Contains the basic:

separation

alignment

cohesion

calculations used by the motion system.

Boids are primarily used while robots are travelling. Classical cohesion
is not used to make holding robots continuously flock together.

config.py

Central configuration for:

grid dimensions

robot counts

speed and turning

neighbour/separation distances

obstacle distances

steering weights

colours

rendering parameters

grid.py

Converts between grid cells and pixel coordinates and clamps positions
to the map.

obstacles.py

Generates random obstacles and provides free-cell utilities. The
generated map maintains connectivity between important areas.

obstacle_avoidance.py

Provides local obstacle awareness.

It currently contains:

Soft repulsion from nearby obstacles.

Hard emergency protection when the robot body gets too close to
an obstacle.

Obstacle cells are treated as actual rectangles rather than point
centres, and robot radius is considered when calculating clearance.

astar.py

Grid-based A* pathfinding around blocked cells.

path_utils.py

Simplifies A* paths into fewer pixel-space waypoints.

regions.py

Defines the six fixed surveillance regions.

Region Allocation

The previous implementation assigned robots only according to the
nearest active region. That could produce highly uneven distributions.

The current implementation instead creates quotas.

For example:

10 robots / 2 active regions

Region 2 → 5
Region 4 → 5

or:

10 robots / 3 active regions

Region 2 → 4
Region 4 → 3
Region 5 → 3

Distance is still considered when assigning robots, but active regions
receive approximately equal populations.

Formation / Coverage

Robots are no longer given random points inside their region.

Each region generates explicit formation slots based on the number of
assigned robots.

Example:

4 robots

┌─────────────────────┐
│                     │
│      ●       ●      │
│                     │
│      ●       ●      │
│                     │
└─────────────────────┘

Six robots:

┌─────────────────────┐
│   ●      ●      ●   │
│                     │
│   ●      ●      ●   │
└─────────────────────┘

The slot generator:

adapts to robot count

spreads robots across the region

keeps them away from region boundaries

checks obstacles

moves blocked slots to the nearest free point

Each robot receives one unique slot.

Movement

A* provides the global route:

Robot → A* → formation slot

Local steering then modifies the movement to account for:

nearby robots

obstacles

heading changes

separation

The robot does not continuously roam after reaching its target.

Heading Alignment

Heading alignment is different from classical flocking.

The objective is:

Robots in the same surveillance formation should face approximately
the same direction while remaining spatially separated.

Example:

       ↑       ↑       ↑
       ●       ●       ●

       ↑       ↑       ↑
       ●       ●       ●

Holding robots do not use cohesion to move toward one another. They
remain stationary unless safety correction is required.

Obstacle Avoidance

Obstacle handling uses several layers.

1. A* pathfinding

A* knows which grid cells are blocked and provides the main route.

2. Local repulsion

Nearby obstacle cells produce a repulsive steering vector.

The current implementation measures the distance to the actual
obstacle-cell rectangle, rather than simply measuring centre-to-centre
distance.

Avoidance becomes stronger as the robot approaches the obstacle.

3. Robot body clearance

ROBOT_RADIUS is included in obstacle calculations.

This prevents treating the robot as a dimensionless point.

4. Emergency protection

When the robot becomes dangerously close to an obstacle, the simulation
restricts it to an emergency creep speed and uses the repulsion
direction to escape.

Robot Collision Avoidance

Robots use separation while travelling and an emergency correction when
they become critically close.

The intended priority is:

A* navigation
     ↓
local steering
     ↓
obstacle avoidance
     ↓
robot separation
     ↓
emergency correction

Collision avoidance is still an active development area. Predictive
velocity-based collision handling is a planned improvement.

Controls

1 - 6

Toggle surveillance regions.

A

Add a robot.

R

Regenerate the map and reset the robot population.

Left click

Remove the nearest robot to the clicked location.

Esc

Quit.

Installation

Python 3.x and Pygame are required.

pip install pygame

If a requirements file is present:

pip install -r requirements.txt

Run:

python main.py