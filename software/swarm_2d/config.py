"""
Central configuration for the swarm_2d simulation.
Tune these to change map size, robot counts, and behaviour weights.
"""
import math

# --- Grid / window ---
CELL_SIZE = 16
GRID_COLS = 70
GRID_ROWS = 42
UI_BAR_HEIGHT = 70
WINDOW_WIDTH = GRID_COLS * CELL_SIZE
WINDOW_HEIGHT = GRID_ROWS * CELL_SIZE + UI_BAR_HEIGHT

FPS = 30

# --- Regions (fixed grid of surveillance zones) ---
REGION_ROWS = 2
REGION_COLS = 3
NUM_REGIONS = REGION_ROWS * REGION_COLS

# --- Obstacles ---
NUM_OBSTACLE_BLOBS = 16
OBSTACLE_BLOB_MIN = 3
OBSTACLE_BLOB_MAX = 7
SPAWN_CLEAR_RADIUS_CELLS = 4   # keep the robot spawn area obstacle-free

# --- Robots ---
INITIAL_ROBOT_COUNT = 10
MAX_ROBOTS = 40
ROBOT_RADIUS = 6.0

MAX_SPEED = 95.0                    # pixels / second
MAX_TURN_RATE = math.radians(220)   # radians / second

NEIGHBOR_RADIUS = 75.0
SEPARATION_RADIUS = 24.0
CRITICAL_DISTANCE = 13.0            # hard emergency stop vs other robots

OBSTACLE_AVOID_RADIUS = 34.0        # soft repulsion field around obstacle cells
OBSTACLE_CRITICAL_DISTANCE = 11.0   # emergency creep-speed threshold vs obstacles

# When inside a critical distance, robots don't freeze to an absolute zero --
# two robots freezing simultaneously would deadlock forever, since neither
# could move to increase the distance again. Instead they're capped to a
# slow creep, steered by whatever separation/repulsion already pushed their
# heading toward, which reliably resolves the conflict within a few frames.
EMERGENCY_CREEP_SPEED = 16.0

WEIGHT_SEPARATION = 1.7
WEIGHT_ALIGNMENT = 0.5
WEIGHT_COHESION = 0.35
WEIGHT_GOAL = 2.4
WEIGHT_OBSTACLE = 2.2
WEIGHT_WANDER = 0.6                 # used only when a robot has no assigned region

WAYPOINT_REACHED_DIST = 24.0        # must exceed the robot's minimum turning
                                     # radius (speed/turn_rate ~ 24.5px) or a
                                     # tight-angle waypoint can never actually
                                     # be reached -- the robot orbits it forever
PATROL_RETARGET_SEC = 7.0           # how often a robot picks a new point inside its region

# Raw per-frame steering (separation/goal/obstacle summed fresh each tick) is
# noisy when several robots crowd the same narrow corridor -- the direction
# can flicker as the local crowd shifts, causing the heading to visibly hunt
# back and forth instead of converging on the goal. Low-pass filtering the
# steering vector across frames (exponential blend with the previous frame's
# vector) removes that jitter without weakening any individual force.
STEERING_SMOOTHING = 0.22           # fraction of the new vector blended in per frame

# --- Colors ---
COLOR_BG = (18, 20, 26)
COLOR_GRID_LINE = (28, 31, 38)
COLOR_OBSTACLE = (90, 95, 105)
COLOR_UI_BG = (12, 13, 17)
COLOR_TEXT = (225, 228, 232)
COLOR_TEXT_DIM = (140, 145, 155)
COLOR_ROBOT = (80, 200, 255)
COLOR_ROBOT_IDLE = (120, 125, 135)
COLOR_PATH = (80, 200, 255)

REGION_ACTIVE_FILL = (46, 90, 60)
REGION_INACTIVE_FILL = (32, 34, 40)
REGION_BORDER = (70, 75, 85)
REGION_LABEL = (200, 205, 210)

REGION_COLORS = [
    (231, 111, 81),
    (233, 196, 106),
    (42, 157, 143),
    (38, 70, 83),
    (244, 162, 97),
    (129, 178, 154),
]
