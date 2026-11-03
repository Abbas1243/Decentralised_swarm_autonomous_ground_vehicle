"""Central configuration for the decentralized swarm simulation."""
import math

# --- Grid / window ---
CELL_SIZE = 16
GRID_COLS = 70
GRID_ROWS = 42
UI_BAR_HEIGHT = 70
WINDOW_WIDTH = GRID_COLS * CELL_SIZE
WINDOW_HEIGHT = GRID_ROWS * CELL_SIZE + UI_BAR_HEIGHT
FPS = 30

# --- Regions ---
REGION_ROWS = 2
REGION_COLS = 3
NUM_REGIONS = REGION_ROWS * REGION_COLS

# --- Obstacles ---
NUM_OBSTACLE_BLOBS = 16
OBSTACLE_BLOB_MIN = 3
OBSTACLE_BLOB_MAX = 7
SPAWN_CLEAR_RADIUS_CELLS = 4

# --- Robots ---
INITIAL_ROBOT_COUNT = 10
MAX_ROBOTS = 40
ROBOT_RADIUS = 6.0
MAX_SPEED = 95.0
MAX_TURN_RATE = math.radians(220)

# Collision neighbourhood: movement/safety only.
NEIGHBOR_RADIUS = 80.0
SEPARATION_RADIUS = 30.0
CRITICAL_DISTANCE = 15.0

# Communication/election neighbourhood. This is deliberately larger than the
# movement radius so adjacent regional clusters can exchange task-load gossip.
CONSENSUS_NEIGHBOR_RADIUS = 430.0

# --- Election protocol ---
ELECTION_ROUND_INTERVAL = 0.8
ELECTION_ROUNDS = 4
ELECTION_WARMUP_ROUNDS = 3
MIGRATION_COOLDOWN = 5.0
CLAIM_TTL = 2.0

# A migration is only proposed by a region that is above its equal-share target.
# The deficit of a destination increases the usefulness of a candidate.
DISTANCE_WEIGHT = 1.0
DEFICIT_WEIGHT = 45.0

# --- Navigation ---
WAYPOINT_REACHED_DIST = 16.0
FINAL_TARGET_REACHED_DIST = 12.0
MAX_SPEED_NEAR_TARGET = 42.0
STEERING_SMOOTHING = 0.32
WEIGHT_GOAL = 2.2
WEIGHT_SEPARATION = 1.5
WEIGHT_OBSTACLE = 2.0
EMERGENCY_CREEP_SPEED = 22.0

# --- Obstacle avoidance ---
OBSTACLE_AVOID_RADIUS = 34.0
OBSTACLE_CRITICAL_DISTANCE = 11.0

# --- Compact flock/cluster around each active region's anchor ---
FLOCK_SLOT_COUNT = 40
FLOCK_SPACING = 30.0
FLOCK_RING_START = 1
FLOCK_CENTER_MARGIN = 35.0

# Safety radius around target anchor. Slots are compact; this is not a
# region-wide formation grid.
FLOCK_RADIUS = 95.0

# --- UI ---
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
