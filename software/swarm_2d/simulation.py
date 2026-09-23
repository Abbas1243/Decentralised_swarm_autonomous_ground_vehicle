"""
Main simulation controller.

Behaviour:

    - Active regions are surveillance tasks.
    - Robots are distributed across active regions as evenly as possible.
    - Each robot receives a specific formation slot inside its region.
    - A* is used to travel to that slot.
    - Once the slot is reached, the robot holds position.
    - Robots in the same region gradually align their headings.
    - Separation is used only as a safety correction.
    - Robots do not randomly wander or continuously patrol.
"""

import math
import random

import pygame

import config
import grid as gridmod
import obstacles as obstaclemod
import astar
import path_utils
import boids

from obstacle_avoidance import repulsion_vector, too_close_to_obstacle
from regions import build_regions
from robot import Robot


def _wrap_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class Simulation:
    def __init__(self, seed=None):
        self.rng = random.Random(seed)

        self.regions = build_regions()
        self.active_ids = set()
        self.robots = []
        self.blocked = set()

        self.spawn_cell = (
            config.GRID_ROWS // 2,
            config.GRID_COLS // 2
        )

        self.reset_map(keep_active=False)

    # ============================================================
    # MAP / POPULATION LIFECYCLE
    # ============================================================

    def reset_map(self, keep_active=True):
        region_centers = [
            r.center_cell()
            for r in self.regions
        ]

        self.blocked = obstaclemod.generate_obstacle_grid(
            self.spawn_cell,
            region_centers,
            rng=self.rng
        )

        if not keep_active:
            self.active_ids = set()

            for region in self.regions:
                region.active = False

        self.robots = []

        for _ in range(config.INITIAL_ROBOT_COUNT):
            self.add_robot(replan=False)

        # Assign every robot to a balanced set of regions.
        self._rebalance_assignments()

    def add_robot(self, replan=True):
        if len(self.robots) >= config.MAX_ROBOTS:
            return None

        free_cell = obstaclemod.nearest_free_cell(
            self.blocked,
            self.spawn_cell
        )

        spawn_px = gridmod.cell_center_px(free_cell)

        pos = self._find_spawn_slot(spawn_px)

        robot = Robot(
            pos,
            rng=self.rng
        )

        self.robots.append(robot)

        if replan:
            self._rebalance_assignments()

        return robot

    def _find_spawn_slot(self, center_px, attempts=60):
        """
        Places a newly created robot near the spawn point without
        stacking it directly on another robot.
        """

        min_gap = config.CRITICAL_DISTANCE * 1.8

        last_candidate = center_px

        for attempt in range(attempts):
            radius = 10 + (attempt // 6) * 10

            angle = self.rng.uniform(
                0,
                2 * math.pi
            )

            x = (
                center_px[0]
                + math.cos(angle) * radius
            )

            y = (
                center_px[1]
                + math.sin(angle) * radius
            )

            x, y = gridmod.clamp_px(x, y)

            last_candidate = (x, y)

            if gridmod.cell_of((x, y)) in self.blocked:
                continue

            if all(
                math.hypot(
                    x - robot.pos[0],
                    y - robot.pos[1]
                ) >= min_gap
                for robot in self.robots
            ):
                return (x, y)

        return gridmod.clamp_px(
            last_candidate[0],
            last_candidate[1]
        )

    def kill_nearest_robot(self, click_pos, max_dist=18.0):
        if not self.robots:
            return

        best = min(
            self.robots,
            key=lambda r: math.hypot(
                r.pos[0] - click_pos[0],
                r.pos[1] - click_pos[1]
            )
        )

        distance = math.hypot(
            best.pos[0] - click_pos[0],
            best.pos[1] - click_pos[1]
        )

        if distance <= max_dist:
            self.robots.remove(best)

            # Rebalance the remaining robots after removal.
            self._rebalance_assignments()

    def toggle_region(self, region_id):
        region = self.regions[region_id]

        region.active = not region.active

        if region.active:
            self.active_ids.add(region_id)
        else:
            self.active_ids.discard(region_id)

        # Changing active regions can completely change the desired
        # distribution, so recalculate all assignments.
        self._rebalance_assignments()

    # ============================================================
    # REGION ALLOCATION
    # ============================================================

    def _rebalance_assignments(self):
        """
        Allocate active surveillance regions through the consensus layer.

        Consensus decides ONLY which region each robot should cover.
        Formation slots and A* remain exactly where they were.

        This keeps the architecture separated:

            consensus.py -> WHAT region should I cover?
            formation    -> WHERE inside that region?
            A*           -> HOW do I get there?
            boids        -> HOW do I move safely with neighbours?
        """

        if not self.robots:
            return

        # --------------------------------------------------------
        # No active region = every robot becomes idle.
        # --------------------------------------------------------

        if not self.active_ids:
            for robot in self.robots:
                robot.assigned_region = None
                robot.path = []
                robot.patrol_target_px = None
                robot.state = Robot.IDLE
                robot.speed = 0.0
                robot.smoothed_steer = (0.0, 0.0)

            return

        active_regions = sorted(self.active_ids)

        # --------------------------------------------------------
        # CONSENSUS
        # --------------------------------------------------------
        #
        # Each robot forms a local opinion using nearby robot state.
        # The consensus module resolves conflicts and preserves the
        # same balanced population target as the previous allocator.
        #
        # Import is local so consensus.py remains optional and easy
        # to replace while the rest of the simulator stays intact.
        # --------------------------------------------------------

        from consensus import allocate_regions

        assignments = allocate_regions(
            robots=self.robots,
            active_regions=active_regions,
            regions=self.regions,
            neighbor_radius=config.NEIGHBOR_RADIUS,
            rounds=3,
        )

        # --------------------------------------------------------
        # Apply region assignments.
        # --------------------------------------------------------

        region_members = {
            region_id: []
            for region_id in active_regions
        }

        for robot in self.robots:
            new_region = assignments.get(robot)

            # Safety fallback. This should never be needed, but a
            # robot must never be left without a valid assignment
            # when active regions exist.
            if new_region not in region_members:
                new_region = min(
                    active_regions,
                    key=lambda region_id: math.hypot(
                        self.regions[region_id].center_px()[0]
                        - robot.pos[0],
                        self.regions[region_id].center_px()[1]
                        - robot.pos[1],
                    ),
                )

            robot.assigned_region = new_region
            robot.state = Robot.MOVING
            robot.speed = 0.0
            robot.smoothed_steer = (0.0, 0.0)
            robot.path = []

            region_members[new_region].append(robot)

        # --------------------------------------------------------
        # Formation slots remain unchanged.
        # --------------------------------------------------------

        for region_id in active_regions:
            members = region_members[region_id]

            slots = self._formation_slots(
                region_id,
                len(members)
            )

            self._assign_slots_to_robots(
                members,
                slots
            )

    # ============================================================
    # FORMATION SLOT GENERATION
    # ============================================================

    def _formation_slots(self, region_id, count):
        """
        Generate evenly distributed positions inside a region.

        The positions form a grid whose shape adapts to the number
        of robots.

        Examples:

            2 robots:

                ●       ●


            4 robots:

                ●       ●

                ●       ●


            6 robots:

                ●    ●    ●

                ●    ●    ●
        """

        if count <= 0:
            return []

        region = self.regions[region_id]

        x, y, width, height = region.rect_px

        # Keep robots away from the exact region boundaries.
        margin_x = max(
            35.0,
            min(70.0, width * 0.12)
        )

        margin_y = max(
            35.0,
            min(55.0, height * 0.12)
        )

        usable_width = max(
            20.0,
            width - 2.0 * margin_x
        )

        usable_height = max(
            20.0,
            height - 2.0 * margin_y
        )

        # Choose a roughly square formation.
        aspect = usable_width / max(
            usable_height,
            1.0
        )

        columns = int(
            math.ceil(
                math.sqrt(count * aspect)
            )
        )

        columns = max(
            1,
            min(count, columns)
        )

        rows = int(
            math.ceil(
                count / columns
            )
        )

        slots = []

        for row in range(rows):
            remaining = count - row * columns

            if remaining <= 0:
                break

            robots_in_row = min(
                columns,
                remaining
            )

            # Evenly distribute rows vertically.
            if rows == 1:
                row_y = y + height * 0.5
            else:
                row_y = (
                    y
                    + margin_y
                    + (
                        row + 0.5
                    )
                    * (
                        usable_height / rows
                    )
                )

            # Center incomplete rows.
            if robots_in_row == 1:
                row_x_positions = [
                    x + width * 0.5
                ]

            else:
                spacing_x = (
                    usable_width
                    / (robots_in_row - 1)
                )

                start_x = (
                    x
                    + width * 0.5
                    - spacing_x
                    * (robots_in_row - 1)
                    / 2.0
                )

                row_x_positions = [
                    start_x + i * spacing_x
                    for i in range(robots_in_row)
                ]

            for slot_x in row_x_positions:
                requested = (
                    slot_x,
                    row_y
                )

                free_point = (
                    self._nearest_free_point_in_region(
                        region,
                        requested
                    )
                )

                slots.append(free_point)

        return slots

    def _nearest_free_point_in_region(
        self,
        region,
        target_px
    ):
        """
        Find the closest free grid cell to target_px while staying
        inside the same surveillance region.

        This prevents an obstacle from destroying the formation.
        """

        rx, ry, rw, rh = region.rect_px

        target_cell = gridmod.cell_of(target_px)

        min_col = max(
            0,
            int(rx // config.CELL_SIZE)
        )

        max_col = min(
            config.GRID_COLS - 1,
            int(
                (rx + rw - 1)
                // config.CELL_SIZE
            )
        )

        min_row = max(
            0,
            int(ry // config.CELL_SIZE)
        )

        max_row = min(
            config.GRID_ROWS - 1,
            int(
                (ry + rh - 1)
                // config.CELL_SIZE
            )
        )

        best_cell = None
        best_distance = math.inf

        for row in range(min_row, max_row + 1):
            for col in range(min_col, max_col + 1):

                cell = (row, col)

                if cell in self.blocked:
                    continue

                cx, cy = gridmod.cell_center_px(cell)

                distance = math.hypot(
                    cx - target_px[0],
                    cy - target_px[1]
                )

                if distance < best_distance:
                    best_distance = distance
                    best_cell = cell

        if best_cell is None:
            # Extremely unlikely because obstacle generation keeps
            # the regions connected.
            return target_px

        return gridmod.cell_center_px(
            best_cell
        )

    def _assign_slots_to_robots(
        self,
        robots,
        slots
    ):
        """
        Give each robot one unique formation slot.

        Greedy nearest-slot assignment keeps robots from making
        unnecessary crossings when a formation is recalculated.
        """

        remaining_slots = list(slots)

        unassigned = list(robots)

        while unassigned and remaining_slots:

            best_robot = None
            best_slot = None
            best_distance = math.inf

            for robot in unassigned:
                for slot in remaining_slots:

                    distance = math.hypot(
                        slot[0] - robot.pos[0],
                        slot[1] - robot.pos[1]
                    )

                    if distance < best_distance:
                        best_distance = distance
                        best_robot = robot
                        best_slot = slot

            best_robot.patrol_target_px = best_slot

            self._plan_target(
                best_robot,
                best_slot
            )

            unassigned.remove(best_robot)
            remaining_slots.remove(best_slot)

    # ============================================================
    # PATH PLANNING
    # ============================================================

    def _plan_target(self, robot, target_px):
        """
        Create an A* route from the robot's current position to
        its assigned formation slot.
        """

        start_cell = robot.cell()

        goal_cell = obstaclemod.nearest_free_cell(
            self.blocked,
            gridmod.cell_of(target_px)
        )

        path_cells = astar.find_path(
            start_cell,
            goal_cell,
            self.blocked
        )

        if path_cells and len(path_cells) > 1:

            raw_waypoints = [
                gridmod.cell_center_px(cell)
                for cell in path_cells[1:]
            ]

            robot.path = (
                path_utils.simplify_waypoints_px(
                    raw_waypoints,
                    tuple(robot.pos),
                    self.blocked
                )
            )

        elif path_cells:
            robot.path = [target_px]

        else:
            # Connectivity should normally make this unnecessary.
            robot.path = [target_px]

        robot.patrol_target_px = target_px
        robot.time_since_retarget = 0.0
        robot.state = Robot.MOVING

    # ============================================================
    # PER-FRAME UPDATE
    # ============================================================

    def update(self, dt):
        # --------------------------------------------------------
        # Build neighbour information.
        # --------------------------------------------------------

        states = [
            robot.state_dict()
            for robot in self.robots
        ]

        for i, robot in enumerate(self.robots):

            neighbors = []

            for j, other in enumerate(self.robots):

                if i == j:
                    continue

                dx = (
                    other.pos[0]
                    - robot.pos[0]
                )

                dy = (
                    other.pos[1]
                    - robot.pos[1]
                )

                distance = math.hypot(
                    dx,
                    dy
                )

                if distance <= config.NEIGHBOR_RADIUS:

                    neighbor_state = dict(
                        states[j]
                    )

                    # Extra information used for formation
                    # alignment. boids.py simply ignores it.
                    neighbor_state["heading"] = (
                        other.heading
                    )

                    neighbor_state["state"] = (
                        other.state
                    )

                    neighbor_state["assigned_region"] = (
                        other.assigned_region
                    )

                    neighbors.append(
                        neighbor_state
                    )

            # ====================================================
            # IDLE
            # ====================================================

            if robot.state == Robot.IDLE:

                robot.speed = 0.0
                robot.smoothed_steer = (
                    0.0,
                    0.0
                )

                continue

            # ====================================================
            # MOVING
            # ====================================================

            if robot.state == Robot.MOVING:

                robot.advance_path_if_reached()

                # Robot may have arrived at its formation slot.
                if robot.state == Robot.HOLDING:

                    robot.speed = 0.0
                    robot.smoothed_steer = (
                        0.0,
                        0.0
                    )

                    continue

                goal_vec = (
                    0.0,
                    0.0
                )

                waypoint = robot.next_waypoint()

                if waypoint is not None:

                    dx = (
                        waypoint[0]
                        - robot.pos[0]
                    )

                    dy = (
                        waypoint[1]
                        - robot.pos[1]
                    )

                    distance = math.hypot(
                        dx,
                        dy
                    )

                    if distance > 1e-6:

                        goal_vec = (
                            dx / distance
                            * config.WEIGHT_GOAL,

                            dy / distance
                            * config.WEIGHT_GOAL
                        )

                # Boids are still allowed during travel, but their
                # job is mainly collision avoidance here.
                boids_vec = boids.blended_boids(
                    tuple(robot.pos),
                    robot.velocity,
                    neighbors
                )

                obst_vec = repulsion_vector(
                    robot.pos,
                    self.blocked
                )

                obst_vec = (
                    obst_vec[0]
                    * config.WEIGHT_OBSTACLE,

                    obst_vec[1]
                    * config.WEIGHT_OBSTACLE
                )

                combined = (
                    boids_vec[0]
                    + goal_vec[0]
                    + obst_vec[0],

                    boids_vec[1]
                    + goal_vec[1]
                    + obst_vec[1]
                )

                # ------------------------------------------------
                # Steering smoothing
                # ------------------------------------------------

                s = config.STEERING_SMOOTHING

                combined = (
                    robot.smoothed_steer[0]
                    * (1.0 - s)
                    + combined[0] * s,

                    robot.smoothed_steer[1]
                    * (1.0 - s)
                    + combined[1] * s
                )

                robot.smoothed_steer = combined

                magnitude = math.hypot(
                    combined[0],
                    combined[1]
                )

                if magnitude > 1e-4:

                    desired_heading = math.atan2(
                        combined[1],
                        combined[0]
                    )

                    diff = _wrap_angle(
                        desired_heading
                        - robot.heading
                    )

                    max_step = (
                        config.MAX_TURN_RATE
                        * dt
                    )

                    diff = max(
                        -max_step,
                        min(max_step, diff)
                    )

                    robot.heading = _wrap_angle(
                        robot.heading + diff
                    )

                    turn_factor = max(
                        0.15,
                        math.cos(diff)
                    )

                    robot.speed = (
                        config.MAX_SPEED
                        * turn_factor
                    )

                else:

                    robot.speed *= 0.8

                # ------------------------------------------------
                # Emergency collision protection
                # ------------------------------------------------

                emergency = (
                    too_close_to_obstacle(
                        robot.pos,
                        self.blocked
                    )
                )

                if not emergency:

                    for neighbor in neighbors:

                        if math.hypot(
                            neighbor["position"][0]
                            - robot.pos[0],

                            neighbor["position"][1]
                            - robot.pos[1]
                        ) < config.CRITICAL_DISTANCE:

                            emergency = True
                            break

                if emergency:

                    robot.speed = min(
                        robot.speed,
                        config.EMERGENCY_CREEP_SPEED
                    )

                # ------------------------------------------------
                # Move robot
                # ------------------------------------------------

                vx, vy = robot.velocity

                new_x = (
                    robot.pos[0]
                    + vx * dt
                )

                new_y = (
                    robot.pos[1]
                    + vy * dt
                )

                new_x, new_y = gridmod.clamp_px(
                    new_x,
                    new_y
                )

                robot.pos[0] = new_x
                robot.pos[1] = new_y

                continue

            # ====================================================
            # HOLDING
            # ====================================================

            if robot.state == Robot.HOLDING:

                # A holding robot should not have translational
                # velocity under normal conditions.
                robot.speed = 0.0
                robot.smoothed_steer = (
                    0.0,
                    0.0
                )

                # ------------------------------------------------
                # FORMATION ALIGNMENT
                # ------------------------------------------------
                #
                # Only robots in the SAME REGION participate.
                #
                # We calculate a circular mean of their headings.
                # This means:
                #
                #       ↑   ↑   ↑
                #       ●   ●   ●
                #
                # rather than robots constantly moving toward one
                # another.
                # ------------------------------------------------

                same_region = [
                    other
                    for other in self.robots
                    if (
                        other is not robot
                        and
                        other.state == Robot.HOLDING
                        and
                        other.assigned_region
                        == robot.assigned_region
                    )
                ]

                if same_region:

                    heading_x = 0.0
                    heading_y = 0.0

                    for other in same_region:

                        heading_x += math.cos(
                            other.heading
                        )

                        heading_y += math.sin(
                            other.heading
                        )

                    heading_magnitude = math.hypot(
                        heading_x,
                        heading_y
                    )

                    if heading_magnitude > 1e-4:

                        desired_heading = math.atan2(
                            heading_y,
                            heading_x
                        )

                        diff = _wrap_angle(
                            desired_heading
                            - robot.heading
                        )

                        # Heading alignment is deliberately slower
                        # than navigation.
                        alignment_turn_rate = (
                            config.MAX_TURN_RATE
                            * 0.45
                        )

                        max_step = (
                            alignment_turn_rate
                            * dt
                        )

                        diff = max(
                            -max_step,
                            min(max_step, diff)
                        )

                        robot.heading = _wrap_angle(
                            robot.heading + diff
                        )

                # ------------------------------------------------
                # SAFETY CORRECTION
                # ------------------------------------------------
                #
                # Normally a holding robot remains completely still.
                #
                # If another robot gets dangerously close, use
                # separation to resolve the collision.
                # ------------------------------------------------

                emergency_vector = (
                    0.0,
                    0.0
                )

                emergency = False

                # Obstacle safety.
                if too_close_to_obstacle(
                    robot.pos,
                    self.blocked
                ):
                    emergency = True

                    obstacle_vec = repulsion_vector(
                        robot.pos,
                        self.blocked
                    )

                    emergency_vector = (
                        emergency_vector[0]
                        + obstacle_vec[0],

                        emergency_vector[1]
                        + obstacle_vec[1]
                    )

                # Robot-to-robot safety.
                for other in self.robots:

                    if other is robot:
                        continue

                    dx = (
                        robot.pos[0]
                        - other.pos[0]
                    )

                    dy = (
                        robot.pos[1]
                        - other.pos[1]
                    )

                    distance = math.hypot(
                        dx,
                        dy
                    )

                    if (
                        distance > 1e-6
                        and
                        distance
                        < config.CRITICAL_DISTANCE
                    ):

                        emergency = True

                        emergency_vector = (
                            emergency_vector[0]
                            + dx / distance,

                            emergency_vector[1]
                            + dy / distance
                        )

                # Only move if there is a genuine collision risk.
                if emergency:

                    magnitude = math.hypot(
                        emergency_vector[0],
                        emergency_vector[1]
                    )

                    if magnitude > 1e-6:

                        direction_x = (
                            emergency_vector[0]
                            / magnitude
                        )

                        direction_y = (
                            emergency_vector[1]
                            / magnitude
                        )

                        robot.heading = math.atan2(
                            direction_y,
                            direction_x
                        )

                        robot.speed = (
                            config.EMERGENCY_CREEP_SPEED
                        )

                        new_x = (
                            robot.pos[0]
                            + direction_x
                            * robot.speed
                            * dt
                        )

                        new_y = (
                            robot.pos[1]
                            + direction_y
                            * robot.speed
                            * dt
                        )

                        new_x, new_y = gridmod.clamp_px(
                            new_x,
                            new_y
                        )

                        robot.pos[0] = new_x
                        robot.pos[1] = new_y

    # ============================================================
    # DRAWING
    # ============================================================

    def draw(self, surface, font, small_font):
        surface.fill(
            config.COLOR_BG
        )

        self._draw_regions(
            surface,
            font
        )

        self._draw_obstacles(
            surface
        )

        for robot in self.robots:
            robot.draw(surface)

        self._draw_ui_bar(
            surface,
            font,
            small_font
        )

    def _draw_regions(self, surface, font):
        for region in self.regions:

            x, y, w, h = region.rect_px

            rect = pygame.Rect(
                x,
                y,
                w,
                h
            )

            fill = (
                config.REGION_ACTIVE_FILL
                if region.active
                else config.REGION_INACTIVE_FILL
            )

            pygame.draw.rect(
                surface,
                fill,
                rect
            )

            pygame.draw.rect(
                surface,
                config.REGION_BORDER,
                rect,
                width=2
            )

            label = font.render(
                region.label,
                True,
                config.REGION_LABEL
            )

            surface.blit(
                label,
                (x + 8, y + 6)
            )

    def _draw_obstacles(self, surface):
        for r, c in self.blocked:

            rect = pygame.Rect(
                c * config.CELL_SIZE,
                r * config.CELL_SIZE,
                config.CELL_SIZE,
                config.CELL_SIZE
            )

            pygame.draw.rect(
                surface,
                config.COLOR_OBSTACLE,
                rect
            )

    def _draw_ui_bar(
        self,
        surface,
        font,
        small_font
    ):
        bar_y = (
            config.GRID_ROWS
            * config.CELL_SIZE
        )

        bar_rect = pygame.Rect(
            0,
            bar_y,
            config.WINDOW_WIDTH,
            config.UI_BAR_HEIGHT
        )

        pygame.draw.rect(
            surface,
            config.COLOR_UI_BG,
            bar_rect
        )

        x = 12

        for region in self.regions:

            color = (
                config.REGION_COLORS[region.id]
                if region.active
                else config.COLOR_TEXT_DIM
            )

            box = pygame.Rect(
                x,
                bar_y + 10,
                26,
                26
            )

            pygame.draw.rect(
                surface,
                color,
                box,
                width=(
                    0
                    if region.active
                    else 2
                )
            )

            label = small_font.render(
                region.label,
                True,
                (
                    (10, 10, 10)
                    if region.active
                    else config.COLOR_TEXT_DIM
                )
            )

            surface.blit(
                label,
                (
                    x + 9,
                    bar_y + 14
                )
            )

            x += 36

        info = (
            f"Robots: {len(self.robots)}    "
            "(keys 1-6 toggle regions, "
            "click a robot to remove, "
            "A add, R reload, Esc quit)"
        )

        text = small_font.render(
            info,
            True,
            config.COLOR_TEXT
        )

        surface.blit(
            text,
            (
                x + 20,
                bar_y + 18
            )
        )