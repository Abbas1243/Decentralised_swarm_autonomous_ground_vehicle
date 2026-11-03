"""Main decentralized swarm simulation.

Task policy:
    * Active regions are surveillance tasks.
    * The swarm aims for an equal-share population per active region.
    * A region added during runtime does NOT trigger a global reassignment.
    * A finite election window runs for several local rounds.
    * Only surplus robots volunteer to move.
    * Nearby candidates elect locally; the closest useful robots tend to move.
    * Existing robots keep their current cluster slots unless they themselves
      migrate.

Movement policy:
    * A* takes each robot to a compact flock slot near its region anchor.
    * Travel uses only goal + short-range separation + obstacle avoidance.
    * Once there, the robot stays still except for genuine collision safety.
"""

import math
import random
import pygame

import config
import grid as gridmod
import obstacles as obstaclemod
import astar
import path_utils

from obstacle_avoidance import repulsion_vector, too_close_to_obstacle
from regions import build_regions
from robot import Robot
from consensus import (
    merge_assignment_knowledge,
    mark_local_assignment,
    local_voting_round,
    apply_migrations,
    expire_claims,
)


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

        # Election state. This is a coordination clock, not an assignment
        # controller: it simply says when local voting rounds may happen.
        self.election_active = False
        self.election_epoch = 0
        self.election_round = 0
        self.election_timer = 0.0
        self.preferred_extra_regions = []

        self.spawn_cell = (
            config.GRID_ROWS // 2,
            config.GRID_COLS // 2,
        )
        self.reset_map(keep_active=False)

    # ============================================================
    # MAP / POPULATION
    # ============================================================

    def reset_map(self, keep_active=True):
        region_centers = [r.center_cell() for r in self.regions]
        self.blocked = obstaclemod.generate_obstacle_grid(
            self.spawn_cell,
            region_centers,
            rng=self.rng,
        )

        if not keep_active:
            self.active_ids.clear()
            for region in self.regions:
                region.active = False

        self.robots = []
        self.election_active = False
        self.election_round = 0
        self.election_timer = 0.0
        self.election_epoch += 1

        for _ in range(config.INITIAL_ROBOT_COUNT):
            self.add_robot(replan=False)

    def add_robot(self, replan=True):
        if len(self.robots) >= config.MAX_ROBOTS:
            return None

        free_cell = obstaclemod.nearest_free_cell(self.blocked, self.spawn_cell)
        spawn_px = gridmod.cell_center_px(free_cell)
        pos = self._find_spawn_slot(spawn_px)
        robot = Robot(pos, rng=self.rng)
        self.robots.append(robot)

        # New robots know themselves. Existing knowledge tables learn the new
        # member through neighbour gossip during the next election.
        if self.active_ids and replan:
            nearest = min(
                self.active_ids,
                key=lambda rid: math.hypot(
                    robot.pos[0] - self.regions[rid].center_px()[0],
                    robot.pos[1] - self.regions[rid].center_px()[1],
                ),
            )
            mark_local_assignment(robot, nearest)
            self._prepare_robot_cluster_slot(robot)
            self._start_election(preferred=[nearest])

        return robot

    def _find_spawn_slot(self, center_px, attempts=80):
        min_gap = config.CRITICAL_DISTANCE * 1.8
        last_candidate = center_px
        for attempt in range(attempts):
            radius = 12 + (attempt // 6) * 10
            angle = self.rng.uniform(0, 2 * math.pi)
            x = center_px[0] + math.cos(angle) * radius
            y = center_px[1] + math.sin(angle) * radius
            x, y = gridmod.clamp_px(x, y)
            last_candidate = (x, y)
            if gridmod.cell_of((x, y)) in self.blocked:
                continue
            if all(
                math.hypot(x - r.pos[0], y - r.pos[1]) >= min_gap
                for r in self.robots
            ):
                return x, y
        return gridmod.clamp_px(*last_candidate)

    def kill_nearest_robot(self, click_pos, max_dist=18.0):
        if not self.robots:
            return
        best = min(
            self.robots,
            key=lambda r: math.hypot(
                r.pos[0] - click_pos[0],
                r.pos[1] - click_pos[1],
            ),
        )
        if math.hypot(best.pos[0] - click_pos[0], best.pos[1] - click_pos[1]) <= max_dist:
            dead_id = best.id
            self.robots.remove(best)
            for robot in self.robots:
                robot.known_assignments.pop(dead_id, None)
                robot.known_bids.pop(dead_id, None)
            if self.active_ids:
                self._start_election()

    # ============================================================
    # TASK ACTIVATION / ELECTION
    # ============================================================

    def toggle_region(self, region_id):
        region = self.regions[region_id]
        was_active = bool(self.active_ids)

        region.active = not region.active
        if region.active:
            self.active_ids.add(region_id)
        else:
            self.active_ids.discard(region_id)

        if not self.active_ids:
            for robot in self.robots:
                self._make_idle(robot)
            self.election_active = False
            return

        # First active task: everyone has only one possible task.
        if not was_active and self.active_ids:
            first_region = next(iter(self.active_ids))
            for robot in self.robots:
                mark_local_assignment(robot, first_region)
                robot.formation_slot_index = None
                self._prepare_robot_cluster_slot(robot)
            self.election_active = False
            return

        # Existing assignments remain untouched. The newly changed task set
        # starts a finite local election period.
        preferred = [region_id] if region.active else []
        self._start_election(preferred=preferred)

        # If a region was deactivated, its former members must immediately
        # become eligible to vote for another active region.
        if not region.active:
            for robot in self.robots:
                if robot.assigned_region == region_id:
                    robot.assigned_region = None
                    robot.assignment_version += 1
                    robot.known_assignments[robot.id] = (
                        robot.assignment_version,
                        None,
                    )
                    robot.formation_slot_index = None
                    robot.path = []
                    robot.state = Robot.IDLE
                    robot.speed = 0.0

    def _start_election(self, preferred=None):
        self.election_epoch += 1
        self.election_active = True
        self.election_round = 0
        self.election_timer = 0.0
        self.preferred_extra_regions = [
            rid for rid in (preferred or []) if rid in self.active_ids
        ]
        for robot in self.robots:
            robot.migrated_epoch = -1
            robot.migration_claim = None

    def _build_neighbor_map(self):
        result = {}
        for robot in self.robots:
            neighbours = []
            for other in self.robots:
                if other is robot:
                    continue
                d = math.hypot(
                    other.pos[0] - robot.pos[0],
                    other.pos[1] - robot.pos[1],
                )
                if d <= config.CONSENSUS_NEIGHBOR_RADIUS:
                    neighbours.append(other.state_dict())
            result[robot] = neighbours
        return result

    def _run_election_round(self):
        if not self.active_ids:
            self.election_active = False
            return

        neighbours = self._build_neighbor_map()
        for robot, states in neighbours.items():
            merge_assignment_knowledge(robot, states)

        winners = local_voting_round(
            robots=self.robots,
            neighbour_map=neighbours,
            active_regions=sorted(self.active_ids),
            regions=self.regions,
            preferred_extra_regions=self.preferred_extra_regions,
            election_epoch=self.election_epoch,
            now=self.election_round * config.ELECTION_ROUND_INTERVAL,
            decision_round=self.election_round,
            warmup_rounds=config.ELECTION_WARMUP_ROUNDS,
        )

        apply_migrations(
            self.robots,
            winners,
            self.election_epoch,
            self.election_round * config.ELECTION_ROUND_INTERVAL,
        )

        for robot in winners:
            self._prepare_robot_cluster_slot(robot)

        self.election_round += 1
        if self.election_round >= config.ELECTION_ROUNDS:
            self.election_active = False
            self.election_timer = 0.0

    # ============================================================
    # COMPACT FLOCK SLOTS
    # ============================================================

    def _region_anchor(self, region_id):
        region = self.regions[region_id]
        return self._nearest_free_point_in_region(region, region.center_px())

    def _flock_slots(self, region_id):
        anchor = self._region_anchor(region_id)
        slots = [anchor]

        # Compact rings: 6, 12, 18, ...
        ring = 1
        while len(slots) < config.FLOCK_SLOT_COUNT:
            count = 6 * ring
            radius = config.FLOCK_SPACING * ring
            for i in range(count):
                angle = 2.0 * math.pi * i / count
                requested = (
                    anchor[0] + math.cos(angle) * radius,
                    anchor[1] + math.sin(angle) * radius,
                )
                point = self._nearest_free_point_in_region(
                    self.regions[region_id],
                    requested,
                )
                if all(math.hypot(point[0] - p[0], point[1] - p[1]) >= config.FLOCK_SPACING * 0.8 for p in slots):
                    slots.append(point)
                    if len(slots) >= config.FLOCK_SLOT_COUNT:
                        break
            ring += 1
            if ring > 8:
                break
        return slots

    def _prepare_robot_cluster_slot(self, robot):
        if robot.assigned_region not in self.active_ids:
            return

        robot.flock_anchor_px = self._region_anchor(robot.assigned_region)
        slots = self._flock_slots(robot.assigned_region)

        occupied = set()
        for other in self.robots:
            if other is robot:
                continue
            if other.assigned_region != robot.assigned_region:
                continue
            if other.formation_slot_index is None:
                continue
            if math.hypot(
                other.pos[0] - robot.pos[0],
                other.pos[1] - robot.pos[1],
            ) <= config.CONSENSUS_NEIGHBOR_RADIUS:
                occupied.add(other.formation_slot_index)

        available = [
            (index, slot)
            for index, slot in enumerate(slots)
            if index not in occupied
        ]
        if not available:
            available = list(enumerate(slots))

        index, target = min(
            available,
            key=lambda item: (
                math.hypot(
                    item[1][0] - robot.pos[0],
                    item[1][1] - robot.pos[1],
                ),
                item[0],
            ),
        )

        robot.formation_slot_index = index
        self._plan_target(robot, target)

    # ============================================================
    # PATH PLANNING
    # ============================================================

    def _nearest_free_point_in_region(self, region, target_px):
        rx, ry, rw, rh = region.rect_px
        target_cell = gridmod.cell_of(target_px)
        min_col = max(0, int(rx // config.CELL_SIZE))
        max_col = min(config.GRID_COLS - 1, int((rx + rw - 1) // config.CELL_SIZE))
        min_row = max(0, int(ry // config.CELL_SIZE))
        max_row = min(config.GRID_ROWS - 1, int((ry + rh - 1) // config.CELL_SIZE))

        best_cell = None
        best_distance = math.inf
        for row in range(min_row, max_row + 1):
            for col in range(min_col, max_col + 1):
                cell = (row, col)
                if cell in self.blocked:
                    continue
                cx, cy = gridmod.cell_center_px(cell)
                d = math.hypot(cx - target_px[0], cy - target_px[1])
                if d < best_distance:
                    best_distance = d
                    best_cell = cell

        if best_cell is None:
            return target_px
        return gridmod.cell_center_px(best_cell)

    def _plan_target(self, robot, target_px):
        start_cell = robot.cell()
        goal_cell = obstaclemod.nearest_free_cell(
            self.blocked,
            gridmod.cell_of(target_px),
        )
        path_cells = astar.find_path(
            start_cell,
            goal_cell,
            self.blocked,
        )

        if path_cells and len(path_cells) > 1:
            raw = [gridmod.cell_center_px(cell) for cell in path_cells[1:]]
            robot.path = path_utils.simplify_waypoints_px(
                raw,
                tuple(robot.pos),
                self.blocked,
            )
        elif path_cells:
            robot.path = [target_px]
        else:
            robot.path = [target_px]

        robot.patrol_target_px = target_px
        robot.state = Robot.MOVING
        robot.speed = 0.0
        robot.smoothed_steer = (0.0, 0.0)

    def _make_idle(self, robot):
        robot.assigned_region = None
        robot.formation_slot_index = None
        robot.flock_anchor_px = None
        robot.path = []
        robot.patrol_target_px = None
        robot.state = Robot.IDLE
        robot.speed = 0.0
        robot.smoothed_steer = (0.0, 0.0)

    # ============================================================
    # UPDATE / MOVEMENT
    # ============================================================

    def update(self, dt):
        # Election timer. Movement continues, but assignments only change at
        # discrete election rounds.
        if self.election_active:
            self.election_timer += dt
            while self.election_timer >= config.ELECTION_ROUND_INTERVAL and self.election_active:
                self.election_timer -= config.ELECTION_ROUND_INTERVAL
                self._run_election_round()

        expire_claims(self.robots, self.election_round * config.ELECTION_ROUND_INTERVAL)

        neighbour_cache = {}
        for robot in self.robots:
            neighbours = []
            for other in self.robots:
                if other is robot:
                    continue
                d = math.hypot(
                    other.pos[0] - robot.pos[0],
                    other.pos[1] - robot.pos[1],
                )
                if d <= config.NEIGHBOR_RADIUS:
                    neighbours.append(other)
            neighbour_cache[robot] = neighbours

        for robot in self.robots:
            neighbours = neighbour_cache[robot]

            if robot.state == Robot.IDLE:
                robot.speed = 0.0
                continue

            if robot.state == Robot.MOVING:
                robot.advance_path_if_reached()
                if robot.state == Robot.HOLDING:
                    robot.speed = 0.0
                    continue

                waypoint = robot.next_waypoint()
                if waypoint is None:
                    robot.speed = 0.0
                    robot.state = Robot.HOLDING
                    continue

                dx = waypoint[0] - robot.pos[0]
                dy = waypoint[1] - robot.pos[1]
                d = math.hypot(dx, dy)
                if d <= 1e-6:
                    robot.speed = 0.0
                    continue

                goal_x = dx / d
                goal_y = dy / d

                sep_x = 0.0
                sep_y = 0.0
                critical = False
                for other in neighbours:
                    rx = robot.pos[0] - other.pos[0]
                    ry = robot.pos[1] - other.pos[1]
                    dist = math.hypot(rx, ry)
                    if dist <= 1e-6:
                        continue
                    if dist < config.SEPARATION_RADIUS:
                        strength = ((config.SEPARATION_RADIUS - dist) / config.SEPARATION_RADIUS) ** 2
                        sep_x += rx / dist * strength
                        sep_y += ry / dist * strength
                    if dist < config.CRITICAL_DISTANCE:
                        critical = True

                obs_x, obs_y = repulsion_vector(robot.pos, self.blocked)

                desired_x = (
                    goal_x * config.WEIGHT_GOAL
                    + sep_x * config.WEIGHT_SEPARATION
                    + obs_x * config.WEIGHT_OBSTACLE
                )
                desired_y = (
                    goal_y * config.WEIGHT_GOAL
                    + sep_y * config.WEIGHT_SEPARATION
                    + obs_y * config.WEIGHT_OBSTACLE
                )

                if critical or too_close_to_obstacle(robot.pos, self.blocked):
                    safe_x = sep_x + obs_x
                    safe_y = sep_y + obs_y
                    safe_mag = math.hypot(safe_x, safe_y)
                    if safe_mag > 1e-6:
                        desired_x, desired_y = safe_x, safe_y

                s = config.STEERING_SMOOTHING
                steer_x = robot.smoothed_steer[0] * (1.0 - s) + desired_x * s
                steer_y = robot.smoothed_steer[1] * (1.0 - s) + desired_y * s
                robot.smoothed_steer = (steer_x, steer_y)

                mag = math.hypot(steer_x, steer_y)
                if mag > 1e-6:
                    desired_heading = math.atan2(steer_y, steer_x)
                    diff = _wrap_angle(desired_heading - robot.heading)
                    max_step = config.MAX_TURN_RATE * dt
                    diff = max(-max_step, min(max_step, diff))
                    robot.heading = _wrap_angle(robot.heading + diff)

                    angle_error = abs(_wrap_angle(desired_heading - robot.heading))
                    turn_factor = max(0.25, math.cos(angle_error))
                    distance_factor = min(1.0, max(0.30, d / 70.0))
                    robot.speed = config.MAX_SPEED * turn_factor * distance_factor
                    if d < 60.0:
                        robot.speed = min(robot.speed, config.MAX_SPEED_NEAR_TARGET)
                else:
                    robot.speed = 0.0

                if critical or too_close_to_obstacle(robot.pos, self.blocked):
                    robot.speed = min(robot.speed, config.EMERGENCY_CREEP_SPEED)

                vx, vy = robot.velocity
                x, y = gridmod.clamp_px(
                    robot.pos[0] + vx * dt,
                    robot.pos[1] + vy * dt,
                )
                robot.pos[0], robot.pos[1] = x, y
                continue

            if robot.state == Robot.HOLDING:
                robot.speed = 0.0
                robot.smoothed_steer = (0.0, 0.0)

                # Holding is stable. Separation is used only when two robots
                # physically violate the safety distance.
                push_x = 0.0
                push_y = 0.0
                emergency = False

                if too_close_to_obstacle(robot.pos, self.blocked):
                    ox, oy = repulsion_vector(robot.pos, self.blocked)
                    push_x += ox
                    push_y += oy
                    emergency = True

                for other in self.robots:
                    if other is robot:
                        continue
                    dx = robot.pos[0] - other.pos[0]
                    dy = robot.pos[1] - other.pos[1]
                    dist = math.hypot(dx, dy)
                    if 1e-6 < dist < config.CRITICAL_DISTANCE:
                        push_x += dx / dist
                        push_y += dy / dist
                        emergency = True

                if emergency:
                    mag = math.hypot(push_x, push_y)
                    if mag > 1e-6:
                        robot.heading = math.atan2(push_y, push_x)
                        robot.speed = config.EMERGENCY_CREEP_SPEED
                        x, y = gridmod.clamp_px(
                            robot.pos[0] + robot.velocity[0] * dt,
                            robot.pos[1] + robot.velocity[1] * dt,
                        )
                        robot.pos[0], robot.pos[1] = x, y

    # ============================================================
    # DRAWING
    # ============================================================

    def draw(self, surface, font, small_font):
        surface.fill(config.COLOR_BG)
        self._draw_regions(surface, font)
        self._draw_obstacles(surface)
        for robot in self.robots:
            robot.draw(surface)
        self._draw_ui_bar(surface, small_font)

    def _draw_regions(self, surface, font):
        for region in self.regions:
            x, y, w, h = region.rect_px
            rect = pygame.Rect(x, y, w, h)
            fill = config.REGION_ACTIVE_FILL if region.active else config.REGION_INACTIVE_FILL
            pygame.draw.rect(surface, fill, rect)
            pygame.draw.rect(surface, config.REGION_BORDER, rect, width=2)
            label = font.render(region.label, True, config.REGION_LABEL)
            surface.blit(label, (x + 8, y + 6))

            if region.active:
                anchor = self._region_anchor(region.id)
                pygame.draw.circle(
                    surface,
                    config.COLOR_TEXT_DIM,
                    (int(anchor[0]), int(anchor[1])),
                    4,
                )

    def _draw_obstacles(self, surface):
        for r, c in self.blocked:
            rect = pygame.Rect(
                c * config.CELL_SIZE,
                r * config.CELL_SIZE,
                config.CELL_SIZE,
                config.CELL_SIZE,
            )
            pygame.draw.rect(surface, config.COLOR_OBSTACLE, rect)

    def _draw_ui_bar(self, surface, small_font):
        bar_y = config.GRID_ROWS * config.CELL_SIZE
        pygame.draw.rect(
            surface,
            config.COLOR_UI_BG,
            pygame.Rect(0, bar_y, config.WINDOW_WIDTH, config.UI_BAR_HEIGHT),
        )

        x = 12
        for region in self.regions:
            color = config.REGION_COLORS[region.id] if region.active else config.COLOR_TEXT_DIM
            box = pygame.Rect(x, bar_y + 10, 26, 26)
            pygame.draw.rect(surface, color, box, width=0 if region.active else 2)
            label = small_font.render(region.label, True, (10, 10, 10) if region.active else config.COLOR_TEXT_DIM)
            surface.blit(label, (x + 9, bar_y + 14))
            x += 36

        counts = {rid: 0 for rid in self.active_ids}
        for robot in self.robots:
            if robot.assigned_region in counts:
                counts[robot.assigned_region] += 1

        count_text = " ".join(
            f"R{rid + 1}:{counts[rid]}" for rid in sorted(counts)
        ) or "No active tasks"

        if self.election_active:
            status = f"ELECTION {self.election_round + 1}/{config.ELECTION_ROUNDS} | "
        else:
            status = "SETTLED | "

        info = (
            f"Robots:{len(self.robots)}  {status}{count_text}  "
            "| 1-6 toggle region | A add | R reset | click robot remove | Esc quit"
        )
        text = small_font.render(info, True, config.COLOR_TEXT)
        surface.blit(text, (x + 15, bar_y + 18))
