"""Milestone-2 autonomy: explore a room without a map.

A small state machine on top of the DWA local planner:

* **WAIT** until the first LiDAR scan arrives.
* **CRUISE** towards an exploration goal. Goals are chosen from the current
  scan: for each candidate direction, how far could the body travel (a swept
  disc against the scan points), scored by distance, by how rarely the robot
  has been near that spot (a coarse visit counter) and by a random factor
  from the robot's own seeded stream. DWA drives there.
  A goal is dropped when reached, when it times out, when the distance to it
  stops shrinking, or when DWA reports that only turning on the spot is safe.
* **BACK OFF** a few centimetres after a bump, if the LiDAR says the space
  behind is clear (there is no rear bumper).
* **TURN** on the spot by a random angle away from the bumped side, then cruise.

Frames: goals and the visit counter live in the *map frame* and use the
SLAM pose (Milestone 3), so the counter no longer smears as odometry drifts;
the bump reflexes (back off a few centimetres, turn by an angle) use
odometry, which is smooth and locally accurate, like the ``odom`` frame of a
ROS robot. Without a SLAM pose, the odometry pose serves for both.

The behaviour itself is still map-free: it picks goals from the current
scan. Milestone 4 replaces goal picking with coverage planning on the map.
See docs/algorithms/dwa.md.
"""

from __future__ import annotations

import math
from enum import StrEnum

import numpy as np

from roombacalypse.core.config import DwaConfig, RobotSpec, WanderConfig
from roombacalypse.core.events import EventKind, EventPublisher
from roombacalypse.core.geometry import Point, Pose2D, angle_difference, transform_points
from roombacalypse.core.messages import STOP, BumperReading, DriveCommand
from roombacalypse.localization.odometry import OdometryTracker
from roombacalypse.navigation.dwa import DwaPlan, DynamicWindowPlanner
from roombacalypse.navigation.local_obstacles import swept_free_distance

_GOAL_STANDOFF = 0.3  # stop this far short of the first obstacle in a direction [m]
_HOP_STANDOFF = 0.05  # cramped fallback: get this close to the first obstacle [m]
_MIN_HOP = 0.08  # shorter hops than this are pointless [m]
_BLOCKED_PATIENCE = 1.5  # seconds of "only turning is safe" before dropping a goal
_BACK_OFF_TIMEOUT = 2.5  # [s]


class WanderState(StrEnum):
    WAIT = "wait"
    CRUISE = "cruise"
    BACK_OFF = "back off"
    TURN = "turn"


class VisitGrid:
    """Counts control ticks spent in each cell of a coarse grid (map frame)."""

    def __init__(self, cell_size: float) -> None:
        self.cell_size = cell_size
        self._counts: dict[tuple[int, int], int] = {}

    def _key(self, x: float, y: float) -> tuple[int, int]:
        return (math.floor(x / self.cell_size), math.floor(y / self.cell_size))

    def mark(self, x: float, y: float) -> None:
        key = self._key(x, y)
        self._counts[key] = self._counts.get(key, 0) + 1

    def count(self, x: float, y: float) -> int:
        return self._counts.get(self._key(x, y), 0)

    def reset(self) -> None:
        self._counts.clear()

    def __len__(self) -> int:
        return len(self._counts)


class WanderBehaviour:
    """Autonomous, map-free exploration with DWA obstacle avoidance and a bump reflex."""

    def __init__(
        self,
        spec: RobotSpec,
        config: WanderConfig,
        dwa_config: DwaConfig,
        control_period: float,
        publish: EventPublisher | None = None,
    ) -> None:
        self.spec = spec
        self.config = config
        self.dwa = DynamicWindowPlanner(spec, dwa_config, control_period)
        self.cruise_speed = dwa_config.max_speed
        self.goal_clearance = spec.radius + dwa_config.safety_margin  # same disc DWA keeps clear
        self.visits = VisitGrid(config.visit_cell_size)
        self._publish = publish
        self._rng = np.random.default_rng(0)
        self.state = WanderState.WAIT
        self.goal: Point | None = None
        """Current exploration goal in the map frame."""
        self.plan: DwaPlan | None = None
        """Latest DWA result (for visualisation)."""
        self.goals_chosen = 0
        self.escapes = 0
        self.abandoned = 0
        self._deadline = 0.0
        self._best_distance = math.inf
        self._progress_time = 0.0
        self._blocked_since: float | None = None
        self._state_since = 0.0
        self._back_off_start = Pose2D(0.0, 0.0, 0.0)
        self._turn_start = 0.0
        self._turn_angle = 0.0
        self._turn_direction = 1.0

    # ------------------------------------------------------------------ lifecycle

    def reset(self, rng: np.random.Generator) -> None:
        """Forget everything, including where the robot has been."""
        self._rng = rng
        self.visits.reset()
        self.goals_chosen = self.escapes = self.abandoned = 0
        self.activate()

    def activate(self) -> None:
        """(Re)start autonomous operation; keeps the visit memory."""
        self.state = WanderState.WAIT
        self.goal = None
        self.plan = None
        self._blocked_since = None

    def _emit(self, time: float, message: str, **data: object) -> None:
        if self._publish is not None:
            self._publish(time, EventKind.BEHAVIOUR, message, **data)

    # ------------------------------------------------------------------ control

    def update(
        self,
        time: float,
        odometry: OdometryTracker,
        obstacles: np.ndarray,
        new_bump: BumperReading | None,
        scan_ready: bool,
        pose: Pose2D | None = None,
    ) -> DriveCommand:
        """One control tick.

        ``obstacles`` are points in the current body frame; ``pose`` is the
        robot's map-frame estimate (default: the odometry pose).
        """
        if pose is None:
            pose = odometry.pose
        self.visits.mark(pose.x, pose.y)
        if new_bump is not None and self.state in (WanderState.CRUISE, WanderState.WAIT):
            self._start_escape(time, odometry.pose, obstacles, new_bump)
        if self.state is WanderState.WAIT:
            if not scan_ready:
                return STOP
            self.state = WanderState.CRUISE
        if self.state is WanderState.BACK_OFF:
            command = self._back_off(time, odometry.pose, obstacles)
            if command is not None:
                return command
        if self.state is WanderState.TURN:
            command = self._turn(time, odometry.pose)
            if command is not None:
                return command
        return self._cruise(time, odometry, pose, obstacles)

    def _cruise(self, time: float, odometry: OdometryTracker, pose: Pose2D, obstacles: np.ndarray) -> DriveCommand:
        if self.goal is not None:
            reason = self._goal_problem(time, pose)
            if reason is not None:
                if reason != "reached":
                    self.abandoned += 1
                self._emit(time, f"goal {reason}", reason=reason)
                self.goal = None
        if self.goal is None and not self._choose_goal(time, pose, obstacles):
            self._begin_turn(time, odometry.pose, float(self._rng.choice((-1.0, 1.0))),
                             math.radians(self._rng.uniform(60, 120)))
            self._emit(time, "boxed in: turning to look for space")
            return DriveCommand(0.0, self._turn_direction * self.config.turn_speed)
        assert self.goal is not None
        goal_body = pose.inverse_transform_point(*self.goal)
        plan = self.dwa.plan(odometry.wheel_speeds, goal_body, obstacles)
        self.plan = plan
        if plan.blocked:
            if self._blocked_since is None:
                self._blocked_since = time
        else:
            self._blocked_since = None
        return plan.command

    def _goal_problem(self, time: float, pose: Pose2D) -> str | None:
        """Why the current goal should be dropped, or ``None`` to keep it."""
        assert self.goal is not None
        distance = math.hypot(self.goal[0] - pose.x, self.goal[1] - pose.y)
        if distance < self.config.goal_reached_tolerance:
            return "reached"
        if distance < self._best_distance - 0.05:
            self._best_distance = distance
            self._progress_time = time
        if time > self._deadline:
            return "timed out"
        if time - self._progress_time > self.config.stall_time:
            return "abandoned (no progress)"
        if self._blocked_since is not None and time - self._blocked_since > _BLOCKED_PATIENCE:
            return "abandoned (blocked)"
        return None

    def _choose_goal(self, time: float, pose: Pose2D, obstacles: np.ndarray) -> bool:
        cfg = self.config
        bearings = np.radians(np.arange(-180.0, 180.0, cfg.candidate_step_deg))
        free = swept_free_distance(obstacles, bearings, self.goal_clearance, cfg.goal_max_distance + _GOAL_STANDOFF)
        reach = np.minimum(free - _GOAL_STANDOFF, cfg.goal_max_distance)
        feasible = reach >= cfg.goal_min_distance
        if not feasible.any():
            return self._choose_hop(time, pose, bearings, free)
        local = np.column_stack((reach * np.cos(bearings), reach * np.sin(bearings)))
        goals = transform_points(pose, local)
        middles = transform_points(pose, 0.5 * local)
        novelty = np.array(
            [
                1.0 / (1.0 + self.visits.count(g[0], g[1]) + 0.5 * self.visits.count(m[0], m[1]))
                for g, m in zip(goals, middles, strict=True)
            ]
        )
        score = novelty * np.sqrt(np.maximum(reach, 0.0) / cfg.goal_max_distance)
        score *= self._rng.uniform(0.6, 1.4, bearings.shape[0])
        score = np.where(feasible, score, -np.inf)
        best = int(np.argmax(score))
        self.goal = (float(goals[best, 0]), float(goals[best, 1]))
        distance = float(reach[best])
        self._deadline = time + min(cfg.goal_timeout, 4.0 + 3.0 * distance / self.cruise_speed)
        self._best_distance = distance
        self._progress_time = time
        self._blocked_since = None
        self.goals_chosen += 1
        self._emit(time, f"new goal {distance:.1f} m at {math.degrees(bearings[best]):+.0f}°", distance=distance)
        return True

    def _choose_hop(self, time: float, pose: Pose2D, bearings: np.ndarray, free: np.ndarray) -> bool:
        """Cramped: no full-length goal exists, so take a short hop towards the most open direction.

        Turning on the spot does not change what a round robot can reach, so
        without this the robot could turn forever in a tight corner (e.g. nose
        between two chair legs). The random factor avoids re-picking the same
        hop every time.
        """
        hop = np.minimum(free - _HOP_STANDOFF, self.config.goal_min_distance)
        score = np.where(hop >= _MIN_HOP, hop * self._rng.uniform(0.7, 1.3, bearings.shape[0]), -np.inf)
        if not np.isfinite(score).any():
            return False
        best = int(np.argmax(score))
        distance = float(hop[best])
        x, y = pose.transform_point(distance * math.cos(bearings[best]), distance * math.sin(bearings[best]))
        self.goal = (x, y)
        self._deadline = time + 4.0 + 3.0 * distance / self.cruise_speed
        self._best_distance = distance
        self._progress_time = time
        self._blocked_since = None
        self.goals_chosen += 1
        bearing = math.degrees(bearings[best])
        self._emit(time, f"cramped: short hop {distance:.2f} m at {bearing:+.0f}°", distance=distance)
        return True

    # ------------------------------------------------------------------ bump reflex

    def _start_escape(self, time: float, pose: Pose2D, obstacles: np.ndarray, bump: BumperReading) -> None:
        cfg = self.config
        if bump.left and not bump.right:
            direction, side = -1.0, "left"
        elif bump.right and not bump.left:
            direction, side = 1.0, "right"
        else:
            direction, side = float(self._rng.choice((-1.0, 1.0))), "front"
        angle = math.radians(self._rng.uniform(cfg.turn_min_deg, cfg.turn_max_deg))
        self.escapes += 1
        self.goal = None
        self.plan = None
        self._turn_direction, self._turn_angle = direction, angle
        if cfg.backup_distance > 0.0 and self._rear_free(obstacles) > cfg.backup_distance + 0.02:
            self.state = WanderState.BACK_OFF
            self._back_off_start = pose
            self._state_since = time
            self._emit(time, f"bump on the {side}: backing off, then turning {math.degrees(angle):.0f}°", side=side)
        else:
            self._begin_turn(time, pose, direction, angle)
            self._emit(time, f"bump on the {side}, no room behind: turning {math.degrees(angle):.0f}°", side=side)

    def _rear_free(self, obstacles: np.ndarray) -> float:
        return float(swept_free_distance(obstacles, np.array([math.pi]), self.spec.radius + 0.01, 1.0)[0])

    def _back_off(self, time: float, pose: Pose2D, obstacles: np.ndarray) -> DriveCommand | None:
        moved = pose.distance_to(self._back_off_start)
        if moved >= self.config.backup_distance or time - self._state_since > _BACK_OFF_TIMEOUT or (
            self._rear_free(obstacles) < 0.03
        ):
            self._begin_turn(time, pose, self._turn_direction, self._turn_angle)
            return None
        return DriveCommand(-self.config.backup_speed, 0.0)

    def _begin_turn(self, time: float, pose: Pose2D, direction: float, angle: float) -> None:
        self.state = WanderState.TURN
        self._turn_direction, self._turn_angle = direction, angle
        self._turn_start = pose.theta
        self._state_since = time

    def _turn(self, time: float, pose: Pose2D) -> DriveCommand | None:
        turned = abs(angle_difference(pose.theta, self._turn_start))
        too_long = time - self._state_since > self._turn_angle / self.config.turn_speed + 2.0
        if turned >= self._turn_angle - 0.05 or too_long:
            self.state = WanderState.CRUISE
            self.goal = None
            return None
        return DriveCommand(0.0, self._turn_direction * self.config.turn_speed)
