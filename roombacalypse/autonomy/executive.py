"""Milestone-4 autonomy: plan on the map and cover all of it (belief side).

A state machine on top of the coverage planner, the path follower and the
Dynamic Window Approach:

* **WAIT** for the first scan and the gyro calibration (the robot sits still on its dock).
* **UNDOCK**: back straight off the dock (see ``wander._undock`` for why).
* **DRIVE** to the current target: a boustrophedon lane, a sideways shift to
  the next lane, a backtrack to the nearest uncovered floor, or a frontier
  (:mod:`roombacalypse.planning.coverage`). The path follower turns the
  path into a goal for DWA, which avoids whatever the LiDAR sees right now.
  The costmap is rebuilt every ``replan_interval`` seconds, and the target
  is re-decided when it is reached, when its path becomes blocked on the
  map, or when the robot makes no progress (the target area is then
  blacklisted for a minute so the robot does not try the same thing again).
* **BACK OFF** and **TURN** after a bump. The bump's location is
  remembered in the map frame and becomes an obstacle in every later
  costmap: the LiDAR cannot see a sock, but the robot learns where it is.
* **RECOVER** when a stuck detector fires (:mod:`roombacalypse.autonomy.recovery`):
  back up, spin on the spot, hop towards open space, or give the target up,
  escalating while trouble keeps coming back at the same place.
* **TRAPPED** at the top of the ladder: stop and call for help.
* **LOST** while SLAM is lost (the robot was picked up and put down
  elsewhere: :mod:`roombacalypse.localization.slam`). Nothing it believes
  about where it is can be used, so it stops and lets SLAM search the whole
  map; if that does not work within a few seconds, it hops a short way to
  a new viewpoint (the hop needs no map) and tries again. After
  ``_LOST_MOVES`` hops it gives up and calls for help. Once found, it simply
  re-plans: the coverage memory and the map are in the map frame, which the
  kidnapping did not change.
* **DONE** when there is neither uncovered floor nor a frontier left.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from enum import StrEnum

import numpy as np

from roombacalypse.autonomy.recovery import RecoveryLadder, Rung, StuckDetector, Trouble
from roombacalypse.core.config import DwaConfig, PlanningConfig, RobotSpec, WanderConfig
from roombacalypse.core.events import EventKind, EventPublisher
from roombacalypse.core.geometry import Point, Pose2D, angle_difference
from roombacalypse.core.messages import STOP, BumperReading, DriveCommand
from roombacalypse.localization.odometry import OdometryTracker
from roombacalypse.mapping.occupancy_grid import OccupancyGrid
from roombacalypse.navigation.dwa import DwaPlan, DynamicWindowPlanner
from roombacalypse.navigation.local_obstacles import swept_free_distance
from roombacalypse.navigation.path_follower import PathFollower
from roombacalypse.planning.costmap import Costmap, build_costmap
from roombacalypse.planning.coverage import (
    CoverageMemory,
    CoveragePlanner,
    CoverageState,
    Stranded,
    Target,
    TargetKind,
)

_BLACKLIST_RADIUS = 0.25  # [m] around a target the robot failed to reach
_BLACKLIST_SECONDS = 60.0
_FRONTIER_BLACKLIST_SECONDS = 300.0
_UNSWEEPABLE_SECONDS = 900.0
_UNSWEEPABLE_RADIUS = 0.1  # [m]
_KEEP_OUT_RADIUS = 0.4  # [m] around a place where every recovery failed
_HOPS_BEFORE_TRAPPED = 2
_RECOVERY_BACK_UP = 0.15  # [m]
_APPROACH_GAIN = 0.75  # [1/s] speed cap per metre left on the path: 0.3 m/s from 0.4 m out
_SLOW_SPEED = 0.15  # [m/s] within _CAREFUL_GAP of anything
_CAREFUL_GAP = 0.1  # [m] between the body and the nearest LiDAR point
_HOP = (0.15, 0.5)  # [m] shortest worthwhile and longest hop towards open space
_HOP_HORIZON = 2.0  # [m]: the hop heads for the most open direction within this range (out of a gap, not in)
_HOP_BEARING_STEP = 5.0  # [deg]: fine enough to line up with a gap 3 cm wider than the robot on each side
_HOP_TOLERANCE = 0.01  # [m] a hop ignores returns this far inside the body's edge: LiDAR noise on walls alongside
_HOP_SPEED = 0.1  # [m/s]
_HOP_ALIGNED = 0.02  # [rad]: turn this close to the hop's heading before creeping (5° off can scrape)
_HOP_TIMEOUT = 5.0  # [s] to turn round, on top of the creeping time
_BACK_OFF_TIMEOUT = 2.5  # [s] beyond the time the distance should take
_DOCK_TOLERANCE = 0.02  # [m]
_ESCAPE_TURN = (0.5, 1.2)  # [rad] range of the turn after a bump
_BUMP_BEARINGS = {"left": 0.6, "right": -0.6, "front": 0.0}
_LOST_WAIT = 3.0  # [s] stand still while SLAM searches the map, before hopping to a new viewpoint
_LOST_MOVES = 20  # hops before a lost robot calls for help (it keeps looking, standing still)
_LOST_HOP = 1.0  # [m] a lost robot hops further: it needs a different view, not a way out


class ExecutiveState(StrEnum):
    WAIT = "wait"
    UNDOCK = "undock"
    DRIVE = "drive"
    BACK_OFF = "back off"
    TURN = "turn"
    RECOVER = "recover"
    TRAPPED = "trapped"
    LOST = "lost"
    DONE = "done"


class Executive:
    """Coverage behaviour: decides targets on the map and drives to them."""

    def __init__(
        self,
        spec: RobotSpec,
        planning: PlanningConfig,
        dwa_config: DwaConfig,
        wander: WanderConfig,
        control_period: float,
        map_source: Callable[[], OccupancyGrid],
        publish: EventPublisher | None = None,
    ) -> None:
        self.spec = spec
        self.config = planning
        self.reflex = wander  # back-off distance and speed, turn rate, undock distance
        self.dwa = DynamicWindowPlanner(spec, dwa_config, control_period)
        self.inscribed = spec.radius + dwa_config.safety_margin + planning.extra_margin
        self.planner = CoveragePlanner(spec.radius, planning.lane_width, planning.frontier_min_size,
                                       planning.goal_tolerance + 0.05)
        self.follower = PathFollower(planning.lookahead, planning.goal_tolerance)
        self.memory = CoverageMemory(0.05, spec.radius)
        self.detector = StuckDetector()
        self.ladder = RecoveryLadder()
        self.map_source = map_source
        self._publish = publish
        self._rng = np.random.default_rng(0)
        self.reset(self._rng)

    # ------------------------------------------------------------------ lifecycle

    def reset(self, rng: np.random.Generator, *, docked: bool = False) -> None:
        self._rng = rng
        self.memory.reset()
        self.bumps: list[Point] = []
        """Where the robot bumped into things, in the map frame (obstacles in every later costmap)."""
        self._blacklist: list[tuple[float, float, float, float]] = []  # (x, y, until, radius)
        self._arrived_at: Point | None = None
        self.goals_chosen = self.escapes = self.abandoned = self.recoveries = 0
        self.finished_at: float | None = None
        """When coverage was first declared complete [s]."""
        self.ladder.reset()
        self.costmap: Costmap | None = None
        self.coverage: CoverageState | None = None
        self._docked = docked
        self._direction = 0
        self._sweep = 1
        self.activate()

    def activate(self) -> None:
        self.state = ExecutiveState.WAIT
        self.target: Target | None = None
        self.plan: DwaPlan | None = None
        self.follower.clear()
        self._planned_at = -math.inf
        self._state_since = 0.0
        self._move_start = Pose2D(0.0, 0.0, 0.0)
        self._move_distance = 0.0
        self._turn_angle = 0.0
        self._turn_direction = 1.0
        self._turned = 0.0
        self._last_heading = 0.0
        self._after_turn = ExecutiveState.DRIVE
        self._hop: Point | None = None
        self._hop_heading = 0.0
        self._hop_aligned = False
        self._hop_deadline = 0.0
        self._hops_here = 0
        self._stranded = False
        self._lost_moves = 0
        self._wake_at = 0.0
        self.detector.clear(0.0)

    @property
    def goal(self) -> Point | None:
        """The end of the current target's path (map frame), for display."""
        return self.follower.end

    def _emit(self, time: float, message: str, **data: object) -> None:
        if self._publish is not None:
            self._publish(time, EventKind.BEHAVIOUR, message, **data)

    # ------------------------------------------------------------------ control

    def update(self, time: float, odometry: OdometryTracker, pose: Pose2D, obstacles: np.ndarray,
               new_bump: BumperReading | None, ready: bool, *, lost: bool = False) -> DriveCommand:
        """One control tick: ``pose`` in the map frame, ``obstacles`` in the body frame.

        ``lost``: SLAM does not know where the robot is, so ``pose`` must not be used.
        """
        if lost:
            return self._while_lost(time, odometry.pose, obstacles, new_bump)
        if self.state is ExecutiveState.LOST:
            self._found(time)
        if self.state is not ExecutiveState.WAIT:
            self.memory.mark(pose)
        if new_bump is not None:
            self._remember_bump(pose, new_bump)
            if self.state in (ExecutiveState.DRIVE, ExecutiveState.WAIT, ExecutiveState.DONE):
                self._start_escape(time, odometry.pose, obstacles, new_bump)
        if self.state is ExecutiveState.WAIT:
            if not ready:
                return STOP
            if self._docked and odometry.distance < _DOCK_TOLERANCE and self._rear_free(obstacles) > (
                self.reflex.undock_distance + 0.02
            ) and self.reflex.undock_distance > 0.0:
                self._begin_move(time, odometry.pose, self.reflex.undock_distance, ExecutiveState.UNDOCK)
                self._emit(time, f"undocking: backing off {100 * self.reflex.undock_distance:.0f} cm")
            else:
                self.state = ExecutiveState.DRIVE
            self._docked = False
        if self.state in (ExecutiveState.UNDOCK, ExecutiveState.BACK_OFF):
            command = self._reverse(time, odometry.pose, obstacles)
            if command is not None:
                return command
        if self.state is ExecutiveState.TURN:
            command = self._turn(time, odometry.pose)
            if command is not None:
                return command
        if self.state is ExecutiveState.RECOVER and self._hop is not None:
            command = self._hop_towards(time, odometry.pose, obstacles)
            if command is not None:
                return command
        if self.state in (ExecutiveState.DONE, ExecutiveState.TRAPPED):
            return STOP
        command = self._drive(time, odometry, pose, obstacles)
        if self._stranded:  # the planner cannot start from here: get somewhere it can
            self._stranded = False
            return self._recover(time, odometry.pose, obstacles, Trouble("stranded", time, pose.position))
        self.ladder.progress(pose.position)
        trouble = self.detector.update(
            time, pose.position, odometry.distance, odometry.angular_velocity,
            driving=self.state is ExecutiveState.DRIVE and self.target is not None,
            blocked=self.plan is not None and self.plan.blocked,
            remaining=self.follower.remaining(pose) if self.target is not None else None,
            bumped=new_bump is not None,
        )
        if trouble is not None:
            return self._recover(time, odometry.pose, obstacles, trouble)
        return command

    def _drive(self, time: float, odometry: OdometryTracker, pose: Pose2D, obstacles: np.ndarray) -> DriveCommand:
        if self.target is None or time - self._planned_at >= self.config.replan_interval:
            self._refresh(time, pose)
        if self.target is not None and self.follower.reached(pose):
            self._arrived(time)
            self._decide(time, pose)
        if self.target is None:
            return STOP
        goal = self.follower.goal(pose)
        assert goal is not None
        plan = self.dwa.plan(odometry.wheel_speeds, pose.inverse_transform_point(*goal), obstacles,
                             self._speed_cap(self.follower.remaining(pose), obstacles))
        self.plan = plan
        return plan.command

    def _speed_cap(self, remaining: float, obstacles: np.ndarray) -> float:
        """Slow down at the end of a path and close to anything.

        Lane ends sit right at the edge of the planner's safety margin, a few
        centimetres from a wall. Arriving there at cruise speed leaves the
        braking to the last moment, and at that range the LiDAR's ±1 cm of
        noise is enough to overrun into the wall.
        """
        cap = max(_SLOW_SPEED * 0.5, _APPROACH_GAIN * remaining)
        if obstacles.shape[0]:
            nearest = float(np.min(np.hypot(obstacles[:, 0], obstacles[:, 1]))) - self.spec.radius
            if nearest < _CAREFUL_GAP:
                cap = min(cap, _SLOW_SPEED)
        return cap

    # ------------------------------------------------------------------ decisions

    def _blacklist_mask(self, costmap: Costmap, time: float) -> np.ndarray | None:
        self._blacklist = [b for b in self._blacklist if b[2] > time]
        if not self._blacklist:
            return None
        rows, cols = costmap.shape
        ys = costmap.origin[1] + (np.arange(rows) + 0.5) * costmap.resolution
        xs = costmap.origin[0] + (np.arange(cols) + 0.5) * costmap.resolution
        mask = np.zeros((rows, cols), dtype=bool)
        for x, y, _, radius in self._blacklist:
            mask |= ((xs[None, :] - x) ** 2 + (ys[:, None] - y) ** 2) <= radius**2
        return mask

    def _refresh(self, time: float, pose: Pose2D) -> None:
        """Rebuild the costmap; re-decide if there is no target or its path became blocked."""
        self._planned_at = time
        grid = self.map_source()
        if not grid.log_odds.size:
            return
        bumps = np.asarray(self.bumps, dtype=float).reshape(-1, 2)
        self.costmap = build_costmap(grid, self.config, self.inscribed, bumps)
        blacklist = self._blacklist_mask(self.costmap, time)
        self.coverage = self.planner.assess(self.costmap, self.memory.aligned(self.costmap), blacklist)
        if self.target is None or not self._path_clear(pose):
            self._decide(time, pose)

    def _path_clear(self, pose: Pose2D) -> bool:
        costmap = self.costmap
        if costmap is None or self.target is None:
            return False
        end = self.target.path[-1]
        return math.isfinite(costmap.cost_at(*end))

    def _decide(self, time: float, pose: Pose2D) -> None:
        if self.costmap is None:
            self._refresh(time, pose)
            if self.costmap is None:
                return
        assert self.costmap is not None
        swept = self.memory.aligned(self.costmap)
        blacklist = self._blacklist_mask(self.costmap, time)
        coverage = self.planner.assess(self.costmap, swept, blacklist)
        if self._arrived_at is not None:
            # Arrived, and the spot is still worth sweeping: it cannot be swept from where the robot can
            # stop (a corner just beyond the arrival tolerance). Do not come back for it.
            cell = self.costmap.cell(*self._arrived_at)
            if self.costmap.inside(cell) and coverage.useful[cell]:
                self._blacklist.append((*self._arrived_at, time + _UNSWEEPABLE_SECONDS, _UNSWEEPABLE_RADIUS))
                blacklist = self._blacklist_mask(self.costmap, time)
                coverage = self.planner.assess(self.costmap, swept, blacklist)
            self._arrived_at = None
        self.coverage = coverage
        try:
            target, self._sweep = self.planner.next_target(coverage, pose, self._direction, self._sweep, blacklist)
        except Stranded:
            self.target = None
            self.follower.clear()
            self._stranded = True
            return
        self.target = target
        self.detector.target_changed(time)
        if target is None:
            self.follower.clear()
            if self.state is not ExecutiveState.DONE:
                self.state = ExecutiveState.DONE
                self.finished_at = time if self.finished_at is None else self.finished_at
                self._emit(time, f"coverage complete: {coverage.uncovered_area:.1f} m² of known floor left unswept",
                           uncovered=coverage.uncovered_area)
            return
        self.state = ExecutiveState.DRIVE
        self.follower.set_path(target.path)
        self.goals_chosen += 1
        if target.kind is TargetKind.LANE:
            self._direction = target.direction
        elif target.kind in (TargetKind.BACKTRACK, TargetKind.FRONTIER):
            length = sum(math.dist(a, b) for a, b in zip(target.path, target.path[1:], strict=False))
            self._emit(time, f"{target.kind.value}: {length:.1f} m away", target=target.kind.value, distance=length)

    def _arrived(self, time: float) -> None:
        assert self.target is not None
        self._arrived_at = self.target.path[-1]
        self.ladder.succeeded()
        if self.target.kind in (TargetKind.SHIFT, TargetKind.BACKTRACK):
            self._direction = self.target.direction
        elif self.target.kind is TargetKind.FRONTIER:
            self._direction = 0
            # Seen from here, a frontier either resolves or never will (a pocket the LiDAR cannot see into).
            end = self.target.path[-1]
            self._blacklist.append((end[0], end[1], time + _FRONTIER_BLACKLIST_SECONDS, _BLACKLIST_RADIUS))
        self.target = None

    def _give_up(self, time: float, reason: str) -> None:
        if self.target is None:
            return
        end = self.target.path[-1]
        self._blacklist.append((end[0], end[1], time + _BLACKLIST_SECONDS, _BLACKLIST_RADIUS))
        self.abandoned += 1
        self._direction = 0
        self._emit(time, f"{self.target.kind.value} abandoned ({reason})", reason=reason)
        self.target = None

    # ------------------------------------------------------------------ recovery

    def _recover(self, time: float, odometry_pose: Pose2D, obstacles: np.ndarray, trouble: Trouble) -> DriveCommand:
        rung = self.ladder.next(trouble)
        self.recoveries += 1
        self.detector.clear(time)
        self.plan = None
        if rung is Rung.BACK_UP and self._rear_free(obstacles) < _RECOVERY_BACK_UP + 0.02:
            rung = Rung.SPIN  # nowhere to back up to
        if rung is Rung.TRAPPED and self._hops_here < _HOPS_BEFORE_TRAPPED and self._open_direction(obstacles):
            rung = Rung.HOP  # there is a way out in the scan: try it before calling for help
        self._hops_here = self._hops_here + 1 if rung is Rung.HOP else (0 if rung is Rung.BACK_UP else self._hops_here)
        self._emit(time, f"stuck ({trouble.kind}): {rung.name.lower().replace('_', ' ')}", trouble=trouble.kind,
                   rung=int(rung))
        if rung is Rung.BACK_UP:
            self._turn_angle = 0.0
            self._begin_move(time, odometry_pose, _RECOVERY_BACK_UP, ExecutiveState.BACK_OFF)
        elif rung is Rung.SPIN:
            self._turn_direction = float(self._rng.choice((-1.0, 1.0)))
            self._turn_angle = 2.0 * math.pi
            self._begin_turn(time, odometry_pose)
        elif rung is Rung.HOP:
            self._start_hop(time, odometry_pose, obstacles)
        elif rung is Rung.ELSEWHERE:
            # Keep out of the trouble spot itself, not just away from the target: else the next target
            # sends the robot straight back into the same tight gap.
            self._blacklist.append((*trouble.where, time + _UNSWEEPABLE_SECONDS, _KEEP_OUT_RADIUS))
            self._give_up(time, trouble.kind)
            self.state = ExecutiveState.DRIVE
            self._planned_at = -math.inf
        else:
            self.state = ExecutiveState.TRAPPED
            self.target = None
            self.follower.clear()
            self._emit(time, "trapped: I need help", trouble=trouble.kind)
            return STOP
        return STOP

    def _hop_options(self, obstacles: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Bearings all round and how far the bare body (not the planner's margin) could drive along each."""
        bearings = np.radians(np.arange(-180.0, 180.0, _HOP_BEARING_STEP))
        return bearings, swept_free_distance(obstacles, bearings, self.spec.radius - _HOP_TOLERANCE, _HOP_HORIZON)

    def _open_direction(self, obstacles: np.ndarray) -> bool:
        return bool(self._hop_options(obstacles)[1].max() >= _HOP[1] + 0.05)

    def _start_hop(self, time: float, odometry_pose: Pose2D, obstacles: np.ndarray, longest: float = _HOP[1]
                   ) -> None:
        """Pick the most open direction in the current scan and aim a short hop at it (odometry frame)."""
        bearings, free = self._hop_options(obstacles)
        best = int(np.argmax(free + self._rng.uniform(0.0, 0.02, free.shape)))
        distance = float(min(free[best] - 0.05, longest))
        if distance < _HOP[0]:
            self._turn_direction = float(self._rng.choice((-1.0, 1.0)))
            self._turn_angle = 2.0 * math.pi
            self._begin_turn(time, odometry_pose)
            return
        bearing = float(bearings[best])
        self._hop = odometry_pose.transform_point(distance * math.cos(bearing), distance * math.sin(bearing))
        self._hop_heading = odometry_pose.theta + bearing
        self._hop_aligned = False
        self._hop_deadline = time + _HOP_TIMEOUT + distance / _HOP_SPEED
        self.state = ExecutiveState.RECOVER
        self._state_since = time

    def _hop_towards(self, time: float, odometry_pose: Pose2D, obstacles: np.ndarray) -> DriveCommand | None:
        """Drive on a heading, like Nav2's DriveOnHeading: turn to it, then creep straight.

        Only the bare body is checked, not DWA's safety margin: the hop is for
        the gaps the margin cannot get out of (nose-in between a sofa and an
        armchair, 3 cm to spare on each side). There, LiDAR noise (σ 0.8 cm)
        puts some returns from the walls alongside inside the body's edge, so
        returns less than ``_HOP_TOLERANCE`` inside it are ignored: driving
        straight with the gyro holding the heading, the robot never closes on
        a parallel wall, and anything across its path is seen well before.
        """
        assert self._hop is not None
        ahead, _ = odometry_pose.inverse_transform_point(*self._hop)
        heading_error = angle_difference(self._hop_heading, odometry_pose.theta)
        if not self._hop_aligned and abs(heading_error) <= _HOP_ALIGNED:
            self._hop_aligned = True
        blocked = self._hop_aligned and float(
            swept_free_distance(obstacles, np.array([0.0]), self.spec.radius - _HOP_TOLERANCE, 0.1)[0]
        ) < 0.03
        if (self._hop_aligned and ahead < 0.02) or blocked or time > self._hop_deadline:
            self._hop = None
            self.state = ExecutiveState.DRIVE
            self._planned_at = -math.inf
            return None
        turn = self.reflex.turn_speed
        if not self._hop_aligned:  # proportional near the end: a full-rate tick turns 0.15 rad
            return DriveCommand(0.0, math.copysign(max(0.2, min(turn, 3.0 * abs(heading_error))), heading_error))
        return DriveCommand(_HOP_SPEED, max(-turn, min(turn, 2.0 * heading_error)))

    # ------------------------------------------------------------------ lost

    def _while_lost(self, time: float, odometry_pose: Pose2D, obstacles: np.ndarray,
                    new_bump: BumperReading | None) -> DriveCommand:
        """Stand still while SLAM searches the map; now and then hop to a new viewpoint (odometry frame)."""
        if self.state is not ExecutiveState.LOST:
            self.state = ExecutiveState.LOST
            self.target = None
            self.plan = None
            self.follower.clear()
            self._hop = None
            self._lost_moves = 0
            self._wake_at = time + _LOST_WAIT
            self._emit(time, "lost: stopping to look around")
        if self._hop is not None:
            command = self._hop_towards(time, odometry_pose, obstacles)
            self.state = ExecutiveState.LOST  # the hop hands over to DRIVE when it is done
            if command is not None and new_bump is None:
                return command
            self._hop = None
            self._wake_at = time + _LOST_WAIT
        if time >= self._wake_at:
            if self._lost_moves >= _LOST_MOVES:
                if self._lost_moves == _LOST_MOVES:
                    self._lost_moves += 1
                    self._emit(time, "lost: I need help (please carry me somewhere I know)", trouble="lost")
                return STOP
            self._lost_moves += 1
            self._start_hop(time, odometry_pose, obstacles, _LOST_HOP)
            self.state = ExecutiveState.LOST
            if self._hop is None:  # nowhere to go: keep looking from here
                self._wake_at = time + _LOST_WAIT
        return STOP

    def _found(self, time: float) -> None:
        """SLAM knows where the robot is again: re-plan from scratch."""
        self.state = ExecutiveState.DRIVE
        self.target = None
        self._hop = None
        self._direction = 0
        self._planned_at = -math.inf
        self.ladder.reset()
        self.detector.clear(time)
        self._emit(time, "found: back to work")

    # ------------------------------------------------------------------ reflexes

    def _remember_bump(self, pose: Pose2D, bump: BumperReading) -> None:
        side = "front" if bump.left == bump.right else ("left" if bump.left else "right")
        reach = self.spec.radius + 0.02
        bearing = _BUMP_BEARINGS[side]
        self.bumps.append(pose.transform_point(reach * math.cos(bearing), reach * math.sin(bearing)))
        self._planned_at = -math.inf  # the next tick rebuilds the costmap with it

    def _rear_free(self, obstacles: np.ndarray) -> float:
        return float(swept_free_distance(obstacles, np.array([math.pi]), self.spec.radius + 0.01, 1.0)[0])

    def _start_escape(self, time: float, odometry_pose: Pose2D, obstacles: np.ndarray, bump: BumperReading) -> None:
        self.escapes += 1
        self.target = None
        self.plan = None
        self.follower.clear()
        if bump.left and not bump.right:
            self._turn_direction = -1.0
        elif bump.right and not bump.left:
            self._turn_direction = 1.0
        else:
            self._turn_direction = float(self._rng.choice((-1.0, 1.0)))
        self._turn_angle = float(self._rng.uniform(*_ESCAPE_TURN))
        distance = self.reflex.backup_distance
        if distance > 0.0 and self._rear_free(obstacles) > distance + 0.02:
            self._begin_move(time, odometry_pose, distance, ExecutiveState.BACK_OFF)
        else:
            self._begin_turn(time, odometry_pose)

    def _begin_move(self, time: float, odometry_pose: Pose2D, distance: float, state: ExecutiveState) -> None:
        self.state = state
        self._move_start = odometry_pose
        self._move_distance = distance
        self._state_since = time

    def _reverse(self, time: float, odometry_pose: Pose2D, obstacles: np.ndarray) -> DriveCommand | None:
        moved = odometry_pose.distance_to(self._move_start)
        timeout = _BACK_OFF_TIMEOUT + self._move_distance / self.reflex.backup_speed
        if moved >= self._move_distance or time - self._state_since > timeout or self._rear_free(obstacles) < 0.03:
            if self.state is ExecutiveState.BACK_OFF and self._turn_angle > 0.0:
                self._begin_turn(time, odometry_pose)
            else:
                self.state = ExecutiveState.DRIVE
                self._planned_at = -math.inf
            return None
        return DriveCommand(-self.reflex.backup_speed, 0.0)

    def _begin_turn(self, time: float, odometry_pose: Pose2D) -> None:
        self.state = ExecutiveState.TURN
        self._turned = 0.0
        self._last_heading = odometry_pose.theta
        self._state_since = time

    def _turn(self, time: float, odometry_pose: Pose2D) -> DriveCommand | None:
        self._turned += abs(angle_difference(odometry_pose.theta, self._last_heading))  # can exceed a half turn
        self._last_heading = odometry_pose.theta
        timeout = self._turn_angle / self.reflex.turn_speed + 2.0
        if self._turned >= self._turn_angle - 0.05 or time - self._state_since > timeout:
            self.state = ExecutiveState.DRIVE
            self._direction = 0
            self._planned_at = -math.inf
            return None
        return DriveCommand(0.0, self._turn_direction * self.reflex.turn_speed)
