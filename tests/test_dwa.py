"""Dynamic Window Approach planner and the swept-disc free-space query."""

from __future__ import annotations

import math

import numpy as np
import pytest

from roombacalypse.core.config import DwaConfig, EncoderConfig, RobotSpec
from roombacalypse.core.geometry import Pose2D
from roombacalypse.core.kinematics import twist_to_wheel_speeds
from roombacalypse.core.messages import DriveCommand, SensorFrame
from roombacalypse.localization.odometry import OdometryTracker
from roombacalypse.navigation.dwa import DynamicWindowPlanner
from roombacalypse.navigation.local_obstacles import ObstacleMemory, swept_free_distance
from roombacalypse.simulation.engine import Simulation
from tests.helpers import IDEAL, box_room, min_separation

SPEC = RobotSpec()
CONFIG = DwaConfig()
PERIOD = 0.1
EMPTY = np.empty((0, 2))


def _planner() -> DynamicWindowPlanner:
    return DynamicWindowPlanner(SPEC, CONFIG, PERIOD)


def _wall(x: float, y_from: float = -2.0, y_to: float = 2.0, spacing: float = 0.02) -> np.ndarray:
    ys = np.arange(y_from, y_to + 1e-9, spacing)
    return np.column_stack((np.full_like(ys, x), ys))


def test_window_is_exactly_what_the_wheels_can_reach() -> None:
    planner = _planner()
    current = (0.1, 0.2)
    v, w = planner.window(current)
    left, right = twist_to_wheel_speeds(v, w, SPEC.wheel_base)
    reach = SPEC.max_wheel_accel * PERIOD
    assert np.all(np.abs(left - current[0]) <= reach + 1e-9)
    assert np.all(np.abs(right - current[1]) <= reach + 1e-9)
    assert np.all(v >= 0.0) and np.all(v <= CONFIG.max_speed + 1e-9)
    from_rest_v, from_rest_w = planner.window((0.0, 0.0))
    assert np.any((from_rest_v == 0.0) & (np.abs(from_rest_w) > 0.5))  # turning on the spot is an option


def test_window_brakes_when_outside_the_envelope() -> None:
    v, _ = _planner().window((-0.15, -0.15))  # still reversing after a back-off
    assert np.all(v == 0.0)  # closest admissible: stop reversing as fast as possible


def test_free_space_goal_ahead_drives_straight_and_speeds_up() -> None:
    plan = _planner().plan((0.2, 0.2), (2.0, 0.0), EMPTY)
    assert plan.command.linear > 0.2
    assert abs(plan.command.angular) < 0.1
    assert not plan.blocked and plan.free_distance == math.inf


@pytest.mark.parametrize(("goal", "sign"), [((1.0, 1.5), 1.0), ((1.0, -1.5), -1.0)])
def test_turns_towards_the_goal(goal: tuple[float, float], sign: float) -> None:
    plan = _planner().plan((0.2, 0.2), goal, EMPTY)
    assert sign * plan.command.angular > 0.1


def test_goal_behind_turns_hard() -> None:
    plan = _planner().plan((0.0, 0.0), (-2.0, 0.1), EMPTY)
    assert abs(plan.command.angular) > 0.5
    assert plan.command.linear < 0.1


def test_wall_too_close_to_brake_at_cruise_speed_slows_down() -> None:
    plan = _planner().plan((0.3, 0.3), (3.0, 0.0), _wall(0.235))  # 3.5 cm left before the safety radius
    assert plan.admissible[plan.chosen]
    braking = plan.command.linear**2 / (2 * SPEC.max_wheel_accel)
    assert plan.command.linear < 0.3 and braking <= plan.free_distance + 1e-9


def test_boxed_in_reports_blocked() -> None:
    angles = np.linspace(-math.pi, math.pi, 120, endpoint=False)
    ring = 0.215 * np.column_stack((np.cos(angles), np.sin(angles)))
    plan = _planner().plan((0.0, 0.0), (2.0, 0.0), ring)
    assert plan.blocked
    assert plan.admissible[plan.chosen]
    assert plan.command.linear == pytest.approx(0.0, abs=1e-9)  # no creeping into the margin


class _DwaGoalSeeker:
    """Minimal closed-loop controller: DWA towards a fixed goal using LiDAR and odometry only."""

    name = "dwa-goal-seeker"

    def __init__(self, goal: tuple[float, float]) -> None:
        self.goal = goal  # odometry frame
        self.planner = _planner()
        self.odometry = OdometryTracker(SPEC, EncoderConfig())
        self.memory = ObstacleMemory(bump_memory_seconds=0.0)

    def reset(self, seed: int = 0) -> None:
        self.odometry.reset()
        self.memory.reset()

    def update(self, frame: SensorFrame, dt: float) -> DriveCommand:
        pose = self.odometry.update(frame.encoders, frame.time)
        if frame.lidar is not None:
            self.memory.update_scan(frame.lidar, pose)
        goal_body = pose.inverse_transform_point(*self.goal)
        if math.hypot(*goal_body) < 0.15:
            return DriveCommand()
        obstacles = self.memory.points_in_body(pose, CONFIG.obstacle_range)
        return self.planner.plan(self.odometry.wheel_speeds, goal_body, obstacles).command


def _closed_loop(obstacles: list[dict[str, object]], goal: tuple[float, float], seconds: float) -> Simulation:
    controller = _DwaGoalSeeker(goal)
    sim = Simulation(IDEAL, box_room(width=6.0, obstacles=obstacles), controller)
    sim.robot.pose = Pose2D(1.0, 1.5, 0.0)  # odometry frame = this start pose
    for _ in range(round(seconds / sim.dt)):
        sim.step()
        assert min_separation(sim) > -1e-9
        assert not sim.robot.contacts, "DWA let the body touch something"
    return sim


PILLAR = {"name": "pillar", "shape": "circle", "center": [2.5, 1.5], "radius": 0.3, "z": [0.0, 2.0]}


def test_closed_loop_detours_around_an_obstacle_partly_in_the_way() -> None:
    post = {"name": "post", "shape": "circle", "center": [2.5, 1.2], "radius": 0.2, "z": [0.0, 2.0]}
    sim = _closed_loop([post], goal=(3.5, 0.0), seconds=20.0)  # straight path would clip the post by 7 cm
    assert sim.robot.pose.distance_to(Pose2D(4.5, 1.5, 0.0)) < 0.2


def test_closed_loop_swings_around_a_pillar_dead_ahead() -> None:
    """The classic DWA local minimum (goal straight behind a round obstacle) is escaped: the intrusion
    penalty makes skirting the pillar beat stopping in front of it. Not guaranteed for every shape
    (e.g. U-shaped traps); the wander behaviour's stall timeout and, later, M4's A* path cover the rest."""
    sim = _closed_loop([PILLAR], goal=(3.5, 0.0), seconds=20.0)
    assert sim.robot.pose.distance_to(Pose2D(4.5, 1.5, 0.0)) < 0.2


def test_closed_loop_stops_at_an_unpassable_wall() -> None:
    partition = {"name": "partition", "kind": "wall", "center": [3.0, 1.5], "size": [0.1, 3.0], "z": [0.0, 2.0]}
    sim = _closed_loop([partition], goal=(3.5, 0.0), seconds=15.0)
    assert sim.robot.pose.x < 2.95 - SPEC.radius  # stopped short of the partition, never touched it


def test_chosen_arc_is_always_admissible_property() -> None:
    rng = np.random.default_rng(11)
    planner = _planner()
    for _ in range(200):
        obstacles = rng.uniform((-1.5, -1.5), (1.5, 1.5), size=(int(rng.integers(1, 60)), 2))
        obstacles = obstacles[np.hypot(obstacles[:, 0], obstacles[:, 1]) > SPEC.radius + 0.04]
        speed = float(rng.uniform(0.0, 0.3))
        turn = float(rng.uniform(-0.1, 0.1))
        goal = (float(rng.uniform(-2, 2)), float(rng.uniform(-2, 2)))
        plan = planner.plan((speed - turn, speed + turn), goal, obstacles)
        if plan.chosen < 0:
            continue
        assert plan.admissible[plan.chosen]
        v = plan.command.linear
        braking = v * v / (2 * SPEC.max_wheel_accel)
        assert braking <= plan.free_distance + 1e-9


@pytest.mark.parametrize(
    ("point", "bearing_deg", "expected"),
    [
        ((1.0, 0.0), 0.0, 1.0 - 0.2),  # dead ahead
        ((1.0, 0.1), 0.0, 1.0 - math.sqrt(0.2**2 - 0.1**2)),  # off-centre: touches later
        ((1.0, 0.25), 0.0, 3.0),  # passes beside: never touched
        ((-1.0, 0.0), 0.0, 3.0),  # behind: ignored
        ((0.0, 1.0), 90.0, 1.0 - 0.2),
    ],
)
def test_swept_free_distance(point: tuple[float, float], bearing_deg: float, expected: float) -> None:
    free = swept_free_distance(np.array([point]), np.radians([bearing_deg]), radius=0.2, limit=3.0)
    assert free[0] == pytest.approx(expected)
