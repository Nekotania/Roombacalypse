"""Coverage planning, path following, stuck detection and the executive in closed loop."""

from __future__ import annotations

import math

import numpy as np
import pytest

from roombacalypse.autonomy.brain import Mode, RobotBrain
from roombacalypse.autonomy.executive import Executive
from roombacalypse.autonomy.recovery import RecoveryLadder, Rung, StuckDetector, Trouble
from roombacalypse.core.config import AppConfig, MappingConfig, PlanningConfig
from roombacalypse.core.events import EventKind, EventLog
from roombacalypse.core.geometry import Pose2D, angle_difference, inverse_transform_points
from roombacalypse.evaluation.coverage import FootprintCoverage
from roombacalypse.mapping.occupancy_grid import OccupancyGrid
from roombacalypse.navigation.path_follower import PathFollower
from roombacalypse.planning.costmap import build_costmap
from roombacalypse.planning.coverage import CoverageMemory, CoveragePlanner, Stranded, TargetKind
from roombacalypse.simulation.engine import Simulation
from tests.helpers import box_room, min_separation

PLANNING = PlanningConfig()
RADIUS = 0.17
INSCRIBED = RADIUS + 0.03
PLANNER = CoveragePlanner(RADIUS, PLANNING.lane_width, PLANNING.frontier_min_size, PLANNING.goal_tolerance + 0.05)


def _room_grid(width: float = 4.0, height: float = 3.0, known_width: float | None = None) -> OccupancyGrid:
    """A mapped rectangular room (walls on the border); only ``known_width`` of it seen so far."""
    grid = OccupancyGrid(MappingConfig())
    grid.ensure_contains(0.0, 0.0, width, height)
    xs = grid.origin[0] + (np.arange(grid.shape[1]) + 0.5) * grid.resolution
    ys = grid.origin[1] + (np.arange(grid.shape[0]) + 0.5) * grid.resolution
    gx, gy = np.meshgrid(xs, ys)
    seen = gx < (known_width if known_width is not None else width + 1.0)
    inside = (gx > 0.0) & (gx < width) & (gy > 0.0) & (gy < height) & seen
    border = inside & ((gx < 0.05) | (gx > width - 0.05) | (gy < 0.05) | (gy > height - 0.05))
    grid.log_odds[inside] = -2.0
    grid.log_odds[border] = 3.5
    return grid


# --------------------------------------------------------------------------- path follower


def test_the_follower_puts_the_carrot_lookahead_metres_along_the_path() -> None:
    follower = PathFollower(lookahead=0.5, tolerance=0.1)
    follower.set_path([(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)])
    goal = follower.goal(Pose2D(0.5, 0.2, 0.0))
    assert goal == pytest.approx((1.0, 0.0))  # projected to x = 0.5, then 0.5 m on
    goal = follower.goal(Pose2D(1.8, 0.0, 0.0))
    assert goal == pytest.approx((2.0, 0.3))  # round the corner
    assert follower.remaining(Pose2D(1.8, 0.0, 0.0)) == pytest.approx(2.2)
    assert follower.goal(Pose2D(2.0, 1.9, 0.0)) == pytest.approx((2.0, 2.0))  # never beyond the end
    assert follower.reached(Pose2D(2.0, 1.95, 0.0)) and not follower.reached(Pose2D(2.0, 1.5, 0.0))


# --------------------------------------------------------------------------- coverage planner


def test_coverage_memory_marks_the_footprint_and_grows() -> None:
    memory = CoverageMemory(0.05, RADIUS)
    memory.mark(Pose2D(1.0, 1.0, 0.0))
    assert memory.area == pytest.approx(math.pi * RADIUS**2, rel=0.15)
    memory.mark(Pose2D(-30.0, 25.0, 0.0))  # far away: the grid grows, the first mark stays
    assert memory.area == pytest.approx(2 * math.pi * RADIUS**2, rel=0.15)
    costmap = build_costmap(_room_grid(), PLANNING, INSCRIBED)
    aligned = memory.aligned(costmap)
    assert aligned[costmap.cell(1.0, 1.0)] and not aligned[costmap.cell(2.0, 2.0)]


def test_an_unswept_room_starts_with_lanes_along_x() -> None:
    costmap = build_costmap(_room_grid(), PLANNING, INSCRIBED)
    state = PLANNER.assess(costmap, np.zeros(costmap.shape, dtype=bool))
    assert state.sweepable_area > 0.9 * (4.0 - 0.1) * (3.0 - 0.1) - 1.0
    target, _ = PLANNER.next_target(state, Pose2D(1.0, 1.0, 0.0), direction=1, sweep=1)
    assert target is not None and target.kind is TargetKind.LANE
    (_, y0), (x1, y1) = target.path
    assert y0 == pytest.approx(y1) and x1 > 3.5  # straight along +x, to the far wall's safety margin
    assert x1 <= 4.0 - 0.05 - INSCRIBED + 0.05


def test_at_the_end_of_a_lane_the_robot_shifts_one_lane_width() -> None:
    costmap = build_costmap(_room_grid(), PLANNING, INSCRIBED)
    memory = CoverageMemory(0.05, RADIUS)
    for x in np.arange(0.3, 3.71, 0.02):  # one lane swept along y = 1.0
        memory.mark(Pose2D(float(x), 1.0, 0.0))
    state = PLANNER.assess(costmap, memory.aligned(costmap))
    target, sweep = PLANNER.next_target(state, Pose2D(3.7, 1.0, 0.0), direction=1, sweep=1)
    assert target is not None and target.kind is TargetKind.SHIFT
    (_, y0), (_, y1) = target.path
    assert y1 - y0 == pytest.approx(PLANNING.lane_width, abs=0.051) and sweep == 1
    assert target.direction == -1  # and comes back the other way


def test_with_lanes_done_it_backtracks_then_explores_then_finishes() -> None:
    costmap = build_costmap(_room_grid(known_width=2.0), PLANNING, INSCRIBED)
    swept = np.zeros(costmap.shape, dtype=bool)
    state = PLANNER.assess(costmap, swept)
    target, _ = PLANNER.next_target(state, Pose2D(1.0, 1.5, 0.0), direction=0, sweep=1)
    assert target is not None and target.kind in (TargetKind.BACKTRACK, TargetKind.SHIFT)
    everything = np.ones(costmap.shape, dtype=bool)
    state = PLANNER.assess(costmap, everything)  # all known floor swept: the unseen half calls
    target, _ = PLANNER.next_target(state, Pose2D(1.0, 1.5, 0.0), direction=0, sweep=1)
    assert target is not None and target.kind is TargetKind.FRONTIER
    assert target.path[-1][0] > 1.5
    whole = build_costmap(_room_grid(), PLANNING, INSCRIBED)  # fully mapped and fully swept: done
    state = PLANNER.assess(whole, np.ones(whole.shape, dtype=bool))
    assert PLANNER.next_target(state, Pose2D(1.0, 1.5, 0.0), direction=0, sweep=1)[0] is None


def test_nowhere_to_start_from_is_not_the_same_as_done() -> None:
    costmap = build_costmap(_room_grid(), PLANNING, INSCRIBED)
    state = PLANNER.assess(costmap, np.zeros(costmap.shape, dtype=bool))
    assert PLANNER.next_target(state, Pose2D(1.0, 1.5, 0.0), direction=0, sweep=1)[0] is not None
    grid = _room_grid()
    centres_x = grid.origin[0] + (np.arange(grid.shape[1]) + 0.5) * grid.resolution
    centres_y = grid.origin[1] + (np.arange(grid.shape[0]) + 0.5) * grid.resolution
    gx, gy = np.meshgrid(centres_x, centres_y)
    grid.log_odds[(np.abs(gx - 2.0) < 0.7) & (np.abs(gy - 1.5) < 0.7)] = 3.5  # a big table the robot is "under"
    costmap = build_costmap(grid, PLANNING, INSCRIBED)
    state = PLANNER.assess(costmap, np.zeros(costmap.shape, dtype=bool))
    with pytest.raises(Stranded):  # nothing traversable within reach of the middle of it
        PLANNER.next_target(state, Pose2D(2.0, 1.5, 0.0), direction=0, sweep=1)


# --------------------------------------------------------------------------- stuck detection


def test_each_detector_fires_on_its_own_trouble() -> None:
    def run(**kwargs: object) -> str | None:
        detector = StuckDetector()
        trouble = None
        for k in range(200):
            t = 0.1 * k
            args = {"where": (0.0, 0.0), "odometry_distance": 0.0, "angular": 0.0, "driving": True,
                    "blocked": False, "remaining": 3.0, "bumped": False}
            args.update({key: value(t, k) if callable(value) else value for key, value in kwargs.items()})
            trouble = detector.update(t, **args)  # type: ignore[arg-type]
            if trouble is not None:
                return trouble.kind
        return None

    assert run(blocked=True) == "cornered"
    assert run() == "no progress"
    assert run(remaining=lambda t, k: 3.0 - 0.1 * t, bumped=lambda t, k: k % 30 == 0) == "bump storm"
    zigzag = lambda t, k: 1.0 if (k // 3) % 2 else -1.0  # noqa: E731
    assert run(remaining=lambda t, k: 3.0 - 0.1 * t, angular=zigzag) == "oscillation"
    assert run(remaining=lambda t, k: 3.0 - 0.1 * t, odometry_distance=lambda t, k: 0.2 * t) == "wheel slip"
    assert run(remaining=lambda t, k: 3.0 - 0.2 * t, where=lambda t, k: (0.2 * t, 0.0),
               odometry_distance=lambda t, k: 0.2 * t) is None  # actually driving: nothing wrong


def test_the_ladder_escalates_at_the_same_place_and_restarts_elsewhere() -> None:
    ladder = RecoveryLadder()
    rungs = [ladder.next(Trouble("cornered", 10.0 * k, (0.1 * k, 0.0))) for k in range(6)]
    assert rungs == [Rung.BACK_UP, Rung.SPIN, Rung.HOP, Rung.ELSEWHERE, Rung.TRAPPED, Rung.TRAPPED]
    assert ladder.next(Trouble("cornered", 70.0, (5.0, 5.0))) is Rung.BACK_UP  # somewhere else
    assert ladder.next(Trouble("cornered", 200.0, (5.0, 5.0))) is Rung.BACK_UP  # much later


# --------------------------------------------------------------------------- closed loop


def test_the_robot_covers_a_room_and_says_so() -> None:
    config = AppConfig()
    events = EventLog(10000)
    brain = RobotBrain(config, publish=events.publisher("robot"), mode=Mode.AUTONOMOUS)
    room = box_room(3.0, 2.5, dock=(0.0, 1.25, 0.0))
    sim = Simulation(config, room, brain, seed=3, events=events)
    coverage = FootprintCoverage(sim.world, config.robot, sim.robot.pose.position)
    for _ in range(round(320.0 / sim.dt)):  # measured: done after about 260 s
        sim.step()
        coverage.mark(sim.robot.pose.x, sim.robot.pose.y)
        assert min_separation(sim) > -1e-9
    assert coverage.fraction > 0.88
    assert events.count(EventKind.COLLISION) == 0
    assert any("coverage complete" in e.message for e in events.recent(10000))
    assert brain.executive.state.value == "done" and sim.robot.linear_velocity == pytest.approx(0.0, abs=1e-6)


def test_the_escape_hop_creeps_straight_out_of_a_gap_three_centimetres_wider_than_the_robot_each_side() -> None:
    """Measured in the apartment: in a 40 cm dead end, LiDAR noise (σ 0.8 cm) on the walls alongside blocked
    every escape that checked the robot's margin, and 15° bearings never lined up with the way out."""
    config = AppConfig()
    executive = Executive(config.robot, config.planning, config.dwa, config.wander, 0.1,
                          lambda: OccupancyGrid(MappingConfig()))
    rng = np.random.default_rng(1)
    axis = math.radians(-3.0)  # the robot is not quite lined up with the gap
    along = np.arange(-0.25, 0.45, 0.01)
    sides = [np.column_stack((along, np.full_like(along, side) + rng.normal(0.0, 0.008, along.shape)))
             for side in (-0.2, 0.2)]
    back = np.column_stack((np.full(41, -0.25), np.linspace(-0.2, 0.2, 41)))
    local = np.vstack((*sides, back))
    c, s = math.cos(axis), math.sin(axis)
    obstacles = np.column_stack((c * local[:, 0] - s * local[:, 1], s * local[:, 0] + c * local[:, 1]))
    start = Pose2D(0.0, 0.0, 0.0)
    assert executive._open_direction(obstacles)
    executive._start_hop(0.0, start, obstacles)
    assert executive._hop is not None
    assert abs(angle_difference(executive._hop_heading, axis)) <= math.radians(5.0)  # within a bearing step
    pose, moved = start, False
    for k in range(120):  # turn on the spot, then creep: integrate the commands over 0.1 s ticks
        command = executive._hop_towards(0.1 * k, pose, inverse_transform_points(pose, obstacles))
        if command is None:
            break
        moved |= command.linear > 0.0
        heading = pose.theta + 0.1 * command.angular
        pose = Pose2D(pose.x + 0.1 * command.linear * math.cos(heading),
                      pose.y + 0.1 * command.linear * math.sin(heading), heading)
    assert moved  # the noisy walls alongside did not stop it
    out = pose.inverse_transform_point(0.0, 0.0)
    assert pose.distance_to(start) > 0.4 and abs(angle_difference(pose.theta, axis)) < math.radians(3.0)
    assert out[0] < -0.4  # out of the 45 cm deep gap, along it


def test_a_robot_boxed_in_by_furniture_calls_for_help() -> None:
    config = AppConfig()
    events = EventLog(10000)
    brain = RobotBrain(config, publish=events.publisher("robot"), mode=Mode.AUTONOMOUS)
    # The dock sits in a pen of heavy boxes just wider than the robot can turn in: no way out.
    walls = [{"name": "crate", "center": c, "size": s, "z": [0.0, 0.5]}
             for c, s in (([0.7, 1.5], [0.1, 1.0]), ([0.35, 1.95], [0.7, 0.1]), ([0.35, 1.05], [0.7, 0.1]))]
    sim = Simulation(config, box_room(4.0, 3.0, obstacles=walls), brain, seed=1, events=events)
    for _ in range(round(240.0 / sim.dt)):
        sim.step()
        if brain.executive.state.value == "trapped":
            break
    assert brain.executive.state.value in ("trapped", "done")
    messages = [e.message for e in events.recent(10000) if e.kind is EventKind.BEHAVIOUR]
    if brain.executive.state.value == "trapped":
        assert any(m.startswith("trapped") for m in messages)
    assert events.count(EventKind.COLLISION) == 0
