"""The kidnapped robot: global relocalisation, the simulator's kidnapping, and the closed loop."""

from __future__ import annotations

import math

import numpy as np
import pytest
from scipy import ndimage

from roombacalypse.autonomy.brain import Mode, RobotBrain
from roombacalypse.core.config import AppConfig, MappingConfig
from roombacalypse.core.events import EventKind, EventLog
from roombacalypse.core.geometry import Pose2D, angle_difference
from roombacalypse.core.messages import LidarScan
from roombacalypse.core.paths import resolve_room_path
from roombacalypse.core.rng import RngStreams
from roombacalypse.environment.rooms import RoomSpec, build_world, load_room
from roombacalypse.environment.shapes import Circle
from roombacalypse.localization.relocalization import GlobalMatcher
from roombacalypse.localization.scan_matcher import voxel_filter
from roombacalypse.mapping.occupancy_grid import OccupancyGrid
from roombacalypse.sensors.lidar import Lidar
from roombacalypse.simulation.engine import Simulation
from tests.helpers import box_room, min_separation

CONFIG = AppConfig()  # realistic LiDAR noise
MATCHER = GlobalMatcher(CONFIG.slam.relocalize_sigma, CONFIG.slam.relocalize_clearance)
FURNISHED = box_room(5.0, 4.0, obstacles=[
    {"name": "pillar", "kind": "pillar", "shape": "circle", "center": [3.5, 2.6], "radius": 0.15},
    {"name": "cabinet", "center": [4.6, 0.6], "size": [0.6, 0.8]},
    {"name": "stool", "kind": "pillar", "shape": "circle", "center": [1.2, 3.4], "radius": 0.2},
], dock=(0.0, 2.0, 0.0))
SURVEY = [Pose2D(x, y, 0.0) for x in (1.0, 2.5, 4.0) for y in (1.2, 2.8)]


def _scan(room: RoomSpec, pose: Pose2D, seed: int) -> LidarScan:
    world = build_world(room, CONFIG.robot, CONFIG.dock)
    return Lidar(CONFIG.lidar, RngStreams(seed).get("sensors.lidar")).scan(world, pose, 0.0)


def _map(room: RoomSpec) -> OccupancyGrid:
    """An occupancy grid surveyed from known poses (map frame = world frame here)."""
    grid = OccupancyGrid(MappingConfig())
    for k, pose in enumerate(SURVEY):
        grid.insert_scan(pose, _scan(room, pose, seed=100 + k))
    return grid


def _error(found: Pose2D, truth: Pose2D) -> tuple[float, float]:
    return found.distance_to(truth), abs(angle_difference(found.theta, truth.theta))


def test_the_whole_map_search_finds_a_scan_taken_anywhere_facing_any_way() -> None:
    grid = _map(FURNISHED)
    world = build_world(FURNISHED, CONFIG.robot, CONFIG.dock)
    rng = np.random.default_rng(7)
    found = 0
    while found < 6:
        x, y = rng.uniform(0.4, 4.6), rng.uniform(0.4, 3.6)
        if not world.is_shape_free(Circle(x, y, 0.3), 0.0, 0.1):
            continue
        truth = Pose2D(float(x), float(y), float(rng.uniform(-math.pi, math.pi)))
        hypothesis = MATCHER.search(grid, _scan(FURNISHED, truth, seed=found).points())
        assert hypothesis is not None
        distance, heading = _error(hypothesis.pose, truth)
        assert distance < 0.08 and heading < math.radians(2.0), (truth, hypothesis)
        assert hypothesis.evaluated < 0.2 * hypothesis.candidates  # branch and bound pruned most of the search
        found += 1


def test_branch_and_bound_finds_the_exhaustive_optimum() -> None:
    grid = _map(FURNISHED)
    truth = Pose2D(2.2, 1.7, 2.0)
    points = _scan(FURNISHED, truth, seed=3).points()
    hypothesis = MATCHER.search(grid, points)
    assert hypothesis is not None
    # Brute force over every candidate cell at the winning heading.
    occupied = grid.occupied_mask()
    distance = ndimage.distance_transform_edt(~occupied) * grid.resolution
    likelihood = np.exp(-0.5 * (distance / MATCHER.sigma) ** 2)
    candidates = grid.free_mask() & ~occupied & (distance >= MATCHER.clearance)
    pts = voxel_filter(points, MATCHER.voxel)
    c, s = math.cos(hypothesis.pose.theta), math.sin(hypothesis.pose.theta)
    d_row = np.rint((s * pts[:, 0] + c * pts[:, 1]) / grid.resolution).astype(int)
    d_col = np.rint((c * pts[:, 0] - s * pts[:, 1]) / grid.resolution).astype(int)
    rows, cols = np.nonzero(candidates)
    r = np.clip(rows[:, None] + d_row[None, :], 0, occupied.shape[0] - 1)
    cc = np.clip(cols[:, None] + d_col[None, :], 0, occupied.shape[1] - 1)
    inside = (rows[:, None] + d_row >= 0) & (rows[:, None] + d_row < occupied.shape[0]) & (
        cols[:, None] + d_col >= 0) & (cols[:, None] + d_col < occupied.shape[1])
    exhaustive = np.where(inside, likelihood[r, cc], 0.0).mean(axis=1).max()
    assert hypothesis.score == pytest.approx(exhaustive, abs=1e-9)


def test_a_symmetric_room_is_reported_as_ambiguous() -> None:
    empty = box_room(5.0, 4.0)
    grid = _map(empty)
    truth = Pose2D(1.5, 1.2, 0.4)  # off-centre: its mirror image through the room's centre looks the same
    hypothesis = MATCHER.search(grid, _scan(empty, truth, seed=5).points(), good_enough=1.0)
    assert hypothesis is not None
    assert hypothesis.ambiguity > 0.9  # so SLAM will not commit to either


def test_kidnapping_moves_the_robot_without_telling_it() -> None:
    room = box_room(4.0, 3.0)
    events = EventLog()
    brain = RobotBrain(CONFIG, publish=events.publisher("robot"), mode=Mode.MANUAL)
    sim = Simulation(CONFIG, room, brain, seed=2, events=events)
    sim.run(1.0)
    travel = (sim.robot.travel_left, sim.robot.travel_right)
    pose = sim.kidnap()
    assert sim.robot.pose == pose
    assert sim.world.is_shape_free(Circle(pose.x, pose.y, CONFIG.robot.radius + 0.2), 0.0, CONFIG.robot.height)
    assert (sim.robot.travel_left, sim.robot.travel_right) == travel  # the wheels did not turn: nothing to feel
    assert events.count(EventKind.KIDNAP) == 1
    again = Simulation(CONFIG, room, RobotBrain(CONFIG, mode=Mode.MANUAL), seed=2)
    again.run(1.0)
    assert again.kidnap() == pose  # deterministic, from its own random stream


def test_a_kidnapped_robot_notices_finds_itself_and_carries_on() -> None:
    config = AppConfig()
    events = EventLog(100_000)
    brain = RobotBrain(config, publish=events.publisher("robot"), mode=Mode.AUTONOMOUS)
    sim = Simulation(config, load_room(resolve_room_path("studio")), brain, seed=2, events=events)
    sim.run(240.0)  # most of the studio is mapped by now
    before = brain.executive.memory.area
    sim.kidnap()
    lost_at = found_at = None
    for _ in range(round(30.0 / sim.dt)):
        sim.step()
        assert min_separation(sim) > -1e-9
        if lost_at is None and brain.slam.lost:
            lost_at = sim.time
        if lost_at is not None and found_at is None and not brain.slam.lost:
            found_at = sim.time
    assert lost_at is not None and lost_at - 240.0 < 1.5  # measured: 0.4 s (three scans that fit nowhere)
    assert found_at is not None and found_at - lost_at < 5.0  # measured: the first global search, 0.2 s later
    located = sim.boot_pose.compose(brain.pose)
    assert located.distance_to(sim.robot.pose) < 0.05
    assert brain.executive.state.value != "lost"
    assert brain.executive.memory.area > before  # back to cleaning
    messages = [e.message for e in events.recent(10_000)]
    assert any(m.startswith("lost:") for m in messages) and any(m.startswith("found myself") for m in messages)
    visible = [e for e in events.recent(10_000) if e.kind is EventKind.COLLISION
               and (e.data.get("obstacle_id") is None or sim.world.obstacle(e.data["obstacle_id"]).z_max > 0.1)]
    assert not visible


def test_a_kidnapping_to_the_mirror_image_of_a_nearby_pose_is_not_taken_for_a_slip() -> None:
    """Measured in the near-symmetric test chamber: ICP pulled a kidnapped scan to the mirror image of the old
    view (0.5 m and 16° from where the robot stood still) with a fit of 0.80, and the scan went into the map."""
    config = AppConfig()
    brain = RobotBrain(config, mode=Mode.MANUAL)
    sim = Simulation(config, load_room(resolve_room_path("test_chamber")), brain, seed=1)
    here = Pose2D(1.4, 1.4, 0.5)
    sim.robot.pose = here  # it stands here (manual mode, no one driving) and maps the room
    sim.run(4.0)
    assert not brain.slam.lost and brain.slam.keyframes[-1].tracked
    near = Pose2D(here.x + 0.35, here.y + 0.1, here.theta + math.radians(12.0))
    mirror = Pose2D(4.0 - near.x, 3.0 - near.y, near.theta + math.pi)  # the room turned half round: same view
    sim.kidnap(mirror)
    sim.run(2.0)
    located = here.compose(brain.pose)  # it booted here, not on the dock: that is its map frame's origin
    assert brain.slam.lost or located.distance_to(sim.robot.pose) < 0.05  # lost, or (already) found: never fooled
