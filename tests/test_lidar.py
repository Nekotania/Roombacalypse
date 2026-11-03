"""LiDAR: ray casting against analytic distances, height bands, range limits, noise statistics."""

from __future__ import annotations

import math

import numpy as np
import pytest

from roombacalypse.core.config import LidarConfig
from roombacalypse.core.geometry import Pose2D
from roombacalypse.environment.rooms import build_world
from roombacalypse.environment.shapes import ConvexPolygon
from roombacalypse.environment.world import ObstacleKind, World
from roombacalypse.sensors.lidar import Lidar, beam_angles
from roombacalypse.sensors.raycast import cast_rays
from tests.helpers import IDEAL, box_room

MOUNT_X = IDEAL.lidar.mount_x


def _world(**room_kwargs: object) -> World:
    return build_world(box_room(**room_kwargs), IDEAL.robot, IDEAL.dock)  # type: ignore[arg-type]


def _lidar(config: LidarConfig = IDEAL.lidar, seed: int = 0) -> Lidar:
    return Lidar(config, np.random.default_rng(seed))


def _beam(lidar: Lidar, degrees: float) -> int:
    return int(np.argmin(np.abs(np.degrees(lidar.angles) - degrees)))


def test_beam_angles_full_circle_and_partial_fov() -> None:
    full = beam_angles(LidarConfig())
    assert full.shape == (360,)
    assert full[0] == pytest.approx(-math.pi)
    assert np.allclose(np.diff(full), math.radians(1.0))
    partial = beam_angles(LidarConfig(fov_deg=90.0, angular_resolution_deg=1.0))
    assert partial.shape == (91,)
    assert partial[0] == pytest.approx(math.radians(-45)) and partial[-1] == pytest.approx(math.radians(45))
    assert not full.flags.writeable


def test_cast_rays_analytic() -> None:
    angles = np.radians([0.0, 90.0, 180.0, 45.0])
    segments = np.array([[2.0, -1.0]])  # x = 2, from y = -1 to y = 1
    deltas = np.array([[0.0, 2.0]])
    circles = np.array([[0.0, 3.0]])
    radii = np.array([0.5])
    ranges = cast_rays(0.0, 0.0, np.cos(angles), np.sin(angles), segments, deltas, circles, radii)
    assert ranges[0] == pytest.approx(2.0)  # segment straight ahead
    assert ranges[1] == pytest.approx(2.5)  # circle near side
    assert ranges[2] == math.inf  # nothing behind
    assert ranges[3] == math.inf  # passes above the segment's end (y = 2 at x = 2)
    parallel = cast_rays(0.0, 0.0, np.array([0.0]), np.array([1.0]), np.array([[1.0, 0.0]]),
                         np.array([[0.0, 5.0]]), np.empty((0, 2)), np.empty(0))
    assert parallel[0] == math.inf


def test_scan_of_empty_room_matches_geometry() -> None:
    world = _world()
    lidar = _lidar()
    ranges = lidar.true_ranges(world, Pose2D(2.0, 1.5, 0.0))
    ox = 2.0 + MOUNT_X
    assert ranges[_beam(lidar, 0)] == pytest.approx(4.0 - ox)
    assert ranges[_beam(lidar, 90)] == pytest.approx(1.5)
    assert ranges[_beam(lidar, -90)] == pytest.approx(1.5)
    assert ranges[_beam(lidar, 45)] == pytest.approx(1.5 / math.sin(math.radians(45)))
    assert ranges[_beam(lidar, -180)] == pytest.approx(ox - IDEAL.dock.depth)  # the dock face is visible


def test_scan_rotates_with_the_robot() -> None:
    world = _world()
    lidar = _lidar()
    ranges = lidar.true_ranges(world, Pose2D(2.0, 1.5, math.pi / 2))  # facing +y; turret sits at y = 1.5 + mount_x
    assert ranges[_beam(lidar, 0)] == pytest.approx(3.0 - (1.5 + MOUNT_X))
    assert ranges[_beam(lidar, -90)] == pytest.approx(4.0 - 2.0)


def test_only_obstacles_in_the_scan_plane_are_seen() -> None:
    world = _world(
        width=6.0,
        obstacles=[
            {"name": "sock", "kind": "clutter", "center": [2.0, 1.5], "size": [0.1, 0.4], "z": [0.0, 0.02]},
            {"name": "bed", "kind": "soft", "center": [2.6, 1.5], "size": [0.2, 0.6], "z": [0.12, 0.5]},
            {"name": "pillar", "shape": "circle", "center": [3.4, 1.5], "radius": 0.1, "z": [0.0, 2.0]},
        ],
    )
    lidar = _lidar()
    ranges = lidar.true_ranges(world, Pose2D(1.0, 1.5, 0.0))
    assert ranges[_beam(lidar, 0)] == pytest.approx(3.4 - 0.1 - (1.0 + MOUNT_X))


def test_max_range_and_points() -> None:
    world = _world(width=12.0)
    lidar = _lidar()
    scan = lidar.scan(world, Pose2D(1.0, 1.5, 0.0), time=0.0)
    assert scan.ranges[_beam(lidar, 0)] == math.inf  # wall 11 m away, beyond the 6 m range
    assert scan.ranges[_beam(lidar, 90)] == pytest.approx(1.5)
    points = scan.points()
    assert points.shape[0] == int(scan.valid.sum())
    up = points[np.argmin(np.abs(points[:, 0] - MOUNT_X) + np.abs(points[:, 1] - 1.5))]
    assert up == pytest.approx((MOUNT_X, 1.5), abs=1e-9)  # body frame: x forward, y left
    assert not scan.ranges.flags.writeable


def test_range_noise_statistics() -> None:
    config = LidarConfig(range_noise_std=0.01, range_noise_per_metre=0.005, dropout_probability=0.0)
    world = _world()
    lidar = _lidar(config, seed=7)
    pose = Pose2D(2.0, 1.5, 0.0)
    truth = lidar.true_ranges(world, pose)
    samples = np.array([lidar.scan(world, pose, 0.0).ranges for _ in range(400)])
    errors = samples - truth
    beam = _beam(lidar, 0)
    expected_sigma = 0.01 + 0.005 * truth[beam]
    assert abs(errors[:, beam].mean()) < 4 * expected_sigma / math.sqrt(400)
    assert errors[:, beam].std() == pytest.approx(expected_sigma, rel=0.12)
    # Across all beams the normalised errors are standard normal.
    normalised = errors / (0.01 + 0.005 * truth)
    assert normalised.std() == pytest.approx(1.0, rel=0.03)


def test_dropouts() -> None:
    config = LidarConfig(range_noise_std=0.0, range_noise_per_metre=0.0, dropout_probability=0.25)
    world = _world()
    lidar = _lidar(config, seed=3)
    missing = np.mean([np.mean(~lidar.scan(world, Pose2D(2.0, 1.5, 0.3), 0.0).valid) for _ in range(50)])
    assert missing == pytest.approx(0.25, abs=0.02)


def test_scan_sees_world_edits() -> None:
    world = _world()
    lidar = _lidar()
    pose = Pose2D(1.0, 1.5, 0.0)
    before = lidar.true_ranges(world, pose)[_beam(lidar, 0)]
    world.add_obstacle("box", ObstacleKind.BOX, ConvexPolygon.box(2.0, 1.5, 0.2, 0.2), z_max=0.3)
    after = lidar.true_ranges(world, pose)[_beam(lidar, 0)]
    assert before == pytest.approx(4.0 - (1.0 + MOUNT_X))
    assert after == pytest.approx(1.9 - (1.0 + MOUNT_X))


def test_scans_are_reproducible_per_seed() -> None:
    world = _world()
    pose = Pose2D(2.0, 1.5, 0.0)
    a = _lidar(LidarConfig(), seed=5).scan(world, pose, 0.0).ranges
    b = _lidar(LidarConfig(), seed=5).scan(world, pose, 0.0).ranges
    c = _lidar(LidarConfig(), seed=6).scan(world, pose, 0.0).ranges
    assert np.array_equal(a, b)
    assert not np.array_equal(a, c)
