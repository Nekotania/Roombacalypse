"""Scan matching: correlative search and Gauss-Newton refinement on real (simulated) LiDAR scans."""

from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest

from roombacalypse.core.config import AppConfig, SlamConfig
from roombacalypse.core.geometry import Pose2D, angle_difference, transform_points
from roombacalypse.core.messages import LidarScan
from roombacalypse.core.rng import RngStreams
from roombacalypse.environment.rooms import RoomSpec, build_world
from roombacalypse.localization.scan_matcher import ScanMatcher, surface_points, voxel_filter
from roombacalypse.mapping.likelihood_field import LikelihoodField
from roombacalypse.sensors.lidar import Lidar
from tests.helpers import box_room

CONFIG = AppConfig()  # realistic LiDAR noise and dropouts
SLAM = CONFIG.slam
MATCHER = ScanMatcher(SLAM)
FURNISHED = box_room(5.0, 4.0, obstacles=[
    {"name": "pillar", "kind": "pillar", "shape": "circle", "center": [3.5, 2.6], "radius": 0.15},
    {"name": "cabinet", "center": [4.6, 0.6], "size": [0.6, 0.8]},
    {"name": "stool", "kind": "pillar", "shape": "circle", "center": [1.2, 3.4], "radius": 0.2},
], dock=(0.0, 2.0, 0.0))


def _scan(room: RoomSpec, pose: Pose2D, seed: int) -> LidarScan:
    world = build_world(room, CONFIG.robot, CONFIG.dock)
    return Lidar(CONFIG.lidar, RngStreams(seed).get("sensors.lidar")).scan(world, pose, 0.0)


def _scan_points(room: RoomSpec, pose: Pose2D, seed: int) -> np.ndarray:
    return voxel_filter(_scan(room, pose, seed).points(), SLAM.scan_voxel)


def _reference(room: RoomSpec, poses: list[Pose2D]) -> LikelihoodField:
    """A local map built from noisy scans taken at known poses (here: the world frame)."""
    points = np.vstack([transform_points(p, surface_points(_scan(room, p, seed=10 + k), SLAM.field_resolution / 2))
                        for k, p in enumerate(poses)])
    return LikelihoodField(points, SLAM.field_resolution, SLAM.field_sigma, margin=0.5)


def _error(found: Pose2D, truth: Pose2D) -> tuple[float, float]:
    return found.distance_to(truth), abs(angle_difference(found.theta, truth.theta))


FIELD = _reference(FURNISHED, [Pose2D(2.0, 1.8, 0.0), Pose2D(2.5, 1.5, 1.0), Pose2D(1.6, 2.2, -2.0)])


def test_surface_points_join_returns_on_one_surface_but_not_across_gaps() -> None:
    angles = np.radians([0.0, 1.0, 2.0, 3.0])
    ranges = np.array([2.0, 2.0, 2.0, 1.0])  # three returns on a wall, then a leg in front of it
    scan = LidarScan(0.0, angles, ranges, 6.0)
    dense = surface_points(scan, 0.01)
    assert dense.shape[0] > 4 + 2 * 3  # the wall got filled in between its returns
    near_leg = np.hypot(dense[:, 0] - math.cos(angles[3]), dense[:, 1] - math.sin(angles[3])) < 0.5
    assert near_leg.sum() == 1  # nothing joins the leg to the wall behind it


def test_voxel_filter_keeps_one_centroid_per_voxel() -> None:
    points = np.array([[0.01, 0.01], [0.03, 0.03], [0.21, 0.01], [-0.01, 0.0]])
    thinned = voxel_filter(points, 0.1)
    assert thinned.shape == (3, 2)
    assert [0.02, 0.02] in thinned.round(6).tolist()


def test_refinement_recovers_a_small_offset() -> None:
    truth = Pose2D(2.3, 1.9, 0.4)
    points = _scan_points(FURNISHED, truth, seed=1)
    prior = Pose2D(truth.x + 0.04, truth.y - 0.03, truth.theta + math.radians(2.0))
    result = MATCHER.refine(FIELD, points, prior)
    distance, heading = _error(result.pose, truth)
    assert distance < 0.006 and heading < math.radians(0.25)
    assert result.score > 0.85


def test_correlative_search_recovers_an_offset_refinement_alone_cannot() -> None:
    truth = Pose2D(2.8, 1.4, -0.7)
    points = _scan_points(FURNISHED, truth, seed=2)
    prior = Pose2D(truth.x + 0.45, truth.y - 0.4, truth.theta - math.radians(11.0))
    alone = MATCHER.refine(FIELD, points, prior)
    assert _error(alone.pose, truth)[0] > 0.2  # ICP locks onto the wrong walls (its basin is wide, not unbounded)
    result = MATCHER.search(FIELD, points, prior, SLAM.loop_window, math.radians(SLAM.loop_window_deg))
    distance, heading = _error(result.pose, truth)
    assert distance < 0.006 and heading < math.radians(0.2)
    assert result.searched and not result.at_boundary


def test_an_offset_beyond_the_window_is_flagged() -> None:
    truth = Pose2D(2.2, 2.0, 0.1)
    points = _scan_points(FURNISHED, truth, seed=3)
    prior = Pose2D(truth.x + 0.45, truth.y, truth.theta)
    result = MATCHER.search(FIELD, points, prior, 0.2, math.radians(5.0))
    assert result.at_boundary or result.score < SLAM.track_min_score


def test_a_well_featured_room_is_not_ambiguous() -> None:
    truth = Pose2D(2.0, 2.0, 0.0)
    points = _scan_points(FURNISHED, truth, seed=4)
    result = MATCHER.search(FIELD, points, Pose2D(2.1, 1.95, 0.05), SLAM.loop_window,
                            math.radians(SLAM.loop_window_deg), check_ambiguity=True)
    assert _error(result.pose, truth)[0] < 0.01
    assert result.ambiguity < SLAM.loop_max_ambiguity


def test_a_featureless_corridor_is_ambiguous_and_leans_on_the_prior() -> None:
    corridor = box_room(20.0, 1.2, dock=(0.0, 0.6, 0.0))  # the ends are beyond the 6 m LiDAR range
    truth = Pose2D(10.0, 0.6, 0.0)
    field = _reference(corridor, [Pose2D(9.8, 0.6, 0.0), Pose2D(10.3, 0.55, 0.1)])
    points = _scan_points(corridor, truth, seed=5)
    prior = Pose2D(truth.x + 0.3, truth.y + 0.05, truth.theta + math.radians(2.0))
    tracked = MATCHER.refine(field, points, prior)
    assert abs(tracked.pose.y - truth.y) < 0.01  # across: pinned by the walls
    assert abs(angle_difference(tracked.pose.theta, truth.theta)) < math.radians(0.5)
    assert abs(tracked.pose.x - prior.x) < 0.05  # along: the scan cannot tell, so the prior stands
    info = tracked.information
    assert info[0, 0] < 0.01 * info[1, 1]  # ...and the information says so
    searched = MATCHER.search(field, points, prior, SLAM.loop_window, math.radians(SLAM.loop_window_deg),
                              check_ambiguity=True)
    assert searched.ambiguity > SLAM.loop_max_ambiguity  # anywhere along the corridor fits: no loop closure


def test_the_prior_is_weak_unless_made_heavy() -> None:
    truth = Pose2D(2.3, 1.9, 0.4)
    points = _scan_points(FURNISHED, truth, seed=6)
    prior = Pose2D(truth.x + 0.03, truth.y + 0.03, truth.theta)
    default = _error(MATCHER.refine(FIELD, points, prior).pose, truth)[0]
    heavy = ScanMatcher(dataclasses.replace(SlamConfig(), prior_translation_weight=200.0))
    assert default < 0.008
    assert _error(heavy.refine(FIELD, points, prior).pose, truth)[0] > 2 * default


@pytest.mark.parametrize("seed", range(5))
def test_search_is_accurate_over_random_offsets(seed: int) -> None:
    rng = np.random.default_rng(seed)
    truth = Pose2D(rng.uniform(1.0, 3.6), rng.uniform(1.0, 3.0), rng.uniform(-math.pi, math.pi))
    points = _scan_points(FURNISHED, truth, seed=20 + seed)
    prior = Pose2D(truth.x + rng.uniform(-0.2, 0.2), truth.y + rng.uniform(-0.2, 0.2),
                   truth.theta + math.radians(rng.uniform(-7, 7)))
    result = MATCHER.search(FIELD, points, prior, SLAM.track_window, math.radians(SLAM.track_window_deg))
    distance, heading = _error(result.pose, truth)
    assert distance < 0.015 and heading < math.radians(0.5)  # one scan against a three-scan map
