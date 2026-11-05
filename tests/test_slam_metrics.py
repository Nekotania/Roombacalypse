"""The ground-truth scoring of localisation and mapping must itself be right."""

from __future__ import annotations

import math

import numpy as np
import pytest

from roombacalypse.core.config import MappingConfig
from roombacalypse.core.geometry import Pose2D, transform_points
from roombacalypse.core.rng import RngStreams
from roombacalypse.environment.rooms import build_world
from roombacalypse.evaluation.slam_metrics import MapAccuracy, TrajectoryRecorder, aligned_rmse, trajectory_error
from roombacalypse.mapping.occupancy_grid import OccupancyGrid
from roombacalypse.sensors.lidar import Lidar
from tests.helpers import IDEAL, box_room


def test_trajectory_error_against_hand_computed_values() -> None:
    truth = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.5]])
    estimate = truth + np.array([[0.0, 0.0, 0.0], [0.03, 0.04, 0.0], [0.0, 0.0, -0.1]])
    error = trajectory_error(estimate, truth)
    assert error.rms_position == pytest.approx(math.sqrt(0.05**2 / 3))
    assert error.max_position == pytest.approx(0.05)
    assert error.max_heading == pytest.approx(0.1)
    assert error.final_position == 0.0


def test_aligned_error_ignores_a_rigid_offset() -> None:
    rng = np.random.default_rng(0)
    truth = rng.uniform(-3, 3, (50, 2))
    pose = Pose2D(0.4, -0.2, 0.3)
    moved = transform_points(pose, truth)
    assert aligned_rmse(moved, truth) == pytest.approx(0.0, abs=1e-9)
    assert trajectory_error(np.column_stack((moved, np.zeros(50))), np.column_stack((truth, np.zeros(50)))
                            ).rms_position > 0.3


def test_recorder_works_in_the_world_frame_through_the_boot_pose() -> None:
    boot = Pose2D(1.0, 2.0, math.pi)
    recorder = TrajectoryRecorder(boot)
    recorder.record(0.0, boot, Pose2D(0.0, 0.0, 0.0))
    recorder.record(0.1, boot.compose(Pose2D(1.0, 0.0, 0.0)), Pose2D(1.0, 0.02, 0.0))
    online = recorder.online_error()
    assert online.max_position == pytest.approx(0.02)
    keyframes = recorder.keyframe_error([0.1], np.array([[1.0, 0.0, 0.0]]))
    assert keyframes.max_position == pytest.approx(0.0, abs=1e-12)


def _scored(map_pose_error: Pose2D) -> tuple[float, float]:
    """Map a box room from four perfect scans, placed in the map with a deliberate pose error."""
    room = box_room(4.0, 3.0, obstacles=[{"name": "pillar", "kind": "pillar", "shape": "circle",
                                          "center": [2.8, 1.8], "radius": 0.15}])
    world = build_world(room, IDEAL.robot, IDEAL.dock)
    boot = world.dock.pose  # type: ignore[union-attr]
    lidar = Lidar(IDEAL.lidar, RngStreams(0).get("sensors.lidar"))
    grid = OccupancyGrid(MappingConfig())
    accuracy = MapAccuracy(world, IDEAL.lidar.mount_height, boot)
    for true_pose in (Pose2D(1.0, 1.0, 0.0), Pose2D(3.0, 0.8, 1.0), Pose2D(1.5, 2.3, -2.0), Pose2D(3.2, 2.4, 2.5)):
        scan = lidar.scan(world, true_pose, 0.0)
        accuracy.observe(scan, true_pose)
        believed = boot.inverse().compose(true_pose).compose(map_pose_error)
        for _ in range(3):
            grid.insert_scan(believed, scan)
    score = accuracy.score(grid)
    return score.precision, score.recall


def test_a_correct_map_scores_full_marks() -> None:
    precision, recall = _scored(Pose2D(0.0, 0.0, 0.0))
    assert precision == 1.0
    assert recall > 0.97


def test_a_misplaced_map_loses_precision_and_recall() -> None:
    precision, recall = _scored(Pose2D(0.15, 0.0, 0.08))  # 15 cm and 4.6° off: phantom walls in free space
    assert precision < 0.8  # measured 0.72: walls that slid along themselves still count
    assert recall < 0.7
