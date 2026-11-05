"""Graph SLAM in closed loop: the real simulator, LiDAR, encoders and odometry feeding GraphSlam."""

from __future__ import annotations

import dataclasses
import math
from collections.abc import Sequence

import numpy as np
import pytest

from roombacalypse.autonomy.brain import Mode, RobotBrain
from roombacalypse.core.config import AppConfig
from roombacalypse.core.events import EventKind, EventLog
from roombacalypse.core.geometry import Pose2D, angle_difference
from roombacalypse.core.messages import DriveCommand, SensorFrame
from roombacalypse.core.paths import resolve_room_path
from roombacalypse.environment.rooms import RoomSpec, load_room
from roombacalypse.localization.odometry import OdometryTracker
from roombacalypse.localization.slam import GraphSlam
from roombacalypse.simulation.engine import Simulation
from tests.helpers import IDEAL, ScriptedController, box_room
from tests.test_simulation import STUDIO_TOUR

DEFAULT = AppConfig()


class SlamProbe:
    """Scripted driving plus the robot's own odometry and SLAM (test tooling, not robot behaviour)."""

    name = "probe"

    def __init__(self, config: AppConfig, segments: Sequence[tuple[float, DriveCommand]]) -> None:
        self.script = ScriptedController(segments)
        self.odometry = OdometryTracker(config.robot, config.encoders)
        self.slam = GraphSlam(config.mapping, config.slam)

    def reset(self, seed: int) -> None:
        self.script.reset(seed)
        self.odometry.reset()
        self.slam.reset()

    def update(self, frame: SensorFrame, dt: float) -> DriveCommand:
        self.slam.update(frame.time, self.odometry.update(frame.encoders, frame.time), frame.lidar)
        return self.script.update(frame, dt)


def _drive(config: AppConfig, room: RoomSpec, segments: Sequence[tuple[float, DriveCommand]], seed: int = 1
           ) -> tuple[Simulation, SlamProbe, list[tuple[float, Pose2D, Pose2D, Pose2D]]]:
    """Run the script; returns (time, truth, SLAM belief, odometry belief) per control tick, world frame."""
    probe = SlamProbe(config, segments)
    sim = Simulation(config, room, probe, seed=seed)
    samples = []
    control_every = round(sim.control_period / sim.dt)
    for _ in range(round(sum(d for d, _ in segments) / sim.dt)):
        tick, time, before = sim.tick, sim.time, sim.robot.pose
        sim.step()
        if tick % control_every == 0:
            samples.append((time, before, sim.boot_pose.compose(probe.slam.pose),
                            sim.boot_pose.compose(probe.odometry.pose)))
    return sim, probe, samples


def _errors(samples: list[tuple[float, Pose2D, Pose2D, Pose2D]], which: int) -> np.ndarray:
    return np.array([(s[which].distance_to(s[1]), abs(angle_difference(s[which].theta, s[1].theta))) for s in samples])


# A loop around a room with furniture: back out of the dock, turn, then two laps of a 2.5 x 1.5 m rectangle.
LAP = [(8.33, DriveCommand(0.3, 0.0)), (1.1, DriveCommand(0.0, 1.5)), (5.0, DriveCommand(0.3, 0.0)),
       (1.1, DriveCommand(0.0, 1.5))] * 2
LOOP = [(1.7, DriveCommand(-0.3, 0.0)), (2.2, DriveCommand(0.0, 1.5)), *LAP * 2]
LOOP_ROOM = box_room(6.0, 5.0, obstacles=[
    {"name": "pillar", "kind": "pillar", "shape": "circle", "center": [2.0, 3.25], "radius": 0.15},
    {"name": "cabinet", "center": [5.5, 1.0], "size": [0.6, 0.9]},
    {"name": "plant", "kind": "plant", "shape": "circle", "center": [0.6, 4.4], "radius": 0.2},
    {"name": "box", "center": [3.6, 0.5], "size": [0.5, 0.4]},
], dock=(0.0, 2.5, 0.0))


def test_with_perfect_sensors_slam_is_millimetre_accurate() -> None:
    _, probe, samples = _drive(IDEAL, LOOP_ROOM, LOOP)
    slam_errors = _errors(samples, 2)
    assert slam_errors[:, 0].max() < 0.01
    assert slam_errors[:, 1].max() < math.radians(0.3)
    # Keyframes respect the spacing thresholds.
    kfs = probe.slam.keyframes
    steps = [kfs[i].odometry.relative_to(kfs[i - 1].odometry) for i in range(1, len(kfs))]
    assert all(math.hypot(s.x, s.y) >= 0.25 - 1e-9 or abs(s.theta) >= math.radians(15) - 1e-9 for s in steps)


def test_slam_beats_dead_reckoning_by_an_order_of_magnitude() -> None:
    config = dataclasses.replace(DEFAULT, drive_errors=dataclasses.replace(DEFAULT.drive_errors, wheel_base_std=0.03))
    _, _, samples = _drive(config, load_room(resolve_room_path("studio")), STUDIO_TOUR, seed=4)
    slam_errors, odometry_errors = _errors(samples, 2), _errors(samples, 3)
    assert odometry_errors[-1, 0] > 0.1  # a 3 % wheel-base error: about 11 cm after this 35 s tour
    # The tour pushes into walls at full speed with the wheels spinning. Between two scans the belief follows
    # the (phantom) odometry, so it can run ahead by up to one scan period at 0.5 m/s before a scan that no
    # longer fits forces a correction.
    assert slam_errors[:, 0].max() < 0.5 * DEFAULT.lidar.scan_period + 0.01
    assert math.sqrt((slam_errors[:, 0] ** 2).mean()) < 0.015
    assert slam_errors[-1, 0] < 0.1 * odometry_errors[-1, 0]
    assert math.sqrt((slam_errors[:, 1] ** 2).mean()) < math.radians(0.5)


def test_driving_a_loop_closes_it() -> None:
    sim, probe, samples = _drive(DEFAULT, LOOP_ROOM, LOOP, seed=2)
    slam = probe.slam
    assert len(slam.loop_closures) >= 2
    assert slam.loops_rejected <= slam.loop_attempts // 2
    errors = _errors(samples, 2)
    assert errors[:, 0].max() < 0.04
    # The final, optimised keyframes are consistent with the truth too.
    truth = {round(t, 6): pose for t, pose, _, _ in samples}
    keyframes = [(sim.boot_pose.compose(slam.graph.pose(k.index)), truth[round(k.time, 6)]) for k in slam.keyframes]
    assert max(est.distance_to(true) for est, true in keyframes) < 0.03


def test_the_map_matches_the_room() -> None:
    sim, probe, _ = _drive(DEFAULT, LOOP_ROOM, LOOP, seed=3)
    from roombacalypse.evaluation.slam_metrics import MapAccuracy

    accuracy = MapAccuracy(sim.world, DEFAULT.lidar.mount_height, sim.boot_pose)
    for keyframe in probe.slam.keyframes:  # the surfaces the robot saw, placed at the true poses
        accuracy.observe(keyframe.scan, sim.boot_pose.compose(probe.slam.graph.pose(keyframe.index)))
    score = accuracy.score(probe.slam.grid)
    assert score.precision > 0.98
    assert score.recall > 0.9


def test_slam_is_deterministic() -> None:
    first = _drive(DEFAULT, LOOP_ROOM, LOOP[:6], seed=5)[1].slam.node_poses()
    second = _drive(DEFAULT, LOOP_ROOM, LOOP[:6], seed=5)[1].slam.node_poses()
    assert np.array_equal(first, second)


def test_reset_forgets_the_map() -> None:
    _, probe, _ = _drive(DEFAULT, LOOP_ROOM, LOOP[:4], seed=1)
    slam = probe.slam
    assert len(slam.keyframes) > 5 and slam.grid.known_area > 1.0
    slam.reset()
    assert slam.keyframes == [] and slam.graph.node_count == 0 and slam.grid.known_area == 0.0
    assert slam.pose == Pose2D(0.0, 0.0, 0.0)


@pytest.mark.parametrize("mode", [Mode.MANUAL, Mode.AUTONOMOUS])
def test_the_brain_maps_in_every_mode_and_reports_loop_closures(mode: Mode) -> None:
    events = EventLog()
    brain = RobotBrain(DEFAULT, publish=events.publisher("robot"), mode=mode)
    sim = Simulation(DEFAULT, load_room(resolve_room_path("test_chamber")), brain, seed=2, events=events)
    if mode is Mode.MANUAL:
        brain.teleop.set_intent(-1.0, 0.3)  # a human backs the robot out of the dock in a curve
        sim.run(3.0)
        brain.teleop.set_intent(0.0, 1.0)
        sim.run(4.0)
        assert len(brain.slam.keyframes) > 5
    else:
        sim.run(90.0)
        assert brain.slam.loop_closures
        assert events.count(EventKind.MAP) >= len(brain.slam.loop_closures)
    while sim.tick % round(sim.control_period / sim.dt):
        sim.step()
    truth = sim.robot.pose  # the brain's next update sees exactly this instant
    sim.step()
    belief = sim.boot_pose.compose(brain.pose)
    assert belief.distance_to(truth) < 0.03
    assert abs(angle_difference(belief.theta, truth.theta)) < math.radians(1.0)
