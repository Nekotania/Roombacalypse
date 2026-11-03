"""Bumper, wheel encoders, the hidden wheel/ground error model, and odometry."""

from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest

from roombacalypse.core.config import AppConfig, BumperConfig, DriveErrorConfig, EncoderConfig, RobotSpec
from roombacalypse.core.geometry import Pose2D, angle_difference
from roombacalypse.core.messages import DriveCommand, EncoderReading
from roombacalypse.localization.odometry import OdometryTracker
from roombacalypse.robot.body import Contact, RobotBody
from roombacalypse.robot.wheels import WheelGroundModel
from roombacalypse.sensors.bumper import Bumper
from roombacalypse.sensors.encoders import WheelEncoders
from tests.helpers import IDEAL, box_room, make_sim

SPEC = RobotSpec()


def _contact_at(bearing_deg: float) -> Contact:
    """A contact on the body rim at ``bearing_deg`` for a robot at the origin facing +x."""
    b = math.radians(bearing_deg)
    return Contact((-math.cos(b), -math.sin(b)), (SPEC.radius * math.cos(b), SPEC.radius * math.sin(b)), 0.0, None)


@pytest.mark.parametrize(
    ("bearing", "left", "right"),
    [(60.0, True, False), (-60.0, False, True), (5.0, True, True), (-10.0, True, True), (120.0, False, False)],
)
def test_bumper_switches_by_bearing(bearing: float, left: bool, right: bool) -> None:
    bumper = Bumper(BumperConfig())
    bumper.observe(RobotBody(pose=Pose2D(0.0, 0.0, 0.0), contacts=(_contact_at(bearing),)))
    reading = bumper.read()
    assert (reading.left, reading.right) == (left, right)


def test_bumper_latches_until_read() -> None:
    bumper = Bumper(BumperConfig())
    bumper.observe(RobotBody(pose=Pose2D(0.0, 0.0, 0.0), contacts=(_contact_at(40.0),)))
    bumper.observe(RobotBody(pose=Pose2D(0.0, 0.0, 0.0)))  # contact over before the software looks
    assert bumper.read().left
    assert not bumper.read().pressed


def test_encoders_quantise_wheel_rotation() -> None:
    encoders = WheelEncoders(SPEC, EncoderConfig())
    ticks_per_metre = 508.8 / (2 * math.pi * SPEC.wheel_radius)
    reading = encoders.read(RobotBody(pose=Pose2D(0, 0, 0), travel_left=1.0, travel_right=-0.5))
    assert reading == EncoderReading(math.floor(ticks_per_metre), math.floor(-0.5 * ticks_per_metre))


def test_ideal_wheels_are_exact() -> None:
    rng = np.random.default_rng(0)
    model = WheelGroundModel(SPEC, DriveErrorConfig(0.0, 0.0, 0.0), rng, rng)
    assert model.ground_travel(0.01, -0.02, 0.02) == (0.01, -0.02)
    assert model.wheel_base == SPEC.wheel_base


def test_slip_process_has_the_configured_statistics() -> None:
    model = WheelGroundModel(SPEC, DriveErrorConfig(0.0, 0.0, slip_std=0.02, slip_time_constant=0.5),
                             np.random.default_rng(1), np.random.default_rng(2))
    samples = []
    for _ in range(100_000):
        model.ground_travel(0.0, 0.0, 0.02)
        samples.append(model.slip_left)
    series = np.array(samples[1000:])
    assert series.std() == pytest.approx(0.02, rel=0.1)
    lag = round(0.5 / 0.02)  # one time constant
    autocorrelation = np.corrcoef(series[:-lag], series[lag:])[0, 1]
    assert autocorrelation == pytest.approx(math.exp(-1.0), abs=0.05)


def test_unequal_wheels_curve_by_the_predicted_amount() -> None:
    """Borenstein & Feng's systematic error: a 'straight' drive turns by d·(s_r − s_l)/L."""
    sim, _ = make_sim(box_room(width=8.0), [(10.0, DriveCommand(0.3, 0.0))], start=Pose2D(1.0, 1.5, 0.0))
    sim.ground.left_scale, sim.ground.right_scale = 0.995, 1.005
    sim.run(6.0)
    distance = sim.robot.odometer
    predicted = distance * (1.005 - 0.995) / SPEC.wheel_base
    assert angle_difference(sim.robot.pose.theta, 0.0) == pytest.approx(predicted, rel=0.03)


def _replay_odometry(frames: list, spec: RobotSpec = SPEC) -> OdometryTracker:
    tracker = OdometryTracker(spec, EncoderConfig())
    for frame in frames:
        tracker.update(frame.encoders, frame.time)
    return tracker


TOUR = [(2.0, DriveCommand(0.3, 0.0)), (1.5, DriveCommand(0.2, 1.0)), (2.0, DriveCommand(0.0, -1.5)),
        (3.0, DriveCommand(0.25, -0.4))]


def test_odometry_matches_truth_with_ideal_wheels() -> None:
    sim, controller = make_sim(box_room(width=6.0, height=5.0), TOUR, start=Pose2D(2.0, 2.5, 0.0))
    sim.run(9.5)  # the tour ends at 8.5 s; frames continue after the robot has braked to a stop
    tracker = _replay_odometry(controller.frames)
    belief = Pose2D(2.0, 2.5, 0.0).compose(tracker.pose)
    assert belief.distance_to(sim.robot.pose) < 0.01  # only tick quantisation remains
    assert abs(angle_difference(belief.theta, sim.robot.pose.theta)) < math.radians(1.0)


def test_odometry_velocity_estimate() -> None:
    sim, controller = make_sim(box_room(width=8.0), [(5.0, DriveCommand(0.3, 0.0))], start=Pose2D(1.0, 1.5, 0.0))
    sim.run(3.0)
    tracker = _replay_odometry(controller.frames)
    assert tracker.linear_velocity == pytest.approx(0.3, rel=0.03)
    assert tracker.angular_velocity == pytest.approx(0.0, abs=0.02)


def test_odometry_drifts_with_realistic_wheels_and_keeps_counting_against_walls() -> None:
    config = dataclasses.replace(AppConfig(), drive_errors=DriveErrorConfig(0.002, 0.01, 0.01))
    sim, controller = make_sim(box_room(), [(12.0, DriveCommand(0.3, 0.0))], start=Pose2D(1.0, 1.5, 0.0), config=config)
    sim.run(12.0)  # reaches the far wall after ~9 s and keeps pushing
    tracker = _replay_odometry(controller.frames)
    assert sim.robot.pose.x == pytest.approx(4.0 - SPEC.radius, abs=1e-6)  # truth: stopped at the wall
    assert tracker.distance > sim.robot.odometer + 0.5  # encoders: wheels kept turning


def test_ideal_config_helper_really_is_ideal() -> None:
    assert IDEAL.drive_errors == DriveErrorConfig(0.0, 0.0, 0.0)
    assert IDEAL.lidar.dropout_probability == 0.0
