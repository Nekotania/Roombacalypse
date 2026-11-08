"""The MEMS gyro (truth side), gyrodometry with bias calibration (belief side), and the undock step."""

from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest

from roombacalypse.autonomy.brain import Mode, RobotBrain
from roombacalypse.core.config import AppConfig, GyroConfig
from roombacalypse.core.events import EventKind, EventLog
from roombacalypse.core.geometry import angle_difference
from roombacalypse.core.messages import STOP, EncoderReading
from roombacalypse.core.paths import resolve_room_path
from roombacalypse.core.rng import RngStreams
from roombacalypse.environment.rooms import load_room
from roombacalypse.localization.odometry import OdometryTracker
from roombacalypse.sensors.gyro import Gyro
from roombacalypse.simulation.engine import Simulation
from tests.helpers import make_sim
from tests.test_simulation import STUDIO_TOUR

CONFIG = AppConfig()
DT = CONFIG.simulation.dt


def test_a_still_gyro_reads_its_bias_plus_white_noise() -> None:
    config = GyroConfig(bias_walk_deg=0.0, resolution_deg=0.0)
    seconds = 200.0
    finals = []
    for seed in range(40):
        gyro = Gyro(config, RngStreams(seed).get("sensors.gyro"))
        for _ in range(round(seconds / DT)):
            gyro.observe(0.0, DT)
        finals.append(gyro.read().angle - gyro.bias * seconds)
    # Angle random walk: the integrated noise has std = density * sqrt(t).
    expected = math.radians(config.noise_density_deg) * math.sqrt(seconds)
    assert np.std(finals) == pytest.approx(expected, rel=0.3)


def test_bias_and_scale_are_drawn_per_power_up_and_hidden() -> None:
    biases = [Gyro(GyroConfig(), RngStreams(seed).get("sensors.gyro")).bias for seed in range(300)]
    assert np.std(biases) == pytest.approx(math.radians(0.5), rel=0.15)
    gyro = Gyro(GyroConfig(noise_density_deg=0.0, bias_std_deg=0.0, bias_walk_deg=0.0, resolution_deg=0.0,
                           scale_error_std=0.01), RngStreams(3).get("sensors.gyro"))
    for _ in range(100):
        gyro.observe(1.0, DT)  # two seconds at 1 rad/s
    assert gyro.read().angle == pytest.approx(2.0 * (1.0 + gyro.scale_error))


def _still(tracker: OdometryTracker, gyro: Gyro, seconds: float, start: float = 0.0) -> float:
    time = start
    for _ in range(round(seconds / CONFIG.simulation.control_period)):
        for _ in range(round(CONFIG.simulation.control_period / DT)):
            gyro.observe(0.0, DT)
        time += CONFIG.simulation.control_period
        tracker.update(EncoderReading(0, 0), time, gyro.read())
    return time


def test_odometry_measures_the_bias_while_standing_on_the_dock() -> None:
    gyro = Gyro(GyroConfig(bias_std_deg=1.0), RngStreams(5).get("sensors.gyro"))
    tracker = OdometryTracker(CONFIG.robot, CONFIG.encoders, CONFIG.odometry)
    tracker.update(EncoderReading(0, 0), 0.0, gyro.read())
    assert tracker.calibrating
    _still(tracker, gyro, CONFIG.odometry.gyro_calibration_time + 0.2)
    assert not tracker.calibrating
    # Averaging 3 s of noise leaves about 0.01 deg/s / sqrt(3 s) of error, against up to ~2 deg/s of bias.
    assert abs(tracker.gyro_bias - gyro.bias) < math.radians(0.02)
    assert tracker.pose.theta == 0.0  # standing still: the heading is held


def test_without_a_gyro_odometry_uses_the_wheels() -> None:
    tracker = OdometryTracker(CONFIG.robot, CONFIG.encoders, CONFIG.odometry)
    tracker.update(EncoderReading(0, 0), 0.0)
    assert not tracker.using_gyro and not tracker.calibrating


def _tour_error(config: AppConfig, seed: int) -> float:
    """Heading error of dead reckoning after the studio tour (wheels slipping against walls included)."""
    sim, _ = make_sim(load_room(resolve_room_path("studio")), [(4.0, STOP), *STUDIO_TOUR], config=config)
    sim.reset(seed)
    tracker = OdometryTracker(config.robot, config.encoders, config.odometry)
    for _ in range(round((4.0 + sum(d for d, _ in STUDIO_TOUR)) / sim.dt)):
        if sim.tick % round(sim.control_period / sim.dt) == 0:
            frame = sim.sense()
            tracker.update(frame.encoders, frame.time, frame.gyro)
        sim.step()
    belief = sim.boot_pose.compose(tracker.pose)
    return abs(angle_difference(belief.theta, sim.robot.pose.theta))


def test_gyrodometry_keeps_heading_where_the_wheels_lose_it() -> None:
    wheels = dataclasses.replace(CONFIG, odometry=dataclasses.replace(CONFIG.odometry, use_gyro=False))
    gyro_errors = [_tour_error(CONFIG, seed) for seed in range(1, 6)]
    wheel_errors = [_tour_error(wheels, seed) for seed in range(1, 6)]
    assert max(gyro_errors) < math.radians(1.5)  # measured 0.5-1.1°: mostly the 0.3 % scale error on ~10 rad of turning
    assert np.mean(wheel_errors) > 5 * np.mean(gyro_errors)


def test_the_robot_calibrates_then_backs_off_its_dock_before_exploring() -> None:
    events = EventLog()
    brain = RobotBrain(CONFIG, publish=events.publisher("robot"), mode=Mode.AUTONOMOUS)
    sim = Simulation(CONFIG, load_room(resolve_room_path("studio")), brain, seed=1, events=events)
    dock = sim.robot.pose
    sim.run(CONFIG.odometry.gyro_calibration_time - 0.5)
    assert sim.robot.pose == dock  # still calibrating
    furthest = 0.0
    for _ in range(round(3.0 / sim.dt)):
        sim.step()
        furthest = max(furthest, -dock.inverse_transform_point(*sim.robot.pose.position)[0])
    assert furthest >= CONFIG.wander.undock_distance - 0.01  # reversed straight off the dock
    assert any("undocking" in e.message for e in events.recent(50) if e.kind is EventKind.BEHAVIOUR)
    assert events.count(EventKind.COLLISION) == 0
