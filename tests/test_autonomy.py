"""The robot's brain and the M2 wander behaviour, in isolation and in closed loop."""

from __future__ import annotations

import math

import numpy as np
import pytest

from roombacalypse.autonomy.brain import Mode, RobotBrain
from roombacalypse.autonomy.wander import WanderBehaviour, WanderState
from roombacalypse.core.config import AppConfig, EncoderConfig
from roombacalypse.core.events import EventKind, EventLog
from roombacalypse.core.geometry import Pose2D
from roombacalypse.core.messages import BumperReading, LidarScan
from roombacalypse.core.paths import resolve_room_path
from roombacalypse.environment.rooms import load_room
from roombacalypse.evaluation.scenario import run_scenario
from roombacalypse.localization.odometry import OdometryTracker
from roombacalypse.navigation.local_obstacles import ObstacleMemory
from roombacalypse.simulation.engine import Simulation
from tests.helpers import IDEAL, box_room, min_separation

CONFIG = AppConfig()


def _wander() -> tuple[WanderBehaviour, OdometryTracker]:
    behaviour = WanderBehaviour(CONFIG.robot, CONFIG.wander, CONFIG.dwa, CONFIG.simulation.control_period)
    behaviour.reset(np.random.default_rng(0))
    return behaviour, OdometryTracker(CONFIG.robot, EncoderConfig())


def test_waits_for_the_first_scan() -> None:
    behaviour, odometry = _wander()
    command = behaviour.update(0.0, odometry, np.empty((0, 2)), None, scan_ready=False)
    assert (command.linear, command.angular) == (0.0, 0.0)
    assert behaviour.state is WanderState.WAIT


def test_open_space_picks_a_goal_and_drives() -> None:
    behaviour, odometry = _wander()
    behaviour.update(0.0, odometry, np.empty((0, 2)), None, scan_ready=True)
    assert behaviour.state is WanderState.CRUISE and behaviour.goal is not None
    goal_distance = math.hypot(*behaviour.goal)
    assert CONFIG.wander.goal_min_distance <= goal_distance <= CONFIG.wander.goal_max_distance


@pytest.mark.parametrize(("bump", "turn_sign"), [(BumperReading(left=True), -1.0), (BumperReading(right=True), 1.0)])
def test_bump_backs_off_then_turns_away(bump: BumperReading, turn_sign: float) -> None:
    behaviour, odometry = _wander()
    behaviour.update(0.0, odometry, np.empty((0, 2)), None, scan_ready=True)
    command = behaviour.update(0.1, odometry, np.empty((0, 2)), bump, scan_ready=True)
    assert behaviour.state is WanderState.BACK_OFF and command.linear < 0.0
    odometry.pose = Pose2D(-0.2, 0.0, 0.0)  # pretend it reversed far enough
    command = behaviour.update(0.2, odometry, np.empty((0, 2)), None, scan_ready=True)
    assert behaviour.state is WanderState.TURN
    assert turn_sign * command.angular > 0.0 and command.linear == 0.0


def test_bump_with_obstacle_behind_skips_the_back_off() -> None:
    behaviour, odometry = _wander()
    behind = np.array([[-0.25, 0.0], [-0.25, 0.05], [-0.25, -0.05]])
    behaviour.update(0.0, odometry, behind, None, scan_ready=True)
    behaviour.update(0.1, odometry, behind, BumperReading(left=True, right=True), scan_ready=True)
    assert behaviour.state is WanderState.TURN


def test_cramped_corner_takes_a_short_hop_instead_of_turning_forever() -> None:
    """Regression: nose between two chair legs, wall behind (studio seed 5 used to spin forever)."""
    behaviour, odometry = _wander()
    legs = np.array([[0.14, 0.15], [0.14, -0.21]])  # robot frame: two legs just ahead
    wall = np.column_stack((np.full(60, -0.63), np.linspace(-1.5, 1.5, 60)))
    behaviour.update(0.0, odometry, np.vstack((legs, wall)), None, scan_ready=True)
    assert behaviour.state is WanderState.CRUISE and behaviour.goal is not None
    assert math.hypot(*behaviour.goal) < CONFIG.wander.goal_min_distance + 1e-9


def test_obstacle_memory_compensates_motion() -> None:
    memory = ObstacleMemory(bump_memory_seconds=5.0)
    scan = LidarScan(0.0, np.array([0.0]), np.array([1.0]), 6.0, 0.0, 0.0)  # one point 1 m ahead
    memory.update_scan(scan, Pose2D(0.0, 0.0, 0.0))
    after_moving = memory.points_in_body(Pose2D(0.4, 0.0, 0.0), max_distance=3.0)
    assert after_moving == pytest.approx(np.array([[0.6, 0.0]]))
    memory.add_bump(0.0, Pose2D(0.4, 0.0, 0.0), 0.0, CONFIG.robot.radius)
    assert len(memory.points_in_body(Pose2D(0.4, 0.0, 0.0), 3.0)) == 4
    memory.expire(10.0)
    assert len(memory.points_in_body(Pose2D(0.4, 0.0, 0.0), 3.0)) == 1


def test_brain_keeps_odometry_across_mode_switches_and_reports_events() -> None:
    events = EventLog()
    brain = RobotBrain(IDEAL, publish=events.publisher("robot"))
    sim = Simulation(IDEAL, box_room(), brain, events=events)
    brain.teleop.set_intent(-1.0, 0.0)  # reverse away from the dock
    sim.run(1.5)
    travelled = brain.odometry.distance
    assert travelled > 0.2
    brain.set_mode(Mode.AUTONOMOUS)
    sim.run(5.0)
    assert brain.odometry.distance > travelled  # continuous, not restarted
    assert events.count(EventKind.MODE) == 1
    assert events.count(EventKind.BEHAVIOUR) >= 1  # it chose a goal


def test_invisible_sock_is_felt_then_avoided() -> None:
    sock = {"name": "sock", "kind": "clutter", "center": [1.0, 1.5], "size": [0.08, 0.6], "z": [0.0, 0.02]}
    config = IDEAL
    events = EventLog()
    brain = RobotBrain(config, publish=events.publisher("robot"), mode=Mode.AUTONOMOUS)
    sim = Simulation(config, box_room(width=6.0, obstacles=[sock]), brain, events=events)
    sim.robot.pose = Pose2D(0.5, 1.5, 0.0)  # facing the sock, which the LiDAR cannot see
    for _ in range(round(30.0 / sim.dt)):
        sim.step()
        assert min_separation(sim) > -1e-9
    assert brain.bumps >= 1  # it had to find the sock the hard way
    assert brain.bumps <= 3  # ...and then remembered it


@pytest.mark.parametrize("room_name", ["test_chamber", "studio"])
def test_short_autonomous_run(room_name: str) -> None:
    result = run_scenario(CONFIG, load_room(resolve_room_path(room_name)), duration=90.0, seed=1)
    assert result.min_separation > -1e-9
    assert result.distance > 15.0
    assert result.coverage > 0.15
    assert result.bumps_per_minute < 3.0
