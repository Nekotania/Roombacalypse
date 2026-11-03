"""Engine behaviour: determinism, control rate, reset, robustness, world editing."""

from __future__ import annotations

import math

import pytest

from roombacalypse.core.config import AppConfig
from roombacalypse.core.geometry import Pose2D
from roombacalypse.core.messages import DriveCommand
from roombacalypse.core.paths import resolve_room_path
from roombacalypse.core.rng import RngStreams
from roombacalypse.environment.catalog import PLACEABLE
from roombacalypse.environment.rooms import load_room
from tests.helpers import box_room, make_sim

# A deterministic regression scenario: back out of the dock, spin, then a tour
# of the studio that includes wall slides and pushing the sock and basket.
STUDIO_TOUR = [
    (1.2, DriveCommand(-0.3, 0.0)),
    (1.5, DriveCommand(0.0, 2.0)),
    (9.0, DriveCommand(0.3, 0.05)),
    (2.0, DriveCommand(0.0, -1.5)),
    (8.0, DriveCommand(0.4, 0.2)),
    (3.0, DriveCommand(-0.2, -1.0)),
    (10.0, DriveCommand(0.5, -0.1)),
]


def _run_tour(seed: int = 42) -> list[tuple[float, float, float]]:
    sim, _ = make_sim(load_room(resolve_room_path("studio")), STUDIO_TOUR)
    sim.reset(seed)
    trajectory = []
    for _ in range(round(sum(d for d, _ in STUDIO_TOUR) / sim.dt)):
        sim.step()
        trajectory.append((sim.robot.pose.x, sim.robot.pose.y, sim.robot.pose.theta))
    return trajectory


def test_simulation_is_bit_for_bit_deterministic() -> None:
    assert _run_tour() == _run_tour()


def test_controller_runs_at_control_rate() -> None:
    sim, controller = make_sim()
    sim.run(1.0)
    assert controller.calls == round(1.0 / sim.control_period)


def test_command_is_held_between_control_ticks() -> None:
    sim, _ = make_sim(segments=[(0.05, DriveCommand(0.3, 0.0)), (10.0, DriveCommand(0.0, 0.0))],
                      start=Pose2D(1.0, 1.5, 0.0))
    sim.run(0.1)  # the 0.05 s segment is sampled once at t=0 and held for the whole 0.1 s period
    assert sim.command == DriveCommand(0.3, 0.0)
    sim.step()
    assert sim.command == DriveCommand(0.0, 0.0)


def test_reset_restores_world_and_robot() -> None:
    room = box_room(obstacles=[{"name": "crate", "kind": "box", "center": [2.0, 1.5], "size": [0.4, 0.4],
                                "z": [0.0, 0.3], "mobility": 0.6}])
    sim, controller = make_sim(room, [(6.0, DriveCommand(-0.3, 0.0)), (2.0, DriveCommand(0.0, 2.0)),
                                      (6.0, DriveCommand(0.3, 0.0))])
    initial_pose = sim.robot.pose
    crate_start = next(o for o in sim.world.obstacles if o.name == "crate").shape.centroid
    sim.run(14.0)
    assert sim.robot.pose != initial_pose
    sim.reset()
    assert sim.time == 0.0
    assert sim.robot.pose == initial_pose
    assert sim.robot.odometer == 0.0
    assert next(o for o in sim.world.obstacles if o.name == "crate").shape.centroid == crate_start
    assert controller.calls == 0


def test_non_finite_command_stops_the_robot() -> None:
    sim, _ = make_sim(segments=[(5.0, DriveCommand(math.nan, 0.0))], start=Pose2D(1.0, 1.5, 0.0))
    sim.run(1.0)
    assert sim.robot.pose == Pose2D(1.0, 1.5, 0.0)
    assert sim.command == DriveCommand()


def test_place_and_remove_obstacles() -> None:
    sim, _ = make_sim(start=Pose2D(1.0, 1.5, 0.0))
    box = PLACEABLE[0]
    assert sim.place_obstacle(box, 2.5, 1.5).ok
    assert not sim.place_obstacle(box, 2.5, 1.5).ok  # occupied now
    assert not sim.place_obstacle(box, 1.0, 1.5).ok  # the robot is there
    assert not sim.place_obstacle(box, 3.95, 1.5).ok  # through the wall
    assert not sim.place_obstacle(box, 9.0, 9.0).ok  # outside the room
    removed = sim.remove_obstacle_at(2.5, 1.5)
    assert [o.name for o in removed] == [box.label]
    assert sim.remove_obstacle_at(2.5, 1.5) == []


def test_dock_cannot_be_removed() -> None:
    sim, _ = make_sim()
    dock = sim.world.obstacle(sim.world.dock.obstacle_id)  # type: ignore[union-attr]
    x, y = dock.shape.centroid
    assert sim.remove_obstacle_at(x, y) == []


def test_removing_a_table_part_removes_the_whole_table() -> None:
    room = box_room(obstacles=[{"name": "table", "shape": "table", "center": [2.5, 1.5], "size": [1.0, 0.6]}])
    sim, _ = make_sim(room)
    removed = sim.remove_obstacle_at(2.5, 1.5)
    assert len(removed) == 5
    assert all(o.group == removed[0].group for o in removed)


def test_rng_streams_are_independent_and_reproducible() -> None:
    a, b = RngStreams(1), RngStreams(1)
    first = a.get("sensors.lidar").random(3)
    a.get("something.else").random(100)  # consuming another stream...
    assert list(a.get("sensors.lidar").random(3)) == list(b.get("sensors.lidar").random(6)[3:])  # ...changes nothing
    assert list(first) != list(RngStreams(2).get("sensors.lidar").random(3))
    with pytest.raises(ValueError):
        RngStreams(-1)


def test_default_config_robot_fits_default_room() -> None:
    sim, _ = make_sim(load_room(resolve_room_path("studio")), config=AppConfig())
    assert sim.world.dock is not None
    assert sim.robot.pose == sim.world.dock.pose
