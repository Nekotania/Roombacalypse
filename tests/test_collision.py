"""Collision detection and response: no penetration, sliding, pushing, height bands."""

from __future__ import annotations

import math

import numpy as np
import pytest

from roombacalypse.core.config import AppConfig
from roombacalypse.core.geometry import Pose2D, point_in_polygon
from roombacalypse.core.messages import DriveCommand
from roombacalypse.core.paths import resolve_room_path
from roombacalypse.environment.rooms import build_world, load_room
from roombacalypse.simulation.physics import ContactSolver
from tests.helpers import box_room, make_sim, min_separation

R = AppConfig().robot.radius
FORWARD = DriveCommand(0.3, 0.0)


def test_solver_projects_robot_out_of_wall() -> None:
    config = AppConfig()
    world = build_world(box_room(), config.robot, config.dock)
    solver = ContactSolver(config.robot, config.physics)
    result = solver.resolve(world, Pose2D(3.80, 1.5, 0.0), Pose2D(3.86, 1.5, 0.0))
    assert result.pose.x == pytest.approx(4.0 - R, abs=1e-12)
    assert not result.blocked
    assert len(result.contacts) == 1
    contact = result.contacts[0]
    assert contact.obstacle_id is None
    assert contact.normal == pytest.approx((-1.0, 0.0))
    assert contact.bearing(result.pose) == pytest.approx(0.0)


def test_driving_into_wall_never_penetrates() -> None:
    sim, _ = make_sim(segments=[(10.0, FORWARD)], start=Pose2D(2.0, 1.5, 0.0))
    for _ in range(500):
        sim.step()
        assert min_separation(sim) > -1e-9
    assert sim.robot.pose.x == pytest.approx(4.0 - R, abs=1e-9)
    assert sim.robot.contacts, "bumper should stay pressed while pushing the wall"
    assert sim.robot.linear_velocity == pytest.approx(0.0, abs=1e-9)
    # Wheels keep turning (slipping) even though the body is stopped: this is
    # exactly the odometry error a real robot gets when it pushes a wall.
    assert sim.robot.travel_left > sim.robot.odometer + 1.0


def test_oblique_hit_slides_along_wall() -> None:
    sim, _ = make_sim(segments=[(6.0, FORWARD)], start=Pose2D(1.0, 2.5, math.radians(45)))
    x_at_contact = None
    for _ in range(300):
        sim.step()
        assert sim.robot.pose.y <= 3.0 - R + 1e-9
        if x_at_contact is None and sim.robot.contacts:
            x_at_contact = sim.robot.pose.x
    assert x_at_contact is not None
    assert sim.robot.pose.y == pytest.approx(3.0 - R, abs=1e-9)
    assert sim.robot.pose.x > x_at_contact + 0.4  # kept moving east along the wall


def test_corner_is_resolved_without_jitter() -> None:
    # Hits the right wall first, then slides up it at 0.3 * sin(30°) = 0.15 m/s into the corner.
    sim, _ = make_sim(segments=[(20.0, FORWARD)], start=Pose2D(2.0, 1.0, math.radians(30)))
    sim.run(20.0)
    assert sim.robot.pose.x == pytest.approx(4.0 - R, abs=1e-6)
    assert sim.robot.pose.y == pytest.approx(3.0 - R, abs=1e-6)
    assert len(sim.robot.contacts) == 2
    assert not sim.robot.blocked


def test_head_on_into_pillar_stops_at_contact() -> None:
    room = box_room(obstacles=[{"name": "pillar", "shape": "circle", "center": [3.0, 1.5], "radius": 0.15}])
    sim, _ = make_sim(room, [(8.0, FORWARD)], start=Pose2D(1.0, 1.5, 0.0))
    sim.run(8.0)
    assert sim.robot.pose.x == pytest.approx(3.0 - 0.15 - R, abs=1e-9)
    assert sim.robot.pose.y == pytest.approx(1.5, abs=1e-12)


def test_random_driving_never_penetrates_or_escapes() -> None:
    """Property test: 60 s of random commands in the furnished studio."""
    config = AppConfig()
    rng = np.random.default_rng(7)
    segments = [
        (float(rng.uniform(0.3, 1.5)), DriveCommand(float(rng.uniform(-0.5, 0.5)), float(rng.uniform(-3.0, 3.0))))
        for _ in range(80)
    ]
    sim, _ = make_sim(load_room(resolve_room_path("studio")), segments, config=config)
    worst = math.inf
    for _ in range(3000):
        sim.step()
        worst = min(worst, min_separation(sim))
        assert point_in_polygon(sim.robot.pose.x, sim.robot.pose.y, sim.world.outline)
    assert worst > -1e-9
    assert sim.robot.odometer > 2.0  # it actually went somewhere


def _crate_room(x: float) -> dict[str, object]:
    return {"name": "crate", "kind": "box", "center": [x, 1.5], "size": [0.4, 0.4], "z": [0.0, 0.3], "mobility": 0.6}


def test_robot_pushes_movable_obstacle() -> None:
    room = box_room(width=6.0, obstacles=[_crate_room(2.0)])
    sim, _ = make_sim(room, [(6.0, FORWARD)], start=Pose2D(1.0, 1.5, 0.0))
    crate = next(o for o in sim.world.obstacles if o.name == "crate")
    sim.run(6.0)
    assert crate.shape.centroid[0] > 2.4  # it moved forward...
    assert crate.shape.centroid[1] == pytest.approx(1.5, abs=1e-9)  # ...straight ahead
    assert min_separation(sim) > -1e-9
    # Pushing costs speed: the robot lags behind where free driving would take it.
    free, _ = make_sim(box_room(width=6.0), [(6.0, FORWARD)], start=Pose2D(1.0, 1.5, 0.0))
    free.run(6.0)
    assert sim.robot.pose.x < free.robot.pose.x - 0.1


def test_movable_obstacle_stops_at_wall_and_blocks_robot() -> None:
    room = box_room(obstacles=[_crate_room(3.0)])
    sim, _ = make_sim(room, [(15.0, FORWARD)], start=Pose2D(2.2, 1.5, 0.0))
    crate = next(o for o in sim.world.obstacles if o.name == "crate")
    sim.run(15.0)
    assert crate.aabb.max_x <= 4.0 + 1e-6
    assert crate.aabb.max_x > 3.9  # pushed all the way to the wall
    assert sim.robot.pose.x == pytest.approx(crate.aabb.min_x - R, abs=1e-3)
    assert min_separation(sim) > -1e-9


def test_height_bands_overhead_and_low_obstacles() -> None:
    room = box_room(
        width=6.0,
        obstacles=[
            {"name": "bed", "center": [2.0, 1.5], "size": [0.8, 1.0], "z": [0.12, 0.5]},
            {"name": "cable", "kind": "clutter", "center": [3.5, 1.5], "size": [0.1, 0.8], "z": [0.0, 0.01]},
        ],
    )
    sim, _ = make_sim(room, [(10.0, FORWARD)], start=Pose2D(1.0, 1.5, 0.0))
    passed_under_bed = False
    for _ in range(500):
        sim.step()
        if 1.8 < sim.robot.pose.x < 2.2:
            passed_under_bed = True
            assert not sim.robot.contacts  # body (0.092 m) clears the bed (0.12 m)
    assert passed_under_bed
    cable = next(o for o in sim.world.obstacles if o.name == "cable")
    assert sim.robot.pose.x == pytest.approx(cable.aabb.min_x - R, abs=1e-6)  # stopped by a 1 cm cable


def test_reversing_into_wall_is_a_rear_contact() -> None:
    sim, _ = make_sim(segments=[(5.0, DriveCommand(-0.3, 0.0))], start=Pose2D(1.0, 0.6, 0.0))
    sim.run(5.0)
    (contact,) = sim.robot.contacts
    assert contact.obstacle_id is None
    assert abs(contact.bearing(sim.robot.pose)) == pytest.approx(math.pi)
