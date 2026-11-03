"""Event log semantics, collision events, and the ground-truth coverage metric."""

from __future__ import annotations

import math

import pytest

from roombacalypse.core.events import EventKind, EventLog
from roombacalypse.core.geometry import Pose2D
from roombacalypse.core.messages import DriveCommand
from roombacalypse.environment.rooms import build_world
from roombacalypse.evaluation.coverage import FootprintCoverage
from tests.helpers import IDEAL, box_room, make_sim

R = IDEAL.robot.radius


def test_event_log_cursor_capacity_and_counts() -> None:
    log = EventLog(capacity=3)
    for i in range(5):
        log.emit(float(i), "sim", EventKind.COLLISION, f"event {i}")
    events, cursor = log.since(0)
    assert [e.message for e in events] == ["event 2", "event 3", "event 4"]  # oldest dropped
    assert log.count(EventKind.COLLISION) == 5  # counters are not limited by capacity
    log.emit(5.0, "robot", EventKind.BUMP, "bump")
    newer, _ = log.since(cursor)
    assert [e.kind for e in newer] == [EventKind.BUMP]
    publish = log.publisher("robot")
    publish(6.0, EventKind.BEHAVIOUR, "hello", detail=1)
    assert log.recent(1)[0].source == "robot" and log.recent(1)[0].data == {"detail": 1}
    log.clear()
    assert log.total == 0 and log.count(EventKind.BUMP) == 0


def test_one_collision_event_per_contact() -> None:
    segments = [(4.0, DriveCommand(0.3, 0.0)), (1.0, DriveCommand(-0.3, 0.0)), (3.0, DriveCommand(0.3, 0.0))]
    sim, _ = make_sim(segments=segments, start=Pose2D(3.0, 1.5, 0.0))
    sim.run(8.0)
    collisions = [e for e in sim.events.recent(50) if e.kind is EventKind.COLLISION]
    assert [e.message for e in collisions] == ["collision with wall", "collision with wall"]


def test_accessible_area_of_an_empty_box() -> None:
    world = build_world(box_room(), IDEAL.robot, IDEAL.dock)
    coverage = FootprintCoverage(world, IDEAL.robot, world.dock.pose.position)  # type: ignore[union-attr]
    # 12 m² minus the dock and the corners a disc cannot reach, give or take the 5 cm grid.
    assert 11.5 < coverage.accessible_area < 12.0
    assert coverage.fraction == 0.0


def test_footprint_marks_a_disc() -> None:
    world = build_world(box_room(), IDEAL.robot, IDEAL.dock)
    coverage = FootprintCoverage(world, IDEAL.robot, (2.0, 1.5))
    coverage.mark(2.0, 1.5)
    assert coverage.covered_area == pytest.approx(math.pi * R * R, rel=0.15)


def test_pocket_behind_a_narrow_gap_is_not_accessible() -> None:
    # A 0.3 m doorway (narrower than the 0.34 m robot) into the right part of the room.
    wall = {"name": "partition", "kind": "wall", "center": [3.0, 0.0], "size": [0.1, 0.0], "z": [0.0, 2.0]}
    upper = dict(wall, center=[3.0, 2.25], size=[0.1, 1.5])
    lower = dict(wall, center=[3.0, 0.6], size=[0.1, 1.2])
    world = build_world(box_room(width=4.0, obstacles=[upper, lower]), IDEAL.robot, IDEAL.dock)
    coverage = FootprintCoverage(world, IDEAL.robot, world.dock.pose.position)  # type: ignore[union-attr]
    right_cell = coverage.cell_of(3.5, 1.5)
    left_cell = coverage.cell_of(1.5, 1.5)
    assert not coverage.accessible[right_cell]
    assert coverage.accessible[left_cell]
