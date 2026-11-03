"""Room files: parsing, furniture macros, validation, world building."""

from __future__ import annotations

import math

import pytest

from roombacalypse.core.config import AppConfig
from roombacalypse.core.geometry import angle_difference
from roombacalypse.core.paths import available_rooms, resolve_room_path
from roombacalypse.environment.rooms import RoomError, build_world, load_room, parse_room
from roombacalypse.environment.shapes import Circle
from roombacalypse.environment.world import ObstacleKind
from tests.helpers import box_room

CONFIG = AppConfig()


def _room(**overrides: object) -> dict[str, object]:
    base: dict[str, object] = {
        "room": {"name": "r", "outline": [[0, 0], [4, 0], [4, 3], [0, 3]]},
        "dock": {"position": [0.0, 1.5], "facing_deg": 0.0},
        "obstacles": [],
    }
    base.update(overrides)
    return base


@pytest.mark.parametrize("name", available_rooms())
def test_builtin_rooms_load_and_validate(name: str) -> None:
    spec = load_room(resolve_room_path(name))
    world = build_world(spec, CONFIG.robot, CONFIG.dock)
    assert world.dock is not None
    assert world.floor_area > 5.0


def test_at_least_two_builtin_rooms() -> None:
    assert {"studio", "test_chamber"} <= set(available_rooms())


def test_dock_pose_faces_the_dock() -> None:
    world = build_world(box_room(), CONFIG.robot, CONFIG.dock)
    assert world.dock is not None
    pose = world.dock.pose
    expected_x = CONFIG.dock.depth + CONFIG.dock.gap + CONFIG.robot.radius
    assert pose.x == pytest.approx(expected_x)
    assert pose.y == pytest.approx(1.5)
    assert abs(angle_difference(pose.theta, math.pi)) < 1e-12  # facing the left wall / dock
    dock_body = world.obstacle(world.dock.obstacle_id)
    assert dock_body.kind is ObstacleKind.DOCK
    assert dock_body.aabb.min_x == pytest.approx(0.0, abs=1e-12)


def test_table_macro_expands_to_legs_and_top() -> None:
    spec = parse_room(_room(obstacles=[{"name": "desk", "shape": "table", "center": [2, 1.5], "size": [1.2, 0.6],
                                        "height": 0.75, "top_thickness": 0.05, "leg_radius": 0.02}]))
    legs = [o for o in spec.obstacles if o.kind is ObstacleKind.LEG]
    tops = [o for o in spec.obstacles if o.kind is ObstacleKind.SURFACE]
    assert len(legs) == 4 and len(tops) == 1
    assert all(isinstance(leg.shape, Circle) and leg.z_max == pytest.approx(0.70) for leg in legs)
    assert tops[0].z_min == pytest.approx(0.70) and tops[0].z_max == pytest.approx(0.75)
    assert len({o.group for o in spec.obstacles}) == 1


def test_chair_macro_and_groups_are_unique_per_entry() -> None:
    spec = parse_room(_room(obstacles=[
        {"name": "chair", "shape": "chair", "center": [1.5, 1.0]},
        {"name": "chair", "shape": "chair", "center": [2.5, 2.0], "angle_deg": 90},
    ]))
    assert len(spec.obstacles) == 12  # 2 x (4 legs + seat + backrest)
    assert len({o.group for o in spec.obstacles}) == 2


@pytest.mark.parametrize(
    ("data", "message"),
    [
        (_room(obstacles=[{"name": "x", "center": [1, 1], "size": [1, 1], "colour": "red"}]), "unknown key"),
        (_room(obstacles=[{"name": "x", "shape": "blob", "center": [1, 1]}]), "unknown shape"),
        (_room(obstacles=[{"name": "x", "center": [1, 1]}]), "missing required key 'size'"),
        (_room(obstacles=[{"name": "x", "kind": "lava", "center": [1, 1], "size": [1, 1]}]), "unknown kind"),
        (_room(obstacles=[{"name": "x", "shape": "polygon", "vertices": [[0, 0], [2, 0], [1, 0.2], [2, 2]]}]),
         "convex"),
        (_room(room={"name": "r", "outline": [[0, 0], [1, 1]]}), "at least 3"),
        (_room(dock={"position": [0, 1.5]}), "facing_deg"),
    ],
)
def test_invalid_rooms_are_rejected(data: dict, message: str) -> None:
    with pytest.raises(RoomError, match=message):
        parse_room(data)


def test_self_intersecting_outline_is_rejected() -> None:
    spec = parse_room(_room(room={"name": "r", "outline": [[0, 0], [4, 3], [4, 0], [0, 3]]}))
    with pytest.raises(ValueError, match="simple"):
        build_world(spec, CONFIG.robot, CONFIG.dock)


def test_obstacle_outside_outline_is_rejected() -> None:
    spec = parse_room(_room(obstacles=[{"name": "ghost", "center": [9, 9], "size": [1, 1]}]))
    with pytest.raises(RoomError, match="outside"):
        build_world(spec, CONFIG.robot, CONFIG.dock)


def test_blocked_dock_is_rejected() -> None:
    spec = parse_room(_room(obstacles=[{"name": "boulder", "shape": "circle", "center": [0.3, 1.5], "radius": 0.2}]))
    with pytest.raises(RoomError, match="does not fit at the dock"):
        build_world(spec, CONFIG.robot, CONFIG.dock)


def test_world_edits_bump_version_and_dock_is_protected() -> None:
    world = build_world(box_room(), CONFIG.robot, CONFIG.dock)
    assert world.dock is not None
    version = world.version
    obstacle = world.add_obstacle("box", ObstacleKind.BOX, Circle(2.0, 1.0, 0.1))
    assert world.version > version
    world.remove_obstacle(obstacle.id)
    with pytest.raises(ValueError):
        world.remove_obstacle(world.dock.obstacle_id)
