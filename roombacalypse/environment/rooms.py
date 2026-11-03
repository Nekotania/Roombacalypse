"""Room files: parse TOML room descriptions and build :class:`World` instances.

A room file describes the floor outline, the dock and a list of obstacles.
Besides primitive shapes (``box``, ``circle``, ``polygon``) it supports two
furniture macros that expand into several obstacles sharing a group:

* ``table``: four legs plus a top. A bed is a table with a thick top whose
  underside is higher than the robot.
* ``chair``: four legs, a seat and a backrest.

Parsing is strict: unknown keys raise :class:`RoomError` with the location,
because a silently ignored typo in a room file wastes an afternoon.
See ``config/rooms/studio.toml`` for an annotated example.
"""

from __future__ import annotations

import logging
import math
import tomllib
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from roombacalypse.core.config import DockConfig, RobotSpec
from roombacalypse.core.geometry import Point, Pose2D, point_segment_distance
from roombacalypse.environment.shapes import Circle, ConvexPolygon, Shape
from roombacalypse.environment.world import Dock, ObstacleKind, World

logger = logging.getLogger(__name__)


class RoomError(ValueError):
    """Invalid room description."""


@dataclass(frozen=True)
class ObstacleSpec:
    """Immutable description of one obstacle, as read from a room file."""

    name: str
    kind: ObstacleKind
    shape: Shape
    z_min: float = 0.0
    z_max: float = 1.0
    mobility: float = 0.0
    group: str | None = None


@dataclass(frozen=True)
class DockPlacement:
    """Where the dock's back touches a wall, and the direction it faces (into the room)."""

    x: float
    y: float
    facing: float


@dataclass(frozen=True)
class RoomSpec:
    """Immutable room description. :func:`build_world` turns it into a fresh mutable world."""

    name: str
    description: str
    outline: tuple[Point, ...]
    dock: DockPlacement
    obstacles: tuple[ObstacleSpec, ...]
    source: str = "<memory>"


# --------------------------------------------------------------------------- strict field access

_MISSING: Any = object()


class _Fields:
    """Typed, strict access to one TOML table that reports unused (unknown) keys."""

    def __init__(self, data: Any, where: str) -> None:
        if not isinstance(data, Mapping):
            raise RoomError(f"{where} must be a table")
        self.data = data
        self.where = where
        self.used: set[str] = set()

    def _raw(self, key: str, default: Any) -> Any:
        self.used.add(key)
        if key in self.data:
            return self.data[key]
        if default is _MISSING:
            raise RoomError(f"{self.where}: missing required key '{key}'")
        return default

    def number(self, key: str, default: Any = _MISSING) -> float:
        value = self._raw(key, default)
        if isinstance(value, bool) or not isinstance(value, int | float):
            raise RoomError(f"{self.where}.{key} must be a number, got {value!r}")
        return float(value)

    def string(self, key: str, default: Any = _MISSING) -> str:
        value = self._raw(key, default)
        if not isinstance(value, str):
            raise RoomError(f"{self.where}.{key} must be a string, got {value!r}")
        return value

    def pair(self, key: str, default: Any = _MISSING) -> tuple[float, float]:
        value = self._raw(key, default)
        if (
            not isinstance(value, list | tuple)
            or len(value) != 2
            or not all(isinstance(v, int | float) and not isinstance(v, bool) for v in value)
        ):
            raise RoomError(f"{self.where}.{key} must be a pair of numbers, got {value!r}")
        return float(value[0]), float(value[1])

    def points(self, key: str) -> tuple[Point, ...]:
        value = self._raw(key, _MISSING)
        if not isinstance(value, list) or len(value) < 3:
            raise RoomError(f"{self.where}.{key} must be a list of at least 3 [x, y] points")
        points = []
        for i, item in enumerate(value):
            if not isinstance(item, list) or len(item) != 2:
                raise RoomError(f"{self.where}.{key}[{i}] must be an [x, y] pair, got {item!r}")
            points.append((float(item[0]), float(item[1])))
        return tuple(points)

    def kind(self, key: str, default: ObstacleKind) -> ObstacleKind:
        value = self.string(key, default.value)
        try:
            return ObstacleKind(value)
        except ValueError:
            options = ", ".join(k.value for k in ObstacleKind)
            raise RoomError(f"{self.where}.{key}: unknown kind {value!r} (choose from {options})") from None

    def finish(self) -> None:
        unknown = sorted(set(self.data) - self.used)
        if unknown:
            raise RoomError(f"{self.where}: unknown key(s) {', '.join(unknown)}")


# --------------------------------------------------------------------------- parsing


def load_room(path: Path) -> RoomSpec:
    """Read and parse a room file."""
    try:
        with path.open("rb") as handle:
            data = tomllib.load(handle)
    except tomllib.TOMLDecodeError as exc:
        raise RoomError(f"{path}: {exc}") from exc
    return parse_room(data, source=str(path))


def parse_room(data: Mapping[str, Any], source: str = "<memory>") -> RoomSpec:
    """Parse an already-loaded room mapping (TOML structure)."""
    top = _Fields(data, source)
    room = _Fields(top._raw("room", _MISSING), f"{source}: [room]")
    name = room.string("name")
    description = room.string("description", "")
    outline = room.points("outline")
    room.finish()

    dock_table = _Fields(top._raw("dock", _MISSING), f"{source}: [dock]")
    dx, dy = dock_table.pair("position")
    dock = DockPlacement(dx, dy, math.radians(dock_table.number("facing_deg")))
    dock_table.finish()

    entries = top._raw("obstacles", [])
    if not isinstance(entries, list):
        raise RoomError(f"{source}: [[obstacles]] must be an array of tables")
    obstacles: list[ObstacleSpec] = []
    for index, entry in enumerate(entries):
        obstacles.extend(_parse_obstacle(entry, index, f"{source}: obstacles[{index}]"))
    top.finish()
    return RoomSpec(name, description, outline, dock, tuple(obstacles), source)


def _parse_obstacle(entry: Any, index: int, where: str) -> list[ObstacleSpec]:
    fields = _Fields(entry, where)
    shape_type = fields.string("shape", "box")
    name = fields.string("name", shape_type)
    where = f"{where} ({name})"
    fields.where = where
    try:
        if shape_type == "table":
            specs = _expand_table(fields, name, group=f"{name}#{index}")
        elif shape_type == "chair":
            specs = _expand_chair(fields, name, group=f"{name}#{index}")
        elif shape_type in ("box", "circle", "polygon"):
            specs = [_parse_primitive(fields, name, shape_type)]
        else:
            raise RoomError(f"{where}: unknown shape {shape_type!r} (box, circle, polygon, table, chair)")
    except RoomError:
        raise
    except ValueError as exc:  # raised by shape constructors
        raise RoomError(f"{where}: {exc}") from exc
    fields.finish()
    return specs


def _parse_primitive(fields: _Fields, name: str, shape_type: str) -> ObstacleSpec:
    kind = fields.kind("kind", ObstacleKind.FURNITURE)
    z_min, z_max = fields.pair("z", (0.0, 1.0))
    mobility = fields.number("mobility", 0.0)
    shape: Shape
    if shape_type == "box":
        cx, cy = fields.pair("center")
        sx, sy = fields.pair("size")
        shape = ConvexPolygon.box(cx, cy, sx, sy, math.radians(fields.number("angle_deg", 0.0)))
    elif shape_type == "circle":
        cx, cy = fields.pair("center")
        shape = Circle(cx, cy, fields.number("radius"))
    else:
        shape = ConvexPolygon(fields.points("vertices"))
    return ObstacleSpec(name, kind, shape, z_min, z_max, mobility)


def _expand_table(fields: _Fields, name: str, group: str) -> list[ObstacleSpec]:
    cx, cy = fields.pair("center")
    sx, sy = fields.pair("size")
    pose = Pose2D(cx, cy, math.radians(fields.number("angle_deg", 0.0)))
    height = fields.number("height", 0.75)
    top_thickness = fields.number("top_thickness", 0.04)
    leg_radius = fields.number("leg_radius", 0.025)
    leg_inset = fields.number("leg_inset", 0.05)
    kind = fields.kind("kind", ObstacleKind.SURFACE)
    if not 0.0 < top_thickness < height:
        raise RoomError(f"{fields.where}: need 0 < top_thickness < height")
    hx, hy = 0.5 * sx - leg_inset, 0.5 * sy - leg_inset
    if hx < leg_radius or hy < leg_radius:
        raise RoomError(f"{fields.where}: legs do not fit inside the top (reduce leg_inset)")
    underside = height - top_thickness
    specs = [
        ObstacleSpec(f"{name} leg", ObstacleKind.LEG, Circle(*pose.transform_point(lx, ly), leg_radius),
                     0.0, underside, 0.0, group)
        for lx, ly in ((hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy))
    ]
    top = ConvexPolygon.box(cx, cy, sx, sy, pose.theta)
    specs.append(ObstacleSpec(f"{name}", kind, top, underside, height, 0.0, group))
    return specs


def _expand_chair(fields: _Fields, name: str, group: str) -> list[ObstacleSpec]:
    cx, cy = fields.pair("center")
    depth, width = fields.pair("size", (0.42, 0.42))
    pose = Pose2D(cx, cy, math.radians(fields.number("angle_deg", 0.0)))  # angle = direction the seat faces
    seat_height = fields.number("seat_height", 0.46)
    back_height = fields.number("back_height", 0.45)
    leg_radius = fields.number("leg_radius", 0.015)
    leg_inset = fields.number("leg_inset", 0.03)
    hx, hy = 0.5 * depth - leg_inset, 0.5 * width - leg_inset
    if hx < leg_radius or hy < leg_radius:
        raise RoomError(f"{fields.where}: legs do not fit under the seat (reduce leg_inset)")
    seat_bottom = seat_height - 0.04
    specs = [
        ObstacleSpec(f"{name} leg", ObstacleKind.LEG, Circle(*pose.transform_point(lx, ly), leg_radius),
                     0.0, seat_bottom, 0.0, group)
        for lx, ly in ((hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy))
    ]
    specs.append(ObstacleSpec(name, ObstacleKind.SURFACE, ConvexPolygon.box(cx, cy, depth, width, pose.theta),
                              seat_bottom, seat_height, 0.0, group))
    bx, by = pose.transform_point(-0.5 * depth + 0.02, 0.0)
    specs.append(ObstacleSpec(f"{name} backrest", ObstacleKind.SURFACE,
                              ConvexPolygon.box(bx, by, 0.04, width, pose.theta),
                              seat_height, seat_height + back_height, 0.0, group))
    return specs


# --------------------------------------------------------------------------- building


def build_world(spec: RoomSpec, robot: RobotSpec, dock_config: DockConfig) -> World:
    """Create a fresh :class:`World` from a room description and validate it."""
    world = World(spec.name, spec.outline)
    for o in spec.obstacles:
        if not world.contains_point(*o.shape.centroid):
            raise RoomError(f"{spec.source}: obstacle '{o.name}' is outside the room outline")
        world.add_obstacle(o.name, o.kind, o.shape, z_min=o.z_min, z_max=o.z_max, mobility=o.mobility, group=o.group)

    placement = spec.dock
    ux, uy = math.cos(placement.facing), math.sin(placement.facing)
    half_depth = 0.5 * dock_config.depth
    body = ConvexPolygon.box(placement.x + ux * half_depth, placement.y + uy * half_depth,
                             dock_config.depth, dock_config.width, placement.facing)
    dock_obstacle = world.add_obstacle("charging dock", ObstacleKind.DOCK, body, z_min=0.0, z_max=dock_config.height)
    offset = dock_config.depth + dock_config.gap + robot.radius
    pose = Pose2D(placement.x + ux * offset, placement.y + uy * offset, placement.facing + math.pi)
    world.set_dock(Dock(pose, dock_obstacle.id))

    _validate(world, spec, robot)
    return world


def _validate(world: World, spec: RoomSpec, robot: RobotSpec) -> None:
    dock = world.dock
    assert dock is not None
    robot_shape = Circle(dock.pose.x, dock.pose.y, robot.radius)
    if not world.is_shape_free(robot_shape, 0.0, robot.height):
        raise RoomError(f"{spec.source}: the robot does not fit at the dock (something overlaps the docking pose)")
    wall_gap = min(point_segment_distance(spec.dock.x, spec.dock.y, *wall) for wall in world.walls)
    if wall_gap > 0.02:
        logger.warning("%s: the dock is %.2f m from the nearest wall; docks usually sit against one",
                       spec.source, wall_gap)
    for obstacle in world.obstacles:
        if obstacle.movable and not world.is_shape_free(
            obstacle.shape, obstacle.z_min, obstacle.z_max, ignore=(obstacle.id,)
        ):
            logger.warning("%s: movable obstacle '%s' starts overlapping something and may not move",
                           spec.source, obstacle.name)
