"""The ground-truth world: a floor outline (walls), obstacles and the dock.

Geometry is continuous (metres), not a grid: the robot's occupancy grid
(milestone 3) has to be *built from sensor data*, and a grid-based world would
make that grid identical to the truth.

Height matters. Every obstacle occupies a vertical band ``[z_min, z_max)``:

* the robot body (``[0, robot.height)``) collides only with obstacles whose
  band overlaps it, so it drives under beds and table tops but not into legs;
* the LiDAR (milestone 2) will see only obstacles whose band contains its
  mounting height, so socks and cables below the scan plane are invisible to
  it and are found by bumping into them, as on real robots.

Walls are the edges of the outline and span all heights.
"""

from __future__ import annotations

import logging
from collections.abc import Iterable, Iterator, Sequence
from dataclasses import dataclass, field
from enum import StrEnum

from roombacalypse.core.geometry import (
    Point,
    Pose2D,
    Segment,
    is_simple_polygon,
    point_in_polygon,
    polygon_signed_area,
)
from roombacalypse.environment.shapes import (
    AABB,
    Shape,
    shape_overlaps_segment,
    shapes_overlap,
)

logger = logging.getLogger(__name__)


class ObstacleKind(StrEnum):
    """What an obstacle is; drives rendering and, later, how the robot narrates it."""

    FURNITURE = "furniture"
    SOFT = "soft"  # upholstered: sofas, beds, ottomans
    LEG = "leg"
    SURFACE = "surface"  # table tops, seats, backrests (usually overhead)
    WALL = "wall"  # interior partitions
    APPLIANCE = "appliance"
    PLANT = "plant"
    PILLAR = "pillar"
    BOX = "box"
    CLUTTER = "clutter"  # socks, cables, toys: low junk below the LiDAR plane
    DOCK = "dock"


@dataclass(eq=False)
class Obstacle:
    """One rigid obstacle. Change its shape only through :meth:`World.move_obstacle`."""

    id: int
    name: str
    kind: ObstacleKind
    shape: Shape
    z_min: float = 0.0
    z_max: float = 1.0
    mobility: float = 0.0
    """How readily it yields when pushed: 0 = immovable, 1 = moves with no resistance."""
    group: str | None = None
    """Parts of one piece of furniture (legs + top of a table) share a group name."""
    aabb: AABB = field(init=False, repr=False)

    def __post_init__(self) -> None:
        if not 0.0 <= self.z_min < self.z_max:
            raise ValueError(f"{self.name}: need 0 <= z_min < z_max, got [{self.z_min}, {self.z_max}]")
        if not 0.0 <= self.mobility <= 1.0:
            raise ValueError(f"{self.name}: mobility must be in [0, 1], got {self.mobility}")
        self.aabb = self.shape.aabb()

    @property
    def movable(self) -> bool:
        return self.mobility > 0.0

    def overlaps_band(self, z0: float, z1: float) -> bool:
        """True if this obstacle's vertical extent overlaps ``[z0, z1)``."""
        return self.z_min < z1 and self.z_max > z0


@dataclass(frozen=True)
class Dock:
    """The charging station.

    ``pose`` is where the robot's centre sits while charging; its heading
    points *at* the dock (the robot drives in forwards, Roomba-style).
    """

    pose: Pose2D
    obstacle_id: int


class World:
    """Mutable ground-truth geometry of one room.

    ``version`` increases on every change so caches (renderers, raycasters)
    can tell when to rebuild.
    """

    def __init__(self, name: str, outline: Sequence[Point]) -> None:
        verts = tuple((float(x), float(y)) for x, y in outline)
        if len(verts) < 3:
            raise ValueError("room outline needs at least 3 vertices")
        area = polygon_signed_area(verts)
        if area < 0.0:
            verts = verts[::-1]
        if not is_simple_polygon(verts):
            raise ValueError("room outline must be a simple (non-self-intersecting) polygon")
        self.name = name
        self.outline: tuple[Point, ...] = verts
        self.walls: tuple[Segment, ...] = tuple(
            (*verts[i], *verts[(i + 1) % len(verts)]) for i in range(len(verts))
        )
        self.wall_boxes: tuple[AABB, ...] = tuple(
            AABB(min(ax, bx), min(ay, by), max(ax, bx), max(ay, by)) for ax, ay, bx, by in self.walls
        )
        self.bounds = AABB.around(verts)
        self.floor_area = abs(area)
        self.dock: Dock | None = None
        self.version = 0
        self._obstacles: dict[int, Obstacle] = {}
        self._next_id = 1
        self._band_cache: dict[tuple[float, float], list[Obstacle]] = {}
        self._band_cache_version = -1

    # ------------------------------------------------------------------ queries

    @property
    def obstacles(self) -> list[Obstacle]:
        """All obstacles in id order (a new list; safe to iterate while editing)."""
        return list(self._obstacles.values())

    def obstacle(self, obstacle_id: int) -> Obstacle:
        return self._obstacles[obstacle_id]

    def obstacles_in_band(self, z0: float, z1: float) -> list[Obstacle]:
        """Obstacles whose vertical extent overlaps ``[z0, z1)`` (cached per version)."""
        if self._band_cache_version != self.version:
            self._band_cache.clear()
            self._band_cache_version = self.version
        key = (z0, z1)
        hits = self._band_cache.get(key)
        if hits is None:
            hits = [o for o in self._obstacles.values() if o.overlaps_band(z0, z1)]
            self._band_cache[key] = hits
        return hits

    def contains_point(self, x: float, y: float) -> bool:
        """True if (x, y) lies on the floor (inside the outline)."""
        return self.bounds.contains_point(x, y) and point_in_polygon(x, y, self.outline)

    def obstacles_at(self, x: float, y: float) -> list[Obstacle]:
        """Obstacles containing the point, topmost (highest ``z_max``) first."""
        hits = [o for o in self._obstacles.values() if o.aabb.contains_point(x, y) and o.shape.contains_point(x, y)]
        return sorted(hits, key=lambda o: o.z_max, reverse=True)

    def group_members(self, obstacle: Obstacle) -> list[Obstacle]:
        """All obstacles in the same furniture group (just ``obstacle`` if ungrouped)."""
        if obstacle.group is None:
            return [obstacle]
        return [o for o in self._obstacles.values() if o.group == obstacle.group]

    def is_shape_free(
        self,
        shape: Shape,
        z_min: float,
        z_max: float,
        *,
        ignore: Iterable[int] = (),
        tolerance: float = 1e-6,
    ) -> bool:
        """True if ``shape`` (spanning ``[z_min, z_max)``) is on the floor and overlaps nothing.

        Touching within ``tolerance`` is allowed, so furniture can sit flush
        against walls and pushed boxes can slide along them.
        """
        box = shape.aabb()
        if not self.bounds.contains_box(box):
            return False
        if not point_in_polygon(*shape.centroid, self.outline):
            return False
        for (ax, ay, bx, by), wall_box in zip(self.walls, self.wall_boxes, strict=True):
            if wall_box.overlaps(box) and shape_overlaps_segment(shape, ax, ay, bx, by, tolerance):
                return False
        skip = set(ignore)
        for other in self.obstacles_in_band(z_min, z_max):
            if other.id in skip or not other.aabb.overlaps(box):
                continue
            if shapes_overlap(shape, other.shape, tolerance):
                return False
        return True

    # ------------------------------------------------------------------ edits

    def add_obstacle(
        self,
        name: str,
        kind: ObstacleKind,
        shape: Shape,
        *,
        z_min: float = 0.0,
        z_max: float = 1.0,
        mobility: float = 0.0,
        group: str | None = None,
    ) -> Obstacle:
        """Add an obstacle and return it (with its new id)."""
        obstacle = Obstacle(self._next_id, name, kind, shape, z_min, z_max, mobility, group)
        self._obstacles[obstacle.id] = obstacle
        self._next_id += 1
        self.version += 1
        return obstacle

    def remove_obstacle(self, obstacle_id: int) -> Obstacle:
        """Remove and return an obstacle. The dock cannot be removed."""
        if self.dock is not None and obstacle_id == self.dock.obstacle_id:
            raise ValueError("the dock cannot be removed")
        obstacle = self._obstacles.pop(obstacle_id)
        self.version += 1
        return obstacle

    def move_obstacle(self, obstacle_id: int, shape: Shape) -> None:
        """Replace an obstacle's shape (used when it is pushed)."""
        obstacle = self._obstacles[obstacle_id]
        obstacle.shape = shape
        obstacle.aabb = shape.aabb()
        self.version += 1

    def set_dock(self, dock: Dock) -> None:
        if dock.obstacle_id not in self._obstacles:
            raise ValueError("dock obstacle must be added to the world first")
        self.dock = dock
        self.version += 1

    def __iter__(self) -> Iterator[Obstacle]:
        return iter(self.obstacles)
