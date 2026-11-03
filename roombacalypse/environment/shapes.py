"""Collision shapes (circles and convex polygons) and the exact tests between them.

The robot is a circle, so the contact queries used every physics step are all
"circle versus X" and return a *signed separation*: the gap between the two
surfaces, negative when they overlap. Obstacle-versus-obstacle queries (used
when a pushed obstacle moves, or when the user places one) are boolean overlap
tests based on the separating axis theorem (SAT).

Non-convex furniture is modelled as several convex parts.
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Iterator
from dataclasses import dataclass, field

import numpy as np

from roombacalypse.core.geometry import (
    Point,
    closest_point_on_segment,
    is_convex,
    polygon_signed_area,
)

_EPS = 1e-12


@dataclass(frozen=True, slots=True)
class AABB:
    """Axis-aligned bounding box, used as a cheap broad-phase filter."""

    min_x: float
    min_y: float
    max_x: float
    max_y: float

    @staticmethod
    def around(points: Iterable[Point]) -> AABB:
        """Smallest box containing ``points``."""
        xs, ys = zip(*points, strict=True)
        return AABB(min(xs), min(ys), max(xs), max(ys))

    @property
    def width(self) -> float:
        return self.max_x - self.min_x

    @property
    def height(self) -> float:
        return self.max_y - self.min_y

    def overlaps(self, other: AABB) -> bool:
        """True if the boxes intersect (touching counts)."""
        return not (
            other.min_x > self.max_x
            or other.max_x < self.min_x
            or other.min_y > self.max_y
            or other.max_y < self.min_y
        )

    def contains_point(self, x: float, y: float) -> bool:
        return self.min_x <= x <= self.max_x and self.min_y <= y <= self.max_y

    def contains_box(self, other: AABB) -> bool:
        return (
            self.min_x <= other.min_x
            and self.min_y <= other.min_y
            and other.max_x <= self.max_x
            and other.max_y <= self.max_y
        )

    def expanded(self, margin: float) -> AABB:
        return AABB(self.min_x - margin, self.min_y - margin, self.max_x + margin, self.max_y + margin)


@dataclass(frozen=True, slots=True)
class Circle:
    """A disc: chair legs, pillars, plant pots, the robot itself."""

    cx: float
    cy: float
    radius: float

    def __post_init__(self) -> None:
        if not self.radius > 0.0:
            raise ValueError(f"circle radius must be positive, got {self.radius}")

    @property
    def center(self) -> Point:
        return (self.cx, self.cy)

    @property
    def centroid(self) -> Point:
        return (self.cx, self.cy)

    def aabb(self) -> AABB:
        r = self.radius
        return AABB(self.cx - r, self.cy - r, self.cx + r, self.cy + r)

    def translated(self, dx: float, dy: float) -> Circle:
        return Circle(self.cx + dx, self.cy + dy, self.radius)

    def contains_point(self, px: float, py: float) -> bool:
        return math.hypot(px - self.cx, py - self.cy) <= self.radius

    def contains_points(self, px: np.ndarray, py: np.ndarray) -> np.ndarray:
        """Vectorised :meth:`contains_point`."""
        return (px - self.cx) ** 2 + (py - self.cy) ** 2 <= self.radius**2


@dataclass(frozen=True, slots=True)
class ConvexPolygon:
    """A convex polygon with vertices stored counter-clockwise.

    Vertices given clockwise are reversed; non-convex or degenerate input
    raises ``ValueError``.
    """

    vertices: tuple[Point, ...]
    _normals: tuple[Point, ...] = field(init=False, repr=False, compare=False)
    _aabb: AABB = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        verts = tuple((float(x), float(y)) for x, y in self.vertices)
        if len(verts) < 3:
            raise ValueError("a polygon needs at least 3 vertices")
        area = polygon_signed_area(verts)
        if abs(area) < _EPS:
            raise ValueError("polygon has zero area")
        if area < 0.0:
            verts = verts[::-1]
        if not is_convex(verts):
            raise ValueError("polygon must be strictly convex (split concave shapes into parts)")
        normals = []
        n = len(verts)
        for i in range(n):
            ax, ay = verts[i]
            bx, by = verts[(i + 1) % n]
            ex, ey = bx - ax, by - ay
            length = math.hypot(ex, ey)
            normals.append((ey / length, -ex / length))  # right normal = outward for CCW
        object.__setattr__(self, "vertices", verts)
        object.__setattr__(self, "_normals", tuple(normals))
        object.__setattr__(self, "_aabb", AABB.around(verts))

    @classmethod
    def box(cls, cx: float, cy: float, size_x: float, size_y: float, angle: float = 0.0) -> ConvexPolygon:
        """Rectangle centred at (cx, cy) with local extents ``size_x`` × ``size_y``, rotated by ``angle``."""
        if size_x <= 0.0 or size_y <= 0.0:
            raise ValueError(f"box size must be positive, got {size_x} x {size_y}")
        hx, hy = 0.5 * size_x, 0.5 * size_y
        c, s = math.cos(angle), math.sin(angle)
        corners = ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))
        return cls(tuple((cx + c * x - s * y, cy + s * x + c * y) for x, y in corners))

    @property
    def normals(self) -> tuple[Point, ...]:
        """Outward unit normal of each edge (edge i runs from vertex i to i+1)."""
        return self._normals

    @property
    def centroid(self) -> Point:
        """Vertex average (inside the polygon because it is convex)."""
        n = len(self.vertices)
        return (sum(v[0] for v in self.vertices) / n, sum(v[1] for v in self.vertices) / n)

    def aabb(self) -> AABB:
        return self._aabb

    def translated(self, dx: float, dy: float) -> ConvexPolygon:
        return ConvexPolygon(tuple((x + dx, y + dy) for x, y in self.vertices))

    def edges(self) -> Iterator[tuple[float, float, float, float]]:
        """Yield edges as ``(ax, ay, bx, by)`` in CCW order."""
        verts = self.vertices
        n = len(verts)
        for i in range(n):
            ax, ay = verts[i]
            bx, by = verts[(i + 1) % n]
            yield ax, ay, bx, by

    def contains_point(self, px: float, py: float) -> bool:
        """True if P is inside or on the boundary."""
        for (ax, ay), (nx, ny) in zip(self.vertices, self._normals, strict=True):
            if (px - ax) * nx + (py - ay) * ny > 1e-12:
                return False
        return True

    def contains_points(self, px: np.ndarray, py: np.ndarray) -> np.ndarray:
        """Vectorised :meth:`contains_point`."""
        inside = np.ones(np.broadcast(px, py).shape, dtype=bool)
        for (ax, ay), (nx, ny) in zip(self.vertices, self._normals, strict=True):
            inside &= (px - ax) * nx + (py - ay) * ny <= 1e-12
        return inside

    def closest_boundary_point(self, px: float, py: float) -> tuple[float, float, int]:
        """Closest point on the boundary to P, and the index of its edge."""
        best = (0.0, 0.0, 0)
        best_d2 = math.inf
        for i, (ax, ay, bx, by) in enumerate(self.edges()):
            qx, qy, _ = closest_point_on_segment(px, py, ax, ay, bx, by)
            d2 = (px - qx) ** 2 + (py - qy) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best = (qx, qy, i)
        return best


Shape = Circle | ConvexPolygon
"""Any obstacle shape."""


# --------------------------------------------------------------------------- circle queries
# Each returns (nx, ny, separation, qx, qy):
#   (nx, ny)    unit normal pointing from the other shape towards the circle centre,
#               i.e. the direction in which the circle must move to separate;
#   separation  surface gap, negative when overlapping (-separation = penetration depth);
#   (qx, qy)    contact point on the other shape's surface.

ContactGeometry = tuple[float, float, float, float, float]


def circle_vs_segment(
    cx: float, cy: float, r: float, ax: float, ay: float, bx: float, by: float
) -> ContactGeometry:
    """Signed separation between a circle and a (zero-thickness) segment."""
    qx, qy, _ = closest_point_on_segment(cx, cy, ax, ay, bx, by)
    dx, dy = cx - qx, cy - qy
    d = math.hypot(dx, dy)
    if d < _EPS:
        # Centre exactly on the segment: fall back to its left normal, which is
        # the room interior for CCW outlines.
        ex, ey = bx - ax, by - ay
        length = math.hypot(ex, ey) or 1.0
        return -ey / length, ex / length, -r, qx, qy
    return dx / d, dy / d, d - r, qx, qy


def circle_vs_circle(cx: float, cy: float, r: float, other: Circle) -> ContactGeometry:
    """Signed separation between two circles."""
    dx, dy = cx - other.cx, cy - other.cy
    d = math.hypot(dx, dy)
    nx, ny = (dx / d, dy / d) if d > _EPS else (1.0, 0.0)
    return nx, ny, d - r - other.radius, other.cx + nx * other.radius, other.cy + ny * other.radius


def circle_vs_polygon(cx: float, cy: float, r: float, poly: ConvexPolygon) -> ContactGeometry:
    """Signed separation between a circle and a convex polygon (handles a centre inside)."""
    qx, qy, edge = poly.closest_boundary_point(cx, cy)
    dx, dy = cx - qx, cy - qy
    d = math.hypot(dx, dy)
    if d < _EPS:
        nx, ny = poly.normals[edge]
        return nx, ny, -r, qx, qy
    if poly.contains_point(cx, cy):
        # Centre inside: escape through the nearest edge point.
        return -dx / d, -dy / d, -d - r, qx, qy
    return dx / d, dy / d, d - r, qx, qy


def circle_vs_shape(cx: float, cy: float, r: float, shape: Shape) -> ContactGeometry:
    """Dispatch to the right circle query for ``shape``."""
    if isinstance(shape, Circle):
        return circle_vs_circle(cx, cy, r, shape)
    return circle_vs_polygon(cx, cy, r, shape)


# --------------------------------------------------------------------------- overlap tests


def _project(vertices: Iterable[Point], nx: float, ny: float) -> tuple[float, float]:
    lo = math.inf
    hi = -math.inf
    for x, y in vertices:
        d = x * nx + y * ny
        lo = min(lo, d)
        hi = max(hi, d)
    return lo, hi


def _separated(a: tuple[Point, ...], b: tuple[Point, ...], axes: Iterable[Point], tolerance: float) -> bool:
    for nx, ny in axes:
        amin, amax = _project(a, nx, ny)
        bmin, bmax = _project(b, nx, ny)
        if amax - bmin <= tolerance or bmax - amin <= tolerance:
            return True
    return False


def shapes_overlap(a: Shape, b: Shape, tolerance: float = 1e-9) -> bool:
    """True if the interiors overlap by more than ``tolerance`` (touching is not overlap)."""
    if not a.aabb().overlaps(b.aabb()):
        return False
    if isinstance(a, Circle):
        return circle_vs_shape(a.cx, a.cy, a.radius, b)[2] < -tolerance
    if isinstance(b, Circle):
        return circle_vs_polygon(b.cx, b.cy, b.radius, a)[2] < -tolerance
    return not _separated(a.vertices, b.vertices, a.normals + b.normals, tolerance)


def shape_overlaps_segment(
    shape: Shape, ax: float, ay: float, bx: float, by: float, tolerance: float = 1e-9
) -> bool:
    """True if ``shape`` crosses segment AB by more than ``tolerance``."""
    if isinstance(shape, Circle):
        return circle_vs_segment(shape.cx, shape.cy, shape.radius, ax, ay, bx, by)[2] < -tolerance
    ex, ey = bx - ax, by - ay
    length = math.hypot(ex, ey)
    if length < _EPS:
        return shape.contains_point(ax, ay)
    segment_axis = (-ey / length, ex / length)
    return not _separated(shape.vertices, ((ax, ay), (bx, by)), (*shape.normals, segment_axis), tolerance)
