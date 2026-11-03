"""Planar geometry primitives.

Conventions (see ``docs/ARCHITECTURE.md``, "Frames and units"):

* right-handed frames: x east/forward, y north/left;
* angles in radians, counter-clockwise positive, wrapped to [-pi, pi);
* polygons are stored counter-clockwise (CCW), so the interior is on the left
  of every edge and the outward normal of edge (a -> b) is its right normal.

Hot-path helpers take and return plain floats/tuples rather than small vector
objects: the physics loop calls them thousands of times per simulated second,
and allocation dominates the cost of tiny vector classes in CPython.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

Point = tuple[float, float]
"""A 2D point or vector ``(x, y)`` in metres."""

Segment = tuple[float, float, float, float]
"""A line segment ``(ax, ay, bx, by)``."""

TAU = 2.0 * math.pi


def wrap_angle(angle: float) -> float:
    """Wrap ``angle`` to the half-open interval [-pi, pi)."""
    return (angle + math.pi) % TAU - math.pi


def angle_difference(a: float, b: float) -> float:
    """Smallest signed rotation that turns heading ``b`` into heading ``a``."""
    return wrap_angle(a - b)


@dataclass(frozen=True, slots=True)
class Pose2D:
    """A rigid planar transform: position in metres, heading in radians.

    A pose can be read two ways: as *where a frame is* (the robot's pose in the
    world) or as *a transform* taking coordinates from that frame to its parent.
    ``theta`` is normalised to [-pi, pi) on construction.
    """

    x: float
    y: float
    theta: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "theta", wrap_angle(self.theta))

    @property
    def position(self) -> Point:
        """The ``(x, y)`` position."""
        return (self.x, self.y)

    def heading_vector(self) -> Point:
        """Unit vector pointing along the heading."""
        return (math.cos(self.theta), math.sin(self.theta))

    def transform_point(self, px: float, py: float) -> Point:
        """Map a point from this pose's local frame into the parent frame."""
        c, s = math.cos(self.theta), math.sin(self.theta)
        return (self.x + c * px - s * py, self.y + s * px + c * py)

    def inverse_transform_point(self, px: float, py: float) -> Point:
        """Map a point from the parent frame into this pose's local frame."""
        c, s = math.cos(self.theta), math.sin(self.theta)
        dx, dy = px - self.x, py - self.y
        return (c * dx + s * dy, -s * dx + c * dy)

    def compose(self, other: Pose2D) -> Pose2D:
        """Return ``self ⊕ other``: ``other`` is expressed in this pose's frame."""
        x, y = self.transform_point(other.x, other.y)
        return Pose2D(x, y, self.theta + other.theta)

    def inverse(self) -> Pose2D:
        """Return the inverse transform, so that ``p.compose(p.inverse())`` is identity."""
        c, s = math.cos(self.theta), math.sin(self.theta)
        return Pose2D(-c * self.x - s * self.y, s * self.x - c * self.y, -self.theta)

    def relative_to(self, base: Pose2D) -> Pose2D:
        """Express this pose in the frame of ``base``."""
        return base.inverse().compose(self)

    def distance_to(self, other: Pose2D) -> float:
        """Euclidean distance between the two positions (ignores heading)."""
        return math.hypot(other.x - self.x, other.y - self.y)


def transform_points(pose: Pose2D, points: np.ndarray) -> np.ndarray:
    """Map ``(N, 2)`` points from ``pose``'s local frame into its parent frame (vectorised)."""
    c, s = math.cos(pose.theta), math.sin(pose.theta)
    x, y = points[:, 0], points[:, 1]
    return np.column_stack((pose.x + c * x - s * y, pose.y + s * x + c * y))


def inverse_transform_points(pose: Pose2D, points: np.ndarray) -> np.ndarray:
    """Map ``(N, 2)`` points from the parent frame into ``pose``'s local frame (vectorised)."""
    c, s = math.cos(pose.theta), math.sin(pose.theta)
    dx, dy = points[:, 0] - pose.x, points[:, 1] - pose.y
    return np.column_stack((c * dx + s * dy, -s * dx + c * dy))


def wrap_angles(angles: np.ndarray) -> np.ndarray:
    """Vectorised :func:`wrap_angle`."""
    return (angles + math.pi) % TAU - math.pi


def points_in_polygon(px: np.ndarray, py: np.ndarray, vertices: Sequence[Point]) -> np.ndarray:
    """Vectorised even-odd test: which of the points (any matching shapes) lie inside the polygon."""
    inside = np.zeros(np.broadcast(px, py).shape, dtype=bool)
    n = len(vertices)
    j = n - 1
    for i in range(n):
        xi, yi = vertices[i]
        xj, yj = vertices[j]
        if yi != yj:
            straddles = (yi > py) != (yj > py)
            x_cross = xi + (py - yi) * (xj - xi) / (yj - yi)
            inside ^= straddles & (px < x_cross)
        j = i
    return inside


def cross(ax: float, ay: float, bx: float, by: float) -> float:
    """Z component of the 3D cross product of two planar vectors."""
    return ax * by - ay * bx


def closest_point_on_segment(
    px: float, py: float, ax: float, ay: float, bx: float, by: float
) -> tuple[float, float, float]:
    """Closest point to P on segment AB.

    Returns ``(qx, qy, t)`` where ``t`` in [0, 1] is the position of Q along AB.
    """
    abx, aby = bx - ax, by - ay
    denom = abx * abx + aby * aby
    if denom <= 0.0:
        return ax, ay, 0.0
    t = ((px - ax) * abx + (py - ay) * aby) / denom
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0
    return ax + t * abx, ay + t * aby, t


def point_segment_distance(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    """Euclidean distance from P to segment AB."""
    qx, qy, _ = closest_point_on_segment(px, py, ax, ay, bx, by)
    return math.hypot(px - qx, py - qy)


def polygon_signed_area(vertices: Sequence[Point]) -> float:
    """Shoelace formula; positive for counter-clockwise vertex order."""
    area = 0.0
    n = len(vertices)
    for i in range(n):
        x0, y0 = vertices[i]
        x1, y1 = vertices[(i + 1) % n]
        area += x0 * y1 - x1 * y0
    return 0.5 * area


def point_in_polygon(px: float, py: float, vertices: Sequence[Point]) -> bool:
    """Even-odd (crossing number) test; works for any simple polygon."""
    inside = False
    n = len(vertices)
    j = n - 1
    for i in range(n):
        xi, yi = vertices[i]
        xj, yj = vertices[j]
        if (yi > py) != (yj > py):
            x_cross = xi + (py - yi) * (xj - xi) / (yj - yi)
            if px < x_cross:
                inside = not inside
        j = i
    return inside


def _orientation(ax: float, ay: float, bx: float, by: float, cx: float, cy: float) -> float:
    return cross(bx - ax, by - ay, cx - ax, cy - ay)


def _on_segment(ax: float, ay: float, bx: float, by: float, px: float, py: float) -> bool:
    return min(ax, bx) <= px <= max(ax, bx) and min(ay, by) <= py <= max(ay, by)


def segments_intersect(a: Point, b: Point, c: Point, d: Point) -> bool:
    """True if closed segments AB and CD share at least one point."""
    d1 = _orientation(c[0], c[1], d[0], d[1], a[0], a[1])
    d2 = _orientation(c[0], c[1], d[0], d[1], b[0], b[1])
    d3 = _orientation(a[0], a[1], b[0], b[1], c[0], c[1])
    d4 = _orientation(a[0], a[1], b[0], b[1], d[0], d[1])
    if ((d1 > 0 > d2) or (d1 < 0 < d2)) and ((d3 > 0 > d4) or (d3 < 0 < d4)):
        return True
    if d1 == 0 and _on_segment(c[0], c[1], d[0], d[1], a[0], a[1]):
        return True
    if d2 == 0 and _on_segment(c[0], c[1], d[0], d[1], b[0], b[1]):
        return True
    if d3 == 0 and _on_segment(a[0], a[1], b[0], b[1], c[0], c[1]):
        return True
    return d4 == 0 and _on_segment(a[0], a[1], b[0], b[1], d[0], d[1])


def is_simple_polygon(vertices: Sequence[Point]) -> bool:
    """True if no two non-adjacent edges intersect (O(n^2), fine for room outlines)."""
    n = len(vertices)
    if n < 3:
        return False
    for i in range(n):
        a, b = vertices[i], vertices[(i + 1) % n]
        if a == b:
            return False
        for j in range(i + 1, n):
            if j == i or (j + 1) % n == i or j == (i + 1) % n:
                continue  # adjacent edges share a vertex by construction
            c, d = vertices[j], vertices[(j + 1) % n]
            if segments_intersect(a, b, c, d):
                return False
    return True


def is_convex(vertices: Sequence[Point]) -> bool:
    """True if the polygon is strictly convex (no collinear or repeated vertices)."""
    n = len(vertices)
    if n < 3:
        return False
    sign = 0.0
    for i in range(n):
        ax, ay = vertices[i]
        bx, by = vertices[(i + 1) % n]
        cx, cy = vertices[(i + 2) % n]
        z = cross(bx - ax, by - ay, cx - bx, cy - by)
        if abs(z) < 1e-12:
            return False
        if sign == 0.0:
            sign = z
        elif (z > 0) != (sign > 0):
            return False
    return True
