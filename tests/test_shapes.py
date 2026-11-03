"""Collision shapes and exact overlap/separation queries."""

from __future__ import annotations

import math

import pytest

from roombacalypse.environment.shapes import (
    AABB,
    Circle,
    ConvexPolygon,
    circle_vs_circle,
    circle_vs_polygon,
    circle_vs_segment,
    shape_overlaps_segment,
    shapes_overlap,
)


def test_box_is_ccw_with_correct_aabb() -> None:
    box = ConvexPolygon.box(1.0, 2.0, 2.0, 1.0)
    assert box.aabb() == AABB(0.0, 1.5, 2.0, 2.5)
    rotated = ConvexPolygon.box(0.0, 0.0, 2.0, 2.0, math.pi / 4)
    half_diagonal = math.sqrt(2.0)
    assert math.isclose(rotated.aabb().max_x, half_diagonal)


def test_clockwise_input_is_reversed() -> None:
    poly = ConvexPolygon(((0.0, 0.0), (0.0, 1.0), (1.0, 1.0), (1.0, 0.0)))
    assert poly.vertices[0] == (1.0, 0.0)  # reversed to CCW
    assert poly.contains_point(0.5, 0.5)


def test_rejects_bad_polygons() -> None:
    with pytest.raises(ValueError):
        ConvexPolygon(((0.0, 0.0), (2.0, 0.0), (1.0, 0.5), (2.0, 2.0), (0.0, 2.0)))  # concave
    with pytest.raises(ValueError):
        ConvexPolygon(((0.0, 0.0), (1.0, 0.0), (2.0, 0.0)))  # zero area
    with pytest.raises(ValueError):
        Circle(0.0, 0.0, 0.0)


def test_outward_normals() -> None:
    box = ConvexPolygon.box(0.0, 0.0, 2.0, 2.0)
    assert box.normals == ((0.0, -1.0), (1.0, 0.0), (0.0, 1.0), (-1.0, 0.0))


def test_circle_vs_segment() -> None:
    nx, ny, separation, qx, qy = circle_vs_segment(1.0, 0.5, 0.2, 0.0, 0.0, 2.0, 0.0)
    assert (nx, ny) == (0.0, 1.0)
    assert math.isclose(separation, 0.3)
    assert (qx, qy) == (1.0, 0.0)
    # Beyond the end point the closest feature is the vertex.
    nx, ny, separation, _, _ = circle_vs_segment(3.0, 0.0, 0.5, 0.0, 0.0, 2.0, 0.0)
    assert (nx, ny) == (1.0, 0.0) and math.isclose(separation, 0.5)


def test_circle_vs_circle() -> None:
    nx, ny, separation, qx, qy = circle_vs_circle(3.0, 0.0, 1.0, Circle(0.0, 0.0, 1.5))
    assert (nx, ny) == (1.0, 0.0)
    assert math.isclose(separation, 0.5)
    assert (qx, qy) == (1.5, 0.0)


def test_circle_vs_polygon_outside_and_inside() -> None:
    box = ConvexPolygon.box(0.0, 0.0, 2.0, 2.0)
    nx, ny, separation, _, _ = circle_vs_polygon(1.5, 0.0, 0.2, box)
    assert (nx, ny) == (1.0, 0.0) and math.isclose(separation, 0.3)
    # Centre inside, 0.1 m from the right edge: escape to the right by 0.1 + r.
    nx, ny, separation, _, _ = circle_vs_polygon(0.9, 0.0, 0.2, box)
    assert math.isclose(nx, 1.0) and math.isclose(separation, -0.3)
    # Near a corner the normal points diagonally.
    nx, ny, separation, _, _ = circle_vs_polygon(2.0, 2.0, 0.1, box)
    assert math.isclose(nx, ny) and math.isclose(separation, math.sqrt(2.0) - 0.1)


def test_polygon_overlap_sat() -> None:
    a = ConvexPolygon.box(0.0, 0.0, 2.0, 2.0)
    assert shapes_overlap(a, ConvexPolygon.box(1.5, 0.0, 2.0, 2.0))
    assert not shapes_overlap(a, ConvexPolygon.box(3.5, 0.0, 2.0, 2.0))
    assert not shapes_overlap(a, ConvexPolygon.box(2.0, 0.0, 2.0, 2.0))  # touching is not overlapping
    # AABBs overlap but a rotated diamond misses the corner: only SAT gets this right.
    diamond = ConvexPolygon.box(1.5, 1.5, 1.0, 1.0, math.pi / 4)
    assert diamond.aabb().overlaps(a.aabb())
    assert not shapes_overlap(a, diamond)


def test_circle_polygon_overlap_both_orders() -> None:
    box = ConvexPolygon.box(0.0, 0.0, 2.0, 2.0)
    assert shapes_overlap(box, Circle(1.1, 0.0, 0.2))
    assert shapes_overlap(Circle(1.1, 0.0, 0.2), box)
    assert not shapes_overlap(Circle(1.3, 0.0, 0.2), box)
    assert shapes_overlap(Circle(0.0, 0.0, 0.5), Circle(0.9, 0.0, 0.5))


def test_shape_overlaps_segment() -> None:
    box = ConvexPolygon.box(0.0, 0.0, 1.0, 1.0)
    assert shape_overlaps_segment(box, -2.0, 0.0, 2.0, 0.0)
    assert not shape_overlaps_segment(box, -2.0, 0.5, 2.0, 0.5)  # flush against the top edge
    assert not shape_overlaps_segment(box, 0.0, 2.0, 1.0, 3.0)
    assert shape_overlaps_segment(Circle(0.0, 0.1, 0.2), -1.0, 0.0, 1.0, 0.0)
