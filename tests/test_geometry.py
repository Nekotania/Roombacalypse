"""Planar geometry primitives."""

from __future__ import annotations

import math

import numpy as np
import pytest

from roombacalypse.core.geometry import (
    Pose2D,
    angle_difference,
    closest_point_on_segment,
    is_convex,
    is_simple_polygon,
    point_in_polygon,
    point_segment_distance,
    polygon_signed_area,
    wrap_angle,
)

L_SHAPE = [(0.0, 0.0), (8.0, 0.0), (8.0, 3.6), (5.4, 3.6), (5.4, 6.0), (0.0, 6.0)]


def test_wrap_angle_range_and_equivalence() -> None:
    for angle in np.linspace(-25.0, 25.0, 2001):
        wrapped = wrap_angle(float(angle))
        assert -math.pi <= wrapped < math.pi
        assert math.isclose(math.cos(wrapped), math.cos(angle), abs_tol=1e-12)
        assert math.isclose(math.sin(wrapped), math.sin(angle), abs_tol=1e-12)


def test_wrap_angle_boundary() -> None:
    assert wrap_angle(math.pi) == -math.pi
    assert wrap_angle(-math.pi) == -math.pi


def test_angle_difference_takes_short_way_round() -> None:
    assert math.isclose(angle_difference(math.radians(170), math.radians(-170)), math.radians(-20))
    assert math.isclose(angle_difference(math.radians(-170), math.radians(170)), math.radians(20))


def _random_poses(count: int) -> list[Pose2D]:
    rng = np.random.default_rng(1)
    return [Pose2D(*rng.uniform(-5, 5, 2), rng.uniform(-math.pi, math.pi)) for _ in range(count)]


def _assert_pose_close(a: Pose2D, b: Pose2D, tol: float = 1e-9) -> None:
    assert math.isclose(a.x, b.x, abs_tol=tol)
    assert math.isclose(a.y, b.y, abs_tol=tol)
    assert abs(angle_difference(a.theta, b.theta)) < tol


def test_pose_normalises_heading() -> None:
    assert math.isclose(Pose2D(0, 0, 3 * math.pi).theta, -math.pi)


def test_compose_with_inverse_is_identity() -> None:
    identity = Pose2D(0.0, 0.0, 0.0)
    for pose in _random_poses(50):
        _assert_pose_close(pose.compose(pose.inverse()), identity)
        _assert_pose_close(pose.inverse().compose(pose), identity)


def test_transform_point_round_trip() -> None:
    for pose in _random_poses(50):
        x, y = pose.transform_point(0.3, -1.2)
        lx, ly = pose.inverse_transform_point(x, y)
        assert math.isclose(lx, 0.3, abs_tol=1e-9) and math.isclose(ly, -1.2, abs_tol=1e-9)


def test_transform_point_known_values() -> None:
    pose = Pose2D(1.0, 2.0, math.pi / 2)
    x, y = pose.transform_point(1.0, 0.0)  # one metre "forward" while facing +y
    assert math.isclose(x, 1.0, abs_tol=1e-12) and math.isclose(y, 3.0)


def test_relative_to_recovers_pose() -> None:
    poses = _random_poses(20)
    for a, b in zip(poses, reversed(poses), strict=True):
        _assert_pose_close(a.compose(b.relative_to(a)), b)


def test_closest_point_on_segment_interior_and_clamped() -> None:
    assert closest_point_on_segment(1.0, 5.0, 0.0, 0.0, 2.0, 0.0) == (1.0, 0.0, 0.5)
    assert closest_point_on_segment(-3.0, 1.0, 0.0, 0.0, 2.0, 0.0) == (0.0, 0.0, 0.0)
    assert closest_point_on_segment(9.0, -1.0, 0.0, 0.0, 2.0, 0.0) == (2.0, 0.0, 1.0)
    assert closest_point_on_segment(1.0, 1.0, 0.5, 0.5, 0.5, 0.5) == (0.5, 0.5, 0.0)  # degenerate
    assert math.isclose(point_segment_distance(3.0, 4.0, 0.0, 0.0, 0.0, 0.0), 5.0)


def test_polygon_orientation() -> None:
    square = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
    assert polygon_signed_area(square) == 1.0
    assert polygon_signed_area(square[::-1]) == -1.0
    assert math.isclose(polygon_signed_area(L_SHAPE), 48.0 - 2.6 * 2.4)


def test_point_in_concave_polygon() -> None:
    assert point_in_polygon(1.0, 1.0, L_SHAPE)
    assert point_in_polygon(7.0, 1.0, L_SHAPE)
    assert point_in_polygon(1.0, 5.0, L_SHAPE)
    assert not point_in_polygon(7.0, 5.0, L_SHAPE)  # the notch
    assert not point_in_polygon(-0.1, 1.0, L_SHAPE)


def test_simple_polygon_detection() -> None:
    assert is_simple_polygon(L_SHAPE)
    bow_tie = [(0.0, 0.0), (1.0, 1.0), (1.0, 0.0), (0.0, 1.0)]
    assert not is_simple_polygon(bow_tie)


def test_convexity() -> None:
    assert is_convex([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)])
    assert not is_convex(L_SHAPE)
    assert not is_convex([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (1.0, 1.0)])  # collinear


@pytest.mark.parametrize("theta", [0.0, 1.0, -2.5])
def test_heading_vector_is_unit(theta: float) -> None:
    hx, hy = Pose2D(0, 0, theta).heading_vector()
    assert math.isclose(math.hypot(hx, hy), 1.0)
