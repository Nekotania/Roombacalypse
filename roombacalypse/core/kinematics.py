"""Differential-drive kinematics: pure math, shared by simulator and robot software.

A differential-drive base has two independently driven wheels on a common
axle, ``wheel_base`` apart. With left/right wheel *surface* speeds v_l, v_r:

    v     = (v_r + v_l) / 2        forward speed of the axle midpoint   [m/s]
    omega = (v_r - v_l) / L        yaw rate, CCW positive                [rad/s]

If both wheel speeds are constant over an interval the robot traces a circular
arc (a straight line when v_l == v_r). :func:`arc_displacement` integrates that
motion exactly, so integration error does not grow with the timestep.

The derivation lives in ``docs/algorithms/differential-drive.md``. Living in
``core`` means the simulator (ground truth) and the robot's odometry (belief,
milestone 2) use one implementation instead of two that could drift apart.
"""

from __future__ import annotations

import math

import numpy as np

from roombacalypse.core.geometry import Pose2D

_SERIES_THRESHOLD = 1e-4
"""Below this turn angle [rad] we use Taylor series to avoid 0/0."""


def twist_to_wheel_speeds(linear: float, angular: float, wheel_base: float) -> tuple[float, float]:
    """Convert a body twist (v, omega) into left/right wheel surface speeds."""
    half = 0.5 * angular * wheel_base
    return linear - half, linear + half


def wheel_speeds_to_twist(v_left: float, v_right: float, wheel_base: float) -> tuple[float, float]:
    """Convert left/right wheel surface speeds into a body twist (v, omega)."""
    return 0.5 * (v_left + v_right), (v_right - v_left) / wheel_base


def arc_displacement(distance: float, dtheta: float) -> tuple[float, float]:
    """Body-frame displacement after moving ``distance`` along an arc turning by ``dtheta``.

    For a constant-curvature path of length ``d`` that changes the heading by
    ``dθ`` the displacement, in the frame of the starting pose, is::

        dx = d * sin(dθ) / dθ
        dy = d * (1 - cos(dθ)) / dθ

    which tends to ``(d, 0)`` as ``dθ -> 0``. Near zero we use the Taylor
    expansions ``sin x / x ≈ 1 - x²/6`` and ``(1 - cos x)/x ≈ x/2 - x³/24``;
    the truncation error is below 1e-17 relative at the switch-over point.
    """
    if abs(dtheta) < _SERIES_THRESHOLD:
        t2 = dtheta * dtheta
        return distance * (1.0 - t2 / 6.0), distance * (0.5 * dtheta - dtheta * t2 / 24.0)
    return (
        distance * math.sin(dtheta) / dtheta,
        distance * (1.0 - math.cos(dtheta)) / dtheta,
    )


def arc_displacement_array(distance: np.ndarray, dtheta: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Vectorised :func:`arc_displacement` (same formula and series branch; tested to agree)."""
    small = np.abs(dtheta) < _SERIES_THRESHOLD
    safe = np.where(small, 1.0, dtheta)
    t2 = dtheta * dtheta
    dx = np.where(small, distance * (1.0 - t2 / 6.0), distance * np.sin(dtheta) / safe)
    dy = np.where(small, distance * (0.5 * dtheta - dtheta * t2 / 24.0), distance * (1.0 - np.cos(dtheta)) / safe)
    return dx, dy


def integrate_wheel_travel(pose: Pose2D, d_left: float, d_right: float, wheel_base: float) -> Pose2D:
    """Advance ``pose`` by wheel travel distances, assuming constant curvature.

    ``d_left``/``d_right`` are the distances each wheel's contact point rolled.
    This is the exact solution whenever the wheel-speed *ratio* is constant over
    the interval, which includes constant speeds and proportional ramps.
    """
    distance = 0.5 * (d_left + d_right)
    dtheta = (d_right - d_left) / wheel_base
    dx, dy = arc_displacement(distance, dtheta)
    x, y = pose.transform_point(dx, dy)
    return Pose2D(x, y, pose.theta + dtheta)


def integrate_twist(pose: Pose2D, linear: float, angular: float, dt: float) -> Pose2D:
    """Advance ``pose`` by a constant twist held for ``dt`` seconds (exact)."""
    dx, dy = arc_displacement(linear * dt, angular * dt)
    x, y = pose.transform_point(dx, dy)
    return Pose2D(x, y, pose.theta + angular * dt)
