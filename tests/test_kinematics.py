"""Differential-drive kinematics and the drivetrain model."""

from __future__ import annotations

import math

import pytest

from roombacalypse.core.config import RobotSpec
from roombacalypse.core.geometry import Pose2D, angle_difference
from roombacalypse.core.kinematics import (
    arc_displacement,
    integrate_twist,
    integrate_wheel_travel,
    twist_to_wheel_speeds,
    wheel_speeds_to_twist,
)
from roombacalypse.core.messages import DriveCommand
from roombacalypse.robot.drivetrain import Drivetrain
from tests.helpers import make_sim

SPEC = RobotSpec()
ORIGIN = Pose2D(0.0, 0.0, 0.0)


def _close(a: Pose2D, b: Pose2D, tol: float = 1e-9) -> bool:
    return (
        math.isclose(a.x, b.x, abs_tol=tol)
        and math.isclose(a.y, b.y, abs_tol=tol)
        and abs(angle_difference(a.theta, b.theta)) < tol
    )


@pytest.mark.parametrize(("v", "w"), [(0.3, 0.0), (0.0, 1.5), (-0.2, 0.7), (0.5, -4.0)])
def test_twist_wheel_round_trip(v: float, w: float) -> None:
    v_left, v_right = twist_to_wheel_speeds(v, w, SPEC.wheel_base)
    assert wheel_speeds_to_twist(v_left, v_right, SPEC.wheel_base) == pytest.approx((v, w))


def test_straight_line() -> None:
    assert _close(integrate_twist(ORIGIN, 0.5, 0.0, 2.0), Pose2D(1.0, 0.0, 0.0))


def test_rotation_in_place() -> None:
    pose = Pose2D(1.0, 2.0, 0.3)
    assert _close(integrate_twist(pose, 0.0, 1.0, math.pi / 2), Pose2D(1.0, 2.0, 0.3 + math.pi / 2))


def test_quarter_circle_has_radius_v_over_omega() -> None:
    v, w = 0.2, 0.5  # radius 0.4 m, turning left
    end = integrate_twist(ORIGIN, v, w, (math.pi / 2) / w)
    assert _close(end, Pose2D(0.4, 0.4, math.pi / 2))


def test_full_circle_closes_after_many_steps() -> None:
    steps, dt = 500, 0.02
    w = 2.0 * math.pi / (steps * dt)
    pose = Pose2D(1.0, -1.0, 0.7)
    end = pose
    for _ in range(steps):
        end = integrate_twist(end, 0.3, w, dt)
    assert _close(end, pose, tol=1e-9)


def test_exact_integration_is_step_size_independent() -> None:
    one_step = integrate_twist(ORIGIN, 0.35, 1.1, 2.0)
    many = ORIGIN
    for _ in range(1000):
        many = integrate_twist(many, 0.35, 1.1, 0.002)
    assert _close(one_step, many, tol=1e-9)


def test_series_branch_is_continuous() -> None:
    """The Taylor branch and the closed form agree where they meet."""
    eps = 1e-12
    below = arc_displacement(1.0, 1e-4 - eps)
    above = arc_displacement(1.0, 1e-4 + eps)
    assert below[0] == pytest.approx(above[0], rel=1e-12)
    assert below[1] == pytest.approx(above[1], rel=1e-6)  # closed form has ~1e-8 cancellation error here
    assert arc_displacement(1.0, 0.0) == (1.0, 0.0)


def test_wheel_travel_matches_twist() -> None:
    v, w, dt = 0.25, -0.8, 0.5
    v_left, v_right = twist_to_wheel_speeds(v, w, SPEC.wheel_base)
    by_wheels = integrate_wheel_travel(ORIGIN, v_left * dt, v_right * dt, SPEC.wheel_base)
    assert _close(by_wheels, integrate_twist(ORIGIN, v, w, dt))


# --------------------------------------------------------------------------- drivetrain


def test_saturation_preserves_curvature() -> None:
    drivetrain = Drivetrain(SPEC)
    command = DriveCommand(0.5, 2.0)
    v_left, v_right = drivetrain.target_wheel_speeds(command)
    assert max(abs(v_left), abs(v_right)) == pytest.approx(SPEC.max_wheel_speed)
    v, w = wheel_speeds_to_twist(v_left, v_right, SPEC.wheel_base)
    assert w / v == pytest.approx(command.angular / command.linear)


def test_no_saturation_within_limits() -> None:
    target = Drivetrain(SPEC).target_wheel_speeds(DriveCommand(0.2, 0.5))
    assert target == pytest.approx(twist_to_wheel_speeds(0.2, 0.5, SPEC.wheel_base))


def test_spin_in_place_saturates_symmetrically() -> None:
    assert Drivetrain(SPEC).target_wheel_speeds(DriveCommand(0.0, 50.0)) == pytest.approx((-0.5, 0.5))


def test_acceleration_limit_ramp_time() -> None:
    """0 -> 0.5 m/s at 1 m/s² takes 0.5 s, i.e. 25 steps of 20 ms."""
    drivetrain = Drivetrain(SPEC)
    speeds = (0.0, 0.0)
    for _ in range(24):
        speeds = drivetrain.step(speeds, (0.5, 0.5), 0.02)
    assert speeds == pytest.approx((0.48, 0.48))
    speeds = drivetrain.step(speeds, (0.5, 0.5), 0.02)
    assert speeds == pytest.approx((0.5, 0.5), abs=1e-12)


def test_acceleration_limit_scales_wheels_jointly() -> None:
    v_left, v_right = Drivetrain(SPEC).step((0.0, 0.0), (0.2, 0.5), 0.02)
    assert v_right == pytest.approx(SPEC.max_wheel_accel * 0.02)
    assert v_left / v_right == pytest.approx(0.2 / 0.5)


def test_engine_ramp_distance_is_exact() -> None:
    """From rest at 1 m/s², the robot covers a*t²/2 = 0.125 m while reaching 0.5 m/s."""
    sim, _ = make_sim(segments=[(10.0, DriveCommand(0.5, 0.0))], start=Pose2D(1.0, 1.5, 0.0))
    for _ in range(25):
        sim.step()
    assert sim.robot.v_left == pytest.approx(0.5)
    assert sim.robot.pose.x - 1.0 == pytest.approx(0.125, abs=1e-12)
