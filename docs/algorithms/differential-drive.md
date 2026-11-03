# Differential-drive kinematics and drivetrain

Code: `roombacalypse/core/kinematics.py` (math, shared by simulator and robot
software) and `roombacalypse/robot/drivetrain.py` (motor limits, truth side).
Tests: `tests/test_kinematics.py`.

## Model

Two driven wheels on a common axle, `L` apart (the wheel base), with surface
speeds `v_l` and `v_r`. Assuming the wheels roll without slipping, the axle
midpoint moves with

```
v = (v_r + v_l) / 2        forward speed        [m/s]
ω = (v_r − v_l) / L        yaw rate, CCW +      [rad/s]
```

and inversely `v_l = v − ωL/2`, `v_r = v + ωL/2`. The robot turns about the
instantaneous centre of curvature, a distance `R = v/ω` to its left. Equal
wheel speeds drive straight; opposite speeds spin in place.

## Exact integration

If both wheel speeds are constant over a step `Δt`, the robot follows a
circular arc of length `d = vΔt` that turns its heading by `Δθ = ωΔt`.
Integrating `ẋ = v cos θ(t)`, `ẏ = v sin θ(t)` with `θ(t) = θ₀ + ωt` gives the
displacement in the frame of the starting pose:

```
Δx = d · sin(Δθ) / Δθ
Δy = d · (1 − cos Δθ) / Δθ
```

which tends to `(d, 0)` as `Δθ → 0`. Below `|Δθ| = 1e-4` the code switches to
the Taylor series `sin x / x ≈ 1 − x²/6` and `(1 − cos x)/x ≈ x/2 − x³/24`,
which avoids `0/0`. Its truncation error there is below 1e-17.

**Why not Euler integration?** Euler (`x += v cos θ Δt`) drifts outward on every
curve, and the error grows with the step size. The arc formula has no
integration error. Its only assumption concerns how the wheel speeds behave
*within* a step. The test `test_exact_integration_is_step_size_independent`
checks that one 2-second step and 1000 steps of 2 ms land in the same place
(to 1e-9 m).

## Within a step: linear speed ramps

The acceleration limit makes wheel speeds change *linearly* during a step, so
each wheel's rolling distance is exactly the trapezoid `½(v₀ + v₁)Δt`. The
engine integrates those two distances (`integrate_wheel_travel`). The
drivetrain scales both wheels' speed changes jointly (see below), so the ratio
between the wheel speeds, and with it the curvature, stays constant during a
ramp from rest, and the arc is still exact. Test: from rest at 1 m/s², the
robot covers exactly `a t²/2 = 0.125 m` in the 0.5 s it takes to reach
0.5 m/s (`test_engine_ramp_distance_is_exact`).

## Speed saturation: preserve curvature

A requested twist may ask a wheel to exceed `max_wheel_speed`. Two choices:

- **clip each wheel** independently: the robot keeps as much speed as possible
  but drives a *different arc* than requested. For a planner that is a silent bug;
- **scale both wheels by the same factor** (implemented): the curvature
  `κ = ω/v`, and so the path, is preserved, and only the speed drops.

## Acceleration limit: scale jointly

Each control period the wheel speeds move toward their targets by at most
`max_wheel_accel · Δt`. If one wheel's change is limited, the other wheel's
change is scaled by the same factor, so both arrive together. Limiting each
wheel independently would bend the path at every speed change: the wheel with
the smaller change would finish its ramp first.

## Approximations and limitations

- **No wheel slip or calibration error yet.** Milestone 2 adds wheel radius
  mismatch, wheel base error and random slip, on the truth side only, so that
  odometry has realistic, *unknown* errors.
- **No motor dynamics:** no time constant, torque limit or load dependence.
  Pushing a heavy box does not slow the wheels; it slows the *body*, through
  the contact solver.
- **Blocked wheels keep turning (they slip).** When the body is stopped by a
  wall, the wheels still roll, so the encoder ground truth
  (`RobotBody.travel_left/right`) keeps growing while the odometer
  (distance over the floor) does not. Real robots behave the same way, and
  localization (Milestone 3) has to cope.
- Casters, weight distribution and the vertical dimension are ignored.

## Default parameters

Roomba-like (`config/default.toml`, `[robot]`): radius 0.17 m, wheel base
0.235 m, wheel radius 0.036 m, 0.5 m/s per wheel (the iRobot Open Interface
limit), 1 m/s² acceleration (an assumption). The resulting in-place spin rate
is `2 · 0.5 / 0.235 ≈ 4.3 rad/s`.
