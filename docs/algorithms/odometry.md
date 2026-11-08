# Wheel encoders, gyro, drive errors and dead reckoning

Code:

- truth side: `roombacalypse/robot/wheels.py` (hidden drive errors),
  `roombacalypse/sensors/encoders.py`, `roombacalypse/sensors/bumper.py`,
  `roombacalypse/sensors/gyro.py` (M4);
- belief side: `roombacalypse/localization/odometry.py`.

Tests: `tests/test_sensors.py`, `tests/test_gyro.py`.

## Two kinds of "metres"

The drivetrain and the encoders work in *nominal metres*: wheel rotation
multiplied by the wheel radius printed on the spec sheet. How far the robot
really moves over the floor is a separate question that only the simulator
can answer. The gap between the two is what makes odometry drift.

## The hidden error model (ground truth)

Following Borenstein & Feng's UMBmark analysis, wheel odometry suffers two
kinds of error.

**Systematic**, drawn once per robot (per reset, from the seed):

| Error | Default std | Effect |
|---|---|---|
| each wheel's true radius vs. nominal | 0.2 % | unequal wheels: a "straight" drive curves by `d · (s_r − s_l) / L` |
| true wheel base vs. nominal | 1 % | every turn comes out slightly too large or too small |

**Random:** slip. Each wheel's ground travel is multiplied by
`(1 + slip)`, where slip follows an Ornstein–Uhlenbeck process: zero mean,
standard deviation 1 %, correlation time 0.5 s. It is discretized exactly:

```
slip ← slip · e^(−dt/τ) + σ · √(1 − e^(−2·dt/τ)) · N(0, 1)
```

Correlated slip (rug versus tiles) matters. Independent per-step noise of
the same size would average out within centimetres and make odometry
unrealistically good.

Tested: ideal settings reproduce the nominal kinematics exactly; the slip
process has its configured standard deviation and an autocorrelation of
e⁻¹ at one time constant; a robot with 0.995/1.005 wheels curves by the
predicted angle to within 3 %.

## Encoders

Encoders count wheel rotation quantized to 508.8 ticks per revolution (the
iRobot Create 2 figure), so one tick is 0.44 mm of nominal travel. They count
*rotation*, not motion: when the robot pushes against a wall its wheels keep
turning and the ticks keep coming. `test_odometry_drifts_with_realistic_wheels_and_keeps_counting_against_walls`
shows 0.5 m+ of phantom travel after 2.5 s of pushing.

## Bumper

Two switches cover the front 180°. A contact at a bearing within ±15° of
straight ahead closes both, otherwise the switch on its side. Contacts
behind the robot are not felt; there is no rear bumper. Switches are latched
between control ticks so short taps are not lost.

## Gyroscope (M4, asked for by the director)

Every real robot vacuum carries a MEMS yaw-rate gyro. The simulated one
measures the body's *true* yaw rate after collisions (so a robot pinned
against a wall with spinning wheels reads no rotation) and corrupts it as a
MEMS gyro does:

    measured = (1 + s) · ω_true + b(t) + n,     n ~ N(0, ARW² / dt)

| Term | Default | Meaning |
|---|---|---|
| `s` | σ 0.3 %, drawn per power-up | scale-factor error |
| `b` | σ 0.5 °/s at power-up, then a random walk of 0.0005 °/s/√s | bias: why gyro headings drift |
| `n` | 0.01 °/s/√Hz | angle random walk |
| quantization | 0.0076 °/s | one LSB of a typical ±250 °/s, 16-bit part |

The rate is integrated at the physics rate into a cumulative angle, as an
IMU's on-board integrator does. Two random numbers are drawn per step
whatever happens, so the stream stays comparable across scenes.

## Dead reckoning (belief side)

`OdometryTracker` integrates tick differences with the *nominal* wheel radius
and wheel base, using the same exact arc formula as the simulator
(`core/kinematics.py`; see [differential-drive.md](differential-drive.md)).
Its frame is the robot's boot pose on the dock. It also estimates wheel
speeds and the body twist from tick deltas, which the DWA planner uses as its
"current velocity".

With ideal wheels, the tracker reproduces the true pose to within 1 cm and
1° after an 8.5 s tour with curves and spins; the remaining error is tick
quantization. With the default errors, drift is real and grows. Over 10
minutes of autonomous wandering (about 160 m):

| Room | Position error | Heading error |
|---|---|---|
| Test Chamber (5 seeds) | 0.7–1.9 m | 4–153° |
| Studio (5 seeds) | 0.6–8.6 m | 11–120° |

The spread comes mostly from the drawn wheel-radius difference: with
L = 0.235 m, a 0.3 % radius mismatch is about 0.7° of heading per metre.
That is why Milestone 3 adds LiDAR-based localization, and why real robot
vacuums carry gyroscopes. The robot's software uses odometry only over short
horizons in M2 (velocity estimates, and moving the latest scan between
sweeps), where the drift is negligible.

**Gyrodometry (M4).** With the gyro, heading changes come from the gyro and
only the distance from the wheels. The bias is averaged while the robot
stands on its dock for the first 3 s after boot (the autonomous behaviours
wait for it), then nudged towards the measured rate (time constant 20 s)
whenever the encoders say the robot is standing still again: a zero-velocity
update, during which the heading is also held. SLAM's odometry noise model
switches accordingly: heading std 0.5 % per radian turned plus 0.0005 rad/s,
instead of 3 % per radian plus 1.5°/m.

Measured over 10 minutes of wandering, seeds 1–5:

| Room | Dead reckoning without gyro (M3) | With gyro | SLAM heading error, online RMS |
|---|---|---|---|
| Test Chamber | 0.3–3.6 m | 0.05–0.42 m, 2–17° | 0.07–0.19° |
| Studio | 0.7–5.2 m | 0.10–1.2 m, 1–19° | 0.08–0.12° |
| Apartment | 0.8–11.3 m | 0.05–1.6 m, 0.5–10° | 0.18–0.40° |

The remaining heading drift is mostly the scale error (0.3 % of the thousands
of degrees a wandering robot turns). SLAM's online heading error fell from
0.4–0.6° (M3) to 0.07–0.40°. The apartment's absolute position error did not
change much (3–5 cm): it is a small rotation of the whole map picked up
across the hallway, which the gyro does not remove.

## Limitations

- **One axis, simple gyro errors.** No temperature drift, g-sensitivity or
  cross-axis coupling; the gyro and the wheels are fused by substitution
  (heading from one, distance from the other), not by a Kalman filter.
- **Symmetric slip.** Slip is symmetric around zero and treats tyre
  compression and true slip alike. Real slip mostly *reduces* ground travel.
- **Constant calibration.** Calibration errors stay fixed for a run; real
  wheels wear and pick up hair.
- **No wheel-drop or stall sensors.** Real robots detect stalls from motor
  current; there are none here yet.
