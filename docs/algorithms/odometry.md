# Wheel encoders, drive errors and dead reckoning

Code:

- truth side: `roombacalypse/robot/wheels.py` (hidden drive errors),
  `roombacalypse/sensors/encoders.py`, `roombacalypse/sensors/bumper.py`;
- belief side: `roombacalypse/localization/odometry.py`.

Tests: `tests/test_sensors.py`.

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

## Limitations

- **No gyroscope.** Real robots fuse one with odometry; this one does not (yet).
- **Symmetric slip.** Slip is symmetric around zero and treats tyre
  compression and true slip alike. Real slip mostly *reduces* ground travel.
- **Constant calibration.** Calibration errors stay fixed for a run; real
  wheels wear and pick up hair.
- **No wheel-drop or stall sensors.** Real robots detect stalls from motor
  current; there are none here yet.
