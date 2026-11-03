# LiDAR simulation

Code: `roombacalypse/sensors/raycast.py` (vectorized ray casting) and
`roombacalypse/sensors/lidar.py` (beam layout, visibility, error model).
Tests: `tests/test_lidar.py`.

## The sensor

A 2D scanning LiDAR on top of the robot, modelled on the cheap "LDS" turrets
of LiDAR robot vacuums. Defaults (`[lidar]` in `config/default.toml`):

| Parameter | Default | Notes |
|---|---|---|
| scan plane height | 0.095 m | just above the 0.092 m body |
| turret position | 2 cm behind the body centre | the robot knows this; scans report it |
| angular resolution | 1° | 360 beams per sweep |
| field of view | 360° | |
| maximum range | 6 m | beyond that: no return |
| range noise | σ = 8 mm + 0.4 % of range | Gaussian |
| dropouts | 1 % of beams | return nothing |
| rate | 5 Hz | one sweep every 0.2 s |

## Ray casting

All beams of a sweep share one origin O, the turret. For a wall or polygon
edge A + u·e (u ∈ [0, 1]), the hit along a beam with unit direction d solves
O + t·d = A + u·e. With the 2D cross product a × b = aₓb_y − a_yb_x and
w = A − O:

```
t = (w × e) / (d × e)      distance along the beam
u = (w × d) / (d × e)      position along the edge
```

The beam hits if d × e ≠ 0 (not parallel), t ≥ 0 and 0 ≤ u ≤ 1. For a circle
with centre C and radius r, substituting into |O + t·d − C|² = r² gives
t = −b − √(b² − c), with b = d·(O − C) and c = |O − C|² − r² (the near root).
The measured range is the smallest t over all edges and circles.

This is computed for every beam/primitive pair at once with NumPy
(360 beams × ~35 edges + ~30 circles in the studio). The visible geometry
(edges and circles) is cached and rebuilt only when `World.version` changes,
for example when the robot pushes a box.

## What the robot can see

An obstacle reflects beams only if its vertical band contains the scan plane:

| Studio object | Band | Seen? |
|---|---|---|
| walls, sofa, cabinets, fridge | from the floor up | yes |
| table and chair legs, bed legs | from the floor up | yes (thin legs can fall between beams far away) |
| table tops, chair seats, bed frame (0.12 m+) | above the plane | no: the robot drives underneath |
| sock (2 cm), dog bowl (6 cm) | below the plane | **no: found only by the bumper** |

The robot is never hidden from its own LiDAR: the turret is above the body,
which is enforced by config validation.

## Error model

Per beam, the sensor:

1. adds Gaussian noise with σ = `range_noise_std + range_noise_per_metre × range`;
2. drops the beam (no return) with probability `dropout_probability`;
3. reports no return (`inf`, as in ROS REP 117) when the range exceeds `max_range`.

The random draws per sweep are fixed (one normal and one uniform number per
beam), whatever is hit. Adding an obstacle therefore never shifts the random
numbers seen by later sweeps, which keeps seeded runs comparable.

Tested:

- every geometric case against hand-computed distances: walls, circles,
  rotated poses, height bands, maximum range, world edits;
- the noise mean and standard deviation over 400 sweeps (within 12 % of σ);
- the normalized noise across all beams (standard normal within 3 %);
- the dropout rate (25 % ± 2 % when configured to 25 %);
- seed reproducibility.

## Approximations and limitations

- **Instant sweep.** A real turret rotates over 0.2 s, so the robot moves during
  a sweep (motion distortion). The simulation captures the whole sweep at one
  instant.
- **Perfect geometry.** No beam divergence, no "mixed pixels" at edges where
  a beam straddles two surfaces, no reflectivity effects (black or glass furniture).
- **Independent errors.** Errors are uncorrelated across beams and sweeps;
  real sensors have some bias and correlation.
- **No minimum range.** Nothing can come closer to the turret than the body
  radius, so a minimum range would never matter here.
