# Algorithm notes

One document per algorithm: the model, the derivation where useful, the
approximations, how it is tested, and measured results. A document is written
when its algorithm is implemented and tested, never before.

| Algorithm | Milestone | Code | Document | Status |
|---|---|---|---|---|
| Differential-drive kinematics and drivetrain limits | M1 | `core/kinematics.py`, `robot/drivetrain.py` | [differential-drive.md](differential-drive.md) | ✅ implemented, tested |
| Collision detection and response | M1 | `environment/shapes.py`, `simulation/physics.py` | [collision.md](collision.md) | ✅ implemented, tested |
| LiDAR ray casting and noise | M2 | `sensors/raycast.py`, `sensors/lidar.py` | [lidar.md](lidar.md) | ✅ implemented, tested |
| Encoders, drive errors, bumper, dead reckoning | M2 | `robot/wheels.py`, `sensors/`, `localization/odometry.py` | [odometry.md](odometry.md) | ✅ implemented, tested |
| Dynamic Window Approach and map-free exploration | M2 | `navigation/dwa.py`, `autonomy/wander.py` | [dwa.md](dwa.md) | ✅ implemented, tested |
| Occupancy grid mapping | M3 | `mapping/` | — | planned |
| Particle filter localization | M3 | `localization/` | — | planned |
| A\* and costmap inflation | M4 | `planning/` | — | planned |
| Coverage planning (BA\* + frontiers) | M4 | `planning/coverage.py` | — | planned |
| Stuck detection and recovery | M4 | `autonomy/recovery.py` | — | planned |
| Cleaning, battery and docking | M5 | `cleaning/`, `battery/`, `navigation/docking.py` | — | planned |
| Personality dynamics | M6 | `personality/` | — | planned |

## References

- S. Thrun, W. Burgard, D. Fox, *Probabilistic Robotics*, MIT Press, 2005.
- D. Fox, W. Burgard, S. Thrun, "The Dynamic Window Approach to Collision Avoidance",
  *IEEE Robotics & Automation Magazine*, 1997.
- J. Borenstein, L. Feng, "Measurement and Correction of Systematic Odometry Errors in Mobile
  Robots" (UMBmark), *IEEE Transactions on Robotics and Automation*, 1996.
- H. H. Viet et al., "BA\*: an online complete coverage algorithm for cleaning robots",
  *Applied Intelligence*, 2013.
- B. Yamauchi, "A frontier-based approach for autonomous exploration", *IEEE CIRA*, 1997.
- G. Grisetti, C. Stachniss, W. Burgard, "Improved Techniques for Grid Mapping with
  Rao-Blackwellized Particle Filters", *IEEE Transactions on Robotics*, 2007.
- G. Fiedler, "Fix Your Timestep!", 2004 (the fixed-step accumulator in the viewer).
