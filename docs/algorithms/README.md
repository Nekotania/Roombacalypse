# Algorithm notes

One document per algorithm: the model, the derivation where useful, the
approximations, how it is tested, and measured results. A document is written
when its algorithm is implemented and tested, never before.

| Algorithm | Milestone | Code | Document | Status |
|---|---|---|---|---|
| Differential-drive kinematics and drivetrain limits | M1 | `core/kinematics.py`, `robot/drivetrain.py` | [differential-drive.md](differential-drive.md) | ✅ implemented, tested |
| Collision detection and response | M1 | `environment/shapes.py`, `simulation/physics.py` | [collision.md](collision.md) | ✅ implemented, tested |
| LiDAR ray casting and noise | M2 | `sensors/raycast.py`, `sensors/lidar.py` | [lidar.md](lidar.md) | ✅ implemented, tested |
| Encoders, drive errors, bumper, dead reckoning; gyro and gyrodometry (M4) | M2, M4 | `robot/wheels.py`, `sensors/`, `localization/odometry.py` | [odometry.md](odometry.md) | ✅ implemented, tested |
| Dynamic Window Approach and map-free exploration | M2 | `navigation/dwa.py`, `autonomy/wander.py` | [dwa.md](dwa.md) | ✅ implemented, tested |
| Graph SLAM: occupancy grid, scan matching, pose graph, loop closure; lost and found (M4) | M3, M4 | `mapping/`, `localization/` | [slam.md](slam.md) | ✅ implemented, tested |
| Costmap, A\*, coverage (online BA\* + frontiers), path following, executive, stuck detection and recovery | M4 | `planning/`, `navigation/path_follower.py`, `autonomy/executive.py`, `autonomy/recovery.py` | [planning.md](planning.md) | ✅ implemented, tested |
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
- S. P. Engelson, D. V. McDermott, "Error correction in mobile robot map learning" (the kidnapped
  robot problem), *IEEE ICRA*, 1992.
- S. Macenski et al., "The Marathon 2: A Navigation System" (Nav2: costmaps, recovery behaviours,
  DriveOnHeading), *IEEE/RSJ IROS*, 2020.
- G. Grisetti, C. Stachniss, W. Burgard, "Improved Techniques for Grid Mapping with
  Rao-Blackwellized Particle Filters", *IEEE Transactions on Robotics*, 2007.
- G. Grisetti, R. Kümmerle, C. Stachniss, W. Burgard, "A Tutorial on Graph-Based SLAM",
  *IEEE Intelligent Transportation Systems Magazine*, 2010.
- E. Olson, "Real-Time Correlative Scan Matching", *IEEE ICRA*, 2009.
- A. Censi, "An ICP variant using a point-to-line metric", *IEEE ICRA*, 2008.
- W. Hess, D. Kohler, H. Rapp, D. Andor, "Real-Time Loop Closure in 2D LIDAR SLAM"
  (Cartographer), *IEEE ICRA*, 2016.
- K. Konolige et al., "Efficient Sparse Pose Adjustment for 2D Mapping" (Karto), *IEEE IROS*, 2010.
- A. Elfes, "Using Occupancy Grids for Mobile Robot Perception and Navigation", *Computer*, 1989.
- J. Sturm et al., "A Benchmark for the Evaluation of RGB-D SLAM Systems" (ATE), *IEEE IROS*, 2012.
- G. Fiedler, "Fix Your Timestep!", 2004 (the fixed-step accumulator in the viewer).
