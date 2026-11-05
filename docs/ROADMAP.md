# Roadmap

Status as of 2026-11-05. Each milestone ends with: all tests green,
earlier functionality re-verified, documentation updated, and a written
report. A milestone is not "done" until its acceptance criteria are
demonstrated by tests or recorded runs.

| Milestone | Theme | Scope | Status |
|---|---|---|---|
| **M1** | *It moves* | room geometry, differential-drive robot, collisions, manual control, viewer | ✅ done |
| **M2** | *It sees* | LiDAR, bumper, encoders, drive errors, DWA obstacle avoidance, autonomous exploration, headless runner, events | ✅ done |
| **M3** | *It knows where it is* | graph SLAM (scan matching, pose graph, loop closure), occupancy grid, multi-room apartment, map/pose metrics | ✅ done |
| **M4** | *It has a plan* | costmap, A\*, coverage planning, executive state machine, stuck detection and recovery | ⏭ next |
| **M5** | *It cleans and comes home* | dirt and vacuum, battery and power, energy-aware return, docking, recharge and resume | planned |
| **M6** | *It has feelings* | personality system, statistics, polished visualization | planned |
| **M7** | *It ships* | documentation, benchmarks, release preparation | planned |

## Dependencies

```
M1 ──► M2 ──► M3 ──► M4 ──► M5 ──► M7
        │             │      │       ▲
        └── events ───┴──────┴─► M6 ─┘
```

- **M2** needs M1's engine and geometry (raycasting reads the same shapes).
- **M3** needs M2's LiDAR and encoders, plus the evaluation harness that compares belief with truth (now in place).
- **M4** plans in M3's map frame using M3's pose estimate (`brain.pose`, `brain.slam.grid`), and reuses M2's
  DWA as its local planner.
- **M5** needs M4's planner (to get home) and executive (to decide when).
- **M6** can start now: M2 publishes events. Its best material
  (getting stuck, near-empty battery) comes from M4 and M5.
- The battery cell and power models have no algorithmic dependencies; they could move earlier if wanted.

## M1: It moves ✅

**Delivered**

- Package skeleton with the truth/belief architecture enforced by tests.
- Config system: frozen dataclasses, layered TOML, strict validation.
- Continuous 2.5D world: concave floor plans, convex and circular obstacles
  with height bands, furniture macros, a charging dock, two built-in rooms.
- Differential-drive kinematics (exact arc integration), drivetrain with
  curvature-preserving saturation and acceleration limits.
- Contact solver: no penetration, sliding along walls, quasi-static pushing of
  movable obstacles, height-aware collisions.
- Pygame viewer: manual driving, pause, speed control, single step, reset,
  obstacle placement and removal, zoom/pan, trail, tooltips, help, screenshots.

**Acceptance criteria (all met):** manual driving works; no penetration in a
60 s random-driving property test; movable obstacles can be pushed while
walls and fixed obstacles stop the robot; runs are deterministic; physics
never imports pygame.

## M2: It sees ✅

**Delivered**

- **Sensors** (truth side):
  - LiDAR with vectorized ray casting, height-band visibility, range-dependent
    Gaussian noise, dropouts and max range;
  - a latched left/right bumper;
  - quantized wheel encoders.
- **Hidden drive errors:** per-robot wheel-radius and wheel-base errors, plus
  correlated Ornstein–Uhlenbeck slip. Odometry now drifts as on a real robot.
- **Robot software** (belief side):
  - `RobotBrain` owns odometry and obstacle memory across modes; Tab switches
    manual and autonomous;
  - dead-reckoning odometry;
  - an obstacle memory that motion-compensates the latest scan and remembers bumps;
  - the Dynamic Window Approach local planner;
  - a map-free exploration behaviour with a bump reflex and a cramped-corner fallback.
- **Event log:** the simulator reports collisions; the robot reports bumps,
  goals, escapes and mode changes. The robot can write events but not read them.
- **Evaluation harness:** ground-truth penetration, pose error, footprint
  coverage of the accessible floor, and a headless scenario runner
  (`main.py --headless`).
- **Viewer:** LiDAR (L), navigation (G) and belief (E) overlays; sensors, belief
  and statistics in the panel; a live event log.
- **Tests:** 314 fast (about 8 s) plus a 10-run acceptance suite
  (`pytest -m slow`, about 70 s). Ruff and mypy are clean.

**Acceptance criteria and measured results.** 10 simulated minutes of
autonomy, default noise, seeds 1–5:

| Criterion | Threshold (enforced) | Test Chamber measured | Studio measured |
|---|---|---|---|
| penetration | none | none | none |
| contacts with LiDAR-visible things | 0 | 0 | 0 |
| bump rate | ≤ 0.2/min (chamber), ≤ 1.2/min (studio) | 0/min | 0.2–0.7/min (sock and dog bowl only) |
| exploration: accessible floor swept | ≥ 70 % (chamber), ≥ 60 % (studio) | 78–85 % | 69–73 % |
| no stalls: distance driven | ≥ 120 m | ~160 m | ~158–161 m |

LiDAR ray casting is checked against hand-computed distances, height bands
and range limits, and its noise statistics against the configured values
(test_lidar.py).

**Found and fixed during M2.** Each issue was surfaced by a test or an
acceptance run and is now covered by a test:

- **Cramped-corner livelock.** Studio, seed 5: the robot was stuck nose-in
  between chair legs for 8 minutes. Fixed with the short-hop fallback.
- **DWA creeping into walls.** The planner predicted instant velocity changes
  and allowed 1 mm of approach per tick. Fixed with ramp-aware prediction and
  hard radii.
- **DWA deadlocked inside its own margin.** Fixed with a two-stage hard radius.
- **DWA's local minimum in front of a pillar.** Fixed with mean-clearance
  scoring and the intrusion penalty.

## M3: It knows where it is ✅

**Decision D7** (the SLAM strategy) was delegated by the director with one
requirement, "something that works in actual robotics, not just simulations".
Chosen: graph SLAM in the Karto / slam_toolbox / Cartographer family, because
it can correct mapped errors through loop closure. Rationale in ARCHITECTURE
§16; algorithm note: [algorithms/slam.md](algorithms/slam.md).

**Delivered**

- `mapping/`:
  - a log-odds occupancy grid with vectorized ray tracing, one update per cell
    per scan, clamping and growth;
  - surface fields: a coarse likelihood grid, plus line and point surface
    elements fitted by PCA;
  - a sparse Levenberg-Marquardt pose-graph optimizer with a Cauchy kernel.
- `localization/`:
  - correlative scan matching with two-level branch and bound;
  - point-to-line ICP refinement with ICP information and an ambiguity check;
  - `GraphSlam` behind a `Slam` protocol:
    - keyframes, and a per-scan wheel-slip check;
    - a local map;
    - loop closure with verification, then re-optimization.
- **The robot uses it.** `RobotBrain` runs SLAM in every mode. Wander's goals and
  visit counter use the map-frame pose; reflexes keep odometry (the `map → odom`
  split, D17). Loop closures and lost scans publish `map` events.
- **A multi-room apartment** (`config/rooms/apartment.toml`, asked for by the
  director): six spaces off a 10 m hallway, with a 13 m loop through living
  room and kitchen.
- **Evaluation:** online and keyframe trajectory error (anchored at the dock,
  plus aligned ATE) and map precision/recall in the headless scorecard.
- **Viewer:**
  - `M` (map, drawn where it really is), on by default;
  - `P` (pose graph and loop closures);
  - `E` now shows the SLAM ghost next to the odometry ghost;
  - a SLAM section in the panel.
- **Tests:** 375 fast (about 30 s on the development container) plus 15 acceptance runs (`pytest -m slow`,
  about 8 minutes). Ruff and mypy are clean.

**Acceptance criteria and measured results.** 10 simulated minutes of
autonomy, default noise, seeds 1–5, each run scored once against both the M2
and the M3 criteria:

| Criterion | Test Chamber: threshold / measured | Studio: threshold / measured | Apartment: threshold / measured |
|---|---|---|---|
| SLAM pose error, online RMS | ≤ 1.5 cm / 0.3–0.5 cm | ≤ 2.5 cm / 0.5–1.4 cm | ≤ 8 cm / 2.9–5.3 cm |
| SLAM pose error, online max | ≤ 4 cm / 1.0–1.4 cm | ≤ 6 cm / 1.8–2.9 cm | ≤ 15 cm / 6.9–9.2 cm |
| keyframe ATE RMS (anchored at the dock) | ≤ 1.5 cm / 0.3–0.5 cm | ≤ 3 cm / 0.5–1.6 cm | ≤ 8 cm / 2.6–5.7 cm |
| keyframe ATE RMS, aligned | ≤ 1 cm / 0.2 cm | ≤ 1.5 cm / 0.4–0.5 cm | ≤ 4 cm / 1.6–2.8 cm |
| map precision (no phantom obstacles) | ≥ 98 % / 99.9–100 % | ≥ 98 % / 99.9–100 % | ≥ 93 % / 95.6–99.8 % |
| map recall | ≥ 90 % / 95.7–97.1 % | ≥ 85 % / 89.3–92.0 % | ≥ 82 % / 86.8–92.0 % |
| loop closures | > 0 / 66–72 | > 0 / 68–71 | > 0 / 54–59 |
| dead reckoning, for comparison | / 0.3–3.6 m | / 0.7–5.2 m | / 0.8–11.3 m |
| M2: penetration; contacts with LiDAR-visible things | none; 0 / none; 0 | none; 0 / none; 0 | none; 0 / none; 0 |
| M2: accessible floor swept | ≥ 70 % / 80–84 % | ≥ 60 % / 68–73 % | ≥ 35 % / 42–55 % |
| M2: bump rate | ≤ 0.2/min / 0 | ≤ 1.2/min / 0.4–0.8 | ≤ 1.2/min / 0.3–0.8 |
| M2: distance driven | ≥ 120 m / ~162 m | ≥ 120 m / ~159 m | ≥ 120 m / ~151 m |

Against the targets set before M3:

- **ATE < 5 cm:** met with wide margins in the chamber and studio (≤ 1.6 cm);
  in the apartment 4 of 5 seeds (seed 5: 5.7 cm).
- **Precision ≥ 0.95:** met everywhere.
- **Recall ≥ 0.85:** met everywhere.

The apartment's absolute error is almost entirely a small rotation of the
map about the dock (about 0.5°), picked up crossing the hallway. The map
itself is consistent (aligned ATE ≤ 2.8 cm), and errors near the dock stay
under 2 cm. The apartment thresholds are therefore set from its own
measurements, as agreed for the stress test.

Map precision counts *phantom obstacles*: occupied cells in free space more
than a cell from any surface. Strict surface precision, which also counts
the harmless thickening of surfaces into solid matter, is 73–89 %; see
algorithms/slam.md, "Evaluation", and decision D19.

**Found and fixed during M3.** Most were found by asking why a number was not
as good as it should be. Details in algorithms/slam.md:

- **Sub-cell accuracy.** Matching against cell centres limited a match to
  ±1 cell. Point-to-line surface elements fixed it: perfect-sensor keyframe
  error fell from 14.6 mm to 3.3 mm.
- **A quartic cost and fake information.** It reported more information
  *along* a corridor than across it.
- **A chiral sampling bias.** Surfaces sat half a step clockwise around the
  sensor: −0.005° per keyframe, the same sign on every seed.
- **Noisy pairwise normals, a stalling solver and a V-shaped point residual.**
  Together they left 6.7 % of each odometry heading error in every match.
  The tell was an even/odd effect in the iteration count.
- **Fake point features along walls.** Lone far returns and blurred far walls
  pinned a corridor along its length.
- **A branch-and-bound border bug.** Blocks just outside the field were
  bounded by 0. A property test found it.
- **Phantom travel.** Spinning wheels against a wall let the pose run 22 cm
  ahead. A per-scan slip check now holds it to one scan period.
- **Wrong test expectations.** Several first-draft expectations were wrong,
  and the tests now state the measured behaviour. For example, ICP converges
  from 45 cm off, not 5 cm, and one scan does not yet mark free space (p = 0.40).

**Limitations (honest):**

- no global relocalization: a kidnapped robot stays lost;
- loop closure searches ±0.5 m and ±12°;
- absolute error grows across long corridors until a loop closes;
- a map re-render costs about 1 ms per keyframe;
- a static-world assumption;
- one session (maps are not saved).

## M4: It has a plan

**Deliverables**

- Costmap with inflation and safety margins.
- A\* with octile heuristic and path pruning.
- BA\* coverage planner with frontier exploration.
- Path following through DWA.
- Executive state machine.
- Stuck detectors (bump rate, no progress, loops, plan failure, slip) and an escalating recovery ladder.
- Overlays: path (`G`), coverage (`C`).

**Acceptance criteria**

- A\* returns optimal costs (checked against Dijkstra on random grids) and
  respects inflation.
- ≥ 90 % of the accessible floor swept in the studio within 30 simulated
  minutes, with the battery disabled. M2's random wandering reaches about 70 %
  in 10 minutes.
- Each stuck detector fires on its dedicated trap scenario, and the recovery
  either succeeds or declares `TRAPPED` within bounded time.

## M5: It cleans and comes home

**Deliverables**

- Dirt field, vacuum pickup, dirt sensor, cleaning metrics (reusing `evaluation/coverage.py`).
- Battery cell, power model, battery gauge.
- Energy-aware return decision.
- Pre-dock navigation plus infrared beacon docking; charge, then resume.
- Right-click spawns dirt.

**Acceptance criteria**

- Dirt mass is conserved (collected + remaining = total).
- Battery energy matches the integrated power draw.
- A full studio mission with at least one recharge completes.
- Docking succeeds in ≥ 95 % of 20 seeded runs.

## M6: It has feelings

**Deliverables**

- `personality/`: emotion integrators fed by the existing event log, an
  appraisal table, a deterministic narrator, incident reports.
- Statistics panel and visual polish.

**Acceptance criteria**

- Emotions respond to scripted event sequences as specified.
- Narration is deterministic per seed.
- The architecture test proves personality cannot influence control.

## M7: It ships

**Deliverables**

- Complete algorithm documentation.
- `scripts/benchmark.py` (seeds × rooms, matplotlib plots).
- Replayable input logs.
- Packaging check, changelog, screenshots and GIFs in the README.

## Technical risks

| # | Risk | Affects | Likelihood | Mitigation / status |
|---|---|---|---|---|
| R1 | SLAM drift or map corruption | M3–M5 | low (measured) | graph SLAM with verified loop closures and a robust kernel; scans that do not fit stay out of the map; measured 0.3–1.6 cm ATE (studio, chamber), 2.6–5.7 cm (apartment). Remaining: heading drift across corridors (a gyro would bound it), no global relocalization |
| R2 | Python too slow at high speed factors | all | low (measured) | M2 ran at about 50–90× real time headless; with SLAM, about 20× on the development container (SLAM costs ~15–20 s per simulated 10 minutes, profiled and vectorized) |
| R3 | DWA local minima and oscillation | M2, M4 | medium (reduced) | M2 fixed the pillar minimum and margin deadlocks; U-traps remain; M4's A\* path and recovery ladder |
| R4 | Docking precision limited by localization error | M5 | medium | infrared beacon for the final approach; tolerance-based contact model |
| R5 | Coverage near walls and under furniture limited by safety margins | M4, M5 | high (by design) | coverage measured against the *accessible* floor (implemented); later, a wall-following edge pass |
| R6 | Floating-point differences across platforms and NumPy versions | tests | low | same-process determinism tests; metric thresholds rather than golden values |
| R7 | pygame wheel availability on new Python versions | install | low (mitigated) | `pygame-ce`; GUI isolated; core and tests run headless |
| R8 | Scope creep in polish and personality | schedule | medium | milestone gates; personality is observer-only |
| R9 | Movable obstacles invalidate the map | M3, M4 | medium | log-odds clamping; replanning on map change; recovery |
| R10 | Simplified pushing (no rotation, no chain pushes) is visible to users | realism | low | documented; upgrade path in the contact solver |
| R11 | Odometry drift makes M2's novelty memory smear in long runs | M2 | resolved (M3) | the visit counter now lives in the SLAM map frame |
| R12 | Map re-rendering cost grows with keyframes (≈1 ms each) | M4+ | medium | lazy, throttled re-render today; Cartographer-style submaps if hour-long runs need it |
| R13 | Kidnapping or a large slip leaves the robot lost | M4, M5 | low in simulation | lost scans are detected and kept out of the map; global relocalization belongs to M4's recovery ladder |

## Open questions for the project director

1. ~~D7, the SLAM strategy~~: delegated by the director; graph SLAM chosen (see M3).
2. ~~Scenarios~~: a multi-room apartment was added in M3. More rooms (a
   "furniture gauntlet" for M4's recovery tests, stairs with cliff sensors)
   remain possible.
3. ~~Licence~~: MIT, confirmed (D13).
4. **New: a gyroscope?** Every real robot vacuum fuses one with its wheel
   odometry, and the apartment's remaining error is mostly heading drift
   across the hallway. Adding a simulated gyro (with bias drift) and a heading
   constraint in the pose graph is cheap and would likely halve that error.
   Proposed for M4 unless you prefer to keep the sensor set minimal.
5. **New: global relocalization** (a kidnapped robot, or one switched on away
   from its dock) fits M4's recovery ladder. Confirm it belongs there.
