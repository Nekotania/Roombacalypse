# Roadmap

Status as of 2026-11-03. Each milestone ends with: all tests green,
earlier functionality re-verified, documentation updated, and a written
report. A milestone is not "done" until its acceptance criteria are
demonstrated by tests or recorded runs.

| Milestone | Theme | Scope | Status |
|---|---|---|---|
| **M1** | *It moves* | room geometry, differential-drive robot, collisions, manual control, viewer | ✅ done |
| **M2** | *It sees* | LiDAR, bumper, encoders, drive errors, DWA obstacle avoidance, autonomous exploration, headless runner, events | ✅ done |
| **M3** | *It knows where it is* | occupancy grid mapping, particle-filter localization, SLAM coupling, map/pose metrics | ⏭ next (needs D7) |
| **M4** | *It has a plan* | costmap, A\*, coverage planning, executive state machine, stuck detection and recovery | planned |
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
- **M4** plans in M3's map frame using M3's pose estimate, and reuses M2's DWA as its local planner.
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

## M3: It knows where it is (next)

**Prerequisite:** the director confirms decision D7 (SLAM strategy;
ARCHITECTURE §16).

**Deliverables**

- `mapping/`: log-odds occupancy grid, inverse sensor model, vectorized ray tracing.
- `localization/`: particle filter with odometry motion model, likelihood field,
  low-variance resampling and an effective-sample-size trigger.
- SLAM coupling behind a `Slam` interface.
- Evaluation: absolute trajectory error and map precision/recall against ground truth.
- Overlays: map (`M`), particles (`P`), estimated versus true pose.
- The wander behaviour switches from odometry to the localized pose, so its
  novelty memory stops smearing.

**Acceptance criteria**

- Unit tests for the grid update, motion model statistics, sensor model,
  resampling, and global localization convergence in a known map.
- After 10 minutes of wandering with default noise, against today's
  dead-reckoning baseline of up to 8.6 m and 153°:
  - map precision ≥ 0.95 and recall ≥ 0.85 on occupied cells;
  - RMS absolute trajectory error < 5 cm.

  These are targets; final thresholds will be set from measured runs and
  justified in the docs.
- Failure modes documented honestly (no loop closure).

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
| R1 | SLAM drift or map corruption (no loop closure) | M3–M5 | medium | apartment-scale rooms, 360° LiDAR; insert scans only when the filter is confident; known-pose diagnostic mode for tests; trajectory-error metric; RBPF as an upgrade path |
| R2 | Python too slow at high speed factors | all | low (measured) | M2 runs at about 90× real time headless with LiDAR + DWA; M3's particle filter will be vectorized and profiled |
| R3 | DWA local minima and oscillation | M2, M4 | medium (reduced) | M2 fixed the pillar minimum and margin deadlocks; U-traps remain; M4's A\* path and recovery ladder |
| R4 | Docking precision limited by localization error | M5 | medium | infrared beacon for the final approach; tolerance-based contact model |
| R5 | Coverage near walls and under furniture limited by safety margins | M4, M5 | high (by design) | coverage measured against the *accessible* floor (implemented); later, a wall-following edge pass |
| R6 | Floating-point differences across platforms and NumPy versions | tests | low | same-process determinism tests; metric thresholds rather than golden values |
| R7 | pygame wheel availability on new Python versions | install | low (mitigated) | `pygame-ce`; GUI isolated; core and tests run headless |
| R8 | Scope creep in polish and personality | schedule | medium | milestone gates; personality is observer-only |
| R9 | Movable obstacles invalidate the map | M3, M4 | medium | log-odds clamping; replanning on map change; recovery |
| R10 | Simplified pushing (no rotation, no chain pushes) is visible to users | realism | low | documented; upgrade path in the contact solver |
| R11 | Odometry drift makes M2's novelty memory smear in long runs | M2 | certain (accepted) | documented; M3 replaces odometry with the localized pose |

## Open questions for the project director

1. **D7, the SLAM strategy (needed before M3):** confirm "particle-filter
   localization against the robot's own growing map" (recommended), or choose
   the heavier Rao-Blackwellized variant.
2. **Scenarios:** any must-have rooms? For example, a multi-room apartment with
   doorways, a "furniture gauntlet" for recovery tests, or stairs with cliff sensors.
3. ~~Licence~~: MIT, confirmed (D13).
