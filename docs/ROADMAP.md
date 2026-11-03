# Roadmap

Status as of 2026-11-03. Each milestone ends with: all tests green,
earlier functionality re-verified, documentation updated, and a written
report. A milestone is not "done" until its acceptance criteria are
demonstrated by tests or recorded runs.

| Milestone | Theme | Scope | Status |
|---|---|---|---|
| **M1** | *It moves* | room geometry, differential-drive robot, collisions, manual control, viewer | ✅ done |
| **M2** | *It sees* | LiDAR, bumper, encoders, drivetrain imperfections, DWA obstacle avoidance, autonomous wandering, headless runner, events | ⏭ next |
| **M3** | *It knows where it is* | occupancy grid mapping, particle-filter localization, SLAM coupling, evaluation metrics | planned |
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
- **M3** needs M2's LiDAR and encoders, plus an evaluation harness that compares belief with truth.
- **M4** plans in M3's map frame using M3's pose estimate, and reuses M2's DWA as its local planner.
- **M5** needs M4's planner (to get home) and executive (to decide when).
- **M6** can start once M2 publishes events, but its best material
  (getting stuck, near-empty battery) comes from M4 and M5.
- The battery cell and power models have no algorithmic dependencies; they could move earlier if wanted.

## M1: It moves ✅

**Delivered**

- Package skeleton with the truth/belief architecture enforced by tests.
- Config system: frozen dataclasses, layered TOML, strict validation.
- Continuous 2.5D world: concave floor outline, convex and circular obstacles
  with height bands, furniture macros, a charging dock, two built-in rooms.
- Differential-drive kinematics (exact arc integration), drivetrain with
  curvature-preserving saturation and acceleration limits.
- Contact solver: no penetration, sliding along walls, quasi-static pushing of
  movable obstacles, height-aware collisions.
- Pygame viewer: manual driving, pause, speed control, single step, reset,
  obstacle placement and removal, zoom/pan, trail, tooltips, help, screenshots.
- 199 tests (Python 3.15 and 3.12); ruff and mypy clean.

**Acceptance criteria (all met)**

- Manual driving works.
- The robot never penetrates walls or obstacles. Tested with a 60 s random-driving
  property test in the furnished studio.
- Movable obstacles can be pushed; immovable ones and walls stop the robot.
- Runs are deterministic.
- Physics never imports pygame.

## M2: It sees (next)

**Deliverables**

- `sensors/lidar.py`: vectorized raycasting against walls, polygons and circles
  in the LiDAR's height band; Gaussian noise, dropouts, max range; configurable
  resolution, range and rate.
- `sensors/bumper.py`: left/right switches over the front arc.
- `sensors/encoders.py`: quantized wheel ticks.
- Drivetrain imperfections, truth side only: wheel radius mismatch, wheel base error, random slip.
- `core/events.py`: an event log (bumps, mode changes, …) for statistics and the future personality.
- `navigation/dwa.py`: the Dynamic Window Approach local planner.
- `autonomy/wander.py`: autonomous wandering (exploration goals + DWA + bump reflex).
  `Tab` switches between manual and autonomous.
- Headless runner (`--headless --duration`) and a LiDAR overlay (`L`).

**Acceptance criteria**

- Raycast tests against analytic distances: walls, circles, polygons, height
  bands, max range. Noise statistics within tolerance.
- 10 minutes of headless autonomous wandering in each built-in room, 5 seeds:
  - zero penetration (always);
  - a bump rate below a documented threshold;
  - the robot explores (visits a documented minimum of floor area).
- The architecture test still passes: the autonomous controller sees only `SensorFrame`s.

## M3: It knows where it is

**Deliverables**

- `mapping/`: log-odds occupancy grid, inverse sensor model, vectorized ray tracing.
- `localization/`: particle filter with odometry motion model, likelihood field,
  low-variance resampling and an effective-sample-size trigger.
- SLAM coupling (decision D7).
- `evaluation/`: trajectory error and map quality against ground truth.
- Viewer overlays: map (`M`), particles (`P`), estimated versus true pose.

**Acceptance criteria**

- Unit tests for the grid update, motion model statistics, sensor model,
  resampling, and global localization convergence in a known map.
- After 10 minutes of wandering with default noise:
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
- ≥ 90 % coverage of the accessible floor in the studio within 30 simulated
  minutes, with the battery disabled.
- Each stuck detector fires on its dedicated trap scenario, and the recovery
  either succeeds or declares `TRAPPED` within bounded time.

## M5: It cleans and comes home

**Deliverables**

- Dirt field, vacuum pickup, dirt sensor, cleaning metrics.
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

- `personality/`: emotion integrators, an appraisal table, a deterministic
  narrator, incident reports.
- Statistics panel, message log, emotion bars.
- Visual polish.

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

| # | Risk | Affects | Likelihood | Mitigation |
|---|---|---|---|---|
| R1 | SLAM drift or map corruption (no loop closure) | M3–M5 | medium | apartment-scale rooms, 360° LiDAR; insert scans only when the filter is confident; known-pose diagnostic mode for tests; trajectory-error metric; RBPF as an upgrade path behind the `Slam` interface |
| R2 | Python too slow (particle filter + DWA + raycasting) at high speed factors | all | medium | NumPy vectorization, beam subsampling, distance-transform lookups, sensor and control rates decoupled from physics, profiling; headless mode for long runs |
| R3 | DWA local minima and oscillation; narrow passages versus inflation radius | M2, M4 | high | global path guidance, recovery behaviours, tunable inflation, scenario tests with doorways and chair clusters |
| R4 | Docking precision limited by localization error | M5 | medium | infrared beacon for the final approach; tolerance-based contact model |
| R5 | Coverage near walls and under furniture limited by safety margins | M4, M5 | high (by design) | report coverage against the *accessible* floor; later, a wall-following edge pass |
| R6 | Floating-point differences across platforms and NumPy versions | tests | low | same-process determinism tests; metric thresholds rather than golden values |
| R7 | pygame wheel availability on new Python versions | install | low (mitigated) | `pygame-ce`; GUI isolated; core and tests run headless |
| R8 | Scope creep in polish and personality | schedule | medium | milestone gates; personality is observer-only |
| R9 | Movable obstacles invalidate the map | M3, M4 | medium | log-odds clamping; replanning on map change; recovery |
| R10 | Simplified pushing (no rotation, no chain pushes) is visible to users | realism | low | documented; upgrade path in the contact solver |

## Open questions for the project director

1. **D7, the SLAM strategy:** confirm "MCL against the self-built map" (recommended) before M3 starts.
2. **Scenarios:** any must-have rooms? For example, a multi-room apartment with
   doorways, a "furniture gauntlet" for recovery tests, or stairs with cliff sensors.
3. **Licence:** MIT was chosen as a placeholder (decision D13).
