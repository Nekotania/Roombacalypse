# ROOMBACALYPSE architecture

> **Status:** living document, revised at every milestone. Sections about later
> milestones describe the *intended* design and are marked **(planned, Mx)**.
> They will be rewritten from the actual implementation when that milestone is
> built and tested. Only Milestone 1 is implemented today.

## 1. Goals and non-goals

**Goals**

- **Genuine robotics.** The robot perceives, maps, localizes, plans and recovers
  using textbook algorithms that are implemented here and tested here.
- **No cheating.** The robot's software never reads ground truth (§2).
- **Deterministic.** Same config, room, seed and inputs give the same run, bit for bit.
- **Readable.** Small modules, explicit data flow, approximations written down.
- **Entertaining.** A personality layer observes real events and narrates them.

**Non-goals**

- High-fidelity rigid-body dynamics (friction cones, torques, restitution).
- Full 3D. The world is 2D plus height bands ("2.5D") where height matters.
- ROS compatibility, real hardware, machine learning.
- Real-time guarantees at extreme speed factors (best effort; headless runs as fast as it can).

## 2. The central rule: the wall of truth

The codebase contains two worlds that must not leak into each other:

| | Ground truth (the simulator) | Belief (the robot's software) |
|---|---|---|
| Knows | exact geometry, true pose, true dirt, true charge | only what its sensors reported |
| Packages | `environment`, `robot`, `sensors`, `simulation`, `cleaning`, `battery` | `mapping`, `localization`, `planning`, `navigation`, `autonomy` |

Exactly two things cross the wall each control tick (`core/messages.py`):

- a `SensorFrame`, from simulator to software;
- a `DriveCommand`, from software to simulator (later joined by other actuator
  commands such as vacuum on/off).

The software may also read its own *nominal* spec (`RobotSpec`: radius, wheel
base, speed limits), which is the information printed on a datasheet. From
Milestone 2 the simulator adds calibration errors that only it knows about.

**Enforcement.** `tests/test_architecture.py` parses every module's imports and fails if:

- a belief-side module imports a truth-side module, or vice versa;
- anything outside `visualization` imports pygame;
- subpackages import each other in a cycle;
- library code calls `print()` instead of using `logging`;
- a new subpackage has not been classified (forcing a deliberate decision).

Why so strict? In a simulator it is trivially easy to "cheat" by reading
`sim.robot.pose` from the robot's software. A robot that cheats looks great and
teaches nothing. Encoding the rule as a test means a cheat cannot slip in
during a refactor. (The rule was checked by planting a forbidden import: all
three checks fired.)

Deliberate, documented exceptions:

- The robot boots on its dock and knows it. That is how real robot vacuums fix
  their map origin, so the map frame's origin is the boot pose.
- The *harness* layer (`visualization`, `evaluation`, `app.py`) reads both sides,
  to draw belief next to truth and to score the robot. It never feeds truth back.

## 3. Package map

```
roombacalypse/
├── core/           shared    geometry, kinematics math, messages, Controller protocol,
│                             config schema + loader, RNG streams, paths
├── environment/    truth     shapes, World (outline, obstacles, dock), room files, placeable catalog
├── robot/          truth     drivetrain (motor limits), RobotBody (true state), Contact
├── simulation/     truth     clock, contact solver, Simulation engine
├── sensors/        truth     (M2) LiDAR, bumper, wheel encoders; (M5) battery gauge, dirt sensor, dock beacon
├── cleaning/       truth     (M5) dirt field, vacuum pickup, ground-truth cleaning metrics
├── battery/        truth     (M5) battery cell and power model
├── mapping/        belief    (M3) occupancy grid, inverse sensor model
├── localization/   belief    (M3) particle filter
├── planning/       belief    (M4) costmap, A*, coverage planner
├── navigation/     belief    (M2) DWA local planner; (M4) path following; (M5) docking
├── autonomy/       belief    teleop (M1); (M2+) executive state machine, behaviours, recovery, energy manager
├── personality/    observer  (M6) emotions, appraisal, narrator, incident reports
├── evaluation/     harness   (M3+) metrics comparing belief with truth, scenario runner
├── visualization/  harness   pygame viewer (the only pygame importer)
└── app.py          harness   command line + composition root
```

Allowed imports: `core` imports nothing internal. Truth packages import `core`
and truth packages. Belief packages import `core` and belief packages. The
observer imports only `core`. The harness imports anything. The simulator calls
the robot's software through the `Controller` protocol in `core/interfaces.py`,
and only `app.py` knows both concrete classes. That is plain dependency
inversion, so neither side imports the other.

**Deviation from the brief's suggested layout (decision D1).** The subsystems
live inside one `roombacalypse` package instead of fourteen top-level packages.
Top-level names like `robot`, `simulation` or `planning` collide with other
installed packages and pollute the import namespace. One package with
subpackages is the standard Python layout and costs nothing. `config/`, `docs/`
and `tests/` stay at the repository root (`assets/` will join them when there
are assets).

## 4. One simulation step

```
every dt (default 20 ms)
 ├─ control tick due? (default every 100 ms)
 │    sense():            ground truth (+ noise from M2) → SensorFrame
 │    controller.update:  SensorFrame → DriveCommand        (held until the next tick)
 ├─ drivetrain:      wheel speeds move toward the command within speed/acceleration limits
 ├─ kinematics:      exact arc integration of each wheel's rolling distance
 ├─ contact solver:  resolve penetration, push movable obstacles, record contacts
 └─ body + clock:    pose, velocities, encoder truth, odometer; tick += 1
```

Planned additions: sensor sampling at each sensor's own rate and drivetrain
imperfections (M2); battery drain and vacuum pickup (M5); event publication
for statistics and personality (M2+).

## 5. Time, rates and determinism

- Fixed physics step `dt` (20 ms). Time is `tick × dt`, so floating-point error never accumulates.
- Every periodic process runs at an integer multiple of `dt` (validated). Control
  runs at 10 Hz; planned rates are LiDAR 5 Hz, encoders every step, battery gauge 1 Hz.
- Commands are held between control ticks (zero-order hold), as on a real microcontroller.
- Rendering is decoupled. The viewer advances the simulation with a
  fixed-timestep accumulator, and the simulation never knows about frames.
- Randomness comes from `RngStreams(seed).get(name)`: independent named
  generators derived from the root seed and a hash of the name. Adding a noisy
  subsystem never changes the random numbers an existing one sees.
- **Guarantee (tested):** the same config, room, seed and command sequence give
  a bit-identical trajectory. Interactive input is not recorded yet; input
  replay logs are on the roadmap.

## 6. Frames and units

- SI everywhere (m, s, rad, kg, W, J). Config keys ending in `_deg` are degrees, converted on load.
- **World frame W:** x right, y up, origin defined by the room file. θ is
  counter-clockwise from +x, wrapped to [−π, π).
- **Body frame B:** x forward, y left (the ROS REP-103 convention).
- **Map frame M** (belief, M3): origin at the robot's boot pose on the dock. The
  transform W←M is known only to the simulator; the viewer uses it to overlay
  belief on truth.
- **Screen frame S:** pixels, y down. Only `visualization/camera.py` converts W↔S.
- **Grid indices** (M3): `(row, col) = (iy, ix)`; cell centres at `origin + (i + 0.5) · resolution`.

## 7. World model

- **Floor:** a simple polygon, possibly concave (the default studio is
  L-shaped). Its edges are walls of unlimited height and zero thickness. The
  renderer draws them thick, outside the floor.
- **Obstacles:** convex polygons or circles. Each has a vertical band
  `[z_min, z_max)`, a kind (`soft`, `leg`, `clutter`, …), a mobility
  (0 = fixed … 1 = yields completely when pushed) and an optional furniture group.
  Concave furniture is several convex parts.
- **Why 2.5D.** Height decides what the body collides with (0–0.092 m) and,
  from M2, what the LiDAR sees (scan plane at 0.095 m). One mechanism
  reproduces several classic robot-vacuum situations:
  - driving under beds and table tops, but not through their legs;
  - socks and cables invisible to the LiDAR but felt by the bumper;
  - later, furniture whose clearance lies between body height and turret
    height (the wedging trap).
- **Room files:** strict TOML with furniture macros (`table` = legs + top,
  `chair` = legs + seat + backrest). See `config/rooms/studio.toml`.
- **Mutable:** movable obstacles move when pushed, and the user can add or
  remove obstacles at run time. `World.version` increments on every change so
  caches (the renderer's, and from M2 the raycaster's) know when to rebuild.
- **Why continuous geometry instead of a grid world (D2).** The robot's
  occupancy grid (M3) must be *inferred* from noisy measurements. In a grid
  world, the map would equal the world and raycasting would be trivial.

## 8. Robot physical model (implemented, M1)

- Round body (r = 0.17 m), differential drive (wheel base 0.235 m), wheel limits
  0.5 m/s and 1 m/s², roughly a Roomba.
- **Drivetrain:** curvature-preserving speed saturation and jointly scaled
  acceleration limits.
- **Integration:** each wheel's exact rolling distance within the step
  (trapezoid rule), then an exact constant-curvature arc.
- **Contacts:** a purpose-built projection solver with quasi-static pushing.
- **Encoder truth:** wheels keep turning when the body is blocked (they slip).
  This is the realistic way odometry goes wrong, and Milestone 3's localization
  has to cope with it.
- **(planned, M2)** per-robot calibration errors (wheel radius mismatch, wheel
  base error) and random slip, known only to the simulator.

Details: [`algorithms/differential-drive.md`](algorithms/differential-drive.md),
[`algorithms/collision.md`](algorithms/collision.md).

## 9. Sensors (planned, M2 and M5)

| Sensor | Model | Rate | Noise and limits |
|---|---|---|---|
| LiDAR (M2) | 2D raycast at 0.095 m against walls and obstacles whose band contains that height; NumPy-vectorized ray–segment and ray–circle intersection | 5 Hz | configurable resolution (1°) and range (6 m); Gaussian range noise, dropouts; rotation distortion ignored |
| Bumper (M2) | front 180° arc split into left/right switches, from body contacts | every step | no rear bumper, so reversing into things is felt by nothing (realistic) |
| Wheel encoders (M2) | ticks from each wheel's rolling distance | every step | quantization; truth-side calibration error and slip |
| Battery gauge (M5) | state of charge and voltage | 1 Hz | noise, 1 % quantization |
| Dirt sensor (M5) | acoustic debris detector: Poisson count of particles picked up | every step | detects only what is under the robot |
| Dock beacon (M5) | infrared bearing to the dock inside a cone, with line of sight | 10 Hz | bearing noise |

Each sensor is a small class with `sample(...) -> Reading`, testable alone.
Readings are immutable dataclasses in `core/messages.py`, the only place both
sides can see.

## 10. Autonomy stack (planned, M2–M5)

### 10.1 Behaviour architecture

The executive is an explicit finite-state machine (`autonomy/executive.py`):

```
DOCKED/CHARGING → UNDOCKING → CLEANING { EXPLORE | COVER | SPOT_CLEAN }
CLEANING → RETURNING → DOCKING → CHARGING → RESUMING → CLEANING … → DONE
any state → RECOVERING → back to previous state | TRAPPED
any state ⇄ MANUAL (user override)
```

Planners and controllers below it are plain objects. Planners return paths and
the local planner returns `DriveCommand`s. The state machine only decides which
goal is active. Every transition publishes an event used by logs, statistics
and the personality.

### 10.2 Mapping and localization (M3; see decision D7)

- **Occupancy grid:** log-odds cells (5 cm). The inverse sensor model traces
  free space along each beam with vectorized sampling, de-duplicated per scan.
  Log-odds are clamped so moved furniture can be forgotten.
- **Localization:** Monte Carlo localization, a particle filter. It uses
  the odometry motion model (Thrun, Burgard & Fox, *Probabilistic Robotics*,
  §5.4) and a likelihood-field sensor model on a distance transform
  (`scipy.ndimage.distance_transform_edt`). Low-variance resampling is triggered
  by the effective sample size. The pose estimate is the weighted mean, with a
  circular mean for θ.
- **SLAM coupling (recommended):** the filter localizes against the robot's own
  incrementally built map, and the map is updated with the estimated pose
  whenever the filter is confident. Section 16, D7, gives the options.
- **Evaluation:** absolute trajectory error against ground truth, and map
  precision/recall against a rasterization of the true world.

### 10.3 Planning (M4)

- **Costmap:** occupied cells are inflated by robot radius plus a safety margin
  (lethal). Beyond that, cost decays exponentially, which keeps paths away from
  furniture. Unknown cells are untraversable except as frontier targets.
- **A\*:** 8-connected grid, octile heuristic, no corner cutting, cost-weighted.
  Paths are pruned by line-of-sight on the costmap.
- **Coverage:** online boustrophedon motion with A\* backtracking (BA\*,
  Viet et al., 2013) on a coarse coverage grid sized to the cleaning width.
  When no reachable uncovered cell remains, frontier exploration takes over
  (Yamauchi, 1997). Coverage is done when neither uncovered reachable cells nor
  frontiers remain.

### 10.4 Navigation (M2, M4, M5)

- **Local planner: Dynamic Window Approach** (Fox, Burgard & Thrun, 1997). It
  samples (v, ω) pairs reachable within one control period under the
  drivetrain's limits, simulates short arcs, discards those that hit LiDAR
  points, and scores the rest on heading, clearance and speed. It honours the
  same limits as the motors, so its commands are physically achievable.
- **M2** drives DWA with wander goals: reactive obstacle avoidance, no map.
  **M4** feeds it lookahead points along the A\* path.
- **Docking (M5):** plan to a pre-dock pose in the map frame, then servo on the
  infrared beacon's bearing for the final approach.

### 10.5 Recovery (M4)

Independent, testable detectors:

- bump rate: N bumps within T seconds;
- no progress: displacement below d over T seconds while motion is commanded;
- navigation loops: revisiting the same cells, or the sign of ω flipping repeatedly;
- plan failure: no path to any target;
- wheel slip: encoders say moving, LiDAR scan matching says not.

Escalating strategies:

1. replan with the goal blacklisted;
2. back up, if the LiDAR shows the rear is clear;
3. rotate 360° to re-scan and re-localize;
4. escape manoeuvre;
5. give up: enter `TRAPPED` and call for help.

### 10.6 Energy management (M5)

The robot goes home when the estimated energy to reach the dock approaches the
remaining energy. The estimate is A\* path length × learned energy per metre ×
safety factor, plus a reserve. There is no fixed battery-percentage threshold.
After charging it resumes from the stored coverage state.

## 11. Cleaning and battery (planned, M5)

- **Dirt:** a density field (grams per 2 cm cell) in the world frame (decision D10).
  Patches come from room files or are spawned with a right-click, and are
  rendered as speckles.
- **Pickup:** cells under the cleaning head lose a fraction `1 − exp(−k·t_under)`
  per pass, so slower passes clean better. The bin has a capacity.
- **Metrics (truth):** total, collected and remaining dirt; coverage of the
  *accessible* floor, meaning cells the footprint can reach (flood fill);
  efficiency in grams per Wh and coverage per minute; time spent cleaning.
- **Battery:** energy-based cell (Wh) with an open-circuit voltage curve and a
  charging taper above 80 %. Power draw is base electronics + LiDAR + motors
  (speed, acceleration, stall) + vacuum + brushes. It is calibrated to roughly
  the 75–90 minute runtime of real robots.

## 12. Personality (planned, M6)

- **Observer only.** It subscribes to events (bump, stuck, recovered, new area,
  dirt found, battery low, docked, …) and never feeds back into robotics logic.
  The architecture test enforces this: `personality` may import only `core`.
- **State:** frustration, confidence, curiosity and battery anxiety, each a
  leaky integrator `x ← x + (baseline − x)·dt/τ + Σ impulses`, clamped to [0, 1].
- **Appraisal:** a table (data, not code) maps events to impulses.
- **Narrator:** template pools keyed by event and mood, chosen deterministically
  from its own RNG stream, with cooldowns against spam. Incident reports are
  written for notable episodes: stuck episodes, collision streaks, near-empty battery.
- No LLM and no network access.

## 13. Visualization

- **Implemented:**
  - pygame viewer with a world view and a status panel;
  - fixed-timestep accumulator; speed 0.125×–64×; pause, single step and reset;
  - obstacle placement and removal with a placement preview;
  - zoom and pan; ground-truth trail; hover tooltips; help overlay; screenshots;
  - `--screenshot` renders offscreen through SDL's dummy driver (used by tests).
- **Layering follows height:** floor, then collidable obstacles, then the
  robot, then overhead obstacles drawn translucent, so the robot stays visible
  under a table.
- **(planned)** overlays:
  - LiDAR rays (M2);
  - occupancy grid and particles (M3);
  - path, costmap and coverage (M4);
  - dirt and battery (M5);
  - personality panel and message log (M6).

  Truth and belief will be drawn side by side, for example the true pose next
  to the estimated pose with its uncertainty.

## 14. Configuration

- Frozen dataclasses in `core/config.py` are the schema and the defaults.
- `config/default.toml` documents every value; a test keeps it equal to the code.
- Layers: defaults ← `default.toml` ← `--config` files ← `--set key=value` ← `--room`/`--seed`.
- Validation is strict: unknown keys, wrong types and inconsistent values
  (for example a timestep that would let the robot tunnel through a chair leg)
  are errors, not warnings.
- Room layouts are separate files in `config/rooms/`.

## 15. Testing strategy

- **Unit tests** per algorithm: geometry, kinematics, drivetrain, shapes,
  contact solver, config, rooms, camera.
- **Property tests:** 60 s of random driving in the furnished studio never
  penetrates anything and never leaves the floor.
- **Determinism test:** a scripted studio tour gives identical trajectories across runs.
- **Architecture tests:** see §2.
- **Viewer smoke tests,** rendered offscreen.
- **(planned) Regression scenarios with metric thresholds,** per milestone:
  - M2: zero penetrations and a bounded bump rate over 10 minutes of autonomous wandering;
  - M3: absolute trajectory error and map precision/recall;
  - M4: coverage of the accessible floor;
  - M5: multi-charge missions complete.

  Thresholds rather than golden numbers, so that intentional physics changes do
  not break every test.
- **(planned, M7) Benchmarks:** headless runs over seeds and rooms, with
  matplotlib plots.

## 16. Decision log

| # | Decision | Alternatives considered | Why | Cost to reverse |
|---|---|---|---|---|
| D1 | One `roombacalypse` package with subpackages | 14 top-level packages (the brief's layout) | avoids namespace collisions; standard packaging | low now, high later |
| D2 | Continuous 2.5D geometry: polygons, circles, height bands | grid world; pure 2D | maps must be inferred; height gives under-furniture and invisible-clutter cases | high |
| D3 | Wall of truth enforced by tests | convention only | cheating is the default failure mode of robot simulators | low |
| D4 | Fixed `dt`, integer-multiple rates, named RNG streams | variable `dt`; one global RNG | determinism and stable regression tests | medium |
| D5 | Frozen dataclasses + TOML (stdlib `tomllib`) | pydantic, YAML, JSON | typed and strict with no new dependency | low |
| D6 | Purpose-built contact solver: circle robot, projection, quasi-static pushing | pymunk / Box2D | slow robots, determinism, explainability, no dependency | medium |
| D7 | **(to confirm before M3)** MCL against the self-built map ("scan-to-map" SLAM, no loop closure) | Rao-Blackwellized particle filter (GMapping-style); pose-graph SLAM (Karto/Cartographer-style) | apartment scale with a 360° LiDAR that sees most walls most of the time; textbook parts; fast in NumPy | medium, if hidden behind a `Slam` interface |
| D8 | (M2) DWA local planner | vector field histogram; pure pursuit with a safety stop | honours velocity and acceleration limits natively; reused in M4 | low |
| D9 | (M4) BA\* plus frontier exploration for coverage | cellular decomposition on a known map; spanning-tree coverage | works online while the map is still being built; designed for cleaning robots | medium |
| D10 | (M5) Dirt as a density grid | individual particles | scales, simple pickup math, still renders as particles | low |
| D11 | (M6) Personality is an observer only | personality biasing behaviour | the brief requires separation; keeps robotics testable | low |
| D12 | `pygame-ce` instead of upstream `pygame` | upstream `pygame` | upstream has no wheels for Python ≥ 3.14; same API, same `import pygame` | low |
| D13 | MIT licence | GPL, Apache-2.0 | permissive default for an educational project; **the director should confirm** | low |

### D7 in more detail

Running mapping and localization at the same time *is* SLAM, the hardest
problem in the brief. The options:

- **(a) MCL against the robot's own growing map** (recommended for M3). It uses
  one shared occupancy grid: the particle filter localizes against it, and each
  scan is inserted at the estimated pose. This is close in spirit to Hector
  SLAM, with a particle filter instead of Gauss–Newton scan matching.
  - *For:* simple, fast, every part is textbook.
  - *Against:* no loop closure. An error baked into the map is never corrected.
    It works because an apartment is small and a 360° LiDAR keeps seeing the
    same walls.
- **(b) Rao-Blackwellized particle filter (GMapping-style).** Each particle
  carries its own map.
  - *For:* proper SLAM that can close loops.
  - *Against:* N maps updated per scan is heavy in Python, and it is much harder to debug.
- **(c) Pose-graph SLAM with scan matching (Karto/Cartographer-style).**
  - *For:* the state of the art.
  - *Against:* needs a nonlinear least-squares back end; overkill for one apartment.

Plan: implement (a) behind a `Slam.update(odometry, scan) -> (pose, grid)`
interface so that (b) can replace it later. A "known-pose" diagnostic mode,
where mapping uses ground truth, will exist **only** in tests and debug
visualization, to separate mapper bugs from localizer bugs. It will never be
an operating mode of the robot.
