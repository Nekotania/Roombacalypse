# ROOMBACALYPSE architecture

> **Status:** living document, revised at every milestone. **Milestones 1 and 2
> are implemented**; sections about later milestones describe the *intended*
> design and are marked **(planned, Mx)**. They will be rewritten from the
> actual implementation when that milestone is built and tested.

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
| Knows | exact geometry, true pose, true calibration errors, true dirt, true charge | only what its sensors reported |
| Packages | `environment`, `robot`, `sensors`, `simulation`, `cleaning`, `battery` | `mapping`, `localization`, `planning`, `navigation`, `autonomy` |

Exactly two things cross the wall each control tick (`core/messages.py`):

- a `SensorFrame`, from simulator to software: wheel encoder ticks, the
  latched bumper switches, and the latest LiDAR scan when a new one has completed;
- a `DriveCommand`, from software to simulator. Later milestones add other
  actuator commands, such as vacuum on/off.

The software may also read its own *nominal* spec (`RobotSpec`: radius,
wheel base, speed limits, encoder resolution, LiDAR mounting), which is the
information printed on a datasheet. The simulator secretly gives each robot
instance slightly different *true* wheel radii and wheel base (§8) that the
software never sees.

Both sides may *write* to the event log (`core/events.py`), for statistics and
the future personality: the simulator reports collisions, the robot reports
bumps and behaviour changes. The robot's software receives only a publish
function, never the log, so it cannot read what the simulator wrote there.

**Enforcement.** `tests/test_architecture.py` parses every module's imports and fails if:

- a belief-side module imports a truth-side module, or vice versa;
- anything outside `visualization` imports pygame;
- subpackages import each other in a cycle;
- library code calls `print()` instead of using `logging`;
- a module imports the composition root (`app.py`);
- a new subpackage has not been classified (forcing a deliberate decision).

Why so strict? In a simulator it is trivially easy to "cheat" by reading
`sim.robot.pose` from the robot's software. A robot that cheats looks great and
teaches nothing. Encoding the rule as a test means a cheat cannot slip in
during a refactor. (The rule was checked by planting a forbidden import: all
three checks fired.)

Deliberate, documented exceptions:

- The robot boots on its dock and knows it. That is how real robot vacuums fix
  their map origin, so the odometry and map frames start at the boot pose.
- The *harness* layer (`visualization`, `evaluation`, `app.py`) reads both sides,
  to draw belief next to truth and to score the robot. It never feeds truth back.

## 3. Package map

```
roombacalypse/
├── core/           shared    geometry (+ vectorised helpers), kinematics math, messages,
│                             events, Controller protocol, config, RNG streams, paths
├── environment/    truth     shapes, World (outline, obstacles, dock), room files, placeable catalog
├── robot/          truth     drivetrain (motor limits), wheel/ground error model, body state
├── sensors/        truth     LiDAR (ray casting + noise), bumper, wheel encoders      [M2]
│                             (M5) battery gauge, dirt sensor, dock beacon
├── simulation/     truth     clock, contact solver, Simulation engine
├── cleaning/       truth     (M5) dirt field, vacuum pickup, cleaning metrics
├── battery/        truth     (M5) battery cell and power model
├── mapping/        belief    (M3) occupancy grid, inverse sensor model
├── localization/   belief    wheel odometry [M2]; (M3) particle filter
├── planning/       belief    (M4) costmap, A*, coverage planner
├── navigation/     belief    obstacle memory, DWA local planner [M2]; (M4) path following; (M5) docking
├── autonomy/       belief    RobotBrain, teleop, wander behaviour [M2]; (M4) executive, recovery
├── personality/    observer  (M6) emotions, appraisal, narrator, incident reports
├── evaluation/     harness   ground-truth metrics, footprint coverage, headless scenarios [M2]
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
and `tests/` stay at the repository root.

## 4. One simulation step (implemented)

```
every dt (default 20 ms)
 ├─ LiDAR due? (every 200 ms)   ray-cast from the true pose, add noise → pending scan
 ├─ control due? (every 100 ms)
 │    SensorFrame(encoders, latched bumper, pending scan) → controller.update → DriveCommand
 │                                                          (held until the next tick)
 ├─ drivetrain:      wheel speeds move toward the command within speed/acceleration limits
 ├─ wheel/ground:    nominal wheel rotation → true ground travel (calibration error, slip)
 ├─ kinematics:      exact arc integration with the *true* wheel base
 ├─ contact solver:  resolve penetration, push movable obstacles, record contacts
 └─ bookkeeping:     body state and encoder truth, latch bumper switches,
                     emit COLLISION events, tick += 1
```

Planned additions: battery drain and vacuum pickup (M5); more events for the
personality (M4+).

## 5. Time, rates and determinism

- Fixed physics step `dt` (20 ms). Time is `tick × dt`, so floating-point error never accumulates.
- Every periodic process runs at an integer multiple of `dt` (validated):
  - control at 10 Hz;
  - LiDAR at 5 Hz;
  - encoders sampled at each control tick;
  - bumper latched every physics step;
  - (planned) battery gauge at 1 Hz.
- Commands are held between control ticks (zero-order hold), as on a real microcontroller.
- Rendering is decoupled. The viewer advances the simulation with a
  fixed-timestep accumulator, and the simulation never knows about frames.
- Randomness comes from `RngStreams(seed).get(name)`: independent named
  generators derived from the root seed and a hash of the name. Truth streams
  are `truth.calibration`, `truth.slip` and `sensors.lidar`; the robot's own
  decisions use `robot.wander`. Adding a noisy subsystem never changes the
  numbers an existing one sees, and the LiDAR draws a fixed amount of noise
  per sweep whatever it hits.
- **Guarantee (tested):** the same config, room, seed and command sequence give
  a bit-identical trajectory, with all noise sources enabled. Different seeds
  give different robots. Interactive input is not recorded yet; input replay
  logs are on the roadmap.

## 6. Frames and units

- SI everywhere (m, s, rad, kg, W, J). Config keys ending in `_deg` are degrees, converted on load.
- **World frame W:** x right, y up, origin defined by the room file. θ is
  counter-clockwise from +x, wrapped to [−π, π).
- **Body frame B:** x forward, y left (the ROS REP-103 convention). LiDAR
  beam angles are relative to the body x axis.
- **Odometry frame O** (belief): origin at the robot's boot pose on the dock.
  The robot's goals and remembered obstacles live here. The viewer draws
  belief in the world through the true boot pose, which is how drift becomes visible.
- **Map frame M** (belief, M3): the same origin as O, corrected by localization.
- **Screen frame S:** pixels, y down. Only `visualization/camera.py` converts W↔S.
- **Grids:** `(row, col) = (iy, ix)`; cell centres at `origin + (i + 0.5) · resolution`.

## 7. World model

- **Floor:** a simple polygon, possibly concave (the default studio is
  L-shaped). Its edges are walls of unlimited height and zero thickness. The
  renderer draws them thick, outside the floor.
- **Obstacles:** convex polygons or circles. Each has a vertical band
  `[z_min, z_max)`, a kind (`soft`, `leg`, `clutter`, …), a mobility
  (0 = fixed … 1 = yields completely when pushed) and an optional furniture group.
  Concave furniture is several convex parts.
- **Why 2.5D.** Height decides what the body collides with (0–0.092 m) and
  what the LiDAR sees (scan plane at 0.095 m). One mechanism reproduces
  several classic robot-vacuum situations:
  - driving under beds and table tops, but not through their legs (implemented);
  - socks and dog bowls invisible to the LiDAR but felt by the bumper
    (implemented: all studio contacts in the acceptance runs are these);
  - later, furniture whose clearance lies between body height and turret
    height (the wedging trap).
- **Room files:** strict TOML with furniture macros (`table` = legs + top,
  `chair` = legs + seat + backrest). See `config/rooms/studio.toml`.
- **Mutable:** movable obstacles move when pushed, and the user can add or
  remove obstacles at run time. `World.version` increments on every change so
  caches (renderer, LiDAR geometry) know when to rebuild.
- **Why continuous geometry instead of a grid world (D2).** The robot's
  occupancy grid (M3) must be *inferred* from noisy measurements. In a grid
  world, the map would equal the world and raycasting would be trivial.

## 8. Robot physical model (implemented)

- Round body (r = 0.17 m), differential drive (wheel base 0.235 m), wheel limits
  0.5 m/s and 1 m/s², roughly a Roomba.
- **Drivetrain:** curvature-preserving speed saturation and jointly scaled
  acceleration limits, in *nominal* wheel metres (rotation × nominal radius).
- **Hidden drive errors (M2):** each reset draws true wheel radii (std 0.2 %)
  and a true wheel base (std 1 %); a correlated Ornstein–Uhlenbeck slip
  (1 %, τ = 0.5 s) varies over time. These are the UMBmark systematic and
  random odometry errors.
- **Integration:** the trapezoid rule within a step, then an exact
  constant-curvature arc using the *true* wheel base.
- **Contacts:** a purpose-built projection solver with quasi-static pushing.
- **Encoders** count wheel rotation, so they keep counting when the body is
  blocked and the wheels slip.

Details: [`algorithms/differential-drive.md`](algorithms/differential-drive.md),
[`algorithms/collision.md`](algorithms/collision.md),
[`algorithms/odometry.md`](algorithms/odometry.md).

## 9. Sensors

| Sensor | Status | Model | Rate | Noise and limits |
|---|---|---|---|---|
| LiDAR | ✅ M2 | 2D ray cast at 0.095 m against walls and obstacles whose band contains that height; NumPy-vectorized ray–segment and ray–circle intersection | 5 Hz | 1° resolution, 6 m range; σ = 8 mm + 0.4 % of range; 1 % dropouts; sweep is instantaneous |
| Bumper | ✅ M2 | front 180° arc, left/right switches, both within ±15°; latched between control ticks | every step | no rear bumper |
| Wheel encoders | ✅ M2 | 508.8 ticks per revolution, counting wheel rotation | every control tick | quantization; true calibration error and slip happen upstream |
| Battery gauge | planned M5 | state of charge and voltage | 1 Hz | noise, 1 % quantization |
| Dirt sensor | planned M5 | acoustic debris detector: Poisson count of particles picked up | every step | detects only what is under the robot |
| Dock beacon | planned M5 | infrared bearing to the dock inside a cone, with line of sight | 10 Hz | bearing noise |

Details: [`algorithms/lidar.md`](algorithms/lidar.md), [`algorithms/odometry.md`](algorithms/odometry.md).

## 10. Autonomy stack

### 10.1 Robot software structure (implemented, M2)

`autonomy/brain.py` holds one `RobotBrain` per robot lifetime. It implements
the `Controller` protocol and owns everything the robot *knows*:

- dead-reckoned odometry;
- the obstacle memory: the latest scan, plus recent bump locations;
- bump edge detection;
- the robot's own seeded random stream.

It delegates *command generation* to the active mode: manual teleoperation,
or the autonomous behaviour. Switching modes (Tab) never loses the robot's
sense of where it has been (tested).

**M2 behaviour: map-free exploration** (`autonomy/wander.py`). A small state
machine on top of DWA:

- WAIT for the first scan;
- CRUISE toward exploration goals scored by free distance, novelty
  (dead-reckoned visit counter) and randomness;
- the cramped-corner hop fallback;
- BACK OFF (only if the LiDAR shows the rear is clear) and TURN after a bump.

See [`algorithms/dwa.md`](algorithms/dwa.md).

**(planned, M4)** The executive state machine replaces "wander":

```
DOCKED/CHARGING → UNDOCKING → CLEANING { EXPLORE | COVER | SPOT_CLEAN }
CLEANING → RETURNING → DOCKING → CHARGING → RESUMING → CLEANING … → DONE
any state → RECOVERING → back to previous state | TRAPPED
any state ⇄ MANUAL (user override)
```

### 10.2 Mapping and localization (planned, M3; see decision D7)

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
  whenever the filter is confident.
- **Evaluation:** absolute trajectory error against ground truth (M2 already
  measures the dead-reckoning baseline: up to metres and tens of degrees per
  10 minutes), and map precision/recall against a rasterization of the true world.

### 10.3 Planning (planned, M4)

- **Costmap:** occupied cells are inflated by robot radius plus a safety margin
  (lethal). Beyond that, cost decays exponentially, which keeps paths away from
  furniture. Unknown cells are untraversable except as frontier targets.
- **A\*:** 8-connected grid, octile heuristic, no corner cutting, cost-weighted.
  Paths are pruned by line-of-sight on the costmap.
- **Coverage:** online boustrophedon motion with A\* backtracking (BA\*,
  Viet et al., 2013) on a coarse coverage grid sized to the cleaning width.
  When no reachable uncovered cell remains, frontier exploration takes over
  (Yamauchi, 1997).

### 10.4 Navigation

- **Local planner: Dynamic Window Approach (implemented, M2).**
  - The window is sampled in wheel-speed space, so it is exactly what the
    motors can reach.
  - Arcs are predicted with the real acceleration ramp.
  - A hard collision radius relaxes in two stages so the robot cannot deadlock
    inside its own safety margin.
  - Scoring is progress, mean clearance, an intrusion penalty, speed and a
    heading tie-breaker.

  Closed-loop tests with the real simulator show it detouring around obstacles
  and stopping at walls without contact. Two versions of the cost function
  were rejected because those tests failed; see the algorithm note.
- **(planned, M4)** lookahead points along the A\* path become DWA's goal.
- **(planned, M5)** docking: plan to a pre-dock pose in the map frame, then
  servo on the infrared beacon's bearing for the final approach.

### 10.5 Recovery (planned, M4)

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

M2's wander behaviour already has goal stall, timeout and blocked detection,
plus the bump reflex.

### 10.6 Energy management (planned, M5)

The robot goes home when the estimated energy to reach the dock approaches the
remaining energy. The estimate is A\* path length × learned energy per metre ×
safety factor, plus a reserve. There is no fixed battery-percentage threshold.

## 11. Cleaning and battery (planned, M5)

- **Dirt:** a density field (grams per 2 cm cell) in the world frame (D10).
  Patches come from room files or are spawned with a right-click.
- **Pickup:** cells under the cleaning head lose `1 − exp(−k·t_under)` per
  pass, so slower passes clean better. The bin has a capacity.
- **Metrics (truth):** reuse `evaluation/coverage.py`, which already measures
  footprint coverage of the *accessible* floor; add dirt collected and remaining,
  efficiency, and time spent cleaning.
- **Battery:** energy-based cell (Wh) with an open-circuit voltage curve and a
  charging taper above 80 %. Power draw is base electronics + LiDAR + motors
  (speed, acceleration, stall) + vacuum + brushes. It is calibrated to roughly
  the 75–90 minute runtime of real robots.

## 12. Personality (planned, M6)

- **Observer only.** It consumes the event log, which already carries
  collisions (simulator) and bumps, goals, escapes and mode changes (robot),
  and never feeds back into robotics logic. The architecture test enforces
  this: `personality` may import only `core`.
- **State:** frustration, confidence, curiosity and battery anxiety, each a
  leaky integrator `x ← x + (baseline − x)·dt/τ + Σ impulses`, clamped to [0, 1].
- **Appraisal:** a table (data, not code) maps events to impulses.
- **Narrator:** template pools keyed by event and mood, chosen deterministically
  from its own RNG stream, with cooldowns against spam. Incident reports are
  written for notable episodes. No LLM and no network access.

## 13. Visualization (implemented, extended per milestone)

- **Viewer:** a pygame window with a world view and a status panel.
- **Panel sections:** simulation; control (mode, behaviour state, goal);
  ground truth; *sensors, as the robot gets them*; *belief* (dead-reckoning
  drift from truth); statistics (distance, live footprint coverage, bumps
  versus contacts); a colour-coded event log.
- **Layers:**
  - L: LiDAR rays and hits, drawn from the true pose at scan time;
  - G: navigation: DWA candidate arcs (admissible and rejected), the chosen
    arc, the exploration goal;
  - E: belief, a purple ghost where odometry thinks the robot is;
  - T: the ground-truth trail.
- **Layering follows height:** floor, then collidable obstacles, then the
  robot, then translucent overhead obstacles, then the overlays.
- **Controls:** Tab (manual/autonomous), pause, single step, speed
  0.125×–64×, reset, obstacle placement and removal with a preview, zoom/pan,
  tooltips, help, screenshots.
- **Offscreen use:** `--screenshot` renders through SDL's dummy driver (used by
  tests); `--headless` skips rendering entirely and prints a scorecard.

## 14. Configuration

- Frozen dataclasses in `core/config.py` are the schema and the defaults.
- `config/default.toml` documents every value; a test keeps it equal to the code.
- Layers: defaults ← `default.toml` ← `--config` files ← `--set key=value` ← `--room`/`--seed`.
- Validation is strict: unknown keys, wrong types and inconsistent values are
  errors, not warnings. Examples: a timestep that would let the robot tunnel
  through a chair leg; a LiDAR mounted below the top of the body; a DWA speed
  above the wheel limit.
- Room layouts are separate files in `config/rooms/`.

## 15. Testing strategy

- **Unit tests** per algorithm:
  - geometry, kinematics (scalar and vectorized), drivetrain, shapes;
  - contact solver, config, rooms, camera;
  - LiDAR (analytic distances, height bands, noise statistics, dropouts);
  - bumper, encoders, the drive-error model (slip statistics, predicted curvature);
  - odometry, DWA (window, admissibility property test), obstacle memory;
  - the wander state machine, the event log, footprint coverage.
- **Closed-loop tests:** DWA with the real simulator, LiDAR and odometry around
  a pillar and at a wall, plus a sock the LiDAR cannot see.
- **Property tests:**
  - 60 s of random driving never penetrates anything;
  - DWA's chosen arc is always admissible, over 200 random scenes.
- **Determinism tests:** bit-identical runs with all noise enabled; different
  seeds give different robots.
- **Architecture tests:** see §2.
- **Viewer and CLI smoke tests,** rendered offscreen, with every key binding exercised.
- **Acceptance suite** (`pytest -m slow`, about 70 s): 10 simulated minutes of
  autonomy × 5 seeds × 2 rooms. It asserts:
  - no penetration;
  - zero contacts with anything the LiDAR can see;
  - no stalls (at least 120 m driven);
  - bump-rate ceilings;
  - coverage floors.

  Thresholds rather than golden numbers, set from measured runs with margin.
- **(planned, M7) Benchmarks:** seeds × rooms with matplotlib plots.

## 16. Decision log

| # | Decision | Alternatives considered | Why | Cost to reverse |
|---|---|---|---|---|
| D1 | One `roombacalypse` package with subpackages | 14 top-level packages (the brief's layout) | avoids namespace collisions; standard packaging | low now, high later |
| D2 | Continuous 2.5D geometry: polygons, circles, height bands | grid world; pure 2D | maps must be inferred; height gives under-furniture and invisible-clutter cases | high |
| D3 | Wall of truth enforced by tests | convention only | cheating is the default failure mode of robot simulators | low |
| D4 | Fixed `dt`, integer-multiple rates, named RNG streams | variable `dt`; one global RNG | determinism and stable regression tests | medium |
| D5 | Frozen dataclasses + TOML (stdlib `tomllib`) | pydantic, YAML, JSON | typed and strict with no new dependency | low |
| D6 | Purpose-built contact solver: circle robot, projection, quasi-static pushing | pymunk / Box2D | slow robots, determinism, explainability, no dependency | medium |
| D7 | **(to confirm before M3)** MCL against the self-built map ("scan-to-map" SLAM, no loop closure) | Rao-Blackwellized particle filter (GMapping-style); pose-graph SLAM | apartment scale with a 360° LiDAR; textbook parts; fast in NumPy | medium, if hidden behind a `Slam` interface |
| D8 | ✅ DWA local planner, window sampled in wheel space, ramp-aware prediction | vector field histogram; pure pursuit with a safety stop | honours velocity and acceleration limits natively; reused in M4 | low |
| D9 | (M4) BA\* plus frontier exploration for coverage | cellular decomposition on a known map; spanning-tree coverage | works online while the map is still being built | medium |
| D10 | (M5) Dirt as a density grid | individual particles | scales, simple pickup math, still renders as particles | low |
| D11 | (M6) Personality is an observer only | personality biasing behaviour | the brief requires separation; keeps robotics testable | low |
| D12 | `pygame-ce` instead of upstream `pygame` | upstream `pygame` | upstream has no wheels for Python ≥ 3.14; same API | low |
| D13 | ✅ MIT licence (confirmed by the director) | GPL, Apache-2.0 | permissive default for an educational project | low |
| D14 | ✅ One long-lived `RobotBrain` owns perception; modes only generate commands | separate controllers swapped on Tab | odometry and memory must survive mode switches, as on a real robot | low |
| D15 | ✅ Hidden per-robot calibration errors plus correlated (OU) slip | i.i.d. per-step noise; perfect wheels | i.i.d. noise averages away; UMBmark systematic errors dominate real odometry | low |
| D16 | ✅ Robot software writes events through a publish-only handle | no events from the robot; a shared readable log | the personality needs robot-side events; reading the log would leak truth | low |

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
an operating mode of the robot. M2 measured the dead-reckoning drift that M3
has to beat: up to 8.6 m and 153° after 10 minutes of wandering.
