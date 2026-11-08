# ROOMBACALYPSE architecture

> **Status:** living document, revised at every milestone. **Milestones 1–4
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
├── sensors/        truth     LiDAR (ray casting + noise), bumper, wheel encoders [M2], gyro [M4]
│                             (M5) battery gauge, dirt sensor, dock beacon
├── simulation/     truth     clock, contact solver, Simulation engine
├── cleaning/       truth     (M5) dirt field, vacuum pickup, cleaning metrics
├── battery/        truth     (M5) battery cell and power model
├── mapping/        belief    occupancy grid, surface fields, pose-graph optimiser [M3]
├── localization/   belief    wheel odometry [M2]; scan matching, graph SLAM [M3]; gyrodometry,
│                             global relocalisation [M4]
├── planning/       belief    costmap, A*, coverage planner [M4]
├── navigation/     belief    obstacle memory, DWA local planner [M2]; path following [M4]; (M5) docking
├── autonomy/       belief    RobotBrain, teleop, wander behaviour [M2]; executive, stuck detection,
│                             recovery ladder [M4]
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
- **Map frame M** (belief, M3): the same origin as O; SLAM's estimate is
  `pose_M = correction ∘ pose_O`, the `map → odom` split of ROS. Odometry stays
  smooth (obstacle memory, bump reflexes); the map frame jumps millimetres at
  keyframes and centimetres at loop closures (exploration goals, visit counter).
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
| Gyroscope | ✅ M4 | MEMS yaw-rate gyro measuring the true body rate after collisions, integrated into an angle | every step (read at 10 Hz) | scale error σ 0.3 %, bias σ 0.5 °/s plus a random walk, 0.01 °/s/√Hz noise, quantization |
| Battery gauge | planned M5 | state of charge and voltage | 1 Hz | noise, 1 % quantization |
| Dirt sensor | planned M5 | acoustic debris detector: Poisson count of particles picked up | every step | detects only what is under the robot |
| Dock beacon | planned M5 | infrared bearing to the dock inside a cone, with line of sight | 10 Hz | bearing noise |

Details: [`algorithms/lidar.md`](algorithms/lidar.md), [`algorithms/odometry.md`](algorithms/odometry.md).

## 10. Autonomy stack

### 10.1 Robot software structure (implemented, M2)

`autonomy/brain.py` holds one `RobotBrain` per robot lifetime. It implements
the `Controller` protocol and owns everything the robot *knows*:

- dead-reckoned odometry;
- SLAM (M3): the map, the pose graph and the map-frame pose, updated in every
  mode, so the map keeps growing while a human drives;
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

**M4 behaviour: the coverage executive** (`autonomy/executive.py`, the
default; `autonomy.behaviour = "wander"` brings M2's back). A state machine
over the coverage planner, the path follower and DWA:

```
WAIT (scan, gyro calibration) → UNDOCK → DRIVE ⇄ BACK OFF / TURN (bump reflex)
DRIVE → RECOVER (stuck: back up · spin · hop · elsewhere) → DRIVE | TRAPPED
any state → LOST (SLAM lost: stand still, hop for a new view) → DRIVE when found
DRIVE → DONE (no uncovered floor, no frontier)
```

M5 adds the energy-aware return to the dock and docking:

```
DOCKED/CHARGING → UNDOCK → CLEANING → RETURNING → DOCKING → CHARGING → RESUMING …
```

### 10.2 Mapping and localization (implemented, M3; decision D7)

Graph SLAM in the Karto / slam_toolbox / Cartographer family. Full write-up,
derivations and measured results: [`algorithms/slam.md`](algorithms/slam.md).

- **Front end.** Odometry predicts; a keyframe is added after 0.25 m or 15°, or
  as soon as a scan stops fitting the prediction (wheel slip). Its scan is
  matched against the surfaces of the last 20 keyframes with point-to-line ICP
  (a correlative search with branch and bound if the fit is poor), and joined
  to the previous keyframe by a constraint carrying the match's information
  plus the odometry noise model.
- **Back end.** Every 2 m, the newest keyframe is matched against an earlier
  visit nearby (correlative search ±0.5 m, ±12°). A strong, unambiguous match
  not on the window's edge becomes a loop-closure constraint with a Cauchy
  kernel, and the sparse pose graph is re-optimised (Levenberg-Marquardt, node 0
  fixed at the dock).
- **Map.** A 5 cm log-odds occupancy grid rendered from the keyframes:
  incrementally, and from scratch (lazily) after a loop closure moved them.
- **Evaluation.** Online and keyframe trajectory error against ground truth
  through the boot pose (no alignment; the aligned ATE is reported too), and
  map precision/recall against the true scan-plane geometry.
- **Lost and found (M4).** Three scans in a row that fit nowhere near the
  prediction: the robot declares itself lost (kidnapped). It then searches the
  whole occupancy grid for each new view (branch-and-bound correlative search,
  as in Cartographer), accepts only a strong, unambiguous match, and joins it
  to the graph with a relocalisation constraint.
- **Not done:** submaps for very long sessions, persistent maps, multi-session
  mapping (a robot set down somewhere unmapped stays lost until it sees a
  mapped place).

### 10.3 Planning (implemented, M4)

Full write-up and measured results: [`algorithms/planning.md`](algorithms/planning.md).

- **Costmap** from the SLAM grid every second: occupied cells and remembered
  bumps, inflated by robot radius plus DWA's safety margin (lethal), strips
  of passable cells one cell wide made lethal too (dead ends the robot fits
  by less than the map's resolution), cost decaying exponentially beyond,
  unknown cells passable at a premium.
- **A\*:** 8-connected grid, octile heuristic, no corner cutting, cost-weighted,
  tested optimal against Dijkstra. Coverage uses SciPy Dijkstra *fields* (one
  start, every cell). Paths are pruned by line of sight.
- **Coverage:** online BA\* (Viet et al., 2013): boustrophedon lanes along map
  x, shifts of one lane width, backtracking to the nearest rich uncovered
  area by path, leftover fringes last; frontier exploration (Yamauchi, 1997)
  when nothing uncovered is reachable; done when neither remains. "Stranded"
  (nowhere to plan from) is trouble, not done.

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
- **Path following (implemented, M4):** a point `lookahead` (0.5 m) along the
  path beyond the robot's projection on it becomes DWA's goal (pure pursuit),
  with a speed cap near the path's end and near obstacles.
- **(planned, M5)** docking: plan to a pre-dock pose in the map frame, then
  servo on the infrared beacon's bearing for the final approach.

### 10.5 Recovery (implemented, M4)

Independent, separately tested detectors (`autonomy/recovery.py`): cornered
(no admissible DWA arc for 1.5 s), no progress along the path, bump storm,
oscillation, wheel slip (odometry moves, the map pose does not), plus
"stranded" from the planner.

The ladder escalates while trouble recurs at the same place (0.5 m, 45 s)
and restarts on arrival or 1 m away:

1. back up 15 cm, if the LiDAR shows the rear is clear;
2. spin 360° to re-scan;
3. hop: drive on a heading towards the most open direction, checking only
   the bare body (Nav2's DriveOnHeading), for gaps the safety margin cannot
   get out of;
4. give the target up and keep out of the spot for 15 minutes;
5. `TRAPPED`: stop and call for help (unless a hop is still possible).

Relocalisation is not a rung: SLAM handles it by itself (§10.2), and the
executive's LOST state only stands still and hops for new views.

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
  - E: belief, a purple ghost where odometry thinks the robot is and a green
    one where SLAM thinks it is;
  - M: the robot's occupancy grid, drawn through the true boot pose, so map
    errors show as they are (on by default);
  - P: the pose graph, keyframes and (yellow) loop closures and relocalisations;
  - C: coverage, the floor the robot believes it swept (green) and what it
    still wants (orange), drawn through the true boot pose (M4);
  - G also shows the planned path (M4);
  - T: the ground-truth trail.
- **Layering follows height:** floor, then collidable obstacles, then the
  robot, then translucent overhead obstacles, then the overlays.
- **Controls:** Tab (manual/autonomous), pause, single step, speed
  0.125×–64×, reset, obstacle placement and removal with a preview, zoom/pan,
  tooltips, help, screenshots; K kidnaps the robot (to the cursor, or with
  Shift anywhere); V toggles the collision sound (M4: a built-in "bonk", or any
  WAV/OGG given with `--sound`).
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
  a pillar and at a wall, plus a sock the LiDAR cannot see. SLAM driven
  through the simulator: millimetre accuracy with perfect sensors, a 10×
  improvement over dead reckoning with a 3 % wheel-base error, loop closure
  on a driven loop, map precision/recall, determinism, slip.
- **SLAM unit tests:** grid updates (once per scan, clamping, growth), field
  values and exact surface distances, branch-and-bound bounds, pose-graph
  Jacobians and robust kernel, scan matching accuracy and its window and
  ambiguity flags, a degenerate corridor, the evaluation metrics themselves.
- **Property tests:**
  - 60 s of random driving never penetrates anything;
  - DWA's chosen arc is always admissible, over 200 random scenes.
- **Determinism tests:** bit-identical runs with all noise enabled; different
  seeds give different robots.
- **Architecture tests:** see §2.
- **Viewer and CLI smoke tests,** rendered offscreen, with every key binding exercised.
- **Acceptance suite** (`pytest -m slow`): 10 simulated minutes of autonomy ×
  5 seeds × 3 rooms. M2 asserts:
  - no penetration;
  - zero contacts with anything the LiDAR can see;
  - no stalls (at least 120 m driven);
  - bump-rate ceilings;
  - coverage floors.

  M3 adds SLAM pose error (online RMS and maximum, keyframe ATE) and map
  precision and recall. These runs use M2's wander behaviour, so the M2 and M3
  criteria keep meaning what they meant. M4 adds 30 simulated minutes of the
  coverage executive × 5 seeds × 3 rooms: coverage of the accessible floor,
  no penetration, zero visible contacts, the M3 SLAM criteria, and "done" (not
  trapped) where the room allows. Thresholds rather than golden numbers, set
  from measured runs with margin.
- **M4 closed-loop tests:** room coverage end to end, a crate pen (trapped or
  done, never a collision), every stuck detector on its own trouble, the
  escape hop in a noisy 40 cm gap, the global search against brute force, a
  symmetric room reported ambiguous, and a kidnapped robot noticing, finding
  itself and carrying on.
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
| D7 | ✅ Graph SLAM: scan-matching front end, pose graph with loop closure (Karto / slam_toolbox / Cartographer family). Chosen by the agent; the director delegated it ("something that works in actual robotics") | MCL against the self-built map (the M2 recommendation); Rao-Blackwellized particle filter (GMapping) | can correct mapped errors; what real LiDAR robots run; scales to the multi-room apartment | medium: behind the `Slam` protocol |
| D8 | ✅ DWA local planner, window sampled in wheel space, ramp-aware prediction | vector field histogram; pure pursuit with a safety stop | honours velocity and acceleration limits natively; reused in M4 | low |
| D9 | ✅ (M4) Online BA\* plus frontier exploration for coverage, re-assessed on the growing SLAM map | cellular decomposition on a known map; spanning-tree coverage | works online while the map is still being built | medium |
| D10 | (M5) Dirt as a density grid | individual particles | scales, simple pickup math, still renders as particles | low |
| D11 | (M6) Personality is an observer only | personality biasing behaviour | the brief requires separation; keeps robotics testable | low |
| D12 | `pygame-ce` instead of upstream `pygame` | upstream `pygame` | upstream has no wheels for Python ≥ 3.14; same API | low |
| D13 | ✅ MIT licence (confirmed by the director) | GPL, Apache-2.0 | permissive default for an educational project | low |
| D14 | ✅ One long-lived `RobotBrain` owns perception; modes only generate commands | separate controllers swapped on Tab | odometry and memory must survive mode switches, as on a real robot | low |
| D15 | ✅ Hidden per-robot calibration errors plus correlated (OU) slip | i.i.d. per-step noise; perfect wheels | i.i.d. noise averages away; UMBmark systematic errors dominate real odometry | low |
| D16 | ✅ Robot software writes events through a publish-only handle | no events from the robot; a shared readable log | the personality needs robot-side events; reading the log would leak truth | low |
| D17 | ✅ `map → odom` split: SLAM corrects odometry; goals live in the map frame, reflexes and obstacle memory in the odometry frame | everything in one frame | the map frame jumps at loop closures; local control must not | low |
| D18 | ✅ Scan matching by point-to-line ICP on fitted surface elements, correspondences from a distance-transform feature map; correlative search only for the mode | likelihood-field Gauss-Newton (Hector, Cartographer); NDT | measured: the likelihood field limited a match to ±1 cell and drifted in heading | low |
| D19 | ✅ Map precision counts phantom obstacles in free space; thickening into solid matter is allowed up to 3 cells; the strict number is reported too | precision within one cell of a surface | every occupancy grid thickens surfaces into solid matter, where beams cannot clear it; that is harmless | low |
| D20 | ✅ (M4) A MEMS gyro; heading from the gyro, distance from the wheels (gyrodometry), bias calibrated on the dock and at every standstill | wheels only; an EKF fusing both | the director said yes; what every robot vacuum does; substitution is simple and measured 5–10× less dead-reckoning drift | low |
| D21 | ✅ (M4) The coverage executive is the default behaviour; M2's wander stays selectable and keeps the M2/M3 acceptance runs meaningful | replace wander | the old acceptance numbers stay comparable; wander is a useful baseline | low |
| D22 | ✅ (M4) Global relocalisation inside SLAM (branch-and-bound correlative search on the occupancy grid, accepted only when unambiguous), not in the recovery ladder | Monte Carlo localisation with uniform re-seeding; a rung "rotate and re-localise" | exact optimum, cheap with branch and bound, no particles; the robot cannot know a kidnapping happened, so detection belongs where the scans are | low: one module |
| D23 | ✅ (M4) The collision sound ships as a synthesised "bonk"; any clip can be played from a local file (`--sound`), git-ignored under `sounds/` | bundling a well-known cartoon scream | copyrighted audio cannot go into an MIT repository | low |

### D7 in more detail

Running mapping and localization at the same time *is* SLAM, the hardest
problem in the brief. Before M3 the options on the table were:

- **(a) MCL against the robot's own growing map** (the M2 recommendation): one
  shared occupancy grid, a particle filter localises against it, and each scan
  is inserted at the estimated pose. Simple, but an error baked into the map is
  never corrected.
- **(b) Rao-Blackwellized particle filter (GMapping-style):** each particle
  carries its own map. Proper SLAM that can close loops, but heavy in Python and
  prone to particle depletion on long loops.
- **(c) Graph SLAM with scan matching (Karto/slam_toolbox/Cartographer-style).**

The director answered: "you choose. something that works in actual robotics,
not just simulations", and asked for a multi-room apartment. Option (c) was
chosen: it is what real LiDAR robots run, and it is the only one of the three
that corrects mapped errors without a particle filter's costs. The nonlinear
least-squares back end turned out to be small: a sparse Levenberg-Marquardt
in about 150 lines of NumPy/SciPy, no new dependency.

The design sits behind the `Slam` protocol (`localization/slam.py`), so another
SLAM can replace `GraphSlam`. A ground-truth "known-pose" mode exists only in
tests (`tests/test_slam_metrics.py`) and debugging scripts, never as robot
behaviour. Measured result: about 1 cm RMS in the furnished studio and 2–5 cm
in the apartment, against metres of dead-reckoning drift (ROADMAP, M3).
