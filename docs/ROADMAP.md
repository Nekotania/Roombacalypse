# Roadmap

Status as of 2026-11-07. Each milestone ends with: all tests green,
earlier functionality re-verified, documentation updated, and a written
report. A milestone is not "done" until its acceptance criteria are
demonstrated by tests or recorded runs.

| Milestone | Theme | Scope | Status |
|---|---|---|---|
| **M1** | *It moves* | room geometry, differential-drive robot, collisions, manual control, viewer | ✅ done |
| **M2** | *It sees* | LiDAR, bumper, encoders, drive errors, DWA obstacle avoidance, autonomous exploration, headless runner, events | ✅ done |
| **M3** | *It knows where it is* | graph SLAM (scan matching, pose graph, loop closure), occupancy grid, multi-room apartment, map/pose metrics | ✅ done |
| **M4** | *It has a plan* | gyro, costmap, A\*, coverage planning, executive state machine, stuck detection and recovery, the kidnapped robot, collision sound | ✅ done |
| **M5** | *It cleans and comes home* | dirt and vacuum, battery and power, energy-aware return, docking, recharge and resume | ⏭ next |
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

- no global relocalization: a kidnapped robot stays lost (resolved in M4);
- loop closure searches ±0.5 m and ±12°;
- absolute error grows across long corridors until a loop closes;
- a map re-render costs about 1 ms per keyframe;
- a static-world assumption;
- one session (maps are not saved).

## M4: It has a plan ✅

Branch `milestone-4/planning` (the director asked for branches named after
their milestone). Algorithm note: [algorithms/planning.md](algorithms/planning.md);
the kidnapped robot is in [algorithms/slam.md](algorithms/slam.md), the gyro in
[algorithms/odometry.md](algorithms/odometry.md).

**Delivered**

- **A gyroscope** (the director's "yes"): a MEMS yaw-rate model with scale
  error, turn-on bias, bias random walk, angle random walk and quantization;
  gyrodometry with bias calibration on the dock and zero-velocity updates;
  SLAM's odometry noise model switches to it. Undocking backs straight off
  the dock first.
- `planning/`: a costmap from the SLAM grid (inscribed inflation, remembered
  bumps, one-cell strips lethal, exponential cost, unknown at a premium); A\*
  (octile, no corner cutting), SciPy Dijkstra fields, line-of-sight pruning;
  an online **BA\*** coverage planner with rich-first backtracking and
  **frontier exploration**.
- `navigation/path_follower.py`: pure pursuit into DWA, with a speed cap near
  path ends and obstacles.
- `autonomy/executive.py`: the coverage state machine (wait, undock, drive,
  bump reflex, recover, lost, trapped, done), now the default behaviour; M2's
  wander remains selectable.
- `autonomy/recovery.py`: five stuck detectors (cornered, no progress, bump
  storm, oscillation, wheel slip) plus "stranded", and a ladder (back up,
  spin, hop, elsewhere, trapped) that escalates only while trouble recurs in
  the same place.
- **The kidnapped robot** (asked for mid-milestone): `Simulation.kidnap`, the
  `K` key; SLAM notices (three misfits in a row), searches the whole map with
  branch and bound (`localization/relocalization.py`), accepts only an
  unambiguous match, and resumes; the executive stands still or hops for new
  views meanwhile.
- **Collision sound** (asked for): `--sound bonk` (synthesised) or
  `--sound FILE`; `V` toggles it. The *Tom and Jerry* scream the director
  wanted is copyrighted, so it is not bundled; `sounds/` is git-ignored for
  personal clips (D23).
- **Viewer:** `C` coverage layer; the planned path on `G`; relocalisation edges
  on `P`; `K` / `Shift+K` kidnap; the panel shows lost/found, and compares
  belief with the truth *at sensing time* (it used to show up to 7° of "error"
  that was only the 0.1 s control period while turning).
- **Scorecard:** final behaviour state, completion time, relocalisations.
- **Tests:** 467 fast (about 45 s), 38 slow. Ruff and mypy are clean; the fast
  suite passes on Python 3.12 and 3.13.

**Acceptance criteria and measured results.** 30 simulated minutes of the
coverage executive, default noise, seeds 1–5 (each run is also held to the
M3 SLAM criteria):

| Criterion | Test Chamber: threshold / measured | Studio: threshold / measured | Apartment: threshold / measured |
|---|---|---|---|
| accessible floor swept in 30 min | ≥ 87 % / 89.6–91.8 % | **≥ 90 % / 91.8–92.4 %** | ≥ 75 % / 79.6–85.5 % |
| job declared done | required / at 297–390 s | required / at 1468–1707 s | not required / still sweeping at 1800 s |
| trapped or lost | never / never | never / never | never / never |
| penetration; contacts with LiDAR-visible things | none; 0 / none; 0 | none; 0 / none; 0 | none; 0 / none; 0 |
| bump rate | ≤ 0.2/min / 0 | ≤ 0.5/min / 0.20–0.23 | ≤ 0.6/min / 0.20–0.33 |
| SLAM online RMS (max) | ≤ 1.5 (4) cm / 0.3–0.9 (≤ 1.4) cm | ≤ 2.5 (6) cm / 0.5–0.9 (≤ 3.1) cm | ≤ 8 (15) cm / 3.1–4.9 (≤ 11.1) cm |
| keyframe ATE (aligned) | ≤ 1.5 (1) cm / 0.3–0.6 (0.2–0.3) cm | ≤ 3 (1.5) cm / 0.5–0.9 (0.4–0.5) cm | ≤ 8 (4) cm / 3.3–5.5 (1.7–2.9) cm |
| map precision; recall | ≥ 98 %; ≥ 90 % / 100 %; 94.8–96.9 % | ≥ 98 %; ≥ 85 % / 99.9–100 %; 87.5–89.7 % | ≥ 93 %; **≥ 77 %** / 94.8–98.0 %; 80.4–87.1 % |
| relocalisations without a kidnapping | 0 / 0 | 0 / 0 | 0 / 0 |

- **The brief's criterion, 90 % of the studio in 30 minutes, is met** on all
  five seeds, each finishing on its own with 24.5–28.5 minutes used. For
  comparison, M2's random wandering reaches 83.8–85.6 % in 30 minutes (seeds 1–3), and never stops. In the
  *first* ten minutes the two are close (wandering 72–76 %, coverage about
  67 %): wandering sweeps the open middle quickly and then keeps re-sweeping
  it, while lanes take longer to start paying off.
- **Apartment recall.** Its coverage runs see more surface at grazing angles
  (lanes along walls) and represent it less completely: recall 80.4–87.1 %,
  below the 82 % set on wandering in M3, so the coverage runs carry their own
  measured floor (77 %). Every other M3 number holds.
- **A\*** is optimal against Dijkstra on 25 random grids; every stuck detector
  fires on its own trouble and on none of the others; the crate pen ends
  "trapped" or "done", never in a collision.

**The M2 and M3 criteria still hold** (10 minutes of M2's wandering, seeds
1–5, re-run with the gyro and every M4 change): coverage 81–86 % / 72–76 % /
46–54 % (chamber / studio / apartment); SLAM online RMS 0.3–0.6 / 0.5–1.0 /
2.2–5.2 cm; aligned ATE 0.2 / 0.3–0.4 / 1.6–2.0 cm; precision ≥ 96.4 %,
recall ≥ 86.4 %; zero visible contacts; no relocalisations.

**The kidnapped robot**, measured (five kidnappings per run to random open
floor, every 120 s from t = 300 s):

| Room | Kidnappings | Noticed after | Found after | Error when found | Wrong relocalisations |
|---|---|---|---|---|---|
| Studio, seeds 1–3 | 15 | 0.4 s | 0.2 s later (the first search, every time) | 0.2–1.3 cm | 0 |
| Apartment, seeds 1–3 | 15 | 0.4 s | 0.2 s; up to 4 min when set down in a room not yet mapped | 0.3–10 cm | 0 |
| Test Chamber, seeds 1–8 | 40 | 0.4 s (a few landed while it was still lost) | 5–130 s: it waits, hopping, for a view the pillar and crate disambiguate | 0.1–0.9 cm | 0 |

The slow acceptance suite repeats eight of these runs (chamber and studio seeds 1–3, apartment seeds 1–2).

**Gyro**, measured (10 minutes of wandering, seeds 1–5): dead reckoning
0.05–0.42 m / 0.10–1.2 m / 0.05–1.6 m (chamber / studio / apartment), against
0.3–3.6 / 0.7–5.2 / 0.8–11.3 m without; SLAM heading error 0.07–0.40° RMS
(M3: 0.4–0.6°). The apartment's absolute position error barely changed (it
is a small rotation of the whole map, not heading drift).

**Found and fixed during M4** (details in planning.md and slam.md):

- **Dock contact** after the gyro calibration: DWA among LiDAR points 5 mm away.
  Both behaviours now back straight off the dock.
- **Dithering** (113 targets in 90 s): targets inside the arrival tolerance.
- **Meandering** over small leftovers while big areas waited: rich-first backtracking.
- **Wall contacts** at lane ends: a speed cap on the approach.
- **Endless corner cycling** at the end of a clean: targets still useful on
  arrival are blacklisted.
- **"Trapped" in an open room:** the ladder kept climbing on unrelated
  troubles; it now resets on arrival or 1 m away.
- **Fake frontiers** inside furniture sent the robot into tight gaps.
- **40 cm dead ends:** every apartment "trapped" was one. One-cell strips are
  now lethal, and the escape hop was rebuilt (5° bearings, a 2 m horizon,
  tolerance for LiDAR noise on the walls alongside, ~1° alignment). Each
  detail was a measured failure.
- **"Coverage complete" with 25 m² unswept:** nowhere to plan from was taken
  for nothing left to do (now "stranded").
- **A dog bowl caged the robot** in its own bump memories while it spun
  against it; the hop then checked the wrong direction.
- **Kidnapped scans in the map:** at the M3 threshold (0.2), a kidnapped scan
  fitting 0.21 went into the map, and every later scan fitted the corrupted
  local map. Now 0.4, and 0.6 × the recent typical fit (0.42–0.43 in the
  near-symmetric chamber). Then, in the chamber, ICP pulled a kidnapped
  scan to the *mirror image* of the robot's old view (fit 0.80): an
  innovation gate now rejects any correction beyond the tracking window.
- **A 2-minute global search loop:** one search per second from the same
  view; now only from new views.
- **Wrong test expectations:** the short-run distance floor was set on
  wandering; a synthetic "40 cm gap" was two cells wide on an ideal map; a
  furnished 5 × 4 m box has near-twin spots. The tests now state measured
  behaviour.

**Limitations (honest):**

- coverage leaves 3–5 cm strips along edges and skips 35–45 cm dead ends
  (about 8 % of the studio's accessible floor); M5's edge pass;
- lanes run along map x, without per-room orientation;
- relocalisation needs a mapped place, and cannot break perfect symmetry;
- a global search can block one control tick for up to a second;
- recovery is local (no planned escape routes).

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
| R1 | SLAM drift or map corruption | M3–M5 | low (measured) | graph SLAM with verified loop closures and a robust kernel; scans that do not fit (absolutely, or relative to the recent fit) stay out of the map; a gyro since M4; measured 0.3–1.0 cm ATE (studio, chamber), 2.5–5.3 cm (apartment) |
| R2 | Python too slow at high speed factors | all | low (measured) | M2 ran at about 50–90× real time headless; with SLAM, about 20× on the development container (SLAM costs ~15–20 s per simulated 10 minutes, profiled and vectorized) |
| R3 | DWA local minima and oscillation | M2, M4 | low (measured) | M2 fixed the pillar minimum and margin deadlocks; M4 plans globally (A\*, coverage), detects five kinds of stuck and climbs a recovery ladder; no TRAPPED in the 30-minute acceptance runs |
| R4 | Docking precision limited by localization error | M5 | medium | infrared beacon for the final approach; tolerance-based contact model |
| R5 | Coverage near walls and under furniture limited by safety margins | M4, M5 | high (by design) | coverage measured against the *accessible* floor; M4 reaches 90–92 % of it in the chamber and studio; the rest is mostly 3–5 cm strips along walls and furniture, and 35–45 cm dead ends M4 will not enter. M5: a wall-following edge pass |
| R6 | Floating-point differences across platforms and NumPy versions | tests | low | same-process determinism tests; metric thresholds rather than golden values |
| R7 | pygame wheel availability on new Python versions | install | low (mitigated) | `pygame-ce`; GUI isolated; core and tests run headless |
| R8 | Scope creep in polish and personality | schedule | medium | milestone gates; personality is observer-only |
| R9 | Movable obstacles invalidate the map | M3, M4 | medium | log-odds clamping; replanning on map change; recovery |
| R10 | Simplified pushing (no rotation, no chain pushes) is visible to users | realism | low | documented; upgrade path in the contact solver |
| R11 | Odometry drift makes M2's novelty memory smear in long runs | M2 | resolved (M3) | the visit counter now lives in the SLAM map frame |
| R12 | Map re-rendering cost grows with keyframes (≈1 ms each) | M4+ | medium | planners read the incremental snapshot, never forcing a re-render; 30-minute runs (≈2500 keyframes) were fine; Cartographer-style submaps if hour-long runs need it |
| R13 | Kidnapping or a large slip leaves the robot lost | M4, M5 | low (measured) | resolved in M4: detection after three misfits (0.4 s), whole-map branch-and-bound search, accepted only when unambiguous; 70 kidnappings, no wrong relocalisation. Remaining: an unmapped drop-off point (the robot hops until it sees a mapped place) |
| R14 | Global search time blocks a control tick (0.3–1 s on the apartment) | M4+ | low | searches only from new viewpoints while lost; the robot stands still meanwhile; a background thread or a coarser first level if maps grow |

## Ideas for later (parked by the director)

- **The Roomba hunts your shins.** Asked for during M4, "later on". It needs a
  person in the simulator first: legs as moving obstacles, a cursor or a
  scripted walker, plus detection (a pair of narrow, moving LiDAR blobs is the
  classic leg detector). Then a behaviour that chases them, which fits
  naturally into M6's personality as a mood ("vengeful" after being kicked or
  kidnapped once too often), with M4's planner and DWA doing the chasing.
  It must stay an *observer-driven option*: the personality may not steer the
  robot (D11), so this would be an explicit behaviour mode, off by default.

## Open questions for the project director

1. ~~D7, the SLAM strategy~~: delegated by the director; graph SLAM chosen (see M3).
2. ~~Scenarios~~: a multi-room apartment was added in M3. More rooms (a
   "furniture gauntlet" for M4's recovery tests, stairs with cliff sensors)
   remain possible.
3. ~~Licence~~: MIT, confirmed (D13).
4. ~~A gyroscope~~: yes (the director); added in M4. It cut dead-reckoning
   drift 5–10×; the apartment's absolute SLAM error did not change much (a
   small global rotation of the map, not heading drift).
5. ~~Global relocalization~~: yes, "add the Kidnapped Robot thing"; done in
   M4, inside SLAM rather than the recovery ladder (D22).
6. **New: the apartment in 30 minutes.** The studio target (90 % in 30
   minutes) is met. The apartment (about 60 m² in six rooms) reaches about 80 %
   in 30 minutes and is still sweeping; the measured runs are in the M4
   section. A real robot vacuum would take 60–90 minutes for that flat. Is
   that an acceptable bar, or should M5's battery-aware missions be measured
   on the apartment too?
7. **New: dead ends.** Gaps the robot fits by less than a map cell (about
   35–45 cm wide) are never entered: in the apartment it got stuck in every
   one it tried. Real robot vacuums do enter them, bumper first. M5's edge
   pass could try them carefully. Worth it?
