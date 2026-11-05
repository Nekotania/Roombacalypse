# AGENTS.md: guide for AI coding agents working on ROOMBACALYPSE

This file is for LLM agents (Claude Code, Codex, etc.) picking up this
project without the original conversation. Read it first, then
`docs/ROADMAP.md` for the current status. Human-facing docs are `README.md`
and `docs/`.

## What this project is

A 2D robot-vacuum simulator in Python: a ridiculous premise, built with real
robotics. The robot must explore, map, localize, plan, clean, manage its
battery and dock using only simulated sensors, without ever reading the
simulator's ground truth. A personality system (planned) narrates real events.

The user acts as **project director**. They write the brief and make the big
decisions; the agent implements. The original brief asked for:

- incremental, verifiable **milestones** (M1–M7; see `docs/ROADMAP.md`);
- after each milestone: run all tests, fix regressions, re-verify earlier
  features, update the docs, and write a report;
- explaining **hard-to-reverse architectural decisions before committing to them**;
- making minor decisions independently, without asking;
- never faking functionality, never silently skipping failing tests, and
  never claiming an algorithm works unless a test shows it.

## Status (check `git log` and `docs/ROADMAP.md`; this section may be stale)

| Milestone | State as of 2026-11-05 |
|---|---|
| M1 "It moves": world, kinematics, collisions, manual control, viewer | committed (`436af18 milestone 1`) |
| M2 "It sees": LiDAR/bumper/encoders, drive errors, DWA, exploration, headless runner | committed (`825a480 milestone 2`) |
| M3 "It knows where it is": graph SLAM, occupancy grid, apartment room, SLAM metrics | complete, verified; on branch `claude/compassionate-euler-e1nan3` |
| M4–M7: planning/coverage/recovery, cleaning/battery/docking, personality, release | planned; designs in `docs/ARCHITECTURE.md` |

**Decisions taken in M3:**

- **D7.** The director delegated it ("you choose. something that works in actual
  robotics, not just simulations"). Graph SLAM was chosen: Karto /
  slam_toolbox / Cartographer style. See ARCHITECTURE §16 and
  `docs/algorithms/slam.md`.
- **Rooms.** The director asked for a multi-room apartment:
  `config/rooms/apartment.toml`.

**Open questions for the director** (ROADMAP, end):

- add a gyroscope in M4?
- global relocalization in M4's recovery ladder?

The licence is MIT and confirmed; the copyright holder is in `LICENSE`.

## Environment

- macOS. The repository lives in a **MEGA cloud-synced folder**. Its root
  `.megaignore` excludes dotfiles, so `.venv/`, `.git/` and caches are not synced.
- Python: the project venv is `.venv/`, using Python 3.15 with the project
  installed editable. Create it with `uv venv && uv pip install -e ".[dev]"`.
  `requires-python >= 3.12`; check compatibility with 3.12 via uv (below).
- **Use `pygame-ce`, not `pygame`.** Upstream pygame has no wheels for Python
  ≥ 3.14. The API and the `import pygame` name are the same.
- Dependencies: numpy, scipy, pygame-ce; pytest for development; matplotlib
  optional. Do not add dependencies without a strong reason (an explicit rule
  of the brief).
- Git: branch `main`; GitHub remote `hucils/Roombacalypse`. Cloud sessions
  work on a `claude/...` branch. In M3's session, pushing failed with a 403:
  the Claude GitHub App had no write access, so the commits stayed local.
- Cloud sessions run on Linux (Python 3.13 venv via `uv venv --python 3.13`);
  the director's own machine is macOS. There, timings were about 2× faster.

## Commands (run all before reporting a milestone)

```bash
.venv/bin/python -m pytest -q                       # fast suite (~375 tests, ~30 s)
.venv/bin/python -m pytest -q -m slow               # acceptance scenarios (~8 min), skipped by default
uvx ruff check roombacalypse tests main.py          # lint (line length 120)
uvx mypy roombacalypse --python-executable .venv/bin/python
# Python 3.12 compatibility (throwaway venv):
uv venv /tmp/rv312 --python 3.12 && uv pip install --python /tmp/rv312/bin/python -e ".[dev]" \
  && /tmp/rv312/bin/python -m pytest -q -p no:cacheprovider; rm -rf roombacalypse.egg-info

.venv/bin/python main.py                                  # GUI (manual; Tab = autonomous)
.venv/bin/python main.py --headless --duration 600 --seed 3   # ground-truth scorecard (incl. SLAM error, map P/R)
.venv/bin/python main.py --headless --room apartment --seed 2  # the SLAM stress test
.venv/bin/python main.py --screenshot out/x.png --frames 60 --mode auto   # offscreen render
```

**Visual verification:** you cannot see the window. Render offscreen
(`--screenshot`, or `Viewer(..., offscreen=True)` + `run_frames` +
`save_screenshot` in a script) and look at the PNG. Offscreen mode ignores
the mouse; `tests/test_viewer_smoke.py` shows how to monkeypatch the mouse and
post key events.

## Architecture rules (enforced by `tests/test_architecture.py`)

1. **The wall of truth.**
   - Truth packages: `environment`, `robot`, `sensors`, `simulation`, `cleaning`, `battery`.
   - Belief packages (the robot's software): `localization`, `navigation`,
     `autonomy`, `mapping`, `planning`.
   - Belief must never import truth, and vice versa. They share only `core`.
   - The robot's software learns about the world **only** through
     `core.messages.SensorFrame` and acts **only** through `DriveCommand`.
   - It may read its own *nominal* spec (`RobotSpec`, sensor configs) but never
     the true calibration errors (`drive_errors`) or anything else in the simulator.
   - Never shortcut this, even in "temporary" code.
2. **Harness:** `visualization`, `evaluation` and `app.py` may read both sides
   (to draw and to score) but never feed truth back. `personality` (planned)
   is an observer that may import only `core` and consumes events.
3. **Only `visualization/` imports pygame.**
4. **No `print()` in library code;** use `logging`. Only `app.py` prints CLI output.
5. **No import cycles between subpackages, and nothing imports `app.py`.**
6. **A new subpackage must be classified** in `LAYERS` in
   `tests/test_architecture.py`, or the test fails. That is deliberate.
7. **The robot writes events through a publish-only callable**
   (`EventLog.publisher("robot")`) and must never read the log, which contains
   simulator truth.

## Conventions

- **Units and frames:**
  - SI everywhere; config keys ending `_deg` are degrees.
  - World frame: x right, y up, θ counter-clockwise, wrapped to [−π, π).
  - Body frame: x forward, y left.
  - Odometry and map frames: origin at the robot's boot pose on the dock.
    SLAM's pose is `correction ∘ odometry` (ROS `map → odom`). Goals and the
    visit counter live in the map frame (`brain.pose`). The obstacle memory and
    bump reflexes stay in odometry, because the map frame jumps at loop closures.
  - Only `visualization/camera.py` converts to screen pixels.
- **Configuration:**
  - Every tunable is a field of a frozen dataclass in `core/config.py`, with
    validation in `__post_init__`.
  - **Every new field must also go into `config/default.toml`.** A test fails
    if the two drift apart.
  - No hard-coded tunables in algorithm code. Small internal constants are
    acceptable as named module constants with a comment.
- **Determinism:**
  - All randomness comes from `RngStreams(seed).get("<stream name>")`. Truth
    streams: `truth.calibration`, `truth.slip`, `sensors.lidar`. Robot
    software: `robot.*`, seeded via `Controller.reset(seed)`.
  - Draw a **fixed number** of random numbers per call whatever the scene
    contains (the LiDAR does this), so runs stay comparable.
  - The same seed and inputs must give bit-identical trajectories; this is tested.
- **Time:**
  - Fixed `dt` of 20 ms. Control runs at 10 Hz and the LiDAR at 5 Hz; every
    period must be an integer multiple of `dt`.
  - Time is `tick * dt`.
- **Geometry:**
  - Hot paths use plain floats and tuples; batch work uses NumPy.
  - There are scalar and vectorized twins (`arc_displacement` and
    `arc_displacement_array`); a test checks they agree. Don't duplicate
    algorithms beyond that.
- **Style:** type hints everywhere (mypy `disallow_untyped_defs`), docstrings
  on public items, and the module docstring explains the model and its
  limitations.

## Testing conventions

- `tests/helpers.py` provides:
  - `IDEAL`: no drive errors and no LiDAR noise, for tests asserting exact
    physics or geometry;
  - `box_room()`: a rectangular test room;
  - `ScriptedController`: open-loop command playback, **test tooling only,
    never robot behaviour**; the brief forbids faking intelligence with
    scripted motion;
  - `make_sim()` and `min_separation()`.
- Prefer:
  - analytic checks (hand-computed distances, kinematics);
  - property tests (random scenes and commands, invariants);
  - **closed-loop tests** through the real simulator, sensors and odometry.
- **Acceptance tests** (`tests/test_acceptance.py`, marked `slow`) use metric
  *thresholds set from measured runs with margin*, not golden numbers. Record
  the measured numbers in `docs/ROADMAP.md`.
- When a test fails, find out whether the code or the test's expectation is
  wrong, and say which in your report. Several M2 "bugs" were wrong test
  expectations; several real bugs were found only by closed-loop and
  acceptance runs.

## Hard-won knowledge (don't undo without reading why)

**DWA** (`navigation/dwa.py`; history in `docs/algorithms/dwa.md`).

Keep:

- the window sampled in **wheel-speed** space (it is exact for this drivetrain);
- arcs predicted with the **acceleration ramp** over the first control period;
- the **two-stage hard radius**:
  - body + `safety_margin` normally;
  - body + 1.5 cm once inside the margin;
  - "never closer" once closer still;
- scoring that combines progress, *mean* clearance along the arc, an
  **intrusion penalty**, speed, and heading as a tie-breaker.

Each of these fixed a real failure:

- creeping into walls;
- deadlocking inside the margin;
- the classic local minimum in front of a pillar.

**Wander** (`autonomy/wander.py`): the **cramped-corner short hop** fixed a
livelock (robot nose-in between chair legs, turning forever). It is covered
by a regression test.

**Odometry drift is deliberate.** Wheel-radius std 0.2 %, wheel-base std 1 %,
OU slip 1 % with τ = 0.5 s; the UMBmark model.

- Drift reaches metres and 100°+ per 10 minutes; that is the M3 baseline.
- Don't "fix" it by removing the errors.
- Unequal wheel radii dominate heading drift.

**What the LiDAR sees:** the scan plane is at 0.095 m, so socks and the dog
bowl are invisible by design.

- In the M2 acceptance runs, *all* studio contacts were with those two.
- The acceptance suite asserts zero contacts with LiDAR-visible things.

**Coverage:** `evaluation/coverage.py` measures coverage of the *accessible*
floor (rasterize, distance transform, flood fill from the dock, dilate). It is
computed once per reset. Reuse it for M5 cleaning metrics.

**Rendering:** the viewer draws the robot's goal and DWA arcs relative to the
**true** pose. Belief (the odometry and SLAM ghosts, E; the map, M; the pose
graph, P) is drawn via `sim.boot_pose`, so belief errors show as they are. A
map re-render after a loop closure costs about 1 ms per keyframe; the viewer
throttles it (`MAP_REFRESH_SECONDS`).

**SLAM** (`localization/slam.py`, `localization/scan_matcher.py`, `mapping/`; history in
`docs/algorithms/slam.md`). Each item below fixed a measured problem. Keep:

- reference maps of **surface elements** (PCA over 5×5 cells, refit over 9×9
  where blurred), classified line, point or blob. Point-to-line residuals give
  sub-cell accuracy. Blobs constrain nothing, because treating them as points
  pinned corridors along their length;
- scan surfaces sampled at **segment midpoints**. Sampling from each return
  toward the next made a chiral heading bias of −0.005° per keyframe;
- **frozen-correspondence ICP** (re-associate, then Gauss-Newton). Letting the
  cost jump inside a line search stalled the solver on the odometry side;
- **vector residuals for point elements**, not `|x − p|`, which oscillates;
- information from **ICP normals** (zero along a corridor), not from `JᵀJ` of
  the likelihood cost, which faked information along walls;
- the **per-scan slip check**: spinning wheels otherwise let the pose run
  22 cm ahead;
- **map precision counts phantom obstacles** in free space. Thickening into
  solid matter is normal and allowed;
- debugging method that worked: record one run (odometry, scans, truth) and
  replay SLAM variants offline on identical data. Compare against an
  *oracle* local map built at true poses to separate matcher bias from
  chaining drift.

## Workflow with the director

1. Before a milestone that involves an expensive-to-reverse choice, explain it
   and recommend an option. Don't start building on an unconfirmed major
   decision (currently D7).
2. Build incrementally; run the full check list; verify visually with offscreen
   screenshots; run the headless batch (5 seeds × both rooms, 600 s) and
   report measured numbers.
3. Update `README.md` (feature table ✅/🗓, controls, limitations),
   `docs/ARCHITECTURE.md` (implemented versus planned, decision log),
   `docs/ROADMAP.md` (status and measured acceptance), and
   `docs/algorithms/<topic>.md`. Documentation claims must match tests or
   measured runs.
4. **Commits:**
   - The director wants each finished milestone committed **before the next
     one starts**.
   - Commit only when asked, and check `git log` first: they sometimes commit
     themselves.
   - Message style: `milestone N`. End commit messages with the
     `Co-Authored-By` trailer your harness specifies.
5. The report should cover: what was delivered, measured results, bugs found
   and fixed, decisions made, honest limitations, and what's needed from the
   director.

## Map of the code

```
main.py / roombacalypse/app.py    CLI, composition root (wires EventLog, RobotBrain, Simulation, Viewer)
roombacalypse/core/               geometry, kinematics, messages (SensorFrame...), events, config, rng, paths
roombacalypse/environment/        shapes (SAT, circle queries), World, rooms (TOML + table/chair macros), catalog
roombacalypse/robot/              drivetrain limits, wheels.py (hidden errors), body state + Contact
roombacalypse/sensors/            raycast.py, lidar.py, bumper.py, encoders.py
roombacalypse/simulation/         clock, physics.py (contact solver, pushing), engine.py (step loop)
roombacalypse/mapping/            occupancy_grid.py, likelihood_field.py (surface fields), pose_graph.py (LM, Cauchy)
roombacalypse/localization/       odometry.py, scan_matcher.py (correlative B&B + PL-ICP), slam.py (GraphSlam, Slam protocol)
roombacalypse/navigation/         local_obstacles.py (memory, swept-disc free distance), dwa.py
roombacalypse/autonomy/           brain.py (RobotBrain, modes), teleop.py, wander.py
roombacalypse/evaluation/         metrics.py, coverage.py, slam_metrics.py (ATE, map P/R), scenario.py (scorecard)
roombacalypse/visualization/      camera (pure math), renderer, hud, viewer, palette
config/default.toml, config/rooms/*.toml    parameters and room layouts (studio, test_chamber, apartment)
tests/                            fast suite + test_acceptance.py (slow) + test_architecture.py
docs/                             ARCHITECTURE.md, ROADMAP.md, algorithms/*.md, images/
```

## Planned next steps (M4 outline)

- Ask the director the two open questions first (gyroscope; global relocalization).
- `planning/`: costmap from `brain.slam.grid`, with inflation by robot radius
  plus margin; A\* with octile heuristic; BA\* coverage on a coarse grid;
  frontier exploration once coverage runs out.
- Path following: lookahead points on the A\* path become DWA goals (DWA is unchanged).
- Executive state machine replaces `wander` (ARCHITECTURE §10.1). Stuck
  detectors and a recovery ladder. "Wheel slip" can reuse SLAM's per-scan fit;
  "rotate and re-localize" needs global relocalization.
- Acceptance: ≥ 90 % of the studio's accessible floor in 30 minutes; the
  apartment gets its own (lower) threshold, measured.
- Watch the map re-render cost (~1 ms per keyframe) in 30-minute runs; submaps
  if needed.
