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

| Milestone | State as of 2026-11-03 |
|---|---|
| M1 "It moves": world, kinematics, collisions, manual control, viewer | committed (`436af18 milestone 1`) |
| M2 "It sees": LiDAR/bumper/encoders, drive errors, DWA, exploration, headless runner | complete, verified; **was uncommitted**; check `git status` |
| M3 "It knows where it is": occupancy grid + particle filter | next. **Blocked on decision D7** (below) |
| M4–M7: planning/coverage/recovery, cleaning/battery/docking, personality, release | planned; designs in `docs/ARCHITECTURE.md` |

**Open decisions for the director** (ask; don't decide alone):

- **D7, the SLAM strategy, needed before M3.** The recommendation on the table:
  Monte Carlo localization against the robot's own incrementally built
  occupancy grid ("scan-to-map", no loop closure), behind a
  `Slam.update(odometry, scan) -> (pose, grid)` interface so a
  Rao-Blackwellized particle filter (GMapping-style) can replace it.
  Full write-up: `docs/ARCHITECTURE.md` §16.
- **Rooms:** any extra scenarios wanted? For example, multi-room with doors, a
  furniture gauntlet, stairs and cliff sensors.

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
- Git: branch `main`, **no remote**.

## Commands (run all before reporting a milestone)

```bash
.venv/bin/python -m pytest -q                       # fast suite (~315 tests, ~8 s)
.venv/bin/python -m pytest -q -m slow               # acceptance scenarios (~70 s), skipped by default
uvx ruff check roombacalypse tests main.py          # lint (line length 120)
uvx mypy roombacalypse --python-executable .venv/bin/python
# Python 3.12 compatibility (throwaway venv):
uv venv /tmp/rv312 --python 3.12 && uv pip install --python /tmp/rv312/bin/python -e ".[dev]" \
  && /tmp/rv312/bin/python -m pytest -q -p no:cacheprovider; rm -rf roombacalypse.egg-info

.venv/bin/python main.py                                  # GUI (manual; Tab = autonomous)
.venv/bin/python main.py --headless --duration 600 --seed 3   # ground-truth scorecard
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
  - Odometry/map frame: origin at the robot's boot pose on the dock.
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
**true** pose. Belief (the odometry ghost, E) is drawn via `sim.boot_pose`.

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
roombacalypse/localization/       odometry.py (dead reckoning)           [M3: particle filter]
roombacalypse/navigation/         local_obstacles.py (memory, swept-disc free distance), dwa.py
roombacalypse/autonomy/           brain.py (RobotBrain, modes), teleop.py, wander.py
roombacalypse/evaluation/         metrics.py, coverage.py, scenario.py (headless scorecard)
roombacalypse/visualization/      camera (pure math), renderer, hud, viewer, palette
config/default.toml, config/rooms/*.toml    parameters and room layouts (studio, test_chamber)
tests/                            fast suite + test_acceptance.py (slow) + test_architecture.py
docs/                             ARCHITECTURE.md, ROADMAP.md, algorithms/*.md, images/
```

## Planned next steps (M3 outline; confirm D7 first)

- `mapping/`: a log-odds occupancy grid (5 cm cells), an inverse sensor model,
  vectorized beam tracing de-duplicated per scan, clamped log-odds.
- `localization/`: MCL with the odometry motion model, a likelihood field from
  `scipy.ndimage.distance_transform_edt`, low-variance resampling triggered by
  effective sample size, and a circular mean for θ.
- SLAM coupling behind a `Slam` interface. A ground-truth "known-pose" mapping
  mode may exist **only** for tests and debugging, never as robot behaviour.
- `evaluation/`: absolute trajectory error and map precision/recall against
  ground truth. Targets: ATE < 5 cm and precision ≥ 0.95 after 10 minutes;
  set final thresholds from measurements.
- Viewer layers: `M` map and `P` particles. Switch wander's visit memory to the
  localized pose.
