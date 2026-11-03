# ROOMBACALYPSE

*An unnecessarily overengineered autonomous robotic vacuum simulator.*

A 2D robotics simulation of a round robot vacuum, built with real robotics
algorithms, honest physics, and (eventually) far too many feelings. The premise
is ridiculous; the engineering is meant to be real. The robot will have to map
its world, find itself in it, plan, clean, and get itself home, without ever
peeking at the simulator's ground truth.

> **Status: Milestone 2 of 7 ("It sees").** Press Tab and the robot drives
> itself. It explores a furnished apartment using only its simulated LiDAR,
> bumper and wheel encoders, never touching anything its LiDAR can see, and
> finding the sock it can't see the hard way. Mapping, localization, planning,
> cleaning, battery and personality are *designed*
> ([docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)) but not built yet. Every
> feature below is marked ✅ implemented or 🗓 planned; nothing is claimed
> before it is tested.

![Milestone 2: autonomous exploration with LiDAR rays, the robot's goal, and its drifting odometry ghost](docs/images/m2-autonomy.png)

## Features

| Feature | Status |
|---|---|
| Differential-drive kinematics: exact arc integration, curvature-preserving wheel speed limits, acceleration limits | ✅ M1 |
| Continuous 2.5D world: concave floor plans, convex furniture, circles, height bands (drive under beds, not through table legs) | ✅ M1 |
| Collision detection and response: no penetration, sliding along walls, pushable obstacles | ✅ M1 |
| Room files in TOML with furniture macros (tables, chairs); two built-in rooms | ✅ M1 |
| Pygame viewer: driving, pause, speed 0.125–64×, single step, reset, obstacle placement and removal, zoom/pan, trail, screenshots | ✅ M1 |
| Typed, validated, layered configuration (no hard-coded parameters) | ✅ M1 |
| Deterministic simulation (same seed and inputs → bit-identical run, noise included) | ✅ M1–M2 |
| Architecture tests that stop the robot's software from reading ground truth | ✅ M1 |
| 2D LiDAR: vectorized ray casting, height-band visibility, range-dependent noise, dropouts | ✅ M2 |
| Bumper (left/right switches) and wheel encoders | ✅ M2 |
| Hidden per-robot wheel calibration errors and correlated slip: odometry drifts realistically | ✅ M2 |
| Autonomous exploration: Dynamic Window Approach obstacle avoidance, bump reflex, map-free goal picking | ✅ M2 |
| Headless runs with a ground-truth scorecard; event log | ✅ M2 |
| Occupancy grid mapping and particle-filter localization | 🗓 M3 |
| A\* path planning, coverage planning, stuck detection and recovery | 🗓 M4 |
| Dirt, vacuum, battery, energy-aware return to dock, docking, recharge and resume | 🗓 M5 |
| Personality (frustration, confidence, curiosity, battery anxiety), incident reports | 🗓 M6 |
| Benchmarks and release | 🗓 M7 |

## Installation

Requires **Python 3.12 or newer** (tested on 3.12 and 3.15).

With [uv](https://docs.astral.sh/uv/):

```bash
uv venv
uv pip install -e ".[dev]"
```

With plain pip:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"
```

Runtime dependencies are NumPy, SciPy and **pygame-ce**. `matplotlib` is optional
(`pip install -e ".[plots]"`) for the diagnostic plots planned for later milestones.

**About pygame-ce.** The project depends on `pygame-ce`, the community-maintained
fork of pygame. It is imported as `pygame` and has the same API. We use it
because upstream `pygame` publishes no wheels for Python 3.14 or 3.15. On
Python 3.12 or 3.13 you may use upstream pygame 2.6 instead
(`pip uninstall pygame-ce && pip install pygame`), but never install both.

## Running

```bash
python main.py                        # the studio apartment; press Tab to let the robot drive
python main.py --mode auto            # start autonomous
python main.py --room test_chamber    # a small empty box
python main.py --list-rooms
python main.py --seed 7 --set dwa.max_speed=0.2 --set lidar.dropout_probability=0.1
python main.py --config my_settings.toml

# No window: simulate 10 minutes as fast as possible and print a ground-truth scorecard
python main.py --headless --duration 600 --seed 3
python main.py --screenshot out/frame.png --frames 60 --mode auto   # render offscreen
```

`python -m roombacalypse` and the installed `roombacalypse` command do the same thing.

A headless scorecard looks like this (studio, seed 3, actual output):

```
coverage        69.0 % of 34.9 m² accessible floor
bumps           2 (0.20/min); simulator saw 2 contacts, 0 with things the LiDAR can see
penetration     none
odometry drift  5.52 m, 104.1° after 600 s
```

The drift line is the robot's *own* dead-reckoning error: what Milestone 3's
localization has to fix.

## Controls

| Key / mouse | Action |
|---|---|
| Tab | switch manual ↔ autonomous |
| Arrows / WASD | drive, in manual mode (Shift: full speed) |
| Space | pause / resume |
| N | single physics step while paused |
| `+` / `-` | faster / slower simulation |
| R / Shift+R | reset (same seed / next seed) |
| L | LiDAR rays and hits |
| G | navigation: exploration goal, DWA candidate arcs, chosen arc |
| E | belief: purple ghost where odometry *thinks* the robot is |
| T | ground-truth trail |
| Left click | place an obstacle (preview follows the mouse) |
| O | cycle obstacle type: cardboard box, pillar, sock, ottoman |
| X | remove the obstacle under the cursor (whole table/chair) |
| Mouse wheel | zoom at cursor |
| Middle drag, or Alt + left drag | pan |
| 0 | fit room to window |
| F12 | save a screenshot to `out/` |
| H / F1 | help overlay |
| Esc / Q | quit |

Hover over furniture to see what it is, its height band, and whether it moves.
Amber outlines mark movable objects. Translucent furniture is overhead: the
robot can drive underneath, and its LiDAR sees straight through it.

Try dropping a sock (O, then click) in front of the autonomous robot. The
LiDAR can't see it, so the robot bumps, backs off and remembers it for 20 s.

The panel separates what is *true* (ground truth) from what the robot
*senses* and *believes*, and keeps a live, colour-coded event log.

Planned bindings: `M` occupancy map and `P` particles (M3), `C` coverage (M4),
right-click to spawn dirt (M5).

## Configuration

All tunables live in typed dataclasses (`roombacalypse/core/config.py`) and are
documented in [`config/default.toml`](config/default.toml). That includes
sensor noise, the hidden drive errors, and DWA and exploration parameters. To
change them, pass a TOML file with only the keys you want (`--config`) or
single values (`--set section.key=value`). Unknown keys and invalid values are
errors, so a typo cannot fall back silently to a default.

Rooms are TOML files in [`config/rooms/`](config/rooms/). The studio file is
annotated and shows every shape type and the `table` and `chair` macros.

## Tests and checks

```bash
pytest                                                          # 314 tests, ~8 s
pytest -m slow                                                  # milestone acceptance runs, ~70 s
uvx ruff check roombacalypse tests                              # lint (optional tool)
uvx mypy roombacalypse --python-executable .venv/bin/python     # type check (optional tool)
```

Highlights:

- **LiDAR:** checked against hand-computed distances and noise statistics.
- **Closed-loop DWA:** tests drive the real simulator through LiDAR and odometry
  around a pillar and up to a wall without contact.
- **Property tests:** random driving never penetrates anything; DWA always
  picks an arc it can brake on.
- **Determinism:** bit-for-bit, with all noise on.
- **Architecture tests:** they fail if the robot's software imports ground truth.
- **Acceptance suite:** 10 simulated minutes × 5 seeds × 2 rooms. It asserts no
  penetration, zero contacts with anything the LiDAR can see, no stalls, a
  bump-rate ceiling and a coverage floor (studio: 69–73 % of the accessible
  floor explored, measured).

## Architecture overview

```
           GROUND TRUTH (simulator)                    BELIEF (robot software)
  environment · robot · sensors · simulation   localization · navigation · autonomy
  (cleaning · battery)                         (mapping · planning)
                 │                                          ▲
                 │  SensorFrame  ─────── the only ───────►  │  encoders, bumper, LiDAR
                 │  ◄─────── DriveCommand  crossing         │
                 ▼                                          │
                              core (shared kernel)
       geometry · kinematics · messages · events · config · RNG streams

  harness (reads both sides, writes neither): visualization · evaluation · app.py
  observer (events only): personality
```

- **The wall of truth.** The robot's software sees only sensor frames, never the
  simulator's state, and not even its own true wheel sizes. Tests enforce this,
  so "autonomy" cannot quietly cheat.
- **Physics is separate from rendering.** The engine runs headless; the pygame
  viewer only reads it.
- **Deterministic.** Fixed timestep, integer-multiple rates, named random
  streams derived from one seed.
- **Continuous geometry, not a grid.** The robot's map has to be *inferred*
  from sensors, as on a real robot.

Read more:

- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md): full design and decision log
- [docs/ROADMAP.md](docs/ROADMAP.md): milestones, acceptance criteria with measured results, risks
- [docs/algorithms/](docs/algorithms/): kinematics, collisions, LiDAR, odometry and drive errors, DWA and exploration

### Repository layout

```
main.py                 launcher
config/                 default.toml + rooms/*.toml
roombacalypse/
  core/                 geometry, kinematics, messages, events, config, RNG, paths
  environment/          shapes, world, room loader, placeable catalog
  robot/                drivetrain, wheel/ground error model, body state
  sensors/              ray casting, LiDAR, bumper, encoders
  simulation/           clock, contact solver, engine
  localization/         wheel odometry
  navigation/           obstacle memory, Dynamic Window Approach
  autonomy/             the robot's brain, teleoperation, exploration behaviour
  evaluation/           ground-truth metrics, coverage, headless scenarios
  visualization/        camera, renderer, HUD, viewer
  app.py                CLI and composition root
tests/                  pytest suite (+ slow acceptance scenarios)
docs/                   architecture, roadmap, algorithm notes
```

The brief suggested fourteen top-level packages. They live inside one
`roombacalypse` package instead, to avoid import-name collisions; see decision
D1 in the architecture document.

## Current limitations

Honest list as of Milestone 2:

- **No map yet.** The robot explores at random, guided by its current scan
  and a dead-reckoned visit counter. It revisits places, and the counter
  smears as odometry drifts. Systematic coverage needs Milestone 3's map and
  Milestone 4's planner.
- **Odometry drifts:** up to several metres and 100°+ per 10 minutes. That is
  deliberate realism; no gyro is modelled (yet).
- **DWA is a local planner.** It handles the obstacles tested, but U-shaped
  traps and dead ends can still fool it; Milestone 4 adds global planning
  and recovery.
- **The LiDAR sweep is instantaneous.** No motion distortion, beam divergence
  or reflectivity effects.
- **Simplified contact physics.**
  - Pushed objects slide but never rotate.
  - No chain pushing (box into box).
  - A push into a wall is cancelled rather than deflected.
  - No friction, mass or momentum.
- **No climbing.** Anything inside the robot's height band blocks it, even a
  1 cm cable; real robots climb about 1.5 cm.
- **Your edits are not reproducible.** Interactive input is not recorded, so
  manual sessions cannot be replayed.
- **Coverage is approximate.** The accessible-floor estimate is computed once
  per reset, so obstacles you place later don't update it.
- The project runs from a source checkout, which is where `config/` is found.
  Override the location with `ROOMBACALYPSE_CONFIG_DIR`.

## Roadmap

| | Milestone | What you will see |
|---|---|---|
| ✅ | **M1: It moves** | drive a robot around a furnished room, push socks |
| ✅ | **M2: It sees** | LiDAR rays, bump reflexes, autonomous exploration without crashing, drifting odometry |
| ⏭ | **M3: It knows where it is** | a map appearing from nothing, a particle cloud collapsing onto the robot |
| 🗓 | **M4: It has a plan** | planned paths, methodical coverage, escaping chair-leg traps |
| 🗓 | **M5: It cleans and comes home** | dirt disappearing, battery anxiety, docking, resuming |
| 🗓 | **M6: It has feelings** | frustration meters, incident reports, unsolicited commentary |
| 🗓 | **M7: It ships** | benchmarks, plots, release |

Details, acceptance criteria and risks: [docs/ROADMAP.md](docs/ROADMAP.md).

## License

MIT; see [LICENSE](LICENSE).
