# ROOMBACALYPSE

*An unnecessarily overengineered autonomous robotic vacuum simulator.*

A 2D robotics simulation of a round robot vacuum, built with real robotics
algorithms, honest physics, and (eventually) far too many feelings. The premise
is ridiculous; the engineering is meant to be real. The robot will have to map
its world, find itself in it, plan, clean, and get itself home, without ever
peeking at the simulator's ground truth.

> **Status: Milestone 4 of 7 ("It has a plan").** Press Tab and the robot
> cleans methodically: boustrophedon lanes planned on the map it is still
> building, backtracking to whatever it missed, exploring frontiers, and
> stopping when the job is done. It uses only its simulated LiDAR, bumper,
> wheel encoders and gyro. Graph SLAM keeps it within about a centimetre of
> the truth. It notices when it is stuck and works its way out, and when you
> pick it up and put it down somewhere else (K), it notices that too and
> finds itself on its map. Cleaning, battery and personality are *designed*
> ([docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)) but not built yet. Every
> feature below is marked ✅ implemented or 🗓 planned; nothing is claimed
> before it is tested.

![Milestone 4: twelve minutes into a clean of the studio. Pale cyan is floor the robot believes it swept, tan is what it still wants; the blue line is its true track, lanes and all; the blue outlines are its own map](docs/images/m4-coverage.png)

![A kidnapping, 1.2 s later: carried 5.1 m without being told, the robot noticed its scans no longer fitted, searched its whole map and found itself (the yellow line in the pose graph). Its wheels and gyro, the purple ghost, still believe it is where it was picked up](docs/images/m4-kidnap.png)

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
| Graph SLAM: scan matching (correlative search + point-to-line ICP), pose graph, loop closure | ✅ M3 |
| Log-odds occupancy grid map, drawn where it really is so map errors show | ✅ M3 |
| A multi-room apartment: six spaces off a 10 m hallway, with a loop | ✅ M3 |
| Ground-truth scoring of localization and mapping (trajectory error, map precision/recall) | ✅ M3 |
| MEMS gyroscope and gyrodometry: 5–10× less dead-reckoning drift | ✅ M4 |
| Costmap and A\* on the robot's own map; online BA\* coverage with frontier exploration | ✅ M4 |
| Stuck detection (cornered, no progress, bump storm, oscillation, wheel slip) and an escalating recovery ladder | ✅ M4 |
| The kidnapped robot: it notices it was moved and relocalises by searching its whole map | ✅ M4 |
| Optional collision sound: a built-in "bonk", or any clip of your own | ✅ M4 |
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
python main.py --room apartment       # six rooms off a hallway: the SLAM stress test
python main.py --sound bonk           # a sound whenever the robot hits something (V toggles it)
python main.py --sound sounds/my_clip.wav   # ...or your own clip (keep it out of git: sounds/ is ignored)
python main.py --set autonomy.behaviour=wander   # M2's map-free wandering instead of coverage
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
coverage        67.1 % of 34.9 m² accessible floor
bumps           3 (0.30/min); simulator saw 3 contacts, 0 with things the LiDAR can see
penetration     none
odometry drift  0.20 m, 7.3° after 600 s
SLAM pose error 0.7 cm RMS, 2.4 cm max; heading 0.13° RMS, 0.5° max
SLAM keyframes  811, ATE 0.7 cm RMS (0.5 cm aligned); 48 loop closures in 50 attempts, 0 relocalisations
map             precision 99.7 % (surface only 75.5 %), recall 90.0 %, 1593 occupied cells
behaviour       135 goals, 0 abandoned, 3 bump escapes; drive
```

The odometry line is what the robot's wheels and gyro alone would believe
(without the gyro: metres off after ten minutes). The SLAM lines are where it
actually believed it was, and how good its map is, scored against ground
truth the robot never sees. Ten minutes is a third of a studio clean: with
`--duration 1800` the same robot reaches 92 % and reports itself done.

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
| G | navigation: the planned path, its goal, DWA candidate arcs, chosen arc |
| E | belief: where odometry (purple) and SLAM (green) *think* the robot is |
| M | the robot's map (on by default), drawn through its true boot pose so errors show |
| P | pose graph: keyframes (green) and loop closures (yellow) |
| C | coverage: floor the robot believes it swept (green), what it still wants (orange) |
| K / Shift+K | kidnap: carry the robot to the cursor / somewhere random (it is not told) |
| V | collision sound on/off (start with `--sound bonk` or `--sound FILE`) |
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
LiDAR can't see it, so the robot bumps, backs off, and puts it on its map.

Or kidnap it (K with the mouse somewhere on the floor). Its wheels never
turned, so nothing tells it; a fraction of a second later its scans stop
fitting, it says it is lost, searches its whole map, and carries on cleaning
from where it really is. Put it down in a room it has never seen and it has
to wander until it recognises something.

The panel separates what is *true* (ground truth) from what the robot
*senses* and *believes*, and keeps a live, colour-coded event log.

Planned bindings: right-click to spawn dirt (M5).

**About the sound.** The director wanted Tom's scream from *Tom and Jerry*.
That clip is copyrighted, so it is not in this repository and never will be.
The built-in sound is a synthesised "bonk". If you own a clip you like, put
it in `sounds/` (git-ignored, so it stays on your machine) and start with
`--sound sounds/your_clip.wav`, or set `viewer.collision_sound` in a config file.

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
pytest                                                          # ~470 tests, ~45 s
pytest -m slow                                                  # milestone acceptance runs, ~27 min
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
- **SLAM:** with perfect sensors it is millimetre-accurate. With a 3 %
  wheel-base error it is ten times better than dead reckoning. It closes
  driven loops, survives wheel slip, and its map is scored against ground
  truth. The scan matcher was checked for bias against an oracle map.
- **Planning:** A\* is checked optimal against Dijkstra on random grids; every
  stuck detector fires on its own trouble and on nothing else; the escape hop
  gets out of a noisy 40 cm gap; the global relocalisation search returns the
  exhaustive optimum, finds a scan taken anywhere, facing any way, and reports
  a symmetric room as ambiguous; a kidnapped robot notices, finds itself and
  goes back to cleaning.
- **Acceptance suite:**
  - M2/M3: 10 simulated minutes of wandering × 5 seeds × 3 rooms: no
    penetration, zero contacts with anything the LiDAR can see, no stalls,
    bump-rate ceilings, coverage floors, SLAM pose error and map
    precision/recall (about 1 cm RMS in the studio, 3–5 cm in the apartment);
  - M4: 30 simulated minutes of coverage × 5 seeds × 3 rooms: at least 90 % of
    the studio's accessible floor, the job declared done, the same safety and
    SLAM criteria, never lost or trapped; and robots kidnapped five times per
    run that always find themselves, within centimetres.

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
- [docs/algorithms/](docs/algorithms/): kinematics, collisions, LiDAR, odometry, gyro and drive errors, DWA
  and exploration, graph SLAM and relocalisation, planning, coverage and recovery

### Repository layout

```
main.py                 launcher
config/                 default.toml + rooms/*.toml
roombacalypse/
  core/                 geometry, kinematics, messages, events, config, RNG, paths
  environment/          shapes, world, room loader, placeable catalog
  robot/                drivetrain, wheel/ground error model, body state
  sensors/              ray casting, LiDAR, bumper, encoders, gyro
  simulation/           clock, contact solver, engine
  mapping/              occupancy grid, surface fields, pose-graph optimizer
  localization/         wheel odometry and gyro, scan matching, graph SLAM, global relocalisation
  planning/             costmap, A*, coverage planner
  navigation/           obstacle memory, Dynamic Window Approach, path follower
  autonomy/             the robot's brain, teleoperation, wander, coverage executive, stuck detection, recovery
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

Honest list as of Milestone 4:

- **Coverage stops short of edges.** The footprint sweeps up to the safety
  margin: strips of 3–5 cm along walls and furniture stay unswept, and gaps
  the robot would fit by less than a map cell (35–45 cm wide dead ends) are
  never entered. That is most of the 8–10 % of accessible floor it misses in
  the studio. Real robots have side brushes and an edge pass (M5).
- **Lanes run along the map's x axis.** No per-room orientation; in the
  apartment (about 60 m² in six rooms), 30 minutes reaches about 80 %.
- **Relocalisation needs a mapped place.** A kidnapped robot put down in a room
  it has never seen hops around until it recognises something (it does not
  start a second map). In a near-symmetric room it waits for a telling view.
  A kidnapping shorter than the tracking window (25 cm) is just corrected as
  a slip. One global search can take up to a second of computing.
- **Absolute SLAM error grows away from the dock** across long corridors until
  a loop closes: about 5 cm in the apartment's far office, mostly a small
  rotation of the whole map. The map itself stays consistent (2 cm aligned).
- **Odometry still drifts**, now 0.05–1.6 m per 10 minutes with the gyro
  (metres without); SLAM corrects it.
- **Recovery is local.** The ladder backs up, spins and hops; it does not plan
  an escape route.
- **The LiDAR sweep is instantaneous.** No motion distortion, beam divergence
  or reflectivity effects.
- **Simplified contact physics.**
  - Pushed objects slide but never rotate.
  - No chain pushing (box into box).
  - A push into a wall is cancelled rather than deflected.
  - No friction, mass or momentum.
- **No climbing.** Anything inside the robot's height band blocks it, even a
  1 cm cable; real robots climb about 1.5 cm.
- **Your edits are not reproducible.** Interactive input (including kidnapping)
  is not recorded, so manual sessions cannot be replayed.
- **Coverage scoring is approximate.** The accessible-floor estimate is
  computed once per reset, so obstacles you place later don't update it.
- The project runs from a source checkout, which is where `config/` is found.
  Override the location with `ROOMBACALYPSE_CONFIG_DIR`.

## Roadmap

| | Milestone | What you will see |
|---|---|---|
| ✅ | **M1: It moves** | drive a robot around a furnished room, push socks |
| ✅ | **M2: It sees** | LiDAR rays, bump reflexes, autonomous exploration without crashing, drifting odometry |
| ✅ | **M3: It knows where it is** | a map appearing from nothing, loops snapping shut, a multi-room apartment |
| ✅ | **M4: It has a plan** | planned paths, methodical coverage, escaping dead ends, finding itself after a kidnapping |
| ⏭ | **M5: It cleans and comes home** | dirt disappearing, battery anxiety, docking, resuming |
| 🗓 | **M6: It has feelings** | frustration meters, incident reports, unsolicited commentary |
| 🗓 | **M7: It ships** | benchmarks, plots, release |

Details, acceptance criteria and risks: [docs/ROADMAP.md](docs/ROADMAP.md).

## License

MIT; see [LICENSE](LICENSE).
