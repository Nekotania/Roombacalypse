# Planning, coverage, recovery (Milestone 4)

Code: `planning/costmap.py`, `planning/astar.py`, `planning/coverage.py`,
`navigation/path_follower.py`, `autonomy/executive.py`, `autonomy/recovery.py`.
Tests: `tests/test_planning.py`, `tests/test_coverage.py`, `tests/test_relocalization.py`,
`tests/test_acceptance.py` (slow).

The robot plans on its own SLAM map, in the map frame, and drives with M2's
Dynamic Window Approach. The layering is the usual one on real robots
(ROS navigation, Nav2):

```
coverage planner ──► target (a path) ──► path follower ──► carrot ──► DWA ──► wheels
      ▲                                                                  ▲
 costmap ◄── SLAM occupancy grid + remembered bumps          LiDAR now + recent bumps
```

The planner decides *where* to go on the map (seconds); DWA decides *how* to
move right now (100 ms) among what the LiDAR sees. The executive is the
state machine that connects them and handles everything that goes wrong.

## Costmap

Built from the SLAM occupancy grid every `replan_interval` (1 s), same lattice (5 cm):

- **Obstacles**: occupied cells, plus discs of `bump_radius` (8 cm) where the robot
  bumped into something its LiDAR cannot see (a sock is invisible, but once
  felt it is avoided for the rest of the run; the obstacle memory DWA uses
  forgets bumps after 20 s).
- **Lethal**: closer than `inscribed = radius + DWA safety margin + extra_margin`
  (0.17 + 0.03 + 0 m) to an obstacle, by Euclidean distance transform. That
  is exactly the region DWA itself refuses, so a plan never asks DWA for the
  impossible.
- **Strips one cell wide are lethal too** (a morphological opening of the
  passable cells with a 2×2 block). Measured: in the apartment every robot
  that declared itself trapped had followed such a strip nose-first into a
  40 cm dead end (between a sofa and an armchair, or a wall and a kitchen
  counter: 3 cm to spare on each side) and could not get out with its safety
  margin. At 5 cm resolution the map cannot vouch for a fit that tight.
- **Cost** beyond that decays as `cost_scale · exp(−(clearance − inscribed) / cost_decay)`,
  so shortest paths keep away from furniture where they can. **Unknown**
  cells are passable at a premium (`unknown_cost`): plans prefer known floor.

`cost` is the extra cost per metre on top of distance; `inf` where lethal.

## A\*, Dijkstra fields, pruning

8-connected grid; a step between neighbours costs its length times
`1 + (cost_a + cost_b)/2`; no corner cutting. A\* with the octile heuristic
(admissible, since a step costs at least its length) is tested optimal
against SciPy's compiled Dijkstra on 25 random grids. Coverage needs "which
of these thousand cells is nearest *by path*", so it runs one Dijkstra
field from the robot (`scipy.sparse.csgraph`, about 10 ms on the apartment)
and reads every candidate off it. Paths are pruned by line of sight through
cells costing at most 1 extra per metre, so the robot drives straight
segments instead of a staircase.

## Coverage: online BA\* with frontiers

An online adaptation of BA\* (Viet et al. 2013) plus frontier exploration
(Yamauchi 1997). Because the map grows while the robot sweeps, everything is
re-assessed each time a decision is due:

1. **Coverage memory**: the footprint (radius 0.17 m) marked along the SLAM
   pose, on a growable grid aligned with the map's lattice.
2. **Uncovered floor**: known-free cells within the footprint's reach from a
   traversable cell, not yet swept. A **useful** position would sweep at least
   6 uncovered cells (0.015 m²); a **rich** one at least half its footprint.
3. **Decision**, in order:
   - **lane**: continue straight along map x to the last useful cell before the free space ends;
   - **shift**: step sideways by up to `lane_width` (0.25 m) onto a row with
     useful cells right there, and reverse (a shorter shift if the free space
     ends sooner, which puts the last lane at the edge);
   - **backtrack** (a BA\* critical point): the nearest *rich* cell by path,
     then a new lane there. Only when no rich cell is left, the nearest
     *useful* fringe. Measured: chasing every fringe as soon as it appeared
     wasted minutes in tight spots while big areas waited;
   - **frontier**: the nearest traversable cell next to unknown space that
     reaches the edge of the map. Unknown pockets *inside* furniture (no beam
     ever gets in) are not frontiers: treating them as such sent the robot
     into tight gaps to look at the inside of boxes;
   - otherwise **done**.
4. Targets within reach of the arrival tolerance are never chosen (they
   would count as reached at once: 113 targets in 90 s, before that rule).
   A target that is still useful after the robot arrived cannot be swept
   from where the robot can stop, so it is blacklisted for 15 minutes.

**Stranded is not done.** If no traversable cell lies within 0.5 m of the
robot, or the cells it can reach are a pocket smaller than 1 m² while useful
floor lies elsewhere, the planner raises `Stranded`. The executive treats
that as trouble (recovery ladder below). Before this rule, a robot squeezed
into a gap declared "coverage complete" with 25 m² unswept.

## Path following and speed

`PathFollower` projects the robot onto the path (searching forward only, so
a path that doubles back is followed in order) and hands DWA the point
`lookahead` (0.5 m) further along: pure pursuit. On a two-point lane path,
this keeps the robot on the lane line. DWA itself is unchanged except for a
**speed cap**: at most `0.75 m/s per metre` of path left (so it arrives at
lane ends slowly), and 0.15 m/s within 10 cm of anything. Lane ends sit at
the edge of the safety margin, a few centimetres from a wall, and arriving at
cruise speed left the braking to the last moment: with ±1 cm of LiDAR noise
at that range, that produced the only wall contacts of the early M4 runs.

## Executive

States: **WAIT** (first scan, gyro calibration) → **UNDOCK** (back straight off
the dock: the dock's 5 mm gap is below LiDAR noise, so turning in place on it
made contact) → **DRIVE** ⇄ **BACK OFF** / **TURN** (bump reflex: the bump also
goes into the costmap) ⇄ **RECOVER** → **TRAPPED**; **LOST** while SLAM is lost;
**DONE**.

## Stuck detection

Five detectors (`StuckDetector`), each simple and separately tested:

| Detector | Fires when | Typical cause |
|---|---|---|
| cornered | DWA finds no admissible forward arc for 1.5 s | nose-in between chair legs |
| no progress | the remaining path has not shrunk by 5 cm in 8 s | dithering, circling a goal |
| bump storm | 3 bumps in 20 s | invisible clutter |
| oscillation | 8 turn reversals in 6 s while moving less than 0.25 m | DWA local minimum |
| wheel slip | odometry says > 0.3 m in 3 s, the map pose < 5 cm | wheels spinning against a wall |

Plus **stranded**, raised by the planner (above).

## Recovery ladder

The ladder escalates while trouble keeps coming back at the same place
(within 0.5 m and 45 s), and starts over anywhere else, or as soon as the
robot reaches a target or gets 1 m away from the last trouble:

1. **back up** 15 cm, if the LiDAR says the space behind is clear (else spin);
2. **spin** 360° on the spot (fresh scans all round, a new costmap);
3. **hop**: drive on a heading towards the most open direction (Nav2's
   `DriveOnHeading`); see below;
4. **elsewhere**: give the target up and keep out of the trouble spot (0.4 m) for 15 minutes;
5. **trapped**: stop and call for help, unless there is still an open
   direction and fewer than two hops have been tried here (then hop again).

**The hop** turns on the spot to the chosen heading (to within about 1°,
proportional near the end), then creeps straight at 0.1 m/s with the gyro
holding the heading, up to 0.5 m (1 m when lost). It is the only manoeuvre
that ignores DWA's safety margin, because its job is the gaps the margin
cannot get out of. Three measured details matter:

- **Bearings every 5°**, not 15°: in a gap 3 cm wider than the robot on each
  side, 9° off the gap's axis scrapes the side within 25 cm.
- **A 2 m horizon** to judge "most open": at 1 m, "into the dead end" and
  "out into the room" tied, and the tie-break chose the dead end.
- **Returns up to 1 cm inside the body's edge are ignored.** LiDAR noise
  (σ 0.8 cm) on the walls alongside puts some returns there; driving
  straight, the robot never closes on a parallel wall, and anything across
  its path is seen well before. Checking the bare body blocked every escape.
- **The obstacle check follows the actual heading**, so the robot must be
  lined up before it creeps: with a 0.1 rad alignment tolerance, the hop
  aimed 5° off the nose but checked straight ahead, found a remembered bump
  there, and gave up at once (a dog bowl the robot had pushed while spinning
  had caged it in bump memories).

## Lost: the kidnapped robot

When SLAM declares itself lost (three keyframes in a row that fit nowhere
near the prediction: see [slam.md](slam.md), "Lost and found"), the
executive stops using the map pose at all: no coverage marks, no bump
memories in the map, no targets. It stands still for 3 s while SLAM searches
the whole map; if that fails, it hops up to 1 m towards the most open
direction for a different view and stands still again. After 20 hops it says
"I need help" and keeps looking, standing still. Once SLAM finds itself, the
executive just re-plans: the coverage memory and the map are in the map
frame, which the kidnapping did not change.

## Measured results

See [ROADMAP.md](../ROADMAP.md), M4, for the acceptance runs (30 simulated
minutes, five seeds, three rooms) and the kidnapping runs.

## Limitations

- **Edge strips.** The footprint sweeps up to 3 cm (the DWA safety margin)
  plus up to half a cell short of walls and furniture: a few per cent of the
  floor. Real robots have side brushes and a wall-following edge pass (M5).
- **Lanes are along map x.** No decomposition into cells oriented with each
  room; in rooms at an angle to the map, lanes are diagonal and shorter.
- **The dead-end rule is coarse.** Gaps whose passable strip is one cell wide
  (roughly 35–45 cm) are never entered, so their floor is left to the edge pass.
- **Re-planning is from scratch** every second (a Dijkstra field over the map,
  about 10 ms on the apartment), not incremental (D\* Lite).
- **Recovery is local.** The ladder backs up, spins and hops; it does not
  plan an escape route through the costmap.
