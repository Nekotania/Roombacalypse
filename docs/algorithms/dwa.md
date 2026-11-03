# Dynamic Window Approach and map-free exploration

Code: `roombacalypse/navigation/dwa.py`, `roombacalypse/navigation/local_obstacles.py`,
`roombacalypse/autonomy/wander.py`, `roombacalypse/autonomy/brain.py`.
Tests: `tests/test_dwa.py`, `tests/test_autonomy.py`, `tests/test_acceptance.py` (slow).

## Dynamic Window Approach (DWA)

Fox, Burgard & Thrun (1997). Every control tick (10 Hz):

**1. Dynamic window.** Only velocities the motors can reach within one
control period are considered. The window is sampled in *wheel-speed*
space: each wheel ±(max acceleration × 0.1 s) around its current
encoder-measured speed, clipped to the wheel-speed limit, on a 9 × 9 grid.
For this drivetrain that is exactly the reachable set, whereas the usual
(v, ω) rectangle over-approximates it. Candidates must have v ≥ 0 (no reverse;
there is no rear bumper) and |ω| ≤ 2 rad/s. If none qualify (the robot is
still reversing, or faster than the cruise cap), the reachable options
closest to the envelope are used, which brakes.

**2. Prediction.** Each candidate is simulated for 1.5 s at 0.1 s steps.
During the first control period the twist *ramps linearly* from the current
one, as the drivetrain's acceleration limit makes it, and then holds.
An earlier version assumed instant velocity changes. That underestimated how
far a braking robot travels, and closed-loop tests caught it creeping into
walls.

**3. Collision check.** Distances from every predicted position to the
obstacle points are found with a KD-tree (`scipy.spatial.KDTree`). An arc's
*free time* is how long it stays outside the **hard radius**; its *free
distance* is v × free time. The arc is **admissible** if the robot could still
brake to a stop within it: v² / 2a ≤ free distance.

The hard radius is:

- the body radius plus `safety_margin` (3 cm) normally;
- the body radius plus 1.5 cm if the robot has already slipped inside that
  margin (it can by a few millimetres while passing an obstacle);
- the current distance, if the robot is closer still (right after a bump):
  it may not get any closer.

A single all-or-nothing radius was tried first. It deadlocked: once inside
the margin, every forward arc "approaches" something.

**4. Scoring.** Admissible arcs are scored by a weighted sum:

| Term | Meaning | Weight |
|---|---|---|
| progress | how much closer to the goal the arc ends, over the max possible | 1.0 |
| clearance | mean distance from the body to obstacles along the arc (capped at 33 cm) | 0.4 |
| intrusion | *penalty*: how deep the arc dips into the 3 cm safety margin | −1.0 |
| speed | v / max speed | 0.2 |
| heading | facing the goal at the end of the arc (mostly a tie-breaker) | 0.2 |

The weights went through two revisions, both driven by failing closed-loop tests:

- With a high heading weight and clearance measured as "free distance"
  (infinite for a robot standing still), *stopping in front of an obstacle
  while facing the goal* outscored every detour: the classic DWA local minimum.
- Measuring clearance as the *mean* distance along the arc, plus the
  intrusion penalty, makes skirting an obstacle beat stopping beside it. The
  robot now swings around a pillar placed dead ahead of its goal
  (`test_closed_loop_swings_around_a_pillar_dead_ahead`).

`blocked` is reported when no admissible arc makes meaningful progress
(v > 0.02 m/s with more than 5 cm of free distance). Creeping a centimetre
towards a wall is not progress.

### What is guaranteed (tested)

- The window matches the wheel-acceleration limits exactly.
- The chosen arc is always admissible: a property test over 200 random
  obstacle clouds, speeds and goals.
- **Closed loop with the real simulator, LiDAR and odometry:**
  - the robot detours around an obstacle partly in its path;
  - it swings around a pillar dead ahead of the goal;
  - it stops in front of an unpassable wall.

  In all three it never touches anything.
- Over 50 simulated minutes in the furnished studio, the robot has **zero
  contacts with anything its LiDAR can see** (acceptance suite).

### Limitations

- **DWA is local.** U-shaped traps and dead ends can still fool it; Milestone 4's
  A\* path (planned on the map) removes most local minima.
- **No reverse.** Reversing is done only by the bump reflex, after a LiDAR check.
- **A static world.** Obstacles are assumed not to move during the 1.5 s horizon.

## Obstacle memory

Between sweeps (5 Hz LiDAR, 10 Hz control) the latest scan is kept in the
*odometry frame* and re-expressed in the current body frame as the robot moves:
motion compensation by odometry. Bump locations are kept as three virtual
points just outside the bumper for 20 s, so DWA steers around socks and
cables the LiDAR cannot see.

## Map-free exploration ("wander", Milestone 2)

A state machine on top of DWA:

- **WAIT** until the first scan.
- **CRUISE** to an exploration goal. Candidate directions every 15°. For each:
  - *how far could the body travel*: a swept disc against the scan points, where
    a point at (along, across) blocks at along − √(R² − across²);
  - *how rarely the robot has been there*: a 0.5 m visit counter in the
    odometry frame;
  - a *random factor* from the robot's own seeded stream.

  The goal is dropped when reached, timed out, stalled (distance not
  shrinking for 4 s) or blocked for 1.5 s.
- **Cramped fallback.** If no direction offers 0.6 m, take a short hop toward
  the most open direction. Without this, a robot nosed between two chair legs
  with a wall behind it turned on the spot forever. That was found by the
  acceptance runs (studio, seed 5: 33 m driven in 10 minutes) and is now a
  regression test.
- **BACK OFF** 12 cm after a bump if the LiDAR shows the space behind is
  clear, then **TURN** 60–150° away from the bumped side.

### Measured performance (10 simulated minutes, default noise, seeds 1–5)

| Room | Accessible floor swept | Bumps per minute | Distance | Contacts with LiDAR-visible things |
|---|---|---|---|---|
| Test Chamber | 78–85 % | 0 | ~160 m | 0 |
| Studio | 69–73 % | 0.2–0.7 | ~160 m | 0 (all 23 contacts: sock and dog bowl) |

Coverage here means the footprint passed over a cell at least once. It is
measured on the *accessible* floor, the cells a robot-sized disc can reach
from the dock (see `evaluation/coverage.py`).

### Limitations

- **Random, not systematic.** Wandering revisits areas; exhaustive coverage
  needs a map and a coverage planner (Milestone 4).
- **Drifting visit counter.** It is dead-reckoned, so over minutes it smears.
- **Invisible clutter.** It is found only by bumping, at about one bump per
  invisible object per 20 s of nearby driving, by design.
