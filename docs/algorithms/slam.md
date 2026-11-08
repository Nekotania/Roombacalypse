# Graph SLAM: mapping and localization

Code:

- `roombacalypse/mapping/occupancy_grid.py`: the robot's log-odds occupancy grid;
- `roombacalypse/mapping/likelihood_field.py`: surface fields that scans are matched against;
- `roombacalypse/mapping/pose_graph.py`: the pose-graph optimiser (the SLAM back end);
- `roombacalypse/localization/scan_matcher.py`: correlative search and point-to-line ICP;
- `roombacalypse/localization/slam.py`: `GraphSlam`, which ties them together;
- `roombacalypse/evaluation/slam_metrics.py`: ground-truth scoring.

Tests: `tests/test_mapping.py`, `tests/test_scan_matching.py`, `tests/test_slam.py`,
`tests/test_slam_metrics.py`, and the M3 acceptance runs in `tests/test_acceptance.py` (slow).

## Why graph SLAM (decision D7)

The director delegated the choice with one requirement: *something that works
in actual robotics, not just simulations*. The LiDAR SLAM systems that run on
real robots today are graph-based: Karto and its successor slam_toolbox (the
ROS 2 navigation default) and Google's Cartographer. They share one structure,
which is what is built here:

- a **front end** that turns each new scan into a pose by matching it against
  recent scans (scan-to-map tracking);
- a **back end** that keeps every keyframe pose in a graph, recognises places
  the robot has seen before (**loop closure**), and re-optimises the whole
  trajectory when it does.

The alternative recommended at the end of M2 (option (a): a particle filter
localising against the robot's own growing map) cannot correct an error once
it is in the map, and the director also asked for a multi-room apartment,
where that matters most. A Rao-Blackwellised particle filter (GMapping) can
close loops, but its particles deplete on long loops and it is slow in
Python. Graph SLAM can correct mapped errors, and it is what the industry
converged on.

## One control tick

```
odometry (encoders) ──► predicted pose = correction ∘ odometry
scan arrived?
 ├─ no ─────────────────────────────────────────────► pose = prediction
 └─ yes: thin it (4 cm voxels); keyframe due?   (moved 0.25 m, turned 15°,
         │                                      or the scan no longer fits
         │                                      at the prediction: slip)
         └─ yes: match against the local map (ICP from the prediction;
                 a correlative search if the fit is poor)
                 → new keyframe = pose-graph node + constraint to the previous one
                 → correction = matched pose ∘ odometry⁻¹
                 → insert the scan into the occupancy grid
                 → every 2 m of travel: try to close a loop;
                   if accepted, re-optimise the graph
```

The robot's pose is reported in the **map frame**, whose origin is its boot
pose on the dock. `correction` is the `map → odom` transform of ROS: odometry
stays smooth for control (the obstacle memory and bump reflexes keep using
it), while the map-frame pose jumps a few millimetres at each keyframe and a
few centimetres at a loop closure.

## The occupancy grid

Log odds `l = log(p / (1 − p))` per 5 cm cell (Elfes 1989; *Probabilistic
Robotics* ch. 9). For every beam with a return:

| Cell | Update | Probability after one scan |
|---|---|---|
| where the return fell | `+0.85` | 0.70 |
| crossed on the way (sampled every cell length, stopping half a cell short of the return) | `−0.4` | 0.40 |

- Each cell is updated **at most once per scan** (fancy-index `+=` with repeated
  indices adds once), and a cell both crossed and hit counts as hit. A scan
  is one measurement of each cell it sees.
- Log odds are **clamped to [−2, 3.5]**, so a pushed box or a moved chair can be
  forgotten (or learnt) within a few scans. A test drives this.
- **No-return beams are ignored.** A 1 % dropout through a wall must not punch
  a hole in it; treating "no return" as "free up to 6 m" would.
- Cells count as occupied above 0.6 (p > 0.65) and free below −0.6 (p < 0.35).
  One scan therefore marks hits occupied but leaves crossed cells uncertain.
- The grid **grows** in 2 m steps, keeping cell boundaries on multiples of the
  resolution.

The grid is rendered from the keyframes at their current poses. While no
loop closure has moved them, each new keyframe is inserted incrementally.
After a loop closure the grid is marked stale and re-rendered from all
keyframes when someone next reads it: once at the end of a headless run, and
at most every 4 s of wall time in the viewer. A re-render costs about 1 ms
per keyframe (0.9 s for 850 keyframes).

## Surface fields: what scans are matched against

A field is built from *reference* scans (the local map, or an old stretch of
trajectory for loop closure), placed at their keyframe poses. It answers two
questions.

**Coarse: how likely is a point here?** `L = exp(−d²/2σ²)` with σ = 5 cm on a
2.5 cm grid, where `d` is the distance from a cell centre to the nearest
occupied cell (a Euclidean distance transform). This is the likelihood field
of *Probabilistic Robotics* §6.4 and the lookup table of Olson's correlative
matcher. It is used only by the exhaustive search, where being right to a
cell is enough.

**Fine: how far is this point from the surface?** Every occupied cell holds a
*surface element* fitted to all reference points in its 5 × 5-cell
neighbourhood (12.5 cm), pooled over every scan in the map:

| Element | Test | Residual |
|---|---|---|
| line | ≥ 4 points, scatter ten times longer than wide (PCA) | `n · (x − p)`: point-to-line |
| point | ≥ 4 points, scatter std < 2.5 cm (a chair leg) | `x − p` as a vector: point-to-point |
| blob | anything else (a corner, a smudge) | none: it constrains no direction |

Where a 5 × 5 fit is a blob, the fit is repeated over 9 × 9 cells (22.5 cm).
That is long enough for a far wall's direction to show through 3 cm of range
noise, and most blobs become lines. The distance transform's *feature map*
gives every cell its nearest occupied cell, so a query point finds its
surface element in O(1). Points more than 3σ from their element are outliers.

Reference scans are first turned into continuous surfaces: consecutive
returns closer than about two beam spacings are joined, and the segment
between them is sampled at its midpoints. Depth discontinuities (a leg in
front of a wall) stay separate.

## Scan matching

**1. Correlative search** (Olson, ICRA 2009). Every pose in a window is scored
by the mean coarse likelihood of the scan's points. Translations step by one
field cell. Rotations step by the angle that moves the 90th-percentile point
by one cell (about 0.4°). It is exhaustive within the window, so it cannot be
fooled by a local minimum. **Two-level branch and bound** keeps it affordable:

- blocks of k × k translations (k = 4, or 8 for windows over 30 cm) are scored
  on a *block-maximum* field, an upper bound on every pose in the block;
- blocks are expanded in order of their bound until the bound falls below the
  best pose found;
- a 2 % tie-breaking penalty at the window edge prefers the prediction.

**2. Point-to-line ICP** (Censi's PL-ICP, with correspondences from the
closest-surface map instead of a KD-tree). The solver alternates two steps:

- re-associate each point with its nearest surface element;
- with associations frozen, take two Gauss-Newton steps on
  `Σ r_k² / Nσ² + prior`, where `r_k` are the line or point residuals above.

It converges when a round moves less than 20 µm (or 10⁻⁵ rad), or after
10 rounds. The prior is a weak pull toward the odometry prediction
(0.5 m⁻², 0.5 rad⁻²). It matters only where the scan is silent, along a
featureless corridor. After a correlative search, the search result becomes
the anchor instead, because the search has already chosen the mode.

**Information.** A match's information matrix is the ICP one:
`Σ_k J_kᵀJ_k / N / σ_p²`, where `J_k` is the residual's derivative with
respect to the pose and σ_p = 1 cm is the expected point-to-surface
disagreement. It is large across walls and zero along a corridor (tested).

**Ambiguity** (loop closure only). After the search, a second branch and
bound looks for the best pose clearly different from the winner: more than
3σ away in translation, or by the matching rotation. If that pose scores
over 90 % of the winner, the match is ambiguous and rejected (tested in a
corridor).

## Front end: tracking and keyframes

- **Keyframes** are added after 0.25 m of travel or 15° of turn. They are
  also added when a scan fits the local map at the predicted pose less than
  85 % as well as the last keyframe did. That catches wheel slip (pushing
  against a wall, the encoders counting motion that did not happen) within
  one scan. The check costs about 0.5 ms per scan.
- **Local map:** the surfaces of the last 20 keyframes, within 8 m. It is
  rebuilt every third keyframe and after every loop closure.
- **Tracking:** ICP from the prediction. If the fit (mean likelihood) is
  below 0.5, a correlative search over ±0.25 m and ±8° runs, and wins if it
  fits better by at least 0.05.
- **Constraint to the previous keyframe:** the matched relative pose, whose
  information is the match's information plus the robot's own odometry
  noise model (3 % of distance, 3 % of rotation, 1.5°/m). The odometry part
  carries the direction the scan cannot see, for example along a corridor.
- **Misfit:** a scan fitting worse than 0.4, or worse than 0.6 × the median
  fit of the last 10 tracked keyframes, is not trusted. The keyframe takes
  the odometry prediction, joins the graph with an odometry-only constraint,
  and stays out of the map and the local map; every following scan is
  matched until one fits again. (M3 used 0.2. Measured in M4 over 13 runs of
  10–20 minutes in all rooms, tracked keyframes never fit worse than 0.53,
  while the first scans after a kidnapping fit 0.06–0.21, *or* 0.42–0.43 in
  the near-symmetric test chamber. A kidnapped scan at 0.21, and later one at
  0.43, went into the map at the wrong place, and every later scan fitted the
  corrupted local map perfectly: hence the relative test.)
- **Innovation gate:** a match more than the tracking window (0.25 m, 8°) from
  the prediction is a misfit too, however well it fits: odometry with a gyro
  cannot be that wrong between two keyframes, so the match came from somewhere
  else. Measured: in the chamber, a robot carried 0.9 m to a spot whose view
  mirrors its old one was pulled by ICP (which converges from tens of
  centimetres off) to that mirror image, 0.5 m and 16° from where it stood
  still, with a fit of 0.80; only the size of the correction gave it away.
  In normal runs the gate never fires (the scorecards are bit-identical with
  and without it).
- **Lost:** three misfits in a row. See "Lost and found" below.

## Back end: loop closure and the pose graph

Every 2 m of travel, SLAM looks for a loop:

1. **Candidates** are old keyframes within 2.5 m of the newest one, and at
   least 5 m of travel back along the trajectory.
2. The **reference** is the nearest candidate and up to 10 keyframes either
   side of it (an earlier visit to the same place), as a surface field.
3. A **correlative search** over ±0.5 m and ±12°, with ICP refinement.
4. The match is **accepted** only if all of these hold: its fit is at least
   0.6, its ambiguity is at most 0.9, and its optimum is not on the edge of
   the window (the truth might lie outside).
5. The **constraint** goes from the nearest candidate to the newest keyframe,
   with a Cauchy robust kernel. The graph is re-optimised, the correction is
   updated, and the grid is marked stale.

**The optimiser** (Grisetti et al., *A Tutorial on Graph-Based SLAM*, 2010)
minimises `Σ ρ(eᵀ Ω e)`, where `e = t2v(Z⁻¹ · X_i⁻¹ · X_j)`. Jacobians of all
edges are computed at once with NumPy and assembled into a sparse system,
which is solved with Levenberg-Marquardt and SuperLU. The Cauchy kernel
`c² log(1 + s/c²)` (c = 2σ) is applied by iteratively reweighted least squares.
Node 0, the boot pose on the dock, is fixed: it defines the map frame.

Tested:

- Jacobians against finite differences;
- a drifted square closes to within 2 cm;
- a deliberately wrong loop closure is shrugged off with the kernel
  (< 5 cm error) but drags the square more than 30 cm without it.

## Lost and found: the kidnapped robot (M4)

The **kidnapped robot problem** (Engelson & McDermott 1992): the robot is
picked up and put down somewhere else (`K` in the viewer, `Simulation.kidnap`
in tests), and nothing tells it. The wheels do not turn while it is carried,
so odometry and the gyro report nothing; only the LiDAR sees a different
place. (Real robot vacuums also have wheel-drop switches; this one has to work
it out from the scans, which is the hard version.)

**Detection.** After three misfits in a row, SLAM declares itself *lost*,
stops adding keyframes, and the executive stops using the pose (see
[planning.md](planning.md)). Measured: 0.4 s after the kidnapping, every
time it was noticed.

**Global search** (`localization/relocalization.py`). Tracking searches a few
centimetres around the prediction; a lost robot must search the *whole* map.
This is Cartographer's approach (Hess et al. 2016) on the occupancy grid:

- The score of a pose is the mean of `exp(−d²/2σ²)` (σ = 7.5 cm) over the scan,
  thinned to one point per 10 cm, where `d` is the distance to the nearest
  occupied cell.
- Candidates: every known-free cell with at least 10 cm of room, times every
  heading at a step that moves the bulk of the scan by one cell (about 1°):
  7–13 million poses in the apartment.
- **Branch and bound:** blocks of 8×8 positions are bounded by a block-maximum
  grid and expanded best bound first; once the next bound is no better than
  the best pose found, nothing left can beat it. The result is the exact
  optimum of the exhaustive search (tested), after scoring about 3–8 % of it.
- **Ambiguity:** the search is repeated with the winner's neighbourhood (0.5 m,
  20°) excluded, stopping early above 0.85 × the winner (ambiguous) and never
  looking below 0.75 × (clearly not). Cost: 0.3–1 s per search on the
  apartment (one search blocks one control tick), 40–70 ms in the chamber.

**Acceptance.** The best pose is refined against the surface map of the 40
nearest keyframes (correlative ±0.25 m, ±8°, then ICP). It is accepted if the
refined fit is at least 0.6 and the ambiguity at most 0.85. It becomes a new
keyframe, joined to the nearest old keyframe by a robust *relocalisation*
constraint (there is no odometry to join it to the keyframes before the
kidnapping); the correction jumps, and tracking resumes. While lost, a new
search runs only when the view has changed (25 cm or 0.5 rad of odometry)
or every 10 s: the same view gives the same answer, and searches are not free.

**Measured** (kidnapped five times per run, every 120 s from t = 300 s, to a
random spot on open floor; a run ends at 1000 s):

| Room | Kidnappings | Noticed after | Found after | Error when found | Wrong relocalisations |
|---|---|---|---|---|---|
| Studio, seeds 1–3 | 15 | 0.4 s | 0.2 s (first search, every time) | 0.2–1.3 cm | 0 |
| Apartment, seeds 1–3 | 15 | 0.4 s | 0.2 s; up to 4 min if put down in a room not yet mapped | 0.3–10 cm | 0 |
| Test Chamber, seeds 1–8 | 40 | 0.4 s (a few landed while it was still lost) | 5–130 s: it waits, hopping, for a view the pillar and crate disambiguate | 0.1–0.9 cm | 0 |

Every run ended with the robot correctly localised (final error 0.1–8.8 cm)
and with no contact with anything the LiDAR can see.

## Evaluation (ground truth, harness only)

- **Trajectory error** is measured in the world frame through the boot pose,
  *without* alignment. A rotated map counts, because it would matter for
  driving home. The aligned ATE (Horn's method, as in SLAM benchmarks) is
  reported alongside.
  - *Online* error is the pose the robot used for control, at every control tick.
  - *Keyframe* error is the final, optimised trajectory.
- **Map precision:** an occupied cell is wrong if its centre lies in *free
  space* more than one cell (5 cm) from any real surface in the scan plane: a
  phantom obstacle. Range noise thickens every occupancy grid by a cell or two
  *into* solid matter (behind walls, inside furniture), where no beam can clear
  it. Such cells count as right up to 15 cm deep: harmless, and real walls are
  thicker than that. The strict variant (within 5 cm of a surface on either
  side) is reported as "surface only".
- **Map recall:** the fraction of surface the robot actually saw that has an
  occupied cell centre within 5 cm. "Saw" means the returns placed at the
  *true* scan pose, on a 1 cm raster.

## Measured performance

See the tables in [ROADMAP.md](../ROADMAP.md) (M3, measured acceptance). In
summary, over 10 minutes of autonomous wandering with default noise, seeds 1–5:

| Room | SLAM pose error, online RMS (max) | Keyframe ATE (aligned) | Map precision (strict) | Map recall | Dead reckoning after 10 min |
|---|---|---|---|---|---|
| Test Chamber | 0.3–0.5 cm (≤ 1.4 cm) | 0.3–0.5 cm (0.2 cm) | 99.9–100 % (80–82 %) | 95.7–97.1 % | 0.3–3.6 m |
| Studio | 0.5–1.4 cm (≤ 2.9 cm) | 0.5–1.6 cm (0.4–0.5 cm) | 99.9–100 % (73–75 %) | 89.3–92.0 % | 0.7–5.2 m |
| Apartment | 2.9–5.3 cm (≤ 9.2 cm) | 2.6–5.7 cm (1.6–2.8 cm) | 95.6–99.8 % (84–89 %) | 86.8–92.0 % | 0.8–11.3 m |

Heading error is 0.4–0.6° RMS online in every room. The apartment's larger
absolute error is almost entirely a rotation of the whole map about the dock
(−0.5° in the worst seed), picked up crossing the hallway. The map stays
consistent: the aligned ATE is 1.6–2.8 cm. Cost: SLAM adds about 15–20 s of
computation per simulated 10 minutes (850–950 keyframes, 55–70 loop closures),
and a headless run takes about 30 s on the development container.

## Found and fixed during M3

Most of these were found by asking why a number was not as good as it should
be. Each is now covered by a test or a measurement:

- **Cell-centre surfaces limit accuracy.** With perfect scans and a perfect
  prior, matching against cell centres moved the pose 11.5 mm RMS: the
  staircase of a wall that is not aligned with the grid. Fixed with sub-cell
  surface elements (point-to-line). Keyframe error with perfect sensors fell
  from 14.6 mm to 3.3 mm.
- **A quartic cost and fake information.** The first refinement minimised
  `Σ(1 − L)²`, which is flat (quartic) at the optimum. Its `JᵀJ` then mostly
  measured small bumps of the field along walls. A corridor reported *more*
  information along its length than across. Fixed with point-to-line
  residuals and ICP information.
- **A chiral sampling bias.** Surfaces were sampled from each return towards
  the next, so every surface sat half a step clockwise around the sensor: a
  steady −0.005° per keyframe, the same sign on every seed. Fixed by sampling
  segment midpoints.
- **Noisy normals.** Normals from pairs of neighbouring returns were 20–30°
  off, which turned walls into sawtooths of local minima. Refinement then
  stopped short on the odometry side: 6.7 % of each odometry heading error
  survived. Fixed with PCA over a pooled neighbourhood.
- **Solver stalls at cell boundaries.** Re-associating inside a
  Levenberg-Marquardt line search made the cost jump at every Voronoi
  boundary between cells. Fixed by alternating frozen-association
  Gauss-Newton with re-association (ICP proper). Started from the truth or
  from the prediction, it now reaches the same answer.
- **A V-shaped residual.** Point elements used the distance `|x − p|`, and
  Gauss-Newton overshot back and forth on it. Two inner iterations were
  worse than one or three, which gave it away. Fixed with vector residuals.
- **Fake features along walls.** Lone far returns, and far walls blurred by
  3 cm of noise, acted as point features and pinned a corridor along its
  length. Fixed with line/point/blob classification and the 9 × 9 refit.
- **A branch-and-bound border bug.** Blocks starting just outside the field
  were bounded by zero, which could prune the true optimum. Found by a
  property test; fixed by padding the block-maximum grid.
- **Phantom travel.** Pushing a wall at full speed with spinning wheels let the
  online pose run 22 cm ahead before the next keyframe. Fixed with the
  per-scan slip check (5 cm worst case, one scan period at 0.5 m/s).
- **The precision metric.** Map precision measured 74 % at first. Every one of
  the "wrong" cells was surface thickening into solid matter; none was in free
  space. The metric now counts what matters (phantom obstacles) and still
  reports the strict number.

## Limitations

- **Relocalisation needs a mapped place.** A robot set down in a room it has
  never seen stays lost until it hops somewhere it has (it does not start a
  second map and merge it later, as multi-session SLAM would).
- **Symmetry cannot be broken.** In a near-symmetric room the robot waits for
  a telling view; in a perfectly symmetric one it would wait forever (and ask
  for help). A kidnapping within the tracking window (25 cm, 8°) is treated as
  a slip and corrected, not noticed as a kidnapping.
- **Loop closure has a window.** Drift beyond ±0.5 m and ±12° between a place
  and its revisit cannot be closed. Apartment-sized loops stay well inside it.
- **Absolute error grows away from the dock.** It is largest after a long
  corridor and is corrected when a loop through known space closes. In the
  apartment it reaches about 5 cm in the far office. The map stays
  self-consistent: aligned ATE is about half the anchored one.
- **Re-rendering costs about 1 ms per keyframe.** That is fine for M3's
  sessions (850 keyframes in 10 minutes); hour-long runs would want
  Cartographer-style submaps.
- **The world is assumed static.** Moved furniture is forgotten only through
  log-odds clamping, and scan matching treats it as outliers.
- **No motion distortion and one session.** The simulated sweep is
  instantaneous, and the map is not saved between runs.
