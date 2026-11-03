# Collision detection and response

Code: `roombacalypse/environment/shapes.py` (geometry queries) and
`roombacalypse/simulation/physics.py` (the solver). Tests:
`tests/test_shapes.py`, `tests/test_collision.py`.

## Why not a physics engine?

A rigid-body engine (pymunk, Box2D) would bring friction, mass and momentum.
A robot vacuum barely needs them: it moves at 0.3 m/s, its motors stop it
almost instantly, and what matters is that it **never passes through
anything**, **slides along walls** the way a round bumper does, and **reports
where it was touched**. A small purpose-built solver gives exactly that. It is
deterministic, explainable in a page, and adds no dependency (decision D6 in
the architecture document).

## Shapes and heights

- **Robot:** a circle of radius `r`, occupying heights `[0, robot.height)`.
- **Walls:** edges of the floor polygon, zero thickness, unlimited height.
- **Obstacles:** convex polygons or circles, each occupying `[z_min, z_max)`.
  The robot only collides with obstacles whose band overlaps its own, so it
  drives under a bed (0.12 m clearance) but stops at a 1 cm cable.

## Detection

1. **Broad phase:** axis-aligned bounding boxes, cached per obstacle and
   updated whenever an obstacle moves.
2. **Narrow phase:** the signed separation between the robot circle and each
   candidate (the surface gap, negative when overlapping), plus a unit normal
   pointing from the obstacle towards the robot centre:
   - circle vs segment: distance to the closest point on the segment, minus `r`;
   - circle vs circle: centre distance minus both radii;
   - circle vs convex polygon: distance to the closest boundary point, minus `r`.
     If the centre is *inside* the polygon, the separation is
     `−(distance + r)` and the normal points out through the nearest edge.
3. **Contact skin:** anything within `contact_skin` (1 mm) counts as touching,
   but only negative separations are resolved. That way a robot resting against
   a wall keeps its bumper pressed instead of flickering.

## Response, per step

```
proposed pose ← drivetrain + kinematics
repeat up to max_solver_iterations (8):
    contacts ← everything penetrating the robot at its proposed position
    if none: done
    c ← the deepest contact (largest penetration depth)
    correction ← depth(c)
    if c is a movable obstacle and it can be pushed (see below):
        move the obstacle by  mobility × depth  along −normal
        correction ← (1 − mobility) × depth
    move the robot by correction along normal
if still penetrating after all passes:
    cancel this step's translation (keep the rotation; a circle can always turn)
contacts reported ← everything within the skin at the final position
```

Only the component of motion *into* a surface is removed, so a robot meeting a
wall at 30° keeps the tangential component and **slides** along it at
`v · sin 30°`. Tests:

- `test_oblique_hit_slides_along_wall`;
- `test_corner_is_resolved_without_jitter`: two walls, resolved to exactly
  `(4 − r, 3 − r)` with two contacts.

## Pushing movable obstacles: a quasi-static model

Each obstacle has a *mobility* `m ∈ [0, 1]`. Per step, the penetration
`depth` is split:

- the obstacle moves `m · depth` away from the robot;
- the robot is pushed back `(1 − m) · depth`.

The net effect is that a robot pushing an obstacle moves at about `m` times its
free speed: a sock (`m = 0.9`) barely slows it, a trash can (`m = 0.4`) feels
heavy. Before the obstacle moves, its new shape is checked with the separating
axis theorem (SAT) against walls and against every obstacle in its own height
band. If that check fails, the push is cancelled and the robot takes the whole
correction. That is how a crate pushed into a wall stops, and the robot stops
behind it (`test_movable_obstacle_stops_at_wall_and_blocks_robot`).

## Tunnelling bound

Discrete collision detection can miss a collision if a body jumps across a thin
obstacle in a single step. Config validation enforces
`max_wheel_speed × dt < r / 4`; with the defaults that is 1 cm per step
against a 17 cm radius. The robot therefore cannot cross a 3 cm chair leg
between two steps, and the push-out direction is always the correct one.

## What is guaranteed (tested)

- No penetration after any step. Property test: 60 s of random commands in the
  furnished studio, worst separation ≥ −1e-9 m.
- The robot centre never leaves the floor polygon.
- A head-on stop at a pillar lands exactly at `pillar radius + r`.
- Contacts carry their direction, so the bumper (M2) can tell left from right
  and front from rear. Reversing into a wall is a rear contact
  (`test_reversing_into_wall_is_a_rear_contact`).

## Approximations and limitations

- Pushed obstacles translate but never **rotate**. Friction, mass, momentum and
  restitution are not modelled.
- **No chain pushing:** a box pushed into another movable box is blocked.
- A push into a wall is **cancelled entirely** rather than sliding the object
  along the wall. The object can stop a few millimetres short of the wall.
- **No climbing:** anything in the body's height band blocks the robot, even a
  1 cm cable. Real robots climb thresholds up to about 1.5 cm.
- **Turret wedging** (an obstacle between body height and LiDAR-turret height)
  is not modelled yet.
- Obstacles that move *into* the robot by themselves are not handled; none exist yet.
- If eight passes do not converge, the step's translation is cancelled
  (logged at DEBUG). A stress run of 60,000 random-driving steps over 20 seeds
  in the studio never triggered it; the worst separation was −2.8e-16 m.
