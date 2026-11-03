"""Contact detection and response for the round robot body.

This is a small purpose-built solver, not a rigid-body engine; see
``docs/algorithms/collision.md`` for the reasoning and the limitations.

Per step, after the drivetrain proposes a new pose:

1. **Detect.** Find walls and obstacles (in the body's height band) whose
   signed separation from the robot circle is below ``contact_skin``.
2. **Resolve.** While anything penetrates, take the deepest contact and move
   the robot out along its normal (position projection). If the obstacle is
   movable, first try to push it away by ``mobility × depth``; the robot
   absorbs the rest. A push that would make the obstacle hit a wall or
   another obstacle is cancelled and the robot takes the whole correction.
3. **Fallback.** If penetration remains after ``max_solver_iterations``
   passes, cancel the step's translation and keep the rotation. A circle can
   always turn in place.

Because the correction only removes the component of motion *into* a
surface, a robot meeting a wall at an angle slides along it.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass

from roombacalypse.core.config import PhysicsConfig, RobotSpec
from roombacalypse.core.geometry import Pose2D
from roombacalypse.environment.shapes import AABB, circle_vs_segment, circle_vs_shape
from roombacalypse.environment.world import Obstacle, World
from roombacalypse.robot.body import Contact

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class MotionResult:
    """Outcome of resolving one proposed motion."""

    pose: Pose2D
    contacts: tuple[Contact, ...]
    pushed: tuple[int, ...]
    blocked: bool


class ContactSolver:
    """Resolves the robot circle against a :class:`World`."""

    def __init__(self, robot: RobotSpec, config: PhysicsConfig) -> None:
        self.radius = robot.radius
        self.height = robot.height
        self.skin = config.contact_skin
        self.max_iterations = config.max_solver_iterations

    def find_contacts(self, world: World, x: float, y: float) -> list[Contact]:
        """All walls/obstacles within ``contact_skin`` of a robot centred at (x, y)."""
        r = self.radius
        reach = r + self.skin
        probe = AABB(x - reach, y - reach, x + reach, y + reach)
        contacts: list[Contact] = []
        for (ax, ay, bx, by), box in zip(world.walls, world.wall_boxes, strict=True):
            if box.overlaps(probe):
                nx, ny, separation, qx, qy = circle_vs_segment(x, y, r, ax, ay, bx, by)
                if separation < self.skin:
                    contacts.append(Contact((nx, ny), (qx, qy), separation, None))
        for obstacle in world.obstacles_in_band(0.0, self.height):
            if obstacle.aabb.overlaps(probe):
                nx, ny, separation, qx, qy = circle_vs_shape(x, y, r, obstacle.shape)
                if separation < self.skin:
                    contacts.append(Contact((nx, ny), (qx, qy), separation, obstacle.id))
        return contacts

    def resolve(self, world: World, previous: Pose2D, proposed: Pose2D) -> MotionResult:
        """Move from ``previous`` towards ``proposed`` without penetrating anything."""
        x, y = proposed.x, proposed.y
        pushed: set[int] = set()
        resolved = False
        for _ in range(self.max_iterations):
            penetrating = [c for c in self.find_contacts(world, x, y) if c.separation < 0.0]
            if not penetrating:
                resolved = True
                break
            deepest = min(penetrating, key=lambda c: c.separation)
            depth = -deepest.separation
            correction = depth
            if deepest.obstacle_id is not None:
                obstacle = world.obstacle(deepest.obstacle_id)
                if obstacle.movable and self._push(world, obstacle, deepest, depth):
                    pushed.add(obstacle.id)
                    correction = depth * (1.0 - obstacle.mobility)
            x += deepest.normal[0] * correction
            y += deepest.normal[1] * correction

        blocked = False
        if not resolved and any(c.separation < -1e-9 for c in self.find_contacts(world, x, y)):
            logger.debug("contact solver did not converge at (%.3f, %.3f); cancelling translation", x, y)
            x, y = previous.x, previous.y
            blocked = True
        final = Pose2D(x, y, proposed.theta)
        return MotionResult(final, tuple(self.find_contacts(world, x, y)), tuple(sorted(pushed)), blocked)

    @staticmethod
    def _push(world: World, obstacle: Obstacle, contact: Contact, depth: float) -> bool:
        """Try to slide ``obstacle`` away from the robot; return False if it is blocked."""
        shift = depth * obstacle.mobility
        moved = obstacle.shape.translated(-contact.normal[0] * shift, -contact.normal[1] * shift)
        if world.is_shape_free(moved, obstacle.z_min, obstacle.z_max, ignore=(obstacle.id,)):
            world.move_obstacle(obstacle.id, moved)
            return True
        return False
