"""Small ground-truth metrics: penetration depth and pose error."""

from __future__ import annotations

import math

from roombacalypse.core.config import RobotSpec
from roombacalypse.core.geometry import Pose2D, angle_difference
from roombacalypse.environment.shapes import AABB, circle_vs_segment, circle_vs_shape
from roombacalypse.environment.world import World


def min_separation(world: World, pose: Pose2D, spec: RobotSpec, reach: float = 0.05) -> float:
    """Smallest signed gap between the robot body and anything it can collide with.

    Only walls and obstacles within ``reach`` of the body are examined (an
    AABB pre-filter), so the result is ``inf`` in open space. Any penetration
    necessarily involves something within reach, so ``>= 0`` (up to rounding)
    means nothing is being penetrated.
    """
    r = spec.radius
    x, y = pose.x, pose.y
    margin = r + reach
    probe = AABB(x - margin, y - margin, x + margin, y + margin)
    gaps = [
        circle_vs_segment(x, y, r, *wall)[2]
        for wall, box in zip(world.walls, world.wall_boxes, strict=True)
        if box.overlaps(probe)
    ]
    gaps.extend(
        circle_vs_shape(x, y, r, o.shape)[2]
        for o in world.obstacles_in_band(0.0, spec.height)
        if o.aabb.overlaps(probe)
    )
    return min(gaps, default=math.inf)


def pose_error(estimate: Pose2D, truth: Pose2D) -> tuple[float, float]:
    """Position error [m] and absolute heading error [rad] of ``estimate``."""
    return estimate.distance_to(truth), abs(angle_difference(estimate.theta, truth.theta))
