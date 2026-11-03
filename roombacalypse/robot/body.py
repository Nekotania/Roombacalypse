"""Ground-truth state of the robot's body."""

from __future__ import annotations

import math
from dataclasses import dataclass

from roombacalypse.core.geometry import Point, Pose2D, wrap_angle


@dataclass(frozen=True, slots=True)
class Contact:
    """Something touching the robot body at the end of a physics step."""

    normal: Point
    """Unit vector from the obstacle surface towards the robot centre."""
    point: Point
    """Contact point on the obstacle surface (world frame)."""
    separation: float
    """Surface gap [m]; about 0 when touching, negative while penetrating."""
    obstacle_id: int | None
    """Id of the obstacle, or ``None`` for a room wall."""

    def bearing(self, pose: Pose2D) -> float:
        """Direction of the contact relative to the robot heading [rad], CCW positive."""
        angle = math.atan2(-self.normal[1], -self.normal[0])
        return wrap_angle(angle - pose.theta)


@dataclass
class RobotBody:
    """Physical state the simulator integrates. The robot's software never sees it."""

    pose: Pose2D
    v_left: float = 0.0
    """Left wheel surface speed [m/s]."""
    v_right: float = 0.0
    """Right wheel surface speed [m/s]."""
    travel_left: float = 0.0
    """Cumulative left wheel rolling distance [m]; what wheel encoders will count."""
    travel_right: float = 0.0
    """Cumulative right wheel rolling distance [m]."""
    linear_velocity: float = 0.0
    """Actual forward speed over the last step, after collisions [m/s]."""
    angular_velocity: float = 0.0
    """Actual yaw rate over the last step [rad/s]."""
    odometer: float = 0.0
    """Total distance actually travelled over the floor [m]."""
    contacts: tuple[Contact, ...] = ()
    """What the body touches at the end of the last step."""
    blocked: bool = False
    """True if the contact solver had to cancel the last step's translation."""

    @property
    def in_contact(self) -> bool:
        return bool(self.contacts)
