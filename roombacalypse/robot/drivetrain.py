"""Motor model of the differential-drive base.

Turns a requested twist into wheel surface speeds the motors can actually
deliver:

1. **Speed saturation.** If either wheel would exceed ``max_wheel_speed``,
   both wheel speeds are scaled down by the same factor. That keeps the
   *curvature* (the shape of the requested arc) and gives up speed, which is
   what a sensible motor controller does: a turn stays a turn.
2. **Acceleration limit.** Each step, wheel speeds move towards their targets
   by at most ``max_wheel_accel * dt``. When the change is limited, both
   wheels' changes are scaled jointly so they reach their targets together.
   Limiting each wheel independently would briefly bend the path during
   every speed change.

Not modelled (yet): motor time constants, torque limits, wheel slip and
calibration errors. Slip and calibration errors arrive with odometry in
milestone 2, because that is where they matter.
"""

from __future__ import annotations

from roombacalypse.core.config import RobotSpec
from roombacalypse.core.kinematics import twist_to_wheel_speeds
from roombacalypse.core.messages import DriveCommand

WheelSpeeds = tuple[float, float]


class Drivetrain:
    """Wheel-speed saturation and acceleration limiting for a :class:`RobotSpec`."""

    def __init__(self, spec: RobotSpec) -> None:
        self.spec = spec

    def target_wheel_speeds(self, command: DriveCommand) -> WheelSpeeds:
        """Wheel speeds for ``command`` after curvature-preserving saturation."""
        v_left, v_right = twist_to_wheel_speeds(command.linear, command.angular, self.spec.wheel_base)
        peak = max(abs(v_left), abs(v_right))
        limit = self.spec.max_wheel_speed
        if peak > limit:
            scale = limit / peak
            v_left *= scale
            v_right *= scale
        return v_left, v_right

    def step(self, current: WheelSpeeds, target: WheelSpeeds, dt: float) -> WheelSpeeds:
        """Move ``current`` towards ``target`` within the acceleration limit over ``dt``."""
        delta_left = target[0] - current[0]
        delta_right = target[1] - current[1]
        max_delta = self.spec.max_wheel_accel * dt
        peak = max(abs(delta_left), abs(delta_right))
        if peak > max_delta:
            scale = max_delta / peak
            delta_left *= scale
            delta_right *= scale
        return current[0] + delta_left, current[1] + delta_right
