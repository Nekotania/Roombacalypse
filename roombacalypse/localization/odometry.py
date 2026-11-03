"""Dead reckoning from wheel encoders (belief side).

Pose is tracked in the *odometry frame*, whose origin is the robot's pose at
reset (boot, on the dock). The tracker can only use the robot's *nominal*
geometry from its spec sheet, so it inherits every calibration error and
every slip as drift. That drift is visible in the viewer (press E) and is the
reason Milestone 3 adds LiDAR-based localization.
"""

from __future__ import annotations

import math

from roombacalypse.core.config import EncoderConfig, RobotSpec
from roombacalypse.core.geometry import Pose2D
from roombacalypse.core.kinematics import integrate_wheel_travel, wheel_speeds_to_twist
from roombacalypse.core.messages import EncoderReading


class OdometryTracker:
    """Integrates encoder ticks into a pose, plus wheel-speed and twist estimates."""

    def __init__(self, spec: RobotSpec, encoders: EncoderConfig) -> None:
        self.wheel_base = spec.wheel_base
        self.metres_per_tick = 2.0 * math.pi * spec.wheel_radius / encoders.ticks_per_revolution
        self.pose = Pose2D(0.0, 0.0, 0.0)
        self.wheel_speeds: tuple[float, float] = (0.0, 0.0)
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.distance = 0.0
        self._last: EncoderReading | None = None
        self._last_time = 0.0

    def reset(self) -> None:
        self.pose = Pose2D(0.0, 0.0, 0.0)
        self.wheel_speeds = (0.0, 0.0)
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.distance = 0.0
        self._last = None
        self._last_time = 0.0

    def update(self, reading: EncoderReading, time: float) -> Pose2D:
        """Fold in a new encoder reading taken at ``time``; returns the updated pose."""
        if self._last is None:
            self._last, self._last_time = reading, time
            return self.pose
        d_left = (reading.left - self._last.left) * self.metres_per_tick
        d_right = (reading.right - self._last.right) * self.metres_per_tick
        dt = time - self._last_time
        self.pose = integrate_wheel_travel(self.pose, d_left, d_right, self.wheel_base)
        if dt > 0.0:
            self.wheel_speeds = (d_left / dt, d_right / dt)
            self.linear_velocity, self.angular_velocity = wheel_speeds_to_twist(
                self.wheel_speeds[0], self.wheel_speeds[1], self.wheel_base
            )
        self.distance += 0.5 * abs(d_left + d_right)
        self._last, self._last_time = reading, time
        return self.pose
