"""Dead reckoning from wheel encoders and, if fitted, the gyro (belief side).

Pose is tracked in the *odometry frame*, whose origin is the robot's pose at
reset (boot, on the dock). The tracker can only use the robot's *nominal*
geometry from its spec sheet, so it inherits every calibration error and
every slip as drift.

**Gyrodometry.** With a gyro, heading changes come from the gyro and only
the distance comes from the wheels. Wheel heading is the weak part of wheel
odometry: a 0.3 % difference between the wheel radii bends a straight line
by 0.7° per metre, and the true wheel base is uncertain by 1 %. A MEMS gyro
does not care about either, nor about a wheel spinning against a wall.

**Bias calibration.** A gyro's bias (here up to about ±1 °/s) would add
hundreds of degrees of heading error in ten minutes, so it has to be
measured. A robot knows when it is standing still: the encoders do not
move. The bias is averaged over the first ``gyro_calibration_time`` seconds
of standing still (on the dock at boot; the autonomous behaviour waits for
it, see :attr:`OdometryTracker.calibrating`), then nudged towards the
measured rate whenever the wheels stand still again (a zero-velocity
update). While the wheels stand still the heading is held, too.

The tracker also estimates wheel speeds and the body twist, which the DWA
planner uses as its "current velocity".
"""

from __future__ import annotations

import math

from roombacalypse.core.config import EncoderConfig, OdometryConfig, RobotSpec
from roombacalypse.core.geometry import Pose2D
from roombacalypse.core.kinematics import arc_displacement, wheel_speeds_to_twist
from roombacalypse.core.messages import EncoderReading, GyroReading


class OdometryTracker:
    """Integrates encoder ticks (and gyro angle) into a pose, plus wheel-speed and twist estimates."""

    def __init__(self, spec: RobotSpec, encoders: EncoderConfig, config: OdometryConfig | None = None) -> None:
        self.wheel_base = spec.wheel_base
        self.metres_per_tick = 2.0 * math.pi * spec.wheel_radius / encoders.ticks_per_revolution
        self.config = config if config is not None else OdometryConfig()
        self.reset()

    def reset(self) -> None:
        self.pose = Pose2D(0.0, 0.0, 0.0)
        self.wheel_speeds: tuple[float, float] = (0.0, 0.0)
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.distance = 0.0
        self.gyro_bias = 0.0
        """Estimated gyro bias [rad/s]."""
        self.calibration_time = 0.0
        """Seconds of standing still that went into the bias estimate."""
        self.using_gyro = False
        """True once gyro readings arrive and ``use_gyro`` is on."""
        self._last: EncoderReading | None = None
        self._last_gyro: GyroReading | None = None
        self._last_time = 0.0

    @property
    def calibrating(self) -> bool:
        """True while the gyro bias is still being measured (never without a gyro)."""
        return self.using_gyro and self.calibration_time < self.config.gyro_calibration_time

    def update(self, reading: EncoderReading, time: float, gyro: GyroReading | None = None) -> Pose2D:
        """Fold in new encoder (and gyro) readings taken at ``time``; returns the updated pose."""
        if self._last is None:
            self._last, self._last_gyro, self._last_time = reading, gyro, time
            self.using_gyro = gyro is not None and self.config.use_gyro
            return self.pose
        d_left = (reading.left - self._last.left) * self.metres_per_tick
        d_right = (reading.right - self._last.right) * self.metres_per_tick
        dt = time - self._last_time
        distance = 0.5 * (d_left + d_right)
        dtheta = (d_right - d_left) / self.wheel_base
        if self.using_gyro and gyro is not None and self._last_gyro is not None:
            dtheta = self._gyro_turn(gyro.angle - self._last_gyro.angle, dt, still=reading == self._last)
        dx, dy = arc_displacement(distance, dtheta)
        x, y = self.pose.transform_point(dx, dy)
        self.pose = Pose2D(x, y, self.pose.theta + dtheta)
        if dt > 0.0:
            self.wheel_speeds = (d_left / dt, d_right / dt)
            self.linear_velocity, self.angular_velocity = wheel_speeds_to_twist(
                self.wheel_speeds[0], self.wheel_speeds[1], self.wheel_base
            )
            if self.using_gyro:
                self.angular_velocity = dtheta / dt
        self.distance += 0.5 * abs(d_left + d_right)
        self._last, self._last_gyro, self._last_time = reading, gyro, time
        return self.pose

    def _gyro_turn(self, raw: float, dt: float, *, still: bool) -> float:
        """Heading change from the gyro, learning its bias whenever the wheels stand still."""
        if not still or dt <= 0.0:
            return raw - self.gyro_bias * dt
        rate = raw / dt
        if self.calibration_time < self.config.gyro_calibration_time:  # boot: a plain average
            total = self.calibration_time + dt
            self.gyro_bias += (rate - self.gyro_bias) * dt / total
            self.calibration_time = total
        else:  # later stops: a slow exponential update
            self.gyro_bias += (rate - self.gyro_bias) * min(1.0, dt / self.config.bias_time_constant)
        return 0.0  # wheels still: the robot is not turning (zero-velocity update)
