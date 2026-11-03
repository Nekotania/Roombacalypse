"""Message types that cross the wall between simulator and robot software.

The robot's software receives a :class:`SensorFrame` and answers with a
:class:`DriveCommand`. Nothing else crosses. Later milestones add sensor
readings (LiDAR scans, bumper, odometry, battery, dirt) as fields of the frame.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class DriveCommand:
    """Velocity command for the drive base (this project's ``cmd_vel``).

    The drivetrain turns it into wheel speeds, enforcing wheel-speed and
    acceleration limits, so a controller can ask for anything and the physics
    decides what actually happens.
    """

    linear: float = 0.0
    """Forward speed [m/s]; negative drives backwards."""
    angular: float = 0.0
    """Yaw rate [rad/s], counter-clockwise (left) positive."""


STOP = DriveCommand()
"""The zero command."""


@dataclass(frozen=True, slots=True)
class SensorFrame:
    """Everything the robot's software may know at one control tick.

    Milestone 1 carries only the clock. Sensors arrive in milestone 2; the
    frame is the *only* channel through which simulator state reaches the
    robot's software.
    """

    time: float
    """Simulation time [s]."""
