"""Message types that cross the wall between simulator and robot software.

Each control tick the robot's software receives one :class:`SensorFrame` and
answers with one :class:`DriveCommand`. Nothing else crosses. Readings are
immutable; array fields are made read-only by the sensors that create them.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


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


@dataclass(frozen=True, slots=True, eq=False)
class LidarScan:
    """One 2D LiDAR sweep.

    ``ranges[i]`` is the distance measured along ``angles[i]`` (radians,
    relative to the robot's heading, CCW positive), or ``inf`` when the beam
    returned nothing (out of range or a dropout), following ROS REP 117.
    ``origin_x/origin_y`` is the sensor's position in the robot body frame;
    the robot knows its own geometry.
    """

    time: float
    angles: np.ndarray
    ranges: np.ndarray
    max_range: float
    origin_x: float = 0.0
    origin_y: float = 0.0

    @property
    def valid(self) -> np.ndarray:
        """Boolean mask of beams that returned something."""
        return np.isfinite(self.ranges)

    def points(self) -> np.ndarray:
        """Valid returns as an ``(M, 2)`` array of points in the robot body frame."""
        mask = np.isfinite(self.ranges)
        r = self.ranges[mask]
        a = self.angles[mask]
        return np.column_stack((self.origin_x + r * np.cos(a), self.origin_y + r * np.sin(a)))


@dataclass(frozen=True, slots=True)
class BumperReading:
    """Bumper switches, latched since the previous reading."""

    left: bool = False
    right: bool = False

    @property
    def pressed(self) -> bool:
        return self.left or self.right


@dataclass(frozen=True, slots=True)
class EncoderReading:
    """Cumulative wheel encoder counts (signed)."""

    left: int = 0
    right: int = 0


@dataclass(frozen=True, slots=True)
class SensorFrame:
    """Everything the robot's software may know at one control tick.

    ``lidar`` holds a scan only if one completed since the previous frame,
    so every scan is delivered exactly once; software that needs the latest
    scan between deliveries keeps it.
    """

    time: float
    """Simulation time [s]."""
    encoders: EncoderReading = EncoderReading()
    bumper: BumperReading = BumperReading()
    lidar: LidarScan | None = None
