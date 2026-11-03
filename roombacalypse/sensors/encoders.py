"""Quadrature wheel encoders: cumulative tick counts per wheel.

Encoders count *wheel rotation*, not motion over the floor. When a wheel
slips, or the robot is pinned against a wall with its wheels still turning,
the ticks keep coming while the robot stands still. That, together with the
calibration errors in :mod:`roombacalypse.robot.wheels`, is what makes
dead reckoning drift.
"""

from __future__ import annotations

import math

from roombacalypse.core.config import EncoderConfig, RobotSpec
from roombacalypse.core.messages import EncoderReading
from roombacalypse.robot.body import RobotBody


class WheelEncoders:
    """Quantises each wheel's cumulative rotation into ticks."""

    def __init__(self, spec: RobotSpec, config: EncoderConfig) -> None:
        self.ticks_per_metre = config.ticks_per_revolution / (2.0 * math.pi * spec.wheel_radius)
        """Ticks per nominal metre of wheel rotation."""

    def read(self, body: RobotBody) -> EncoderReading:
        return EncoderReading(
            math.floor(body.travel_left * self.ticks_per_metre),
            math.floor(body.travel_right * self.ticks_per_metre),
        )
