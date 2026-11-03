"""Front bumper with two switches, like a Roomba's.

The bumper covers an arc of the front of the body. A contact on the left of
the arc closes the left switch, on the right the right switch, and a hit
close to straight ahead (within the dead band) closes both. Contacts outside
the arc, such as reversing into a wall, are not felt at all, as on the real
robot.

Switches are *latched*: a contact during any physics step counts until the
robot's software reads the bumper, so short taps between two control ticks
are not lost.
"""

from __future__ import annotations

import math

from roombacalypse.core.config import BumperConfig
from roombacalypse.core.messages import BumperReading
from roombacalypse.robot.body import RobotBody


class Bumper:
    """Left/right bumper switches derived from body contacts."""

    def __init__(self, config: BumperConfig) -> None:
        self.half_arc = 0.5 * math.radians(config.arc_deg)
        self.deadband = math.radians(config.center_deadband_deg)
        self._left = False
        self._right = False

    def observe(self, body: RobotBody) -> None:
        """Latch switches for the contacts present after a physics step."""
        for contact in body.contacts:
            bearing = contact.bearing(body.pose)
            if abs(bearing) > self.half_arc:
                continue
            if bearing > self.deadband:
                self._left = True
            elif bearing < -self.deadband:
                self._right = True
            else:
                self._left = self._right = True

    def read(self) -> BumperReading:
        """Return the latched state and clear the latches."""
        reading = BumperReading(self._left, self._right)
        self._left = self._right = False
        return reading

    def reset(self) -> None:
        self._left = self._right = False
