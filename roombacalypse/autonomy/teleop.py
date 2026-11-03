"""Manual driving: the human is the controller."""

from __future__ import annotations

from roombacalypse.core.config import TeleopConfig
from roombacalypse.core.messages import DriveCommand, SensorFrame


def _clamp_unit(value: float) -> float:
    return max(-1.0, min(1.0, value))


class TeleopController:
    """Maps a held-key intent to a :class:`DriveCommand`.

    The UI calls :meth:`set_intent` every frame with ``throttle`` and ``turn``
    in [-1, 1]. The controller knows nothing about keyboards or pygame, so it
    can be driven from tests or scripts the same way.
    """

    name = "manual"

    def __init__(self, config: TeleopConfig) -> None:
        self.config = config
        self.throttle = 0.0
        self.turn = 0.0
        self.boost = False

    def set_intent(self, throttle: float, turn: float, boost: bool = False) -> None:
        """Set the driver's intent: +throttle = forward, +turn = left (CCW)."""
        self.throttle = _clamp_unit(throttle)
        self.turn = _clamp_unit(turn)
        self.boost = boost

    def reset(self, seed: int = 0) -> None:
        self.set_intent(0.0, 0.0)

    def update(self, frame: SensorFrame, dt: float) -> DriveCommand:
        speed = self.config.boost_linear_speed if self.boost else self.config.linear_speed
        return DriveCommand(linear=self.throttle * speed, angular=self.turn * self.config.angular_speed)
