"""Protocols that let the simulator drive the robot's software without importing it."""

from __future__ import annotations

from typing import Protocol

from roombacalypse.core.messages import DriveCommand, SensorFrame


class Controller(Protocol):
    """The robot's software, seen from the simulator.

    The simulation engine calls :meth:`update` at the control rate and holds
    the returned command until the next call (zero-order hold). Implementations
    live on the belief side (``autonomy`` package) and must derive everything
    from the frames they receive.
    """

    name: str
    """Short human-readable mode name shown in the UI (e.g. ``"manual"``)."""

    def reset(self) -> None:
        """Forget all internal state (called on simulation reset)."""
        ...

    def update(self, frame: SensorFrame, dt: float) -> DriveCommand:
        """Consume one sensor frame and return the next drive command.

        ``dt`` is the control period in seconds.
        """
        ...
