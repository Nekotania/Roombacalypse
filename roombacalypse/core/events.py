"""Event log: notable things that happened, for statistics, the UI and (M6) the personality.

Both sides of the wall write here (the simulator reports collisions, the
robot reports bumps and behaviour changes), but the robot's software only
ever receives a *publish* function, never the log itself, so it cannot read
what the simulator wrote.
"""

from __future__ import annotations

from collections import Counter, deque
from collections.abc import Mapping
from dataclasses import dataclass, field
from enum import StrEnum
from typing import Any, Protocol


class EventKind(StrEnum):
    RESET = "reset"
    COLLISION = "collision"  # simulator: the body started touching something
    BUMP = "bump"  # robot: a bumper switch closed
    MODE = "mode"  # robot: manual <-> autonomous
    BEHAVIOUR = "behaviour"  # robot: new goal, escape manoeuvre, stall, ...
    MAP = "map"  # robot: loop closed (map corrected), scan did not fit the map
    WORLD_EDIT = "world_edit"  # user: obstacle placed or removed


@dataclass(frozen=True, slots=True)
class Event:
    seq: int
    """Sequence number, counting every event since the last clear."""
    time: float
    """Simulation time [s]."""
    source: str
    """``"sim"``, ``"robot"`` or ``"user"``."""
    kind: EventKind
    message: str
    data: Mapping[str, Any] = field(default_factory=dict)


class EventPublisher(Protocol):
    """Write-only handle given to code that may report events but not read them."""

    def __call__(self, time: float, kind: EventKind, message: str, **data: Any) -> None: ...


class EventLog:
    """Bounded, append-only log. Consumers keep a cursor and call :meth:`since`."""

    def __init__(self, capacity: int = 500) -> None:
        self._events: deque[Event] = deque(maxlen=capacity)
        self._next_seq = 0
        self._counts: Counter[EventKind] = Counter()

    def emit(self, time: float, source: str, kind: EventKind, message: str, **data: Any) -> Event:
        event = Event(self._next_seq, time, source, kind, message, dict(data))
        self._events.append(event)
        self._next_seq += 1
        self._counts[kind] += 1
        return event

    def publisher(self, source: str) -> EventPublisher:
        """A publish-only function that stamps events with ``source``."""

        def publish(time: float, kind: EventKind, message: str, **data: Any) -> None:
            self.emit(time, source, kind, message, **data)

        return publish

    def since(self, cursor: int) -> tuple[list[Event], int]:
        """Events with ``seq >= cursor`` still held, and the cursor for next time."""
        return [e for e in self._events if e.seq >= cursor], self._next_seq

    def recent(self, count: int) -> list[Event]:
        return list(self._events)[-count:] if count > 0 else []

    def count(self, kind: EventKind) -> int:
        """Events of ``kind`` since the last clear (not limited by capacity)."""
        return self._counts[kind]

    @property
    def total(self) -> int:
        return self._next_seq

    def clear(self) -> None:
        self._events.clear()
        self._next_seq = 0
        self._counts.clear()
