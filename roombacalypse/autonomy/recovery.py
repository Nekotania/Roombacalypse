"""Stuck detection and the recovery ladder (belief side).

**Detectors** are small, independent and testable. Each looks at what the
robot's software knows (its own pose, odometry, bumper, the local
planner's verdict) and says whether the robot is stuck, and how:

* ``cornered``: the local planner has found no forward arc at all for a while
  (nose-in between chair legs, wedged against a cabinet corner);
* ``no progress``: driving towards a target without getting closer;
* ``bump storm``: several bumps in a short time;
* ``oscillation``: turning left, right, left, ... without getting anywhere;
* ``wheel slip``: the encoders say the robot is driving, the map says it is not.

**The ladder** escalates while trouble keeps coming back at the same place
(within ``_SAME_PLACE`` metres and ``_SAME_TROUBLE`` seconds), and starts
again from the bottom anywhere else:

1. **back up**: reverse a little, if the LiDAR says the space behind is clear;
2. **spin**: turn on the spot through 360°, re-scanning everything around;
3. **hop**: drive a short way towards the most open direction in the scan;
4. **elsewhere**: give up on the current target and blacklist it;
5. **trapped**: stop and call for help. A real robot vacuum beeps here.

The ladder starts over as soon as the robot reaches a target or gets a
metre away from the trouble. Without that, an endgame spent sweeping
leftovers in one corner climbed the whole ladder on unrelated troubles and
declared a robot trapped in an open room. The ladder only *chooses* the
manoeuvre; the executive drives it.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from enum import IntEnum

from roombacalypse.core.geometry import Point


class Rung(IntEnum):
    BACK_UP = 1
    SPIN = 2
    HOP = 3
    ELSEWHERE = 4
    TRAPPED = 5


_SAME_PLACE = 0.5  # [m]
_LEFT_BEHIND = 1.0  # [m] from the last trouble: the ladder starts over
_SAME_TROUBLE = 45.0  # [s]
_CORNERED_SECONDS = 1.5
_NO_PROGRESS_SECONDS = 8.0
_NO_PROGRESS_DISTANCE = 0.05  # [m] of progress that resets the no-progress clock
_BUMP_STORM = (3, 20.0)  # bumps, within seconds
_OSCILLATION = (8, 6.0, 0.25)  # turn reversals, within seconds, while moving less than metres
_SLIP = (3.0, 0.3, 0.05)  # over seconds: odometry says more than metres, the map less than metres


@dataclass(frozen=True)
class Trouble:
    kind: str
    """``"cornered"``, ``"no progress"``, ``"bump storm"``, ``"oscillation"``, ``"wheel slip"``, or
    ``"stranded"`` (raised by the executive: the planner cannot start from where the robot is)."""
    time: float
    where: Point


class StuckDetector:
    """Watches the robot and reports :class:`Trouble`; :meth:`clear` after each recovery."""

    def __init__(self) -> None:
        self.clear(0.0)

    def clear(self, time: float) -> None:
        self._blocked_since: float | None = None
        self._best_remaining = math.inf
        self._progress_time = time
        self._bumps: deque[float] = deque()
        self._turns: deque[tuple[float, float]] = deque()  # (time, sign of yaw rate)
        self._track: deque[tuple[float, Point, float]] = deque()  # (time, map position, odometry distance)

    def target_changed(self, time: float) -> None:
        self._best_remaining = math.inf
        self._progress_time = time

    def update(self, time: float, where: Point, odometry_distance: float, angular: float, *, driving: bool,
               blocked: bool, remaining: float | None, bumped: bool) -> Trouble | None:
        """One control tick. ``remaining``: distance left to the current target (``None``: no target)."""
        # Cornered: the local planner sees no forward arc at all.
        if driving and blocked:
            self._blocked_since = time if self._blocked_since is None else self._blocked_since
            if time - self._blocked_since > _CORNERED_SECONDS:
                return Trouble("cornered", time, where)
        else:
            self._blocked_since = None
        # No progress towards the target.
        if driving and remaining is not None:
            if remaining < self._best_remaining - _NO_PROGRESS_DISTANCE:
                self._best_remaining, self._progress_time = remaining, time
            elif time - self._progress_time > _NO_PROGRESS_SECONDS:
                return Trouble("no progress", time, where)
        else:
            self._progress_time = time
        # Bump storm.
        if bumped:
            self._bumps.append(time)
        count, window = _BUMP_STORM
        while self._bumps and time - self._bumps[0] > window:
            self._bumps.popleft()
        if len(self._bumps) >= count:
            return Trouble("bump storm", time, where)
        # Track recent motion for oscillation and slip.
        self._track.append((time, where, odometry_distance))
        horizon = max(_OSCILLATION[1], _SLIP[0])
        while self._track and time - self._track[0][0] > horizon:
            self._track.popleft()
        if abs(angular) > 0.3:
            sign = math.copysign(1.0, angular)
            if not self._turns or self._turns[-1][1] != sign:
                self._turns.append((time, sign))
        flips, window, still = _OSCILLATION
        while self._turns and time - self._turns[0][0] > window:
            self._turns.popleft()
        if driving and len(self._turns) >= flips and self._moved(time, window) < still:
            return Trouble("oscillation", time, where)
        seconds, wheels, map_motion = _SLIP
        old = self._oldest(time, seconds)
        if driving and old is not None and odometry_distance - old[2] > wheels and (
            math.dist(where, old[1]) < map_motion
        ):
            return Trouble("wheel slip", time, where)
        return None

    def _oldest(self, time: float, seconds: float) -> tuple[float, Point, float] | None:
        for sample in self._track:
            if time - sample[0] <= seconds:
                return sample if time - sample[0] >= 0.8 * seconds else None
        return None

    def _moved(self, time: float, seconds: float) -> float:
        points = [p for t, p, _ in self._track if time - t <= seconds]
        if len(points) < 2:
            return math.inf
        return max(math.dist(points[0], p) for p in points)


class RecoveryLadder:
    """Chooses the next rung for a trouble: higher if trouble keeps coming back at the same place."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.rung = Rung.BACK_UP
        self._last: Trouble | None = None
        self.history: list[tuple[Trouble, Rung]] = []

    def progress(self, where: Point) -> None:
        """The robot is at ``where``: once well away from the last trouble, the ladder starts over."""
        if self._last is not None and math.dist(where, self._last.where) > _LEFT_BEHIND:
            self.rung = Rung.BACK_UP
            self._last = None

    def succeeded(self) -> None:
        """The robot reached what it was heading for: whatever the trouble was, it is over."""
        self.rung = Rung.BACK_UP
        self._last = None

    def next(self, trouble: Trouble) -> Rung:
        last = self._last
        again = last is not None and trouble.time - last.time < _SAME_TROUBLE and (
            math.dist(trouble.where, last.where) < _SAME_PLACE
        )
        self.rung = Rung(min(self.rung + 1, Rung.TRAPPED)) if again else Rung.BACK_UP
        self._last = trouble
        self.history.append((trouble, self.rung))
        return self.rung
