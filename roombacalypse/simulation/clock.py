"""Fixed-timestep simulation clock."""

from __future__ import annotations


def steps_per_period(period: float, dt: float) -> int:
    """Number of physics steps in ``period``; it must be an integer multiple of ``dt``."""
    ratio = period / dt
    steps = round(ratio)
    if steps < 1 or abs(ratio - steps) > 1e-6 * max(1.0, ratio):
        raise ValueError(f"period {period} s is not a positive multiple of dt {dt} s")
    return steps


class SimClock:
    """Counts fixed steps. Time is ``tick * dt``, so it never accumulates rounding error."""

    def __init__(self, dt: float) -> None:
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        self.dt = dt
        self.tick = 0

    @property
    def time(self) -> float:
        """Simulation time [s]."""
        return self.tick * self.dt

    def advance(self) -> None:
        self.tick += 1

    def reset(self) -> None:
        self.tick = 0
