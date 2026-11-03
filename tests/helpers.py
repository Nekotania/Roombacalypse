"""Test utilities: tiny rooms built in code and open-loop command playback."""

from __future__ import annotations

import dataclasses
from collections.abc import Sequence
from typing import Any

from roombacalypse.core.config import AppConfig, DriveErrorConfig, LidarConfig
from roombacalypse.core.geometry import Pose2D
from roombacalypse.core.messages import STOP, DriveCommand, SensorFrame
from roombacalypse.environment.rooms import RoomSpec, parse_room
from roombacalypse.evaluation import metrics
from roombacalypse.simulation.engine import Simulation

IDEAL = dataclasses.replace(
    AppConfig(),
    drive_errors=DriveErrorConfig(wheel_radius_std=0.0, wheel_base_std=0.0, slip_std=0.0),
    lidar=LidarConfig(range_noise_std=0.0, range_noise_per_metre=0.0, dropout_probability=0.0),
)
"""Perfect wheels and a noise-free LiDAR: for tests that check exact physics or geometry."""


def box_room(
    width: float = 4.0,
    height: float = 3.0,
    obstacles: Sequence[dict[str, Any]] = (),
    dock: tuple[float, float, float] = (0.0, 1.5, 0.0),
) -> RoomSpec:
    """A rectangular room with the dock on the left wall by default."""
    return parse_room(
        {
            "room": {"name": "test box", "outline": [[0, 0], [width, 0], [width, height], [0, height]]},
            "dock": {"position": [dock[0], dock[1]], "facing_deg": dock[2]},
            "obstacles": list(obstacles),
        },
        source="<test>",
    )


class ScriptedController:
    """Open-loop playback of ``(duration, command)`` segments.

    Test tooling only: it is the opposite of autonomy and must never be used
    as robot behaviour.
    """

    name = "scripted"

    def __init__(self, segments: Sequence[tuple[float, DriveCommand]] = ()) -> None:
        self.segments = list(segments)
        self.calls = 0
        self.frames: list[SensorFrame] = []

    def reset(self, seed: int = 0) -> None:
        self.calls = 0
        self.frames.clear()

    def update(self, frame: SensorFrame, dt: float) -> DriveCommand:
        self.calls += 1
        self.frames.append(frame)
        elapsed = 0.0
        for duration, command in self.segments:
            elapsed += duration
            if frame.time < elapsed - 1e-9:
                return command
        return STOP


def make_sim(
    room: RoomSpec | None = None,
    segments: Sequence[tuple[float, DriveCommand]] = (),
    *,
    start: Pose2D | None = None,
    config: AppConfig | None = None,
) -> tuple[Simulation, ScriptedController]:
    """Simulation driven by a :class:`ScriptedController` (ideal wheels unless ``config`` says otherwise)."""
    controller = ScriptedController(segments)
    sim = Simulation(config or IDEAL, room or box_room(), controller)
    if start is not None:
        sim.robot.pose = start
    return sim, controller


def min_separation(sim: Simulation) -> float:
    """Smallest signed gap between the robot body and anything near it."""
    return metrics.min_separation(sim.world, sim.robot.pose, sim.spec)
