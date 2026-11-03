"""Test utilities: tiny rooms built in code and open-loop command playback."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

from roombacalypse.core.config import AppConfig
from roombacalypse.core.geometry import Pose2D
from roombacalypse.core.messages import STOP, DriveCommand, SensorFrame
from roombacalypse.environment.rooms import RoomSpec, parse_room
from roombacalypse.environment.shapes import circle_vs_segment, circle_vs_shape
from roombacalypse.simulation.engine import Simulation


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

    def reset(self) -> None:
        self.calls = 0

    def update(self, frame: SensorFrame, dt: float) -> DriveCommand:
        self.calls += 1
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
    """Simulation driven by a :class:`ScriptedController`, optionally teleported to ``start``."""
    controller = ScriptedController(segments)
    sim = Simulation(config or AppConfig(), room or box_room(), controller)
    if start is not None:
        sim.robot.pose = start
    return sim, controller


def min_separation(sim: Simulation) -> float:
    """Smallest signed gap between the robot body and anything it can collide with."""
    pose = sim.robot.pose
    r = sim.spec.radius
    gaps = [circle_vs_segment(pose.x, pose.y, r, *wall)[2] for wall in sim.world.walls]
    gaps += [
        circle_vs_shape(pose.x, pose.y, r, o.shape)[2]
        for o in sim.world.obstacles_in_band(0.0, sim.spec.height)
    ]
    return min(gaps)
