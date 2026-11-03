"""Obstacles the user can drop into the room at run time."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from roombacalypse.environment.shapes import Circle, ConvexPolygon, Shape
from roombacalypse.environment.world import ObstacleKind


@dataclass(frozen=True)
class ObstacleTemplate:
    """A placeable obstacle type; :meth:`make_shape` positions it at a point."""

    label: str
    kind: ObstacleKind
    z_min: float
    z_max: float
    mobility: float
    factory: Callable[[float, float], Shape]

    def make_shape(self, x: float, y: float) -> Shape:
        return self.factory(x, y)

    @property
    def description(self) -> str:
        traits = "movable" if self.mobility > 0.0 else "fixed"
        if self.z_max < 0.05:
            traits += ", below LiDAR plane"
        return f"{self.label} ({traits})"


PLACEABLE: tuple[ObstacleTemplate, ...] = (
    ObstacleTemplate("cardboard box", ObstacleKind.BOX, 0.0, 0.35, 0.75,
                     lambda x, y: ConvexPolygon.box(x, y, 0.40, 0.30)),
    ObstacleTemplate("pillar", ObstacleKind.PILLAR, 0.0, 2.5, 0.0,
                     lambda x, y: Circle(x, y, 0.12)),
    ObstacleTemplate("sock", ObstacleKind.CLUTTER, 0.0, 0.02, 0.9,
                     lambda x, y: ConvexPolygon.box(x, y, 0.20, 0.07, 0.6)),
    ObstacleTemplate("ottoman", ObstacleKind.SOFT, 0.0, 0.42, 0.3,
                     lambda x, y: ConvexPolygon.box(x, y, 0.50, 0.50)),
)
"""Cycled with the ``O`` key in the viewer."""
