"""World <-> screen coordinate mapping (pure math, no pygame, unit-tested).

World coordinates are metres with y pointing up; screen coordinates are pixels
with y pointing down. The camera owns that flip so no drawing code has to.
"""

from __future__ import annotations

from dataclasses import dataclass

from roombacalypse.core.geometry import Point
from roombacalypse.environment.shapes import AABB

Viewport = tuple[int, int, int, int]
"""``(left, top, width, height)`` in pixels."""

MIN_SCALE = 5.0
MAX_SCALE = 3000.0


@dataclass
class Camera:
    """Maps world metres to screen pixels inside a viewport."""

    viewport: Viewport
    center_x: float
    """World x shown at the viewport centre [m]."""
    center_y: float
    """World y shown at the viewport centre [m]."""
    scale: float
    """Pixels per metre."""

    @classmethod
    def fit(cls, bounds: AABB, viewport: Viewport, margin: float = 0.35) -> Camera:
        """Camera showing ``bounds`` (plus ``margin`` metres) as large as possible."""
        _, _, width, height = viewport
        scale = min(width / (bounds.width + 2 * margin), height / (bounds.height + 2 * margin))
        return cls(
            viewport,
            0.5 * (bounds.min_x + bounds.max_x),
            0.5 * (bounds.min_y + bounds.max_y),
            max(MIN_SCALE, min(MAX_SCALE, scale)),
        )

    @property
    def key(self) -> tuple[Viewport, float, float, float]:
        """Changes whenever the mapping changes (for render caches)."""
        return (self.viewport, self.center_x, self.center_y, self.scale)

    def to_screen(self, x: float, y: float) -> tuple[float, float]:
        """World point -> screen pixel (floating point)."""
        left, top, width, height = self.viewport
        return (
            left + 0.5 * width + (x - self.center_x) * self.scale,
            top + 0.5 * height - (y - self.center_y) * self.scale,
        )

    def to_screen_int(self, x: float, y: float) -> tuple[int, int]:
        """World point -> nearest screen pixel."""
        sx, sy = self.to_screen(x, y)
        return round(sx), round(sy)

    def to_world(self, sx: float, sy: float) -> Point:
        """Screen pixel -> world point."""
        left, top, width, height = self.viewport
        return (
            self.center_x + (sx - left - 0.5 * width) / self.scale,
            self.center_y - (sy - top - 0.5 * height) / self.scale,
        )

    def length(self, metres: float) -> float:
        """World length -> pixels."""
        return metres * self.scale

    def contains_screen_point(self, sx: float, sy: float) -> bool:
        left, top, width, height = self.viewport
        return left <= sx < left + width and top <= sy < top + height

    def zoom_at(self, factor: float, sx: float, sy: float) -> None:
        """Zoom by ``factor`` keeping the world point under pixel (sx, sy) fixed."""
        wx, wy = self.to_world(sx, sy)
        self.scale = max(MIN_SCALE, min(MAX_SCALE, self.scale * factor))
        left, top, width, height = self.viewport
        self.center_x = wx - (sx - left - 0.5 * width) / self.scale
        self.center_y = wy + (sy - top - 0.5 * height) / self.scale

    def pan(self, dx_pixels: float, dy_pixels: float) -> None:
        """Move the view so the scene follows a mouse drag of (dx, dy) pixels."""
        self.center_x -= dx_pixels / self.scale
        self.center_y += dy_pixels / self.scale
