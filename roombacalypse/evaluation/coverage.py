"""Ground-truth floor coverage by the robot's footprint.

Coverage is measured against the *accessible* floor, not the whole floor:
the cells a robot-sized disc can actually reach from the dock. A robot that
cannot fit between two chair legs should not be penalised for not cleaning
between them. Construction:

1. rasterise the floor and every obstacle in the body's height band
   (5 cm cells, padded so walls appear as occupied cells);
2. a distance transform gives each cell's clearance; the robot's *centre*
   fits where clearance >= radius + half a cell;
3. keep the connected component containing the start (8-connectivity);
4. dilate it by the robot's radius: that is the floor the footprint can sweep.

Obstacles are rasterised where they start; pushing one later does not
update the accessible area (documented approximation).
"""

from __future__ import annotations

import math

import numpy as np
from scipy import ndimage

from roombacalypse.core.config import RobotSpec
from roombacalypse.core.geometry import Point, points_in_polygon
from roombacalypse.environment.world import World

_PAD = 2  # cells of guaranteed "outside" around the room's bounding box


class FootprintCoverage:
    """Tracks which accessible floor cells the robot's disc has passed over."""

    def __init__(self, world: World, spec: RobotSpec, start: Point, resolution: float = 0.05) -> None:
        bounds = world.bounds
        self.resolution = resolution
        self.origin = (bounds.min_x - _PAD * resolution, bounds.min_y - _PAD * resolution)
        cols = math.ceil(bounds.width / resolution) + 2 * _PAD
        rows = math.ceil(bounds.height / resolution) + 2 * _PAD
        xs = self.origin[0] + (np.arange(cols) + 0.5) * resolution
        ys = self.origin[1] + (np.arange(rows) + 0.5) * resolution
        gx, gy = np.meshgrid(xs, ys)  # (rows, cols); row index follows y

        self.floor = points_in_polygon(gx, gy, world.outline)
        occupied = ~self.floor
        for obstacle in world.obstacles_in_band(0.0, spec.height):
            box = obstacle.aabb
            i0, i1 = self._span(box.min_y, box.max_y, self.origin[1], rows)
            j0, j1 = self._span(box.min_x, box.max_x, self.origin[0], cols)
            if i0 < i1 and j0 < j1:
                occupied[i0:i1, j0:j1] |= obstacle.shape.contains_points(gx[i0:i1, j0:j1], gy[i0:i1, j0:j1])
        self.occupied = occupied

        clearance = ndimage.distance_transform_edt(~occupied) * resolution
        fits = clearance >= spec.radius + 0.5 * resolution
        start_cell = self._nearest_true(fits, self.cell_of(*start))
        labels, _ = ndimage.label(fits, structure=np.ones((3, 3), dtype=bool))
        self.reachable_centres = labels == labels[start_cell] if start_cell is not None else np.zeros_like(fits)

        k = math.ceil(spec.radius / resolution)
        offsets = np.arange(-k, k + 1)
        self._disc = (offsets[:, None] ** 2 + offsets[None, :] ** 2) * resolution**2 <= spec.radius**2
        self._k = k
        self.accessible = ndimage.binary_dilation(self.reachable_centres, structure=self._disc) & ~occupied
        self.visited = np.zeros_like(self.accessible)
        self._last_mark: Point | None = None

    def _span(self, lo: float, hi: float, origin: float, size: int) -> tuple[int, int]:
        """Half-open index range of cells whose centres may lie within [lo, hi]."""
        first = max(0, math.floor((lo - origin) / self.resolution))
        last = min(size, math.ceil((hi - origin) / self.resolution) + 1)
        return first, last

    def cell_of(self, x: float, y: float) -> tuple[int, int]:
        """(row, col) of the cell containing (x, y)."""
        return (math.floor((y - self.origin[1]) / self.resolution), math.floor((x - self.origin[0]) / self.resolution))

    @staticmethod
    def _nearest_true(mask: np.ndarray, cell: tuple[int, int], search: int = 6) -> tuple[int, int] | None:
        rows, cols = mask.shape
        best: tuple[int, int] | None = None
        best_d = math.inf
        for i in range(max(0, cell[0] - search), min(rows, cell[0] + search + 1)):
            for j in range(max(0, cell[1] - search), min(cols, cell[1] + search + 1)):
                d = (i - cell[0]) ** 2 + (j - cell[1]) ** 2
                if mask[i, j] and d < best_d:
                    best, best_d = (i, j), d
        return best

    def mark(self, x: float, y: float) -> None:
        """Record the footprint of a robot centred at (x, y)."""
        last = self._last_mark
        if last is not None and math.hypot(x - last[0], y - last[1]) < 0.5 * self.resolution:
            return
        self._last_mark = (x, y)
        i, j = self.cell_of(x, y)
        k = self._k
        rows, cols = self.visited.shape
        i0, i1, j0, j1 = max(0, i - k), min(rows, i + k + 1), max(0, j - k), min(cols, j + k + 1)
        if i0 >= i1 or j0 >= j1:
            return
        disc = self._disc[i0 - (i - k): i1 - (i - k), j0 - (j - k): j1 - (j - k)]
        self.visited[i0:i1, j0:j1] |= disc

    @property
    def cell_area(self) -> float:
        return self.resolution**2

    @property
    def accessible_area(self) -> float:
        """Floor area the footprint can reach [m²]."""
        return float(self.accessible.sum()) * self.cell_area

    @property
    def covered_area(self) -> float:
        """Accessible floor area swept so far [m²]."""
        return float((self.visited & self.accessible).sum()) * self.cell_area

    @property
    def fraction(self) -> float:
        accessible = int(self.accessible.sum())
        return float((self.visited & self.accessible).sum()) / accessible if accessible else 0.0
