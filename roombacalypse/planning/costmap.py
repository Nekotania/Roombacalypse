"""Costmap: where the robot's centre may go on its own map, and how much it costs (belief side).

Built from the SLAM occupancy grid (same lattice, map frame), as a ROS costmap is:

* **obstacles** are occupied cells, plus the places the robot has *bumped*
  into things its LiDAR cannot see (socks, cables), remembered as small discs;
* **lethal**: every cell closer than ``inscribed = radius + DWA safety margin +
  extra margin`` to an obstacle. The robot's centre must not be there, which
  is exactly what the local planner would refuse anyway (so a plan never
  asks DWA for the impossible);
* **cost** beyond that decays exponentially with clearance, so shortest paths
  keep away from furniture where they can;
* **unknown** cells (never observed) are passable at a premium: plans prefer
  known floor, but can cross a patch the LiDAR has not seen yet.

``cost`` is the *extra* cost per metre on top of the distance itself, and is
``inf`` where lethal. Everything outside the grid is lethal.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy import ndimage

from roombacalypse.core.config import PlanningConfig
from roombacalypse.core.geometry import Point
from roombacalypse.mapping.occupancy_grid import OccupancyGrid

Cell = tuple[int, int]
"""``(row, col)``."""


@dataclass(frozen=True, eq=False)
class Costmap:
    """A snapshot of where the robot's centre may go (map frame)."""

    origin: Point
    resolution: float
    cost: np.ndarray
    """Extra cost per metre (0 in open known floor), ``inf`` where lethal."""
    clearance: np.ndarray
    """Distance from each cell centre to the nearest obstacle cell centre [m]."""
    free: np.ndarray
    """Cells the robot has seen to be free."""
    unknown: np.ndarray
    """Cells never classified as free or occupied."""
    occupied: np.ndarray
    """Obstacle cells (mapped, or remembered bumps)."""
    inscribed: float
    """Clearance below which a cell is lethal [m]."""

    @property
    def shape(self) -> tuple[int, int]:
        rows, cols = self.cost.shape
        return rows, cols

    @property
    def lethal(self) -> np.ndarray:
        return ~np.isfinite(self.cost)

    @property
    def traversable(self) -> np.ndarray:
        return np.isfinite(self.cost)

    def cell(self, x: float, y: float) -> Cell:
        return (math.floor((y - self.origin[1]) / self.resolution), math.floor((x - self.origin[0]) / self.resolution))

    def centre(self, cell: Cell) -> Point:
        return (self.origin[0] + (cell[1] + 0.5) * self.resolution, self.origin[1] + (cell[0] + 0.5) * self.resolution)

    def inside(self, cell: Cell) -> bool:
        rows, cols = self.shape
        return 0 <= cell[0] < rows and 0 <= cell[1] < cols

    def cost_at(self, x: float, y: float) -> float:
        cell = self.cell(x, y)
        return float(self.cost[cell]) if self.inside(cell) else math.inf

    def nearest_traversable(self, x: float, y: float, search: float = 0.3) -> Cell | None:
        """The traversable cell nearest to (x, y) within ``search`` metres (the robot may sit in a lethal cell
        after slipping close to something; planning has to start from somewhere it may be)."""
        centre = self.cell(x, y)
        if self.inside(centre) and math.isfinite(self.cost[centre]):
            return centre
        k = max(1, math.ceil(search / self.resolution))
        rows, cols = self.shape
        r0, r1 = max(0, centre[0] - k), min(rows, centre[0] + k + 1)
        c0, c1 = max(0, centre[1] - k), min(cols, centre[1] + k + 1)
        if r0 >= r1 or c0 >= c1:
            return None
        window = self.traversable[r0:r1, c0:c1]
        if not window.any():
            return None
        rr, cc = np.nonzero(window)
        best = int(np.argmin((rr + r0 - centre[0]) ** 2 + (cc + c0 - centre[1]) ** 2))
        return (int(rr[best] + r0), int(cc[best] + c0))


def build_costmap(grid: OccupancyGrid, config: PlanningConfig, inscribed: float,
                  extra_obstacles: np.ndarray | None = None) -> Costmap:
    """The costmap of ``grid``; ``extra_obstacles`` are map-frame points (bumps) marked as small discs."""
    res = grid.resolution
    occupied = grid.occupied_mask().copy()
    if extra_obstacles is not None and extra_obstacles.shape[0] and occupied.size:
        rows, cols = grid.cells(extra_obstacles[:, 0], extra_obstacles[:, 1])
        marks = np.zeros_like(occupied)
        inside = (rows >= 0) & (rows < occupied.shape[0]) & (cols >= 0) & (cols < occupied.shape[1])
        marks[rows[inside], cols[inside]] = True
        k = max(0, math.ceil(config.bump_radius / res))
        offsets = np.arange(-k, k + 1)
        disc = (offsets[:, None] ** 2 + offsets[None, :] ** 2) * res * res <= config.bump_radius**2
        occupied |= ndimage.binary_dilation(marks, structure=disc) if k else marks
    free = grid.free_mask()
    unknown = ~(free | occupied)
    clearance = ndimage.distance_transform_edt(~occupied) * res if occupied.any() else np.full(occupied.shape, np.inf)
    cost = config.cost_scale * np.exp(-np.maximum(clearance - inscribed, 0.0) / config.cost_decay)
    cost = cost + np.where(unknown, config.unknown_cost, 0.0)
    cost = np.where(clearance < inscribed, np.inf, cost)
    return Costmap(grid.origin, res, cost, clearance, free & ~occupied, unknown, occupied, inscribed)
