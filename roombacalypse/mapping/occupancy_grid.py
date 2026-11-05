"""Log-odds occupancy grid with a vectorised inverse sensor model (belief side).

Each cell stores the log odds ``l = log(p / (1 - p))`` that it is occupied
(Elfes 1989; Thrun, Burgard & Fox, *Probabilistic Robotics*, ch. 9). A beam
from the sensor to a return is evidence that

* the cell the return fell in is occupied: ``l += hit_log_odds``;
* the cells the beam crossed on the way are free: ``l += miss_log_odds``.

Log odds are clamped to ``[min_log_odds, max_log_odds]`` after every scan,
so the map never becomes so certain that a moved chair could not be
forgotten (or a new box learnt) within a few scans.

Implementation notes:

* **Vectorised ray tracing.** Every beam is sampled once per cell length (a
  ragged set of samples built with ``np.repeat``), and all samples of a scan
  are converted to cells at once. Sampling instead of Bresenham can skip a
  cell where a beam clips its corner; neighbouring beams and later scans
  cover it. (Half-cell sampling took twice as long for the same measured map
  precision and recall.)
* **One update per cell per scan.** A cell crossed by twenty beams near the
  robot gets one miss, not twenty, and a cell that is both crossed and hit
  counts as hit. A scan is one measurement of each cell it sees.
* **No-return beams are ignored.** A beam without a return is either a
  dropout or out of range; treating it as "free up to max range" would let
  dropouts punch holes in walls.
* **The grid grows** in 2 m steps when a scan reaches beyond it; cell
  boundaries stay on multiples of the resolution, so growing never shifts
  the map.

The frame is the robot's map frame (origin at its boot pose on the dock).
Rows follow y and columns follow x; cell ``(r, c)`` spans
``[origin + c·res, origin + (c+1)·res)`` in x (likewise for y).
"""

from __future__ import annotations

import math

import numpy as np

from roombacalypse.core.config import MappingConfig
from roombacalypse.core.geometry import Point, Pose2D, transform_points
from roombacalypse.core.messages import LidarScan

GROWTH_MARGIN = 2.0  # [m] of extra room added whenever the grid has to grow
_SAMPLE_STEP = 1.0  # free-space sample spacing along a beam, in cells (half a cell: twice the work, same map)


class OccupancyGrid:
    """A growable 2D log-odds occupancy grid in the robot's map frame."""

    def __init__(self, config: MappingConfig) -> None:
        self.config = config
        self.resolution = config.resolution
        self.origin: Point = (0.0, 0.0)
        """Map-frame coordinates of the lower-left corner of cell (0, 0)."""
        self.log_odds = np.zeros((0, 0), dtype=np.float32)
        self.version = 0
        """Increases on every change (renderers cache on it)."""
        self.scans_inserted = 0
        self._hit_scratch = np.zeros((0, 0), dtype=bool)

    # ------------------------------------------------------------------ geometry

    @property
    def shape(self) -> tuple[int, int]:
        """``(rows, cols)``."""
        rows, cols = self.log_odds.shape
        return rows, cols

    @property
    def bounds(self) -> tuple[float, float, float, float]:
        """``(min_x, min_y, max_x, max_y)`` of the area the grid covers."""
        rows, cols = self.shape
        ox, oy = self.origin
        return ox, oy, ox + cols * self.resolution, oy + rows * self.resolution

    def cells(self, xs: np.ndarray, ys: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """``(rows, cols)`` of the cells containing the points (not bounds-checked)."""
        res = self.resolution
        rows = np.floor((ys - self.origin[1]) / res).astype(np.intp)
        cols = np.floor((xs - self.origin[0]) / res).astype(np.intp)
        return rows, cols

    def cell_centres(self, mask: np.ndarray) -> np.ndarray:
        """Map-frame centres of the cells where ``mask`` is true, shape ``(M, 2)``."""
        rows, cols = np.nonzero(mask)
        res = self.resolution
        return np.column_stack((self.origin[0] + (cols + 0.5) * res, self.origin[1] + (rows + 0.5) * res))

    def ensure_contains(self, min_x: float, min_y: float, max_x: float, max_y: float) -> None:
        """Grow the grid (keeping its contents) until it covers the given box."""
        if self.log_odds.size:
            x0, y0, x1, y1 = self.bounds
            if min_x >= x0 and min_y >= y0 and max_x < x1 and max_y < y1:
                return
            min_x, min_y = min(min_x, x0), min(min_y, y0)
            max_x, max_y = max(max_x, x1), max(max_y, y1)
        res = self.resolution
        nx0 = math.floor((min_x - GROWTH_MARGIN) / res)
        ny0 = math.floor((min_y - GROWTH_MARGIN) / res)
        nx1 = math.ceil((max_x + GROWTH_MARGIN) / res)
        ny1 = math.ceil((max_y + GROWTH_MARGIN) / res)
        grown = np.zeros((ny1 - ny0, nx1 - nx0), dtype=np.float32)
        if self.log_odds.size:
            r0 = round(self.origin[1] / res) - ny0
            c0 = round(self.origin[0] / res) - nx0
            rows, cols = self.shape
            grown[r0:r0 + rows, c0:c0 + cols] = self.log_odds
        self.log_odds = grown
        self.origin = (nx0 * res, ny0 * res)
        self._hit_scratch = np.zeros(grown.shape, dtype=bool)
        self.version += 1

    # ------------------------------------------------------------------ updates

    def clear(self) -> None:
        self.log_odds = np.zeros((0, 0), dtype=np.float32)
        self._hit_scratch = np.zeros((0, 0), dtype=bool)
        self.origin = (0.0, 0.0)
        self.scans_inserted = 0
        self.version += 1

    def insert_scan(self, pose: Pose2D, scan: LidarScan) -> None:
        """Insert a scan taken from ``pose`` (the robot's pose in the map frame)."""
        origin = pose.transform_point(scan.origin_x, scan.origin_y)
        self.insert_rays(origin, transform_points(pose, scan.points()))

    def insert_rays(self, origin: Point, endpoints: np.ndarray) -> None:
        """Update the grid with beams from ``origin`` to each of ``endpoints`` (map frame, ``(N, 2)``)."""
        cfg = self.config
        ox, oy = origin
        if endpoints.shape[0] == 0:
            return
        self.ensure_contains(
            min(ox, float(endpoints[:, 0].min())), min(oy, float(endpoints[:, 1].min())),
            max(ox, float(endpoints[:, 0].max())), max(oy, float(endpoints[:, 1].max())),
        )
        res = self.resolution
        dx = endpoints[:, 0] - ox
        dy = endpoints[:, 1] - oy
        length = np.hypot(dx, dy)
        # Free space: samples every cell length from the sensor to half a cell short of the return.
        step = _SAMPLE_STEP * res
        counts = np.maximum(np.ceil((length - 0.5 * res) / step), 0).astype(np.intp)
        beam = np.repeat(np.arange(endpoints.shape[0]), counts)
        first = np.cumsum(counts) - counts
        t = (np.arange(beam.shape[0]) - np.repeat(first, counts)) * step
        scale = t / np.maximum(length[beam], 1e-12)
        free_r, free_c = self.cells(ox + dx[beam] * scale, oy + dy[beam] * scale)
        hit_r, hit_c = self.cells(endpoints[:, 0], endpoints[:, 1])

        # A cell both crossed and hit in this scan counts as hit. Fancy-index "+=" with
        # repeated indices adds once per distinct cell, which is exactly "one update per scan".
        self._hit_scratch[hit_r, hit_c] = True
        keep = ~self._hit_scratch[free_r, free_c]
        self._hit_scratch[hit_r, hit_c] = False
        free_r, free_c = free_r[keep], free_c[keep]
        grid = self.log_odds
        grid[free_r, free_c] += np.float32(cfg.miss_log_odds)
        grid[hit_r, hit_c] += np.float32(cfg.hit_log_odds)
        grid[free_r, free_c] = np.maximum(grid[free_r, free_c], np.float32(cfg.min_log_odds))
        grid[hit_r, hit_c] = np.minimum(grid[hit_r, hit_c], np.float32(cfg.max_log_odds))
        self.scans_inserted += 1
        self.version += 1

    # ------------------------------------------------------------------ queries

    def probability(self) -> np.ndarray:
        """Occupancy probability of every cell (0.5 where nothing is known)."""
        return 1.0 / (1.0 + np.exp(-self.log_odds.astype(np.float64)))

    def occupied_mask(self) -> np.ndarray:
        return self.log_odds > self.config.occupied_log_odds

    def free_mask(self) -> np.ndarray:
        return self.log_odds < self.config.free_log_odds

    def known_mask(self) -> np.ndarray:
        """Cells that have been observed at all."""
        return self.log_odds != 0.0

    @property
    def known_area(self) -> float:
        """Area classified as free or occupied [m²]."""
        return float((self.occupied_mask() | self.free_mask()).sum()) * self.resolution**2
