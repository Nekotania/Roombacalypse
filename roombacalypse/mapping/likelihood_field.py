"""Surface fields: how well does a point agree with a set of mapped surfaces?

A field is built from *reference* surface points (a local map, or an old
stretch of trajectory for loop closure) and answers two questions:

* **Coarse:** ``L(x) = exp(-d(x)² / 2σ²)`` on a grid, where ``d`` is the
  distance from a cell centre to the nearest occupied cell centre
  (``scipy.ndimage.distance_transform_edt``). This is the likelihood-field
  sensor model of *Probabilistic Robotics* §6.4 and the "lookup table" of
  Olson's correlative scan matcher (ICRA 2009). It is read at integer cell
  offsets by the exhaustive correlative search (:meth:`lookup`), where
  accuracy to a cell is all that is needed.
* **Fine:** the exact, signed **point-to-surface distance** (:meth:`correspond`, :meth:`distances`).
  Every occupied cell stores the mean of the reference points around it and,
  where they form a straight surface, its normal (PCA of the neighbourhood):
  a *line* element. A compact cluster (a chair leg) is a *point* element.
  Anything else (a far wall blurred by range noise, a corner) is a *blob*:
  it counts for the fit score but constrains no direction.
  The distance transform's *feature map* tells every cell which occupied cell
  is nearest, so a query point finds its nearest surface element in O(1) and
  its distance is ``n · (x − p)``: point-to-line, as in PL-ICP (Censi 2008),
  with correspondences from a precomputed closest-point map instead of a
  KD-tree. This is sub-cell accurate whatever the orientation of the wall:
  measured with ideal scans, cell-centre representation alone limited a match
  to about ±1 cell (11 mm RMS on a 2.5 cm grid); see docs/algorithms/slam.md.

Points farther than ``cutoff`` from every reference point are outliers: they
contribute neither likelihood nor gradient, so a moved box or a glimpse
through a doorway cannot drag an alignment around.
"""

from __future__ import annotations

import math

import numpy as np
from scipy import ndimage

from roombacalypse.core.geometry import Point

CUTOFF_SIGMAS = 3.0  # points farther than this many sigmas from every reference are outliers
_NEIGHBOURHOOD = 2  # surface elements are fitted to the points of the surrounding 5x5 cells (12.5 cm)...
_WIDE_NEIGHBOURHOOD = 4  # ...or 9x9 cells (22.5 cm) where that is too short to see a noisy wall's direction
_MIN_NORMAL_POINTS = 4  # a surface normal needs at least this many points in the neighbourhood
_MAX_FLATNESS = 0.1  # ...and a scatter at least ten times longer than wide (a line, not a corner or a leg)
_POINT_EXTENT = 0.025  # [m] a scatter with a smaller standard deviation is a compact feature (a leg)


class LikelihoodField:
    """Coarse likelihood grid plus an exact closest-surface map, around reference points (map frame)."""

    def __init__(
        self,
        points: np.ndarray,
        resolution: float,
        sigma: float,
        margin: float) -> None:
        """Build the field around ``points`` (``(N, 2)``), extending ``margin`` metres beyond them."""
        self.resolution = resolution
        self.sigma = sigma
        self.cutoff = CUTOFF_SIGMAS * sigma
        margin = max(margin, (_WIDE_NEIGHBOURHOOD + 1) * resolution)
        if points.shape[0] == 0:
            points = np.array([[1e6, 1e6]])  # nowhere: everything reads as "no surface here"
        c0 = math.floor((float(points[:, 0].min()) - margin) / resolution)
        r0 = math.floor((float(points[:, 1].min()) - margin) / resolution)
        c1 = math.ceil((float(points[:, 0].max()) + margin) / resolution)
        r1 = math.ceil((float(points[:, 1].max()) + margin) / resolution)
        self.origin: Point = (c0 * resolution, r0 * resolution)
        shape = (r1 - r0 + 1, c1 - c0 + 1)
        rows = np.floor(points[:, 1] / resolution).astype(np.intp) - r0
        cols = np.floor(points[:, 0] / resolution).astype(np.intp) - c0
        cell = rows * shape[1] + cols

        # One surface element per occupied cell: a point and, where the surface is locally straight, a normal.
        # Both come from the moments of all points in the cell's 5x5 neighbourhood (pooled over every scan
        # in the map): the mean, and the principal axes of the scatter (PCA, as in NDT and PL-ICP). Normals
        # from pairs of neighbouring returns would be far too noisy: with 1 cm of range noise on returns
        # 2-3 cm apart they are off by 20-30°, which turns every wall into a sawtooth of local minima;
        # a 3x3 neighbourhood (7.5 cm) of a single noisy scan is still 20° off.
        cells, inverse = np.unique(cell, return_inverse=True)
        inverse = inverse.ravel()
        x, y = points[:, 0], points[:, 1]
        moments = np.zeros((6, shape[0] * shape[1]))
        for k, weight in enumerate((np.ones_like(x), x, y, x * x, x * y, y * y)):
            moments[k, cells] = np.bincount(inverse, weights=weight)
        self._px, self._py, self._nx, self._ny, line, extent = self._fit(moments, cells, shape[1], _NEIGHBOURHOOD)
        # Far walls are blurred by range noise (3 cm at 6 m): where a 5x5 fit is neither a line nor compact,
        # fit again over 9x9 cells (22.5 cm), long enough for the wall's direction to show through the noise.
        retry = np.flatnonzero(~line & (extent >= _POINT_EXTENT**2))
        if retry.shape[0]:
            wide = self._fit(moments, cells[retry], shape[1], _WIDE_NEIGHBOURHOOD)
            better = wide[4]
            for target, values in zip((self._px, self._py, self._nx, self._ny, line, extent), wide, strict=True):
                target[retry[better]] = values[better]
        self._has_normal = line
        # Neither a line nor a compact feature (a corner, a smudge): such an element says "there is something
        # here" but not in which direction, so it constrains nothing. Treating it as a point would invent
        # constraints along walls (a corridor would look pinned along its length).
        self._informative = line | (extent < _POINT_EXTENT**2)

        occupied = np.zeros(shape, dtype=bool)
        occupied.flat[cells] = True
        distance, nearest = ndimage.distance_transform_edt(~occupied, return_indices=True)
        lookup = np.full(occupied.size, -1, dtype=np.intp)
        lookup[cells] = np.arange(cells.shape[0])
        self._nearest = lookup[(nearest[0] * shape[1] + nearest[1]).ravel()].reshape(shape)
        self.values = np.exp(-0.5 * (distance * resolution / sigma) ** 2)
        self._padded = np.zeros((shape[0] + 2, shape[1] + 2))
        self._padded[1:-1, 1:-1] = self.values

    @staticmethod
    def _fit(moments: np.ndarray, cells: np.ndarray, width: int, reach: int
             ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Fit a surface element to the points within ``reach`` cells of each given cell.

        Returns the mean (x, y), the unit normal (x, y), whether the scatter is a
        line, and its variance along the major axis.
        """
        span = range(-reach, reach + 1)
        pooled = np.zeros((6, cells.shape[0]))
        for dr in span:
            for dc in span:
                pooled += moments[:, cells + dr * width + dc]
        n, sx, sy, sxx, sxy, syy = pooled
        mx, my = sx / n, sy / n
        cxx, cxy, cyy = sxx / n - mx * mx, sxy / n - mx * my, syy / n - my * my
        spread = np.sqrt(np.maximum((cxx - cyy) ** 2 + 4 * cxy * cxy, 0.0))
        minor = 0.5 * (cxx + cyy - spread)
        major = 0.5 * (cxx + cyy + spread)
        line = (n >= _MIN_NORMAL_POINTS) & (minor < _MAX_FLATNESS * np.maximum(major, 1e-12))
        direction = 0.5 * np.arctan2(2 * cxy, cxx - cyy)  # of the major axis
        # A compact feature needs support too: a lone far return is an under-sampled wall, not a chair leg.
        extent = np.where(n >= _MIN_NORMAL_POINTS, major, np.inf)
        return mx, my, -np.sin(direction), np.cos(direction), line, extent

    @property
    def shape(self) -> tuple[int, int]:
        rows, cols = self.values.shape
        return rows, cols

    # ------------------------------------------------------------------ coarse grid

    def base_cells(self, xs: np.ndarray, ys: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """``(rows, cols)`` of the cells containing the points."""
        res = self.resolution
        rows = np.floor((ys - self.origin[1]) / res).astype(np.intp)
        cols = np.floor((xs - self.origin[0]) / res).astype(np.intp)
        return rows, cols

    def lookup(self, rows: np.ndarray, cols: np.ndarray, padded: np.ndarray | None = None, pad: int = 1
               ) -> np.ndarray:
        """Coarse likelihood at integer cells (0 outside the grid).

        ``padded``/``pad`` may substitute a grid from :meth:`padded_block_maximum` (whose border is ``pad``).
        """
        grid = self._padded if padded is None else padded
        n_rows, n_cols = grid.shape
        r = np.minimum(np.maximum(rows + pad, 0), n_rows - 1)
        c = np.minimum(np.maximum(cols + pad, 0), n_cols - 1)
        return grid.ravel()[r * n_cols + c]

    def padded_block_maximum(self, size: int) -> np.ndarray:
        """Block maxima for branch and bound; read it with ``lookup(rows, cols, result, pad=size)``.

        Entry (r, c) is ``max(values[r:r+size, c:c+size])`` (cells outside the grid
        count as 0), defined for blocks that start up to ``size`` cells outside the
        grid. A point that lands in cell (r, c) after a coarse shift lands somewhere
        in that block after any finer shift within it, so the mean of these maxima
        over a scan bounds the score of every fine candidate in the block.
        """
        rows, cols = self.shape
        out_rows, out_cols = rows + 2 * size, cols + 2 * size
        embedded = np.zeros((out_rows + size - 1, out_cols + size - 1))
        embedded[size:size + rows, size:size + cols] = self.values
        along = embedded[:, :out_cols].copy()
        for k in range(1, size):
            np.maximum(along, embedded[:, k:k + out_cols], out=along)
        result = along[:out_rows].copy()
        for k in range(1, size):
            np.maximum(result, along[k:k + out_rows], out=result)
        return result

    # ------------------------------------------------------------------ exact surfaces

    def correspond(self, xs: np.ndarray, ys: np.ndarray, *, constraints_only: bool = True
                   ) -> tuple[np.ndarray, np.ndarray]:
        """Nearest surface element of every point, and whether the point is an inlier of it.

        Inliers are inside the grid and within ``cutoff`` of the element. With
        ``constraints_only`` (for matching), elements that constrain no direction
        (blurred blobs) do not count as inliers either.
        """
        rows, cols = self.base_cells(xs, ys)
        n_rows, n_cols = self.shape
        inside = (rows >= 0) & (rows < n_rows) & (cols >= 0) & (cols < n_cols)
        element = self._nearest[np.clip(rows, 0, n_rows - 1), np.clip(cols, 0, n_cols - 1)]
        span = np.hypot(xs - self._px[element], ys - self._py[element])
        inlier = inside & (span <= self.cutoff)
        if constraints_only:
            inlier &= self._informative[element]
        return element, inlier

    def distances(self, element: np.ndarray, xs: np.ndarray, ys: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Residuals of each point against its given surface element: two per point, shape ``(N, 2)`` each.

        Returns ``(residuals, gradients)``: ``residuals[:, k]`` is a signed distance
        whose gradient with respect to the point is ``gradients[:, k, :]`` (shape
        ``(N, 2, 2)``). For an element with a normal ``n`` at ``p`` the first
        residual is ``n · (x − p)`` (point-to-line) and the second is zero; for an
        isolated point they are the components of ``x − p`` (point-to-point).
        Using the vector rather than its length ``|x − p|`` matters: the length is
        V-shaped at the optimum, and Gauss-Newton on it overshoots back and forth.
        """
        vx, vy = xs - self._px[element], ys - self._py[element]
        nx, ny, line = self._nx[element], self._ny[element], self._has_normal[element]
        residuals = np.column_stack((np.where(line, nx * vx + ny * vy, vx), np.where(line, 0.0, vy)))
        gradients = np.zeros((xs.shape[0], 2, 2))
        gradients[:, 0, 0] = np.where(line, nx, 1.0)
        gradients[:, 0, 1] = np.where(line, ny, 0.0)
        gradients[:, 1, 1] = np.where(line, 0.0, 1.0)
        return residuals, gradients

    def likelihood(self, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        """``exp(-d²/2σ²)`` with the exact distance to the nearest surface element (0 for outliers)."""
        element, inlier = self.correspond(xs, ys, constraints_only=False)
        residuals, _ = self.distances(element, xs, ys)
        squared = (residuals**2).sum(axis=1)
        return np.where(inlier, np.exp(-0.5 * squared / self.sigma**2), 0.0)
