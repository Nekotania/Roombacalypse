"""Global relocalisation: where on the whole map could this scan have been taken? (belief side)

The **kidnapped robot problem** (Engelson & McDermott, 1992): the robot is
picked up and put down somewhere else, and nothing tells it. Odometry carries
on as if nothing happened; only the LiDAR sees a different place. Tracking,
which searches a few centimetres around the predicted pose, cannot recover
from that. Real robot vacuums (and AMCL's "global localisation", or
Cartographer's "global SLAM" constraints) search the *whole* map instead.

This module does it the way Cartographer does: a **correlative search over
every pose of the map** (Olson, "Real-Time Correlative Scan Matching",
ICRA 2009), made affordable by **branch and bound** over precomputed block
maxima (Hess et al., "Real-Time Loop Closure in 2D LIDAR SLAM", ICRA 2016):

* The score of a pose is the mean of ``exp(-d²/2σ²)`` over the scan points,
  ``d`` being the distance to the nearest occupied cell of the occupancy grid.
* Candidate positions are known free cells far enough from obstacles for the
  robot to stand there; headings are every multiple of an angular step that
  moves the bulk of the scan by about one cell.
* Level 1 scores ``k × k`` blocks of positions at once with a block-maximum
  grid: an upper bound for every position in the block. Blocks are expanded
  best bound first; once the next bound is no better than the best pose
  found, nothing left can beat it. The answer is the exact optimum of the
  exhaustive search, typically for a few per cent of its cost.
* **Ambiguity**: the search is repeated with the winner's neighbourhood
  excluded. A symmetric room or a corridor gives a runner-up nearly as good
  as the winner, and the caller should then wait for a more telling view
  rather than guess.

The result is coarse (a grid cell, an angular step); the caller refines it
against its fine surface map. Limitations: only places already mapped can
be recognised; a robot set down somewhere it has never been stays lost until
it drives somewhere it has.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy import ndimage

from roombacalypse.core.geometry import Pose2D
from roombacalypse.localization.scan_matcher import voxel_filter
from roombacalypse.mapping.occupancy_grid import OccupancyGrid

_BLOCK = 8  # cells per side of a level-1 block
_BATCH = 64  # (heading, block) pairs expanded per round
_ANGLE_CHUNK = 48  # headings bounded at once (memory)


@dataclass(frozen=True)
class Hypothesis:
    """Best pose of a global search, and how much better it is than any other."""

    pose: Pose2D
    """Map frame (coarse: a cell and an angular step)."""
    score: float
    """Mean likelihood of the scan at that pose, in [0, 1]."""
    ambiguity: float
    """Best score of a clearly different pose divided by ``score`` (near 1: could be elsewhere).
    Runner-ups below ``floor`` are not looked for: the value is then ``floor``, meaning "at most"."""
    candidates: int
    """Poses the search could have chosen from."""
    evaluated: int
    """Poses it actually scored (block bounds count as one each)."""


class GlobalMatcher:
    """Branch-and-bound correlative search of a scan over a whole occupancy grid."""

    def __init__(self, sigma: float, clearance: float, voxel: float = 0.1) -> None:
        self.sigma = sigma
        """Width of the likelihood around occupied cells [m]."""
        self.clearance = clearance
        """The robot's centre is only looked for where the map has at least this much room [m]."""
        self.voxel = voxel
        """Scan points are thinned to one per voxel this size [m]."""

    def search(self, grid: OccupancyGrid, points: np.ndarray, *, exclude_radius: float = 0.5,
               exclude_angle: float = math.radians(20.0), good_enough: float = 0.9, floor: float = 0.75
               ) -> Hypothesis | None:
        """The best pose for ``points`` (robot frame) anywhere on ``grid``, or ``None`` if there is no map.

        The runner-up search (for ``ambiguity``) ignores poses within ``exclude_radius`` and
        ``exclude_angle`` of the winner, and stops as soon as it finds one scoring
        ``good_enough`` times the winner (it is then ambiguous; by how much does not matter), and does
        not look below ``floor`` times the winner (clearly unambiguous; by how much does not matter either).
        Both limits keep it to a fraction of the first search's cost.
        """
        occupied = grid.occupied_mask()
        free = grid.free_mask() & ~occupied
        pts = voxel_filter(points, self.voxel)
        if not occupied.any() or not free.any() or pts.shape[0] < 3:
            return None
        res = grid.resolution
        distance = ndimage.distance_transform_edt(~occupied) * res
        candidates = free & (distance >= self.clearance)
        if not candidates.any():
            return None

        reach = max(float(np.percentile(np.hypot(pts[:, 0], pts[:, 1]), 90)), res)
        steps = max(8, math.ceil(2.0 * math.pi / (res / reach)))
        angles = np.arange(steps) * (2.0 * math.pi / steps)
        cos, sin = np.cos(angles)[:, None], np.sin(angles)[:, None]
        d_row = np.rint((sin * pts[:, 0] + cos * pts[:, 1]) / res).astype(np.intp)  # (A, N)
        d_col = np.rint((cos * pts[:, 0] - sin * pts[:, 1]) / res).astype(np.intp)

        k = _BLOCK
        pad = int(max(np.abs(d_row).max(), np.abs(d_col).max())) + k + 1
        values = np.pad(np.exp(-0.5 * (distance / self.sigma) ** 2), pad)
        block_max = _block_maximum(values, k)
        rows, cols = occupied.shape
        block_rows, block_cols = np.meshgrid(np.arange(0, rows, k), np.arange(0, cols, k), indexing="ij")
        has_candidate = np.add.reduceat(np.add.reduceat(candidates.astype(np.int32), np.arange(0, rows, k), axis=0),
                                        np.arange(0, cols, k), axis=1) > 0
        br, bc = block_rows[has_candidate], block_cols[has_candidate]
        width = values.shape[1]

        bounds = np.empty((steps, br.shape[0]))
        flat_block_max = block_max.ravel()
        for a0 in range(0, steps, _ANGLE_CHUNK):
            a1 = min(steps, a0 + _ANGLE_CHUNK)
            r = br[None, :, None] + pad + d_row[a0:a1, None, :]
            c = bc[None, :, None] + pad + d_col[a0:a1, None, :]
            bounds[a0:a1] = flat_block_max[r * width + c].mean(axis=2)
        order = np.argsort(-bounds, axis=None, kind="stable")
        pairs = np.column_stack(np.unravel_index(order, bounds.shape))
        sorted_bounds = bounds.ravel()[order]

        grid_r, grid_c = np.meshgrid(np.arange(k), np.arange(k), indexing="ij")
        offsets_r, offsets_c = grid_r.ravel(), grid_c.ravel()
        padded_candidates = np.pad(candidates, (0, k))
        flat_values = values.ravel()
        evaluated = bounds.size

        def expand(chosen: np.ndarray, exclude: tuple[int, int, int] | None) -> tuple[float, tuple[int, int, int]]:
            nonlocal evaluated
            a = pairs[chosen, 0]
            cell_r = br[pairs[chosen, 1]][:, None] + offsets_r[None, :]  # (P, k*k)
            cell_c = bc[pairs[chosen, 1]][:, None] + offsets_c[None, :]
            ok = padded_candidates[cell_r, cell_c]
            if exclude is not None:
                ea, er, ec = exclude
                turn = np.abs((angles[a] - angles[ea] + math.pi) % (2.0 * math.pi) - math.pi)
                near = np.hypot(cell_r - er, cell_c - ec) * res <= exclude_radius
                ok &= ~(near & (turn <= exclude_angle)[:, None])
            r = cell_r[:, :, None] + pad + d_row[a][:, None, :]
            c = cell_c[:, :, None] + pad + d_col[a][:, None, :]
            scores = np.where(ok, flat_values[r * width + c].mean(axis=2), -math.inf)
            evaluated += int(ok.sum())
            flat = int(np.argmax(scores))
            p, q = divmod(flat, scores.shape[1])
            return float(scores[p, q]), (int(a[p]), int(cell_r[p, q]), int(cell_c[p, q]))

        def branch_and_bound(exclude: tuple[int, int, int] | None, stop_at: float, start: float
                             ) -> tuple[float, tuple[int, int, int]]:
            best, best_index = start, (0, 0, 0)
            for first in range(0, sorted_bounds.shape[0], _BATCH):
                if sorted_bounds[first] <= best or best >= stop_at:
                    break
                value, index = expand(np.arange(first, min(first + _BATCH, sorted_bounds.shape[0])), exclude)
                if value > best:
                    best, best_index = value, index
            return best, best_index

        best, winner = branch_and_bound(None, math.inf, -math.inf)
        if not math.isfinite(best) or best <= 0.0:
            return None
        runner_up, _ = branch_and_bound(winner, good_enough * best, floor * best)
        a, r, c = winner
        ox, oy = grid.origin
        pose = Pose2D(ox + (c + 0.5) * res, oy + (r + 0.5) * res, float(angles[a]))
        return Hypothesis(pose, best, max(runner_up, 0.0) / best, int(candidates.sum()) * steps, evaluated)


def _block_maximum(values: np.ndarray, k: int) -> np.ndarray:
    """``out[r, c] = max(values[r:r+k, c:c+k])``, with zeros beyond the far edges."""
    rows, cols = values.shape
    embedded = np.zeros((rows + k - 1, cols + k - 1))
    embedded[:rows, :cols] = values
    along = embedded[:, :cols].copy()
    for j in range(1, k):
        np.maximum(along, embedded[:, j:j + cols], out=along)
    out = along[:rows].copy()
    for j in range(1, k):
        np.maximum(out, along[j:j + rows], out=out)
    return out
