"""Scan matching: where must the robot have been for this scan to fit the map? (belief side)

Two stages, as in Cartographer and Karto:

1. **Correlative search** (Olson, "Real-Time Correlative Scan Matching",
   ICRA 2009). Every pose in a window around the prediction (translation on
   the likelihood field's grid, rotation in steps that move the farthest
   point about one cell) is scored by the mean likelihood of the scan points.
   This is global within the window: it cannot get stuck in a local
   minimum, which is what makes loop closure and recovery after a bump
   possible. A two-level **branch and bound** keeps it cheap: blocks of
   k×k translations are scored on a block-maximum field, which bounds every
   pose inside the block, and a block is only expanded if its bound beats
   the best pose found so far. A small penalty for distance from the
   prediction breaks ties toward it.
2. **Point-to-line ICP refinement** (PL-ICP, Censi 2008): alternate
   correspondences (each point's nearest surface element, read in O(1) from
   the field's closest-surface map; points beyond the cutoff are dropped as
   outliers) and a Gauss-Newton solve of ``Σ d_k² / Nσ² + prior``. This
   gives sub-cell precision. The prior term (a
   weak pull toward the odometry prediction) only matters where the scan
   cannot tell: along a featureless corridor.

The result carries the scan's **information matrix**, the ICP one: the sum
over matched points of ``J_kᵀJ_k`` with ``J_k = n_kᵀ[I | ∂q/∂θ]`` (the surface
normal at each point), divided by the number of points and by
``match_point_std²``. It is large in directions the scan pins down and zero
along a corridor, whose walls have no normal along it. The pose graph uses
it to weight constraints, so degenerate matches do not pretend to be certain.

Optionally the search also reports **ambiguity**: the best score of any pose
clearly different from the winner, relative to the winner's. A corridor or
a symmetric room gives values near 1, and loop closure rejects those matches.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from roombacalypse.core.config import SlamConfig
from roombacalypse.core.geometry import Pose2D, wrap_angle
from roombacalypse.core.messages import LidarScan
from roombacalypse.mapping.likelihood_field import LikelihoodField

_TIE_BREAK = 0.02  # score penalty at the edge of the search window (pulls ties toward the prediction)
_MAX_ITERATIONS = 10  # ICP rounds (re-association + Gauss-Newton)
_INNER_ITERATIONS = 2  # Gauss-Newton steps per round, with correspondences frozen
_MIN_STEP = 1e-6  # [m or rad] a Gauss-Newton solve has converged
_MIN_ROUND = 2e-5  # [m or rad] ICP has converged when a round moves less than this
_SAME_MODE_SIGMAS = 3.0  # poses within this many field sigmas (and the matching rotation) are one mode
_BATCH = 8  # coarse blocks expanded per round of branch and bound


@dataclass(frozen=True)
class MatchResult:
    """Outcome of matching one scan against a likelihood field."""

    pose: Pose2D
    """Best pose of the scan's robot frame in the field's frame."""
    score: float
    """Mean likelihood of the scan points at that pose, in [0, 1]."""
    information: np.ndarray
    """3x3 information matrix of the pose from the scan alone (no prior)."""
    ambiguity: float = 0.0
    """Best score of a clearly different pose divided by ``score`` (0 when not checked)."""
    at_boundary: bool = False
    """The search optimum lay on the edge of the window, so the truth may lie outside it."""
    searched: bool = False
    """True if the correlative search ran (False: refinement only)."""


def voxel_filter(points: np.ndarray, size: float) -> np.ndarray:
    """One point (the centroid) per occupied ``size``-metre voxel: evens out the density of a scan."""
    if points.shape[0] == 0:
        return points
    keys = np.floor(points / size).astype(np.int64)
    _, inverse, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    inverse = inverse.ravel()
    sums_x = np.bincount(inverse, weights=points[:, 0])
    sums_y = np.bincount(inverse, weights=points[:, 1])
    return np.column_stack((sums_x / counts, sums_y / counts))


def surface_points(scan: LidarScan, step: float, max_gap: float = 0.15) -> np.ndarray:
    """The scan as continuous surfaces: returns joined to their neighbours, sampled every ``step`` metres.

    Consecutive beams whose returns are close (less than ``max_gap`` and about
    two beam spacings at that range) lie on the same surface, so the line
    between them is filled in. Depth discontinuities (a chair leg in front of
    a wall) stay separate. Points are in the robot frame.

    Reference maps need this: far returns are sparse, and a map of isolated
    dots gives the surface fit (and so the matching accuracy) little to work with.
    """
    valid = np.isfinite(scan.ranges)
    points = scan.points()
    if points.shape[0] < 2:
        return points
    ranges = scan.ranges[valid]
    angles = scan.angles[valid]
    nxt = np.roll(np.arange(points.shape[0]), -1)
    spacing = np.abs((angles[nxt] - angles + np.pi) % (2.0 * np.pi) - np.pi)
    delta = points[nxt] - points
    gap = np.hypot(delta[:, 0], delta[:, 1])
    limit = np.minimum(max_gap, 0.02 + 2.0 * np.minimum(ranges, ranges[nxt]) * spacing)
    index = np.flatnonzero((gap <= limit) & (spacing < 0.1) & (gap > 0.0))
    counts = np.ceil(gap[index] / step).astype(np.intp)
    segment = np.repeat(index, counts)
    first = np.cumsum(counts) - counts
    # Midpoints of n equal pieces: symmetric along the segment. (Starting each segment at its first return
    # instead shifts every surface half a step clockwise around the sensor, a tiny but systematic rotation.)
    fraction = (np.arange(segment.shape[0]) - np.repeat(first, counts) + 0.5) / np.repeat(counts, counts)
    return np.vstack((points, points[segment] + fraction[:, None] * delta[segment]))


class ScanMatcher:
    """Correlative search plus Gauss-Newton refinement against a :class:`LikelihoodField`."""

    def __init__(self, config: SlamConfig) -> None:
        self.config = config
        self.prior_weights = np.array(
            [config.prior_translation_weight, config.prior_translation_weight, config.prior_rotation_weight]
        )

    # ------------------------------------------------------------------ public API

    def refine(self, field: LikelihoodField, points: np.ndarray, prior: Pose2D, start: Pose2D | None = None
               ) -> MatchResult:
        """Gauss-Newton only, starting at ``start`` (default: the prior). Fast; needs a good start."""
        pose, score, information = self._refine(field, points, prior, start or prior)
        return MatchResult(pose, score, information)

    def search(
        self,
        field: LikelihoodField,
        points: np.ndarray,
        prior: Pose2D,
        window: float,
        window_theta: float,
        *,
        check_ambiguity: bool = False,
        search_voxel: float = 0.0,
    ) -> MatchResult:
        """Correlative search over ``±window`` metres and ``±window_theta`` radians, then refinement.

        ``search_voxel`` (if positive) thins the scan further for the search stage only;
        refinement always uses every point.
        """
        coarse_points = voxel_filter(points, search_voxel) if search_voxel > 0.0 else points
        best, at_boundary, ambiguity = self._correlative(
            field, coarse_points, prior, window, window_theta, check_ambiguity
        )
        # The search has chosen the mode; refining towards a prior that may be decimetres away would bias it.
        pose, score, information = self._refine(field, points, best, best)
        return MatchResult(pose, score, information, ambiguity, at_boundary, searched=True)

    def score(self, field: LikelihoodField, points: np.ndarray, pose: Pose2D) -> float:
        """Mean likelihood of ``points`` (robot frame) placed at ``pose``."""
        if points.shape[0] == 0:
            return 0.0
        c, s = math.cos(pose.theta), math.sin(pose.theta)
        return float(field.likelihood(c * points[:, 0] - s * points[:, 1] + pose.x,
                                      s * points[:, 0] + c * points[:, 1] + pose.y).mean())

    # ------------------------------------------------------------------ correlative search

    def _correlative(
        self, field: LikelihoodField, points: np.ndarray, prior: Pose2D, window: float, window_theta: float,
        check_ambiguity: bool,
    ) -> tuple[Pose2D, bool, float]:
        res = field.resolution
        n = max(1, math.ceil(window / res))
        # Rotation step: the bulk of the scan (90th percentile range) moves about one cell per step.
        reach = max(float(np.percentile(np.hypot(points[:, 0], points[:, 1]), 90)), res)
        m = max(1, math.ceil(window_theta / (res / reach)))
        angle_offsets = np.arange(-m, m + 1) * (window_theta / m)
        k = 4 if n <= 12 else 8
        starts = np.arange(-n, n + 1, k)

        # Every rotation of the scan, and the cell each point lands in with zero translation offset.
        c, s = np.cos(prior.theta + angle_offsets), np.sin(prior.theta + angle_offsets)
        px = c[:, None] * points[:, 0] - s[:, None] * points[:, 1] + prior.x
        py = s[:, None] * points[:, 0] + c[:, None] * points[:, 1] + prior.y
        base_r, base_c = field.base_cells(px, py)  # (A, N)

        # Level 1: an upper bound for every (angle, block of k x k translations).
        coarse = field.padded_block_maximum(k)
        rows = base_r[:, None, None, :] + starts[None, :, None, None]
        cols = base_c[:, None, None, :] + starts[None, None, :, None]
        bounds = field.lookup(rows, cols, coarse, pad=k).mean(axis=3)  # (A, S, S)
        nearest = np.clip(0, starts, np.minimum(starts + k - 1, n))  # block offset closest to the prediction
        bounds = bounds - self._penalty(nearest[:, None] * res, nearest[None, :] * res, 0.0, window, window_theta)
        bounds = bounds - self._penalty(0.0, 0.0, angle_offsets, window, window_theta)[:, None, None]
        order = np.argsort(-bounds, axis=None, kind="stable")
        flat_bounds = bounds.ravel()[order]
        blocks = np.column_stack(np.unravel_index(order, bounds.shape))

        def expand(selection: np.ndarray, exclude: tuple[int, int, int] | None) -> tuple[float, tuple[int, int, int]]:
            """Best fine candidate inside the selected blocks (optionally outside one mode)."""
            best_value, best_index = -math.inf, (0, 0, 0)
            offsets = np.arange(k)
            for a, by, bx in blocks[selection]:
                oy = starts[by] + offsets
                ox = starts[bx] + offsets
                oy, ox = oy[oy <= n], ox[ox <= n]
                values = field.lookup(base_r[a][None, None, :] + oy[:, None, None],
                                      base_c[a][None, None, :] + ox[None, :, None]).mean(axis=2)
                values = values - self._penalty(oy[:, None] * res, ox[None, :] * res, angle_offsets[a],
                                                window, window_theta)
                if exclude is not None:
                    ea, ey, ex = exclude
                    same = (np.hypot((oy[:, None] - ey) * res, (ox[None, :] - ex) * res)
                            <= _SAME_MODE_SIGMAS * field.sigma) & (
                        abs(angle_offsets[a] - angle_offsets[ea]) * reach <= _SAME_MODE_SIGMAS * field.sigma)
                    values = np.where(same, -math.inf, values)
                flat = int(np.argmax(values))
                if values.flat[flat] > best_value:
                    iy, ix = np.unravel_index(flat, values.shape)
                    best_value, best_index = float(values.flat[flat]), (int(a), int(oy[iy]), int(ox[ix]))
            return best_value, best_index

        def branch_and_bound(
            exclude: tuple[int, int, int] | None, good_enough: float
        ) -> tuple[float, tuple[int, int, int]]:
            best_value, best_index = -math.inf, (m, 0, 0)
            for first in range(0, flat_bounds.shape[0], _BATCH):
                if flat_bounds[first] <= best_value or best_value >= good_enough:
                    break
                value, index = expand(np.arange(first, min(first + _BATCH, flat_bounds.shape[0])), exclude)
                if value > best_value:
                    best_value, best_index = value, index
            return best_value, best_index

        best_value, (a, oy, ox) = branch_and_bound(None, math.inf)
        at_boundary = abs(oy) >= n or abs(ox) >= n or abs(a - m) >= m
        ambiguity = 0.0
        if check_ambiguity and best_value > 0.0:
            alternative, _ = branch_and_bound((a, oy, ox), self.config.loop_max_ambiguity * best_value)
            ambiguity = max(alternative, 0.0) / best_value
        pose = Pose2D(prior.x + ox * res, prior.y + oy * res, prior.theta + float(angle_offsets[a]))
        return pose, at_boundary, ambiguity

    @staticmethod
    def _penalty(dx: np.ndarray | float, dy: np.ndarray | float, dtheta: np.ndarray | float,
                 window: float, window_theta: float) -> np.ndarray:
        return _TIE_BREAK * ((np.square(dx) + np.square(dy)) / window**2 + np.square(dtheta) / window_theta**2)

    # ------------------------------------------------------------------ refinement

    def _refine(self, field: LikelihoodField, points: np.ndarray, prior: Pose2D, start: Pose2D
                ) -> tuple[Pose2D, float, np.ndarray]:
        """Point-to-line ICP: alternate nearest-surface correspondences and a Gauss-Newton solve.

        With correspondences frozen, the cost (point-to-surface distances in units
        of the field's sigma, plus the prior) is smooth and nearly quadratic, so
        Gauss-Newton converges in a step or two; then points are re-associated.
        Re-associating inside the solve instead (letting the cost jump whenever a
        point crosses into another cell's Voronoi region) stalled the solver a
        few hundredths of a degree short of the optimum, always on the side of
        the odometry prediction: a small, systematic heading drift.
        """
        count = points.shape[0]
        if count == 0:
            return start, 0.0, np.zeros((3, 3))
        px, py = points[:, 0], points[:, 1]
        anchor = np.array([prior.x, prior.y, prior.theta])
        weights = self.prior_weights
        x = np.array([start.x, start.y, start.theta])
        normals = np.zeros((count, 3))
        for _ in range(_MAX_ITERATIONS):
            c, s = math.cos(x[2]), math.sin(x[2])
            element, inlier = field.correspond(c * px - s * py + x[0], s * px + c * py + x[1])
            moved = np.zeros(3)
            for _ in range(_INNER_ITERATIONS):
                c, s = math.cos(x[2]), math.sin(x[2])
                qx, qy = c * px - s * py + x[0], s * px + c * py + x[1]
                distance, gradient_q = field.distances(element, qx, qy)
                # d(point)/d(pose) = [I | R'(θ) p]; chain rule for both residuals of every point.
                dq_dtheta = np.column_stack((-s * px - c * py, c * px - s * py))
                rows = np.concatenate((gradient_q[:, 0, :], gradient_q[:, 1, :]))
                normals = np.column_stack((rows, (rows * np.vstack((dq_dtheta, dq_dtheta))).sum(axis=1)))
                both = np.concatenate((inlier, inlier))
                normals[~both] = 0.0
                residual = np.where(both, np.concatenate((distance[:, 0], distance[:, 1])), 0.0) / field.sigma
                jacobian = normals / field.sigma
                offset = x - anchor
                offset[2] = wrap_angle(offset[2])
                hessian = jacobian.T @ jacobian / count + np.diag(weights) + 1e-9 * np.eye(3)
                gradient = jacobian.T @ residual / count + weights * offset
                step = np.linalg.solve(hessian, -gradient)
                x = x + step
                moved += np.abs(step)
                if float(np.abs(step).max()) < _MIN_STEP:
                    break
            if float(moved.max()) < _MIN_ROUND:
                break
        pose = Pose2D(float(x[0]), float(x[1]), float(x[2]))
        # Information of the pose from the scan alone: point-to-surface normals (ICP), per-point noise in metres.
        information = normals.T @ normals / count / self.config.match_point_std**2
        return pose, self.score(field, points, pose), 0.5 * (information + information.T)
