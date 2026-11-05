"""Ground-truth scoring of localisation and mapping (harness: reads both sides, writes neither).

**Trajectory error.** The robot's map frame is anchored at its boot pose on
the dock, which it knows, so belief can be compared with truth directly
(``world = boot_pose ∘ map``). No alignment is applied: an error in the
global heading of the map counts, because it would matter for driving home.
For comparison with SLAM benchmarks (Sturm et al., 2012) the *aligned*
absolute trajectory error (best rigid fit, Horn's method in 2D) is reported
as well.

Two trajectories are scored:

* **online**: the pose the robot used for control at every control tick,
  the error that matters for driving;
* **keyframes**: the final, optimised keyframe poses, the error that
  matters for the map (loop closures correct it after the fact).

**Map accuracy** compares the robot's occupancy grid with the real geometry
in the LiDAR's scan plane:

* *precision*: the fraction of occupied cells that are right. A cell is
  right if its centre is within ``tolerance`` (one cell) of a real surface
  (a wall, or the outline of an obstacle the LiDAR can see), or lies inside
  solid matter (behind a wall, inside furniture) within three cells of one.
  Range noise makes every occupancy grid thicken surfaces by a cell or two
  *into* the solid side, where beams never come to clear it; that is
  harmless, and real walls are thicker than that. What precision must catch
  are phantom obstacles in free space. ``surface_precision`` is the strict
  variant (one cell, either side) for reference;
* *recall*: the fraction of real surface the robot actually saw (its returns
  placed at the *true* scan pose, kept on a 1 cm raster) that has an
  occupied map cell centre within ``tolerance``. Surfaces never in view
  cannot be expected in the map.

Obstacles pushed during a run are scored where they end up, so pushing
furniture costs precision, as it should.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.spatial import KDTree

from roombacalypse.core.geometry import (
    Pose2D,
    inverse_transform_points,
    points_in_polygon,
    transform_points,
    wrap_angles,
)
from roombacalypse.core.messages import LidarScan
from roombacalypse.environment.shapes import Circle
from roombacalypse.environment.world import World
from roombacalypse.mapping.occupancy_grid import OccupancyGrid

_OBSERVED_CELL = 0.01  # [m] raster of the "surface the robot saw" set
_SOLID_DEPTH = 3.0  # occupied cells inside solid matter count as right up to this many tolerances deep


@dataclass(frozen=True)
class TrajectoryError:
    """Position and heading errors of an estimated trajectory against the truth."""

    samples: int
    rms_position: float
    """[m]"""
    max_position: float
    """[m]"""
    rms_heading: float
    """[rad]"""
    max_heading: float
    """[rad]"""
    final_position: float
    """[m]"""
    aligned_rms_position: float
    """[m] after the best rigid alignment (the usual ATE of SLAM benchmarks)."""


def trajectory_error(estimate: np.ndarray, truth: np.ndarray) -> TrajectoryError:
    """Compare ``(N, 3)`` arrays of ``(x, y, θ)`` poses in the same (world) frame."""
    if estimate.shape[0] == 0:
        return TrajectoryError(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    distance = np.hypot(estimate[:, 0] - truth[:, 0], estimate[:, 1] - truth[:, 1])
    heading = np.abs(wrap_angles(estimate[:, 2] - truth[:, 2]))
    return TrajectoryError(
        samples=int(estimate.shape[0]),
        rms_position=float(np.sqrt(np.mean(distance**2))),
        max_position=float(distance.max()),
        rms_heading=float(np.sqrt(np.mean(heading**2))),
        max_heading=float(heading.max()),
        final_position=float(distance[-1]),
        aligned_rms_position=aligned_rmse(estimate[:, :2], truth[:, :2]),
    )


def aligned_rmse(estimate: np.ndarray, truth: np.ndarray) -> float:
    """RMS position error after the rigid transform that best maps ``estimate`` onto ``truth`` (Horn, 2D)."""
    if estimate.shape[0] < 2:
        return float(np.hypot(*(estimate - truth).T).mean()) if estimate.shape[0] else 0.0
    p = estimate - estimate.mean(axis=0)
    q = truth - truth.mean(axis=0)
    angle = math.atan2(float((p[:, 0] * q[:, 1] - p[:, 1] * q[:, 0]).sum()), float((p * q).sum()))
    c, s = math.cos(angle), math.sin(angle)
    rotated = np.column_stack((c * p[:, 0] - s * p[:, 1], s * p[:, 0] + c * p[:, 1]))
    return float(np.sqrt(np.mean(np.sum((rotated - q) ** 2, axis=1))))


class TrajectoryRecorder:
    """Collects (truth, belief) pairs during a run, in the world frame."""

    def __init__(self, boot_pose: Pose2D) -> None:
        self.boot_pose = boot_pose
        self._truth: list[tuple[float, float, float]] = []
        self._belief: list[tuple[float, float, float]] = []
        self._truth_at: dict[float, tuple[float, float, float]] = {}

    def record(self, time: float, truth: Pose2D, belief_in_map: Pose2D) -> None:
        """One sample: the true pose and what the robot believed (map frame) at ``time``."""
        world = self.boot_pose.compose(belief_in_map)
        self._truth.append((truth.x, truth.y, truth.theta))
        self._belief.append((world.x, world.y, world.theta))
        self._truth_at[round(time, 6)] = (truth.x, truth.y, truth.theta)

    def online_error(self) -> TrajectoryError:
        return trajectory_error(np.asarray(self._belief).reshape(-1, 3), np.asarray(self._truth).reshape(-1, 3))

    def keyframe_error(self, times: list[float], poses_in_map: np.ndarray) -> TrajectoryError:
        """Error of (optimised) keyframe poses, given the times their scans were taken."""
        pairs = [(self._truth_at[round(t, 6)], pose) for t, pose in zip(times, poses_in_map, strict=True)
                 if round(t, 6) in self._truth_at]
        if not pairs:
            return trajectory_error(np.zeros((0, 3)), np.zeros((0, 3)))
        truth = np.asarray([p[0] for p in pairs])
        local = np.asarray([p[1] for p in pairs])
        world = transform_points(self.boot_pose, local[:, :2])
        estimate = np.column_stack((world, local[:, 2] + self.boot_pose.theta))
        return trajectory_error(estimate, truth)


@dataclass(frozen=True)
class MapScore:
    precision: float
    recall: float
    surface_precision: float
    """Strict precision: occupied cells within one tolerance of a surface, on either side."""
    occupied_cells: int
    observed_cells: int


class MapAccuracy:
    """Scores the robot's occupancy grid against the true geometry in the LiDAR's scan plane."""

    def __init__(self, world: World, scan_height: float, boot_pose: Pose2D) -> None:
        self.world = world
        self.scan_height = scan_height
        self.boot_pose = boot_pose
        bounds = world.bounds
        self._origin = (bounds.min_x - 1.0, bounds.min_y - 1.0)
        self._shape = (math.ceil((bounds.height + 2.0) / _OBSERVED_CELL),
                       math.ceil((bounds.width + 2.0) / _OBSERVED_CELL))
        self.observed = np.zeros(self._shape, dtype=bool)

    def observe(self, scan: LidarScan, true_pose: Pose2D) -> None:
        """Remember which surfaces this scan saw: its returns placed at the *true* pose."""
        points = transform_points(true_pose, scan.points())
        rows = np.floor((points[:, 1] - self._origin[1]) / _OBSERVED_CELL).astype(np.intp)
        cols = np.floor((points[:, 0] - self._origin[0]) / _OBSERVED_CELL).astype(np.intp)
        inside = (rows >= 0) & (rows < self._shape[0]) & (cols >= 0) & (cols < self._shape[1])
        self.observed[rows[inside], cols[inside]] = True

    def surface_distance(self, points: np.ndarray) -> np.ndarray:
        """Distance from each world point to the nearest true surface in the scan plane [m]."""
        best = np.full(points.shape[0], np.inf)
        segments = list(self.world.walls)
        h = self.scan_height
        for obstacle in self.world.obstacles_in_band(h, h + 1e-9):
            if isinstance(obstacle.shape, Circle):
                c = obstacle.shape
                best = np.minimum(best, np.abs(np.hypot(points[:, 0] - c.cx, points[:, 1] - c.cy) - c.radius))
            else:
                segments.extend(obstacle.shape.edges())
        seg = np.asarray(segments, dtype=float).reshape(-1, 4)
        a = seg[:, :2]
        ab = seg[:, 2:] - a
        length2 = np.maximum((ab**2).sum(axis=1), 1e-18)
        for chunk in range(0, points.shape[0], 2048):
            p = points[chunk:chunk + 2048]
            ap = p[:, None, :] - a[None, :, :]
            t = np.clip((ap * ab[None, :, :]).sum(axis=2) / length2[None, :], 0.0, 1.0)
            closest = a[None, :, :] + t[:, :, None] * ab[None, :, :]
            d = np.hypot(p[:, None, 0] - closest[:, :, 0], p[:, None, 1] - closest[:, :, 1]).min(axis=1)
            best[chunk:chunk + 2048] = np.minimum(best[chunk:chunk + 2048], d)
        return best

    def solid(self, points: np.ndarray) -> np.ndarray:
        """True where a world point lies in solid matter at the scan plane (outside the floor or in an obstacle)."""
        solid = ~points_in_polygon(points[:, 0], points[:, 1], self.world.outline)
        h = self.scan_height
        for obstacle in self.world.obstacles_in_band(h, h + 1e-9):
            solid |= obstacle.shape.contains_points(points[:, 0], points[:, 1])
        return solid

    def score(self, grid: OccupancyGrid, tolerance: float | None = None) -> MapScore:
        """Precision and recall of ``grid``'s occupied cells (``tolerance`` defaults to one map cell)."""
        tol = grid.resolution if tolerance is None else tolerance
        occupied = grid.occupied_mask()
        centres = grid.cell_centres(occupied)
        if centres.shape[0] == 0:
            return MapScore(0.0, 0.0, 0.0, 0, int(self.observed.sum()))
        world_centres = transform_points(self.boot_pose, centres)
        distance = self.surface_distance(world_centres)
        near = distance <= tol
        right = near | (self.solid(world_centres) & (distance <= _SOLID_DEPTH * tol))
        precision, surface_precision = float(right.mean()), float(near.mean())

        rows, cols = np.nonzero(self.observed)
        seen = np.column_stack((self._origin[0] + (cols + 0.5) * _OBSERVED_CELL,
                                self._origin[1] + (rows + 0.5) * _OBSERVED_CELL))
        if seen.shape[0] == 0:
            return MapScore(precision, 0.0, surface_precision, int(centres.shape[0]), 0)
        distance, _ = KDTree(centres).query(inverse_transform_points(self.boot_pose, seen), distance_upper_bound=tol)
        recall = float(np.isfinite(distance).mean())
        return MapScore(precision, recall, surface_precision, int(centres.shape[0]), int(seen.shape[0]))
