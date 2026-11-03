"""Short-term obstacle memory and free-space queries (belief side).

The robot has no map until Milestone 3, but it still needs to remember two
things between LiDAR sweeps:

* the latest scan, stored in the odometry frame so it can be re-expressed in
  the current body frame as the robot moves (motion compensation by odometry);
* where it recently *bumped* into something. The LiDAR cannot see socks or
  cables below its scan plane, so a bump location is the only evidence of
  them; it is kept for a while as virtual obstacle points.
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np

from roombacalypse.core.geometry import Pose2D, inverse_transform_points, transform_points
from roombacalypse.core.messages import LidarScan

_BUMP_SPREAD = np.radians([-14.0, 0.0, 14.0])
_EMPTY = np.empty((0, 2))


def swept_free_distance(points: np.ndarray, bearings: np.ndarray, radius: float, limit: float) -> np.ndarray:
    """How far a disc of ``radius`` at the origin can move along each bearing before touching a point.

    For a point at (s, l) in coordinates along/across the direction of travel,
    the disc's centre touches it after ``s − sqrt(radius² − l²)`` when
    ``|l| < radius``; otherwise the point is never touched. Points behind the
    disc are ignored. The result is clipped to ``[0, limit]``.
    """
    if points.shape[0] == 0:
        return np.full(bearings.shape[0], float(limit))
    ux, uy = np.cos(bearings)[:, None], np.sin(bearings)[:, None]
    along = points[:, 0][None, :] * ux + points[:, 1][None, :] * uy
    across = -points[:, 0][None, :] * uy + points[:, 1][None, :] * ux
    blocking = (np.abs(across) < radius) & (along > 0.0)
    stop = along - np.sqrt(np.maximum(radius * radius - across * across, 0.0))
    stop = np.where(blocking, stop, limit)
    return np.clip(stop.min(axis=1), 0.0, limit)


class ObstacleMemory:
    """Latest scan plus recent bump locations, in the odometry frame."""

    def __init__(self, bump_memory_seconds: float) -> None:
        self.bump_memory_seconds = bump_memory_seconds
        self._scan_points = _EMPTY
        self._bumps: deque[tuple[float, np.ndarray]] = deque()

    def reset(self) -> None:
        self._scan_points = _EMPTY
        self._bumps.clear()

    def update_scan(self, scan: LidarScan, odom_pose: Pose2D) -> None:
        """Replace the remembered scan; ``odom_pose`` is the odometry pose when it was taken."""
        self._scan_points = transform_points(odom_pose, scan.points())

    def add_bump(self, time: float, odom_pose: Pose2D, bearing: float, robot_radius: float) -> None:
        """Remember an invisible obstacle just outside the bumper at ``bearing`` (body frame)."""
        reach = robot_radius + 0.02
        angles = bearing + _BUMP_SPREAD
        local = np.column_stack((reach * np.cos(angles), reach * np.sin(angles)))
        self._bumps.append((time, transform_points(odom_pose, local)))

    def expire(self, time: float) -> None:
        while self._bumps and time - self._bumps[0][0] > self.bump_memory_seconds:
            self._bumps.popleft()

    @property
    def bump_points(self) -> np.ndarray:
        """Remembered bump points in the odometry frame."""
        return np.vstack([p for _, p in self._bumps]) if self._bumps else _EMPTY

    def points_in_body(self, odom_pose: Pose2D, max_distance: float) -> np.ndarray:
        """All remembered points within ``max_distance``, in the current body frame."""
        stacked = np.vstack((self._scan_points, self.bump_points))
        if stacked.shape[0] == 0:
            return _EMPTY
        local = inverse_transform_points(odom_pose, stacked)
        near = np.hypot(local[:, 0], local[:, 1]) <= max_distance
        return local[near]

    def __len__(self) -> int:
        return int(self._scan_points.shape[0]) + 3 * len(self._bumps)


def nearest_distance(points: np.ndarray) -> float:
    """Distance from the origin to the closest point (``inf`` if there are none)."""
    if points.shape[0] == 0:
        return math.inf
    return float(np.min(np.hypot(points[:, 0], points[:, 1])))
