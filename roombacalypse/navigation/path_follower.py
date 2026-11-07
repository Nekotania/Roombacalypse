"""Following a planned path with the local planner: the "carrot" on a stick (belief side).

The global planner produces waypoints in the map frame; the Dynamic Window
Approach avoids what the LiDAR sees right now. The follower connects them:
it projects the robot onto the path, walks ``lookahead`` metres further
along it, and hands that point to DWA as its goal (pure-pursuit style). The
projection only moves forward, so a path that doubles back near itself is
followed in order. A straight two-point path is a boustrophedon lane: the
carrot keeps the robot on the lane line instead of letting it cut corners.
"""

from __future__ import annotations

import math

from roombacalypse.core.geometry import Point, Pose2D, closest_point_on_segment


class PathFollower:
    """Turns a polyline path and the robot's pose into a local goal."""

    def __init__(self, lookahead: float, tolerance: float) -> None:
        self.lookahead = lookahead
        self.tolerance = tolerance
        self.path: list[Point] = []
        self._segment = 0

    @property
    def active(self) -> bool:
        return len(self.path) >= 1

    def set_path(self, path: list[Point]) -> None:
        self.path = list(path)
        self._segment = 0

    def clear(self) -> None:
        self.set_path([])

    @property
    def end(self) -> Point | None:
        return self.path[-1] if self.path else None

    def reached(self, pose: Pose2D) -> bool:
        end = self.end
        return end is not None and math.hypot(end[0] - pose.x, end[1] - pose.y) <= self.tolerance

    def _project(self, pose: Pose2D) -> tuple[int, float]:
        """Closest segment (searching forward from the last one) and the position along it [0, 1]."""
        if len(self.path) == 1:
            return 0, 1.0
        best, best_t, best_d = self._segment, 0.0, math.inf
        for i in range(self._segment, min(self._segment + 4, len(self.path) - 1)):
            (ax, ay), (bx, by) = self.path[i], self.path[i + 1]
            qx, qy, t = closest_point_on_segment(pose.x, pose.y, ax, ay, bx, by)
            d = math.hypot(qx - pose.x, qy - pose.y)
            if d < best_d - 1e-9:
                best, best_t, best_d = i, t, d
        self._segment = best
        return best, best_t

    def goal(self, pose: Pose2D) -> Point | None:
        """The point ``lookahead`` metres along the path beyond the robot's projection on it (map frame)."""
        if not self.path:
            return None
        if len(self.path) == 1:
            return self.path[0]
        index, t = self._project(pose)
        (ax, ay), (bx, by) = self.path[index], self.path[index + 1]
        x, y = ax + t * (bx - ax), ay + t * (by - ay)
        left = self.lookahead
        while True:
            nx, ny = self.path[index + 1]
            step = math.hypot(nx - x, ny - y)
            if step >= left:
                return (x + (nx - x) * left / step, y + (ny - y) * left / step)
            left -= step
            x, y = nx, ny
            index += 1
            if index >= len(self.path) - 1:
                return self.path[-1]

    def remaining(self, pose: Pose2D) -> float:
        """Distance along the path from the robot's projection to the end [m]."""
        if not self.path:
            return 0.0
        if len(self.path) == 1:
            return math.hypot(self.path[0][0] - pose.x, self.path[0][1] - pose.y)
        index, t = self._project(pose)
        (ax, ay), (bx, by) = self.path[index], self.path[index + 1]
        total = (1.0 - t) * math.hypot(bx - ax, by - ay)
        for (px, py), (qx, qy) in zip(self.path[index + 1:], self.path[index + 2:], strict=False):
            total += math.hypot(qx - px, qy - py)
        return total + math.hypot(pose.x - (ax + t * (bx - ax)), pose.y - (ay + t * (by - ay)))
