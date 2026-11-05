"""Graph-based SLAM: where am I, and what does the place look like? (belief side)

This is the design used by the LiDAR SLAM systems that run on real robots
(Karto and its successor slam_toolbox, the ROS 2 default; Cartographer),
cut down to what an apartment-sized robot vacuum needs. See
docs/algorithms/slam.md for the derivations and measured performance.

**Front end (tracking).** Odometry predicts the pose. Whenever the robot has
travelled ``node_distance`` or turned ``node_angle_deg`` since the last
keyframe, the new scan is matched against a *local map*, the likelihood
field of the last ``local_map_nodes`` keyframes' scans:

1. Gauss-Newton refinement from the odometry prediction (fast; enough almost always);
2. if the scan fits poorly, a correlative search over a wider window (after
   a bump or a wheel slip the prediction can be centimetres off).

The matched pose becomes a new **keyframe**: a node in the pose graph,
joined to the previous keyframe by a constraint whose information combines
what the scan pins down with the robot's own odometry noise model (so a
featureless corridor leans on odometry in the direction the scan cannot see).

**Back end.** Every ``loop_interval`` metres the newest keyframe is matched
against a *chain* of old keyframes nearby (an earlier visit to the same
place), with a wider window, and only if the match is strong, unambiguous
and not at the edge of the window is a **loop-closure** constraint added.
The pose graph is then re-optimised (Levenberg-Marquardt with a Cauchy kernel
on loop closures), which bends the whole trajectory back into consistency.
This is what option (a) of decision D7 could never do: an error that has
been mapped can be corrected.

**Output.** ``pose`` is the robot's estimate in the *map frame* (origin at the
boot pose on the dock). Internally it is ``correction ∘ odometry``, the
``map → odom`` transform of ROS: odometry stays smooth for control, and the
correction jumps a few millimetres at each keyframe and a few centimetres at
a loop closure. The occupancy grid (``grid``) is rendered from all keyframes
at their current poses: incrementally while nothing moves, and completely,
lazily, after an optimisation moved them.

**Limitations** (honest, documented): keyframes whose scan does not fit at
all are kept out of the map, but there is no global relocalisation, so a
robot picked up and carried elsewhere ("kidnapped") stays lost. Loop
closure searches a bounded window, so drift beyond it cannot be corrected.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import Protocol

import numpy as np

from roombacalypse.core.config import MappingConfig, SlamConfig
from roombacalypse.core.geometry import Pose2D, transform_points
from roombacalypse.core.messages import LidarScan
from roombacalypse.localization.scan_matcher import MatchResult, ScanMatcher, surface_points, voxel_filter
from roombacalypse.mapping.likelihood_field import LikelihoodField
from roombacalypse.mapping.occupancy_grid import OccupancyGrid
from roombacalypse.mapping.pose_graph import PoseGraph

logger = logging.getLogger(__name__)

_LOST_SCORE = 0.2  # a scan fitting worse than this is not trusted (kept out of the map)
_BETTER_BY = 0.05  # the wide search must beat refinement by this much to override it
_SCAN_FLOOR = (0.01, 0.01, math.radians(0.25))  # std floors of a scan-match constraint [m, m, rad]
_LOOP_FLOOR = (0.02, 0.02, math.radians(0.5))  # std floors of a loop-closure constraint
_ODOMETRY_FLOOR = (0.002, math.radians(0.1))  # std floors of an odometry step [m, rad]
_UNOBSERVED = 1.0  # information added to every direction before inverting (caps std at 1 m / 1 rad)
_LOCAL_MAP_REFRESH = 3  # rebuild the local map every this many keyframes (it changes slowly)
_SEARCH_VOXEL = 0.08  # [m] the correlative stage of loop closure uses a further thinned scan
_LOCAL_MAP_RADIUS = 8.0  # [m] the local map keeps mapped points this close to the newest keyframe


class Slam(Protocol):
    """What the rest of the robot's software needs from a SLAM system."""

    @property
    def pose(self) -> Pose2D:
        """Current pose estimate in the map frame."""
        ...

    @property
    def grid(self) -> OccupancyGrid:
        """The occupancy grid, consistent with the current pose estimates."""
        ...

    def reset(self) -> None: ...

    def update(self, time: float, odometry: Pose2D, scan: LidarScan | None) -> Pose2D:
        """Fold in the latest odometry pose and (if one arrived) a scan; returns ``pose``."""
        ...


@dataclass(frozen=True, eq=False)
class Keyframe:
    """A scan the map is built from, and the node of the pose graph it belongs to."""

    index: int
    time: float
    odometry: Pose2D
    """Odometry pose when the scan was taken."""
    travel: float
    """Distance travelled (by odometry) from boot to this keyframe [m]."""
    points: np.ndarray
    """Thinned scan points in the robot frame (what gets matched)."""
    surface: np.ndarray
    """The scan's surfaces, densely sampled, in the robot frame (what others are matched against)."""
    scan: LidarScan
    """The full scan (for the occupancy grid)."""
    score: float
    """How well the scan fitted the local map when it was added (1 for the first keyframe)."""
    tracked: bool
    """False if the scan did not fit at all; such keyframes stay out of the map."""


@dataclass(frozen=True)
class LoopClosure:
    """A loop closure that was accepted."""

    time: float
    node: int
    other: int
    score: float
    correction: float
    """How far the optimisation moved the newest keyframe [m]."""


def _information(std: tuple[float, float, float]) -> np.ndarray:
    return np.diag(1.0 / np.square(std))


def _constraint_information(scan_information: np.ndarray, floor: tuple[float, float, float]) -> np.ndarray:
    """Turn a scan's information into a constraint's: cap the certainty, keep the shape."""
    covariance = np.linalg.inv(scan_information + _UNOBSERVED * np.eye(3)) + np.diag(np.square(floor))
    information = np.linalg.inv(covariance)
    return 0.5 * (information + information.T)


class GraphSlam:
    """Keyframes, scan matching, loop closure and pose-graph optimisation; implements :class:`Slam`."""

    def __init__(self, mapping: MappingConfig, config: SlamConfig) -> None:
        self.mapping_config = mapping
        self.config = config
        self.matcher = ScanMatcher(config)
        self.graph = PoseGraph()
        self.keyframes: list[Keyframe] = []
        self.loop_closures: list[LoopClosure] = []
        self._grid = OccupancyGrid(mapping)
        self._grid_dirty = False
        self._local_field: LikelihoodField | None = None
        self._travel = np.zeros(0)
        self.correction = Pose2D(0.0, 0.0, 0.0)
        """The map → odometry transform: ``pose = correction ∘ odometry``."""
        self.odometry = Pose2D(0.0, 0.0, 0.0)
        self.loop_attempts = 0
        self.loops_rejected = 0
        self.wide_searches = 0
        self._last_loop_travel = 0.0

    # ------------------------------------------------------------------ Slam protocol

    def reset(self) -> None:
        self.graph.clear()
        self.keyframes.clear()
        self.loop_closures.clear()
        self._grid.clear()
        self._grid_dirty = False
        self._local_field = None
        self._travel = np.zeros(0)
        self.correction = Pose2D(0.0, 0.0, 0.0)
        self.odometry = Pose2D(0.0, 0.0, 0.0)
        self.loop_attempts = self.loops_rejected = self.wide_searches = 0
        self._last_loop_travel = 0.0

    @property
    def pose(self) -> Pose2D:
        return self.correction.compose(self.odometry)

    @property
    def grid(self) -> OccupancyGrid:
        if self._grid_dirty:
            self._render_grid()
        return self._grid

    @property
    def grid_is_stale(self) -> bool:
        """True if reading :attr:`grid` would re-render it from every keyframe (a loop closure moved them)."""
        return self._grid_dirty

    def update(self, time: float, odometry: Pose2D, scan: LidarScan | None) -> Pose2D:
        self.odometry = odometry
        if scan is not None and self._keyframe_due(odometry):
            self._add_keyframe(time, odometry, scan)
        return self.pose

    # ------------------------------------------------------------------ statistics

    @property
    def tracking_score(self) -> float:
        """How well the latest keyframe fitted the local map (1 before the first comparison)."""
        return self.keyframes[-1].score if self.keyframes else 1.0

    @property
    def lost(self) -> bool:
        """The latest keyframe's scan did not fit the map at all."""
        return bool(self.keyframes) and not self.keyframes[-1].tracked

    def node_poses(self) -> np.ndarray:
        """Keyframe poses ``(n, 3)`` in the map frame (a copy)."""
        return self.graph.poses.copy()

    # ------------------------------------------------------------------ front end

    def _keyframe_due(self, odometry: Pose2D) -> bool:
        if not self.keyframes:
            return True
        moved = odometry.relative_to(self.keyframes[-1].odometry)
        return (math.hypot(moved.x, moved.y) >= self.config.node_distance
                or abs(moved.theta) >= math.radians(self.config.node_angle_deg))

    def _add_keyframe(self, time: float, odometry: Pose2D, scan: LidarScan) -> None:
        cfg = self.config
        points = voxel_filter(scan.points(), cfg.scan_voxel)
        surface = surface_points(scan, 0.5 * cfg.field_resolution)
        predicted = self.correction.compose(odometry)
        if not self.keyframes:
            index = self.graph.add_node(predicted)
            keyframe = Keyframe(index, time, odometry, 0.0, points, surface, scan, 1.0, True)
        else:
            previous = self.keyframes[-1]
            step = odometry.relative_to(previous.odometry)
            travel = previous.travel + math.hypot(step.x, step.y)
            result = self._track(points, predicted)
            tracked = result.score >= _LOST_SCORE
            pose = result.pose if tracked else predicted
            information = self._odometry_information(step)
            if tracked:
                information = information + _constraint_information(result.information, _SCAN_FLOOR)
            index = self.graph.add_node(pose)
            self.graph.add_edge(previous.index, index, pose.relative_to(self.graph.pose(previous.index)), information)
            keyframe = Keyframe(index, time, odometry, travel, points, surface, scan, result.score, tracked)
            if not tracked:
                logger.warning("t=%.1f s: scan does not fit the map (score %.2f); keeping it out", time, result.score)
        self.keyframes.append(keyframe)
        self._travel = np.append(self._travel, keyframe.travel)
        self.correction = self.graph.pose(keyframe.index).compose(odometry.inverse())
        if keyframe.tracked and not self._grid_dirty:
            self._grid.insert_scan(self.graph.pose(keyframe.index), scan)
        closed = keyframe.tracked and self._try_loop_closure(time)
        if closed or self._local_field is None or len(self.keyframes) % _LOCAL_MAP_REFRESH == 0:
            self._rebuild_local_field()

    def _track(self, points: np.ndarray, predicted: Pose2D) -> MatchResult:
        cfg = self.config
        assert self._local_field is not None
        result = self.matcher.refine(self._local_field, points, predicted)
        if result.score < cfg.track_min_score:
            self.wide_searches += 1
            wide = self.matcher.search(
                self._local_field, points, predicted, cfg.track_window, math.radians(cfg.track_window_deg)
            )
            if wide.score > result.score + _BETTER_BY:
                result = wide
        return result

    def _odometry_information(self, step: Pose2D) -> np.ndarray:
        cfg = self.config
        distance = math.hypot(step.x, step.y)
        sigma_xy = max(cfg.odometry_translation_noise * distance, _ODOMETRY_FLOOR[0])
        sigma_theta = max(
            cfg.odometry_rotation_noise * abs(step.theta) + cfg.odometry_heading_drift * distance, _ODOMETRY_FLOOR[1]
        )
        return _information((sigma_xy, sigma_xy, sigma_theta))

    def _reference(self, indices: list[int], margin: float, near: Pose2D | None = None) -> LikelihoodField:
        """A surface field from the given keyframes' scans at their current poses (map frame)."""
        cfg = self.config
        parts = [transform_points(self.graph.pose(self.keyframes[i].index), self.keyframes[i].surface)
                 for i in indices if self.keyframes[i].tracked]
        points = np.vstack(parts) if parts else np.empty((0, 2))
        if near is not None:
            points = points[np.hypot(points[:, 0] - near.x, points[:, 1] - near.y) <= _LOCAL_MAP_RADIUS]
        return LikelihoodField(points, cfg.field_resolution, cfg.field_sigma, margin)

    def _rebuild_local_field(self) -> None:
        cfg = self.config
        first = max(0, len(self.keyframes) - cfg.local_map_nodes)
        newest = self.graph.pose(self.keyframes[-1].index)
        self._local_field = self._reference(list(range(first, len(self.keyframes))),
                                            cfg.track_window + 4.0 * cfg.field_sigma, near=newest)

    # ------------------------------------------------------------------ back end

    def _try_loop_closure(self, time: float) -> bool:
        """Look for a loop closure at the newest keyframe; True if one was added (the graph moved)."""
        cfg = self.config
        current = self.keyframes[-1]
        if current.travel - self._last_loop_travel < cfg.loop_interval:
            return False
        self._last_loop_travel = current.travel
        poses = self.graph.poses
        n = current.index
        old = self._travel[:n] <= current.travel - cfg.loop_min_travel
        tracked = np.array([kf.tracked for kf in self.keyframes[:n]], dtype=bool)
        distance = np.hypot(poses[:n, 0] - poses[n, 0], poses[:n, 1] - poses[n, 1])
        candidates = np.flatnonzero(old & tracked & (distance <= cfg.loop_search_radius))
        if candidates.shape[0] == 0:
            return False
        nearest = int(candidates[np.argmin(distance[candidates])])
        chain = [j for j in range(max(0, nearest - cfg.loop_chain_nodes), min(n, nearest + cfg.loop_chain_nodes + 1))
                 if old[j]]
        field = self._reference(chain, cfg.loop_window + 4.0 * cfg.field_sigma)
        self.loop_attempts += 1
        prior = self.graph.pose(n)
        result = self.matcher.search(field, current.points, prior, cfg.loop_window, math.radians(cfg.loop_window_deg),
                                     check_ambiguity=True, search_voxel=_SEARCH_VOXEL)
        if (result.score < cfg.loop_min_score or result.ambiguity > cfg.loop_max_ambiguity
                or result.at_boundary):
            self.loops_rejected += 1
            logger.debug("t=%.1f s: loop closure %d -> %d rejected (score %.2f, ambiguity %.2f, boundary %s)",
                         time, nearest, n, result.score, result.ambiguity, result.at_boundary)
            return False
        self.graph.add_edge(nearest, n, result.pose.relative_to(self.graph.pose(nearest)),
                            _constraint_information(result.information, _LOOP_FLOOR), robust=True, kind="loop")
        before = self.graph.pose(n)
        self.graph.optimize(cfg.optimizer_iterations, cfg.loop_robust_scale)
        after = self.graph.pose(n)
        self.loop_closures.append(LoopClosure(time, n, nearest, result.score, before.distance_to(after)))
        self.correction = after.compose(current.odometry.inverse())
        self._grid_dirty = True
        logger.debug("t=%.1f s: loop closure %d -> %d (score %.2f) moved the robot %.3f m",
                     time, nearest, n, result.score, before.distance_to(after))
        return True

    def _render_grid(self) -> None:
        self._grid.clear()
        for keyframe in self.keyframes:
            if keyframe.tracked:
                self._grid.insert_scan(self.graph.pose(keyframe.index), keyframe.scan)
        self._grid_dirty = False
