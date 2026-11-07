"""Coverage planning: what to sweep next, on a map that is still being built (belief side).

An online adaptation of BA* (Viet et al., "BA*: an online complete coverage
algorithm for cleaning robots", 2013) with frontier exploration (Yamauchi,
1997), on the costmap's grid:

* :class:`CoverageMemory` remembers where the robot's footprint has been
  (map frame, the SLAM grid's lattice).
* **Uncovered floor** is known-free floor within reach of the footprint (the
  costmap's traversable cells grown by the robot radius) that the footprint
  has not swept. A **useful** position is a traversable cell from which the
  footprint would sweep at least ``min_gain`` uncovered cells.
* **Boustrophedon motion.** Lanes run along the map's x axis. A lane is
  driven to the last useful cell before the free space ends. Then the robot
  shifts sideways by up to ``lane_width`` (by less if the free space ends
  sooner, which puts the last lane right at the edge and sweeps up to the
  wall) and reverses.
* **Backtracking.** When neither lane continuation nor a shift is possible
  (a "critical point" in BA*), the nearest *rich* cell by path (a Dijkstra
  field from the robot), one where the footprint would sweep at least half
  its area anew, is the next target, and a new lane starts there. Only when
  no rich cell is left does the robot go for the leftover fringes around
  furniture, nearest first: chasing every fringe as soon as it appears
  wasted minutes in tight spots while big areas waited.
* **Frontier exploration.** When no useful cell is reachable, the nearest
  frontier (known free floor next to never-seen space) is visited, so the map
  grows. Coverage is complete when there is neither.

The planner only decides *where* to go; :mod:`roombacalypse.autonomy.executive`
drives there through the path follower and the Dynamic Window Approach.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import StrEnum

import numpy as np
from scipy import ndimage

from roombacalypse.core.geometry import Point, Pose2D
from roombacalypse.planning.astar import dijkstra_field, line_is_clear, path_to, prune_path
from roombacalypse.planning.costmap import Cell, Costmap

_MIN_GAIN = 6  # uncovered cells (of 5 cm) a position must newly sweep to be worth visiting: 0.015 m²
_RICH_FRACTION = 0.5  # backtrack first to where the footprint would sweep at least this fraction anew
_EDGE_COST = 1.0  # pruned paths may cut through cells up to this extra cost (stay off the walls)


_STRANDED_AREA = 1.0  # [m²]: a reachable pocket smaller than this, with useful floor elsewhere, is a planning failure


class Stranded(Exception):
    """The planner cannot start from where the robot is: no traversable cell nearby, or only a small pocket
    cut off from floor that still needs sweeping. Not the same as done: the robot has to get out first."""


class TargetKind(StrEnum):
    LANE = "lane"
    """Continue or start a boustrophedon lane: a straight run."""
    SHIFT = "shift"
    """Step sideways onto the next lane."""
    BACKTRACK = "backtrack"
    """Travel to the nearest uncovered floor (BA* backtracking)."""
    FRONTIER = "frontier"
    """Travel to the edge of the known map to see more of it."""


@dataclass(frozen=True)
class Target:
    kind: TargetKind
    path: list[Point]
    """Waypoints in the map frame, from (about) the robot to the target."""
    direction: int = 0
    """Lane direction along x (+1 or -1) once the target is reached (0: choose then)."""


class CoverageMemory:
    """Where the robot believes its footprint has been, on a growable grid of the map's lattice."""

    def __init__(self, resolution: float, radius: float) -> None:
        self.resolution = resolution
        self.radius = radius
        k = math.ceil(radius / resolution)
        offsets = np.arange(-k, k + 1)
        self._disc = (offsets[:, None] ** 2 + offsets[None, :] ** 2) * resolution**2 <= radius**2
        self._k = k
        self.swept = np.zeros((0, 0), dtype=bool)
        self._row0 = self._col0 = 0  # lattice index of swept[0, 0]
        self._last: Point | None = None

    def reset(self) -> None:
        self.swept = np.zeros((0, 0), dtype=bool)
        self._row0 = self._col0 = 0
        self._last = None

    def _grow(self, row: int, col: int) -> None:
        k, margin = self._k, 40
        rows, cols = self.swept.shape
        if self.swept.size and self._row0 <= row - k and row + k < self._row0 + rows and (
            self._col0 <= col - k and col + k < self._col0 + cols
        ):
            return
        if not self.swept.size:
            r0, c0, r1, c1 = row - k - margin, col - k - margin, row + k + margin + 1, col + k + margin + 1
        else:
            r0 = min(self._row0, row - k - margin)
            c0 = min(self._col0, col - k - margin)
            r1 = max(self._row0 + rows, row + k + margin + 1)
            c1 = max(self._col0 + cols, col + k + margin + 1)
        grown = np.zeros((r1 - r0, c1 - c0), dtype=bool)
        if self.swept.size:
            grown[self._row0 - r0:self._row0 - r0 + rows, self._col0 - c0:self._col0 - c0 + cols] = self.swept
        self.swept, self._row0, self._col0 = grown, r0, c0

    def mark(self, pose: Pose2D) -> None:
        """The footprint swept the floor around ``pose`` (map frame)."""
        last = self._last
        if last is not None and math.hypot(pose.x - last[0], pose.y - last[1]) < 0.5 * self.resolution:
            return
        self._last = pose.position
        row, col = math.floor(pose.y / self.resolution), math.floor(pose.x / self.resolution)
        self._grow(row, col)
        r, c, k = row - self._row0, col - self._col0, self._k
        self.swept[r - k:r + k + 1, c - k:c + k + 1] |= self._disc

    def aligned(self, costmap: Costmap) -> np.ndarray:
        """The swept mask on ``costmap``'s cells (which share the lattice)."""
        rows, cols = costmap.shape
        out = np.zeros((rows, cols), dtype=bool)
        if not self.swept.size:
            return out
        orow = round(costmap.origin[1] / costmap.resolution)
        ocol = round(costmap.origin[0] / costmap.resolution)
        r0, c0 = self._row0 - orow, self._col0 - ocol
        sr, sc = self.swept.shape
        a0, b0 = max(0, r0), max(0, c0)
        a1, b1 = min(rows, r0 + sr), min(cols, c0 + sc)
        if a0 < a1 and b0 < b1:
            out[a0:a1, b0:b1] = self.swept[a0 - r0:a1 - r0, b0 - c0:b1 - c0]
        return out

    @property
    def area(self) -> float:
        """Floor swept so far [m²]."""
        return float(self.swept.sum()) * self.resolution**2


@dataclass(frozen=True, eq=False)
class CoverageState:
    """What the planner computed from one costmap (also drawn by the viewer)."""

    costmap: Costmap
    uncovered: np.ndarray
    """Known free floor within reach that the footprint has not swept."""
    gain: np.ndarray
    """Uncovered cells the footprint would sweep from each cell."""
    useful: np.ndarray
    """Traversable cells worth driving to (gain >= _MIN_GAIN), blacklist removed."""
    rich: np.ndarray
    """Useful cells whose footprint would sweep at least half its area anew: where new lanes should start."""
    sweepable_area: float
    """Known floor the footprint can reach [m²]."""
    uncovered_area: float
    """Of which still unswept [m²]."""


class CoveragePlanner:
    """Chooses the next :class:`Target` from a costmap and the coverage memory."""

    def __init__(self, robot_radius: float, lane_width: float, frontier_min_size: float, min_move: float) -> None:
        self.radius = robot_radius
        self.lane_width = lane_width
        self.frontier_min_size = frontier_min_size
        self.min_move = min_move
        """Targets closer than this would count as reached at once; never choose them."""

    def assess(self, costmap: Costmap, swept: np.ndarray, blacklist: np.ndarray | None = None) -> CoverageState:
        res = costmap.resolution
        k = math.ceil(self.radius / res)
        offsets = np.arange(-k, k + 1)
        disc = (offsets[:, None] ** 2 + offsets[None, :] ** 2) * res * res <= self.radius**2
        traversable = costmap.traversable
        reach = ndimage.binary_dilation(traversable, structure=disc) if traversable.any() else traversable
        sweepable = reach & costmap.free
        uncovered = sweepable & ~swept
        gain = ndimage.convolve(uncovered.astype(np.int32), disc.astype(np.int32), mode="constant")
        useful = traversable & (gain >= _MIN_GAIN)
        if blacklist is not None:
            useful &= ~blacklist
        rich = useful & (gain >= _RICH_FRACTION * disc.sum())
        cell_area = res * res
        return CoverageState(costmap, uncovered, gain, useful, rich, float(sweepable.sum()) * cell_area,
                             float(uncovered.sum()) * cell_area)

    # ------------------------------------------------------------------ decisions

    def next_target(self, state: CoverageState, pose: Pose2D, direction: int, sweep: int,
                    blacklist: np.ndarray | None = None) -> tuple[Target | None, int]:
        """The next thing to do, and the sweep direction (+1 / -1 along y) to keep. ``None``: nothing left.

        Raises :class:`Stranded` if the robot is somewhere the planner cannot start from.
        """
        costmap = state.costmap
        start = costmap.nearest_traversable(pose.x, pose.y)
        if start is None:
            raise Stranded(f"no traversable cell within 0.5 m of ({pose.x:.2f}, {pose.y:.2f})")
        lane = self._lane(state, start, direction) if direction else None
        if lane is not None:
            return lane, sweep
        for side in (sweep, -sweep):
            shift = self._shift(state, start, side)
            if shift is not None:
                return shift, side
        distance, predecessors = dijkstra_field(costmap.cost, start, costmap.resolution)
        away = self._beyond(costmap, pose)  # anything nearer counts as reached at once
        # Big uncovered areas first (new lanes), the fringes left around furniture last.
        target = self._nearest(distance, state.rich & away)
        if target is None:
            target = self._nearest(distance, state.useful & away)
        if target is not None:
            row_useful = np.flatnonzero(state.useful[target[0]])
            heading = 1 if row_useful.size and row_useful.mean() >= target[1] else -1
            return Target(TargetKind.BACKTRACK, self._route(costmap, predecessors, target), heading), sweep
        frontier = self._frontier(costmap, distance, ~away if blacklist is None else blacklist | ~away)
        if frontier is not None:
            return Target(TargetKind.FRONTIER, self._route(costmap, predecessors, frontier)), sweep
        reachable = float(np.isfinite(distance).sum()) * costmap.resolution**2
        if reachable < _STRANDED_AREA and (state.useful & ~np.isfinite(distance)).any():
            raise Stranded(f"only {reachable:.2f} m² reachable from ({pose.x:.2f}, {pose.y:.2f})")
        return None, sweep

    def _lane(self, state: CoverageState, start: Cell, direction: int) -> Target | None:
        """The rest of the current lane: straight along x to the last useful cell before the free space ends."""
        costmap = state.costmap
        row, col = start
        cols = costmap.shape[1]
        traversable = costmap.traversable[row]
        end = col
        last_useful = -1
        c = col
        while 0 <= c + direction < cols and traversable[c + direction]:
            c += direction
            if state.useful[row, c]:
                last_useful = c
        end = last_useful
        if end < 0 or abs(end - col) * costmap.resolution < max(0.5 * self.lane_width, self.min_move):
            return None
        return Target(TargetKind.LANE, [costmap.centre(start), costmap.centre((row, end))], direction)

    def _shift(self, state: CoverageState, start: Cell, side: int) -> Target | None:
        """Step sideways by up to one lane width onto a row with useful cells right there."""
        costmap = state.costmap
        rows = costmap.shape[0]
        steps = max(1, round(self.lane_width / costmap.resolution))
        shortest = max(1, math.ceil(self.min_move / costmap.resolution))
        for offset in range(steps, shortest - 1, -1):
            row = start[0] + side * offset
            if not 0 <= row < rows:
                continue
            for dc in (0, -1, 1, -2, 2):
                cell = (row, start[1] + dc)
                if not costmap.inside(cell) or not state.useful[cell]:
                    continue
                if line_is_clear(costmap.cost, start, cell, _EDGE_COST * 4.0):
                    useful_cols = np.flatnonzero(state.useful[row])
                    heading = 1 if useful_cols.mean() >= cell[1] else -1
                    return Target(TargetKind.SHIFT, [costmap.centre(start), costmap.centre(cell)], heading)
            if offset <= steps // 2:
                break  # a much shorter shift would mostly re-sweep the current lane
        return None

    def _beyond(self, costmap: Costmap, pose: Pose2D) -> np.ndarray:
        rows, cols = costmap.shape
        ys = costmap.origin[1] + (np.arange(rows) + 0.5) * costmap.resolution
        xs = costmap.origin[0] + (np.arange(cols) + 0.5) * costmap.resolution
        return ((xs[None, :] - pose.x) ** 2 + (ys[:, None] - pose.y) ** 2) > self.min_move**2

    @staticmethod
    def _nearest(distance: np.ndarray, candidates: np.ndarray) -> Cell | None:
        reachable = candidates & np.isfinite(distance)
        if not reachable.any():
            return None
        flat = int(np.argmin(np.where(reachable, distance, np.inf)))
        return divmod(flat, distance.shape[1])

    def _frontier(self, costmap: Costmap, distance: np.ndarray, blacklist: np.ndarray | None) -> Cell | None:
        """Nearest traversable cell next to a big enough frontier (known free floor bordering unknown space).

        Only unknown space that reaches the edge of the map counts. The inside of
        a piece of furniture is unknown too (no beam ever gets in), but it is a
        sealed pocket: treating the floor around every box as a frontier sent the
        robot into tight gaps to look at things that cannot be seen.
        """
        pockets, _ = ndimage.label(costmap.unknown)
        edge = np.unique(np.concatenate((pockets[0], pockets[-1], pockets[:, 0], pockets[:, -1])))
        open_unknown = np.isin(pockets, edge[edge > 0])
        frontier = costmap.free & ndimage.binary_dilation(open_unknown, structure=np.ones((3, 3), dtype=bool))
        labels, count = ndimage.label(frontier, structure=np.ones((3, 3), dtype=bool))
        if not count:
            return None
        sizes = np.bincount(labels.ravel())
        big = sizes * costmap.resolution >= self.frontier_min_size
        big[0] = False
        cells = big[labels]
        # The robot's centre cannot stand on a frontier next to a wall; anywhere within its footprint will do.
        k = math.ceil(self.radius / costmap.resolution)
        near = ndimage.binary_dilation(cells, structure=np.ones((2 * k + 1, 2 * k + 1), dtype=bool))
        candidates = near & costmap.traversable
        if blacklist is not None:
            candidates &= ~blacklist
        return self._nearest(distance, candidates)

    @staticmethod
    def _route(costmap: Costmap, predecessors: np.ndarray, target: Cell) -> list[Point]:
        cells = prune_path(path_to(predecessors, target), costmap.cost, _EDGE_COST)
        return [costmap.centre(c) for c in cells]
