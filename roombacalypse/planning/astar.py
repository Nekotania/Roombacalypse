"""Shortest paths on a costmap: A*, Dijkstra fields, and path pruning (belief side).

The graph is the grid, 8-connected. Moving between neighbouring cells ``a``
and ``b`` costs their distance (1 or √2 cells, in metres) times
``1 + (cost[a] + cost[b]) / 2``; lethal cells (``inf``) cannot be entered,
and a diagonal step is allowed only if both cells it squeezes past are
passable (no cutting corners of obstacles).

* :func:`astar`: one start, one goal; the octile distance is an admissible
  heuristic because every step costs at least its length, so A* returns an
  optimal path (tested against Dijkstra on random grids).
* :func:`dijkstra_field`: one start, *every* cell, with SciPy's compiled
  Dijkstra (``scipy.sparse.csgraph``): the right tool for "which of these
  thousand candidate cells is nearest by path?", as coverage and frontier
  exploration ask.
* :func:`prune_path`: line-of-sight shortcuts through cheap cells, so the robot
  drives straight segments instead of a staircase of grid steps.
"""

from __future__ import annotations

import heapq
import math

import numpy as np
from scipy import sparse
from scipy.sparse import csgraph

Cell = tuple[int, int]

_SQRT2 = math.sqrt(2.0)
_NEIGHBOURS = ((-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
               (-1, -1, _SQRT2), (-1, 1, _SQRT2), (1, -1, _SQRT2), (1, 1, _SQRT2))


def octile(a: Cell, b: Cell) -> float:
    """Octile distance in cells: the length of the shortest 8-connected path on an empty grid."""
    dr, dc = abs(a[0] - b[0]), abs(a[1] - b[1])
    return max(dr, dc) + (_SQRT2 - 1.0) * min(dr, dc)


def astar(cost: np.ndarray, start: Cell, goal: Cell, resolution: float = 1.0) -> tuple[list[Cell], float] | None:
    """Optimal path from ``start`` to ``goal`` (both included) and its cost [m], or ``None`` if unreachable."""
    rows, cols = cost.shape
    if not (0 <= goal[0] < rows and 0 <= goal[1] < cols) or not math.isfinite(cost[goal]):
        return None
    if not (0 <= start[0] < rows and 0 <= start[1] < cols) or not math.isfinite(cost[start]):
        return None
    flat = cost.ravel()
    g = np.full(cost.size, np.inf)
    parent = np.full(cost.size, -1, dtype=np.int64)
    closed = np.zeros(cost.size, dtype=bool)
    s, t = start[0] * cols + start[1], goal[0] * cols + goal[1]
    g[s] = 0.0
    heap: list[tuple[float, float, int]] = [(octile(start, goal) * resolution, 0.0, s)]
    while heap:
        _, g_here, node = heapq.heappop(heap)
        if closed[node]:
            continue
        if node == t:
            break
        closed[node] = True
        r, c = divmod(node, cols)
        here = flat[node]
        for dr, dc, length in _NEIGHBOURS:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue
            nxt = nr * cols + nc
            there = flat[nxt]
            if closed[nxt] or not math.isfinite(there):
                continue
            if dr and dc and not (math.isfinite(flat[r * cols + nc]) and math.isfinite(flat[nr * cols + c])):
                continue  # would cut the corner of something lethal
            candidate = g_here + length * resolution * (1.0 + 0.5 * (here + there))
            if candidate < g[nxt]:
                g[nxt] = candidate
                parent[nxt] = node
                heapq.heappush(heap, (candidate + octile((nr, nc), goal) * resolution, candidate, nxt))
    if not math.isfinite(g[t]):
        return None
    path = [t]
    while path[-1] != s:
        path.append(int(parent[path[-1]]))
    return [divmod(n, cols) for n in reversed(path)], float(g[t])


def grid_graph(cost: np.ndarray, resolution: float = 1.0) -> sparse.csr_matrix:
    """The same graph :func:`astar` searches, as a sparse matrix (each undirected edge once)."""
    rows, cols = cost.shape
    passable = np.isfinite(cost)
    index = np.arange(cost.size).reshape(rows, cols)
    sources, targets, weights = [], [], []
    for dr, dc, length in ((0, 1, 1.0), (1, 0, 1.0), (1, 1, _SQRT2), (1, -1, _SQRT2)):
        r0, r1 = max(0, -dr), rows - max(0, dr)
        c0, c1 = max(0, -dc), cols - max(0, dc)
        a = (slice(r0, r1), slice(c0, c1))
        b = (slice(r0 + dr, r1 + dr), slice(c0 + dc, c1 + dc))
        ok = passable[a] & passable[b]
        if dr and dc:  # both cells squeezed past must be passable too
            ok &= passable[slice(r0 + dr, r1 + dr), slice(c0, c1)] & passable[slice(r0, r1), slice(c0 + dc, c1 + dc)]
        sources.append(index[a][ok])
        targets.append(index[b][ok])
        weights.append(length * resolution * (1.0 + 0.5 * (cost[a][ok] + cost[b][ok])))
    return sparse.csr_matrix(
        (np.concatenate(weights), (np.concatenate(sources), np.concatenate(targets))), shape=(cost.size, cost.size)
    )


def dijkstra_field(cost: np.ndarray, start: Cell, resolution: float = 1.0) -> tuple[np.ndarray, np.ndarray]:
    """Path cost from ``start`` to every cell (``inf`` if unreachable) and the predecessor of each, as grids."""
    rows, cols = cost.shape
    distance, predecessors = csgraph.dijkstra(
        grid_graph(cost, resolution), directed=False, indices=start[0] * cols + start[1], return_predecessors=True
    )
    return distance.reshape(rows, cols), predecessors.reshape(rows, cols)


def path_to(predecessors: np.ndarray, target: Cell) -> list[Cell]:
    """Follow a :func:`dijkstra_field` predecessor grid back from ``target`` to the start."""
    cols = predecessors.shape[1]
    node = target[0] * cols + target[1]
    path = [node]
    while predecessors.flat[node] >= 0:
        node = int(predecessors.flat[node])
        path.append(node)
    return [divmod(n, cols) for n in reversed(path)]


def line_is_clear(cost: np.ndarray, a: Cell, b: Cell, max_cost: float) -> bool:
    """True if every cell on the straight segment from ``a`` to ``b`` costs at most ``max_cost``."""
    steps = max(1, math.ceil(2.0 * max(abs(b[0] - a[0]), abs(b[1] - a[1]))))
    t = np.linspace(0.0, 1.0, steps + 1)
    rows = np.round(a[0] + t * (b[0] - a[0])).astype(np.intp)
    cols = np.round(a[1] + t * (b[1] - a[1])).astype(np.intp)
    return bool(np.all(cost[rows, cols] <= max_cost))


def prune_path(path: list[Cell], cost: np.ndarray, max_cost: float) -> list[Cell]:
    """Keep only the corners: greedily extend each straight segment while its line of sight stays cheap."""
    if len(path) <= 2:
        return list(path)
    pruned = [path[0]]
    anchor = 0
    while anchor < len(path) - 1:
        reach = anchor + 1
        while reach + 1 < len(path) and line_is_clear(cost, path[anchor], path[reach + 1], max_cost):
            reach += 1
        pruned.append(path[reach])
        anchor = reach
    return pruned
