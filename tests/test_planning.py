"""Costmap and shortest paths: inflation, optimality against Dijkstra, corner cutting, pruning."""

from __future__ import annotations

import math
from itertools import pairwise

import numpy as np
import pytest

from roombacalypse.core.config import MappingConfig, PlanningConfig
from roombacalypse.mapping.occupancy_grid import OccupancyGrid
from roombacalypse.planning.astar import astar, dijkstra_field, line_is_clear, octile, path_to, prune_path
from roombacalypse.planning.costmap import build_costmap

PLANNING = PlanningConfig()
INSCRIBED = 0.17 + 0.03 + PLANNING.extra_margin


def _random_cost(rng: np.random.Generator, shape: tuple[int, int]) -> np.ndarray:
    cost = rng.uniform(0.0, 3.0, shape)
    cost[rng.random(shape) < 0.25] = np.inf
    return cost


@pytest.mark.parametrize("seed", range(25))
def test_astar_is_optimal_against_dijkstra(seed: int) -> None:
    rng = np.random.default_rng(seed)
    cost = _random_cost(rng, (rng.integers(5, 30), rng.integers(5, 30)))
    free = np.argwhere(np.isfinite(cost))
    start, goal = (tuple(int(v) for v in free[i]) for i in rng.choice(len(free), 2, replace=False))
    distance, predecessors = dijkstra_field(cost, start, resolution=0.05)
    found = astar(cost, start, goal, resolution=0.05)
    if not math.isfinite(distance[goal]):
        assert found is None
        return
    assert found is not None
    path, total = found
    assert total == pytest.approx(distance[goal], rel=1e-9)
    assert path[0] == start and path[-1] == goal
    assert path_to(predecessors, goal)[-1] == goal and path_to(predecessors, goal)[0] == start
    for a, b in pairwise(path):  # a connected, legal path
        assert max(abs(a[0] - b[0]), abs(a[1] - b[1])) == 1
        assert math.isfinite(cost[b])


def test_astar_does_not_cut_corners() -> None:
    cost = np.zeros((3, 3))
    cost[0, 1] = cost[1, 0] = np.inf  # a diagonal gap between two blocked cells
    assert astar(cost, (1, 1), (0, 0)) is None
    cost[1, 0] = 0.0
    path, total = astar(cost, (1, 1), (0, 0))  # type: ignore[misc]
    assert path == [(1, 1), (1, 0), (0, 0)] and total == pytest.approx(2.0)


def test_astar_prefers_cheap_cells_and_the_heuristic_is_octile() -> None:
    cost = np.zeros((5, 9))
    cost[2, 1:8] = 5.0  # an expensive strip straight between start and goal
    path, total = astar(cost, (2, 0), (2, 8))  # type: ignore[misc]
    assert all(cost[c] == 0.0 for c in path)  # it goes round
    assert total < 8 * 6.0
    assert octile((0, 0), (3, 5)) == pytest.approx(5 + (math.sqrt(2) - 1) * 3)


def test_pruning_keeps_corners_and_respects_cost() -> None:
    cost = np.zeros((10, 10))
    cost[3:7, 5] = np.inf  # a wall
    path, _ = astar(cost, (5, 0), (5, 9))  # type: ignore[misc]
    pruned = prune_path(path, cost, max_cost=0.5)
    assert pruned[0] == path[0] and pruned[-1] == path[-1]
    assert len(pruned) < len(path) // 2
    for a, b in pairwise(pruned):
        assert line_is_clear(cost, a, b, 0.5)


def _room_grid() -> OccupancyGrid:
    """A 4 x 3 m room mapped by hand: walls on the border, a pillar, everything else free."""
    grid = OccupancyGrid(MappingConfig())
    grid.ensure_contains(0.0, 0.0, 4.0, 3.0)
    centres_x = grid.origin[0] + (np.arange(grid.shape[1]) + 0.5) * grid.resolution
    centres_y = grid.origin[1] + (np.arange(grid.shape[0]) + 0.5) * grid.resolution
    gx, gy = np.meshgrid(centres_x, centres_y)
    inside = (gx > 0.0) & (gx < 4.0) & (gy > 0.0) & (gy < 3.0)
    border = inside & ((gx < 0.05) | (gx > 3.95) | (gy < 0.05) | (gy > 2.95))
    pillar = np.hypot(gx - 2.0, gy - 1.5) < 0.1
    grid.log_odds[inside] = -2.0
    grid.log_odds[border | pillar] = 3.5
    return grid


def test_costmap_inflates_obstacles_by_the_inscribed_radius() -> None:
    costmap = build_costmap(_room_grid(), PLANNING, INSCRIBED)
    assert math.isinf(costmap.cost_at(2.0, 1.5 + 0.1 + INSCRIBED - 0.04))  # too close to the pillar
    assert math.isfinite(costmap.cost_at(2.0, 1.5 + 0.1 + INSCRIBED + 0.06))
    near = costmap.cost_at(1.0, 0.05 + INSCRIBED + 0.06)
    far = costmap.cost_at(1.0, 1.0)
    assert near > far >= 0.0  # cost falls with clearance
    assert math.isinf(costmap.cost_at(-10.0, 1.0))  # beyond the grid
    assert costmap.cost_at(-1.0, 1.0) == pytest.approx(PLANNING.unknown_cost, abs=0.01)  # unknown: passable, dear
    assert costmap.unknown.sum() > 0  # the margin around the room was never seen


def test_remembered_bumps_become_obstacles() -> None:
    grid = _room_grid()
    plain = build_costmap(grid, PLANNING, INSCRIBED)
    sock = np.array([[1.0, 2.0]])
    bumped = build_costmap(grid, PLANNING, INSCRIBED, extra_obstacles=sock)
    assert math.isfinite(plain.cost_at(1.0, 2.0)) and math.isinf(bumped.cost_at(1.0, 2.0))
    assert math.isinf(bumped.cost_at(1.0 + PLANNING.bump_radius + INSCRIBED - 0.03, 2.0))


def test_a_path_through_the_room_goes_round_the_pillar_with_clearance() -> None:
    costmap = build_costmap(_room_grid(), PLANNING, INSCRIBED)
    start, goal = costmap.cell(0.5, 1.5), costmap.cell(3.5, 1.5)
    path, _ = astar(costmap.cost, start, goal, costmap.resolution)  # type: ignore[misc]
    points = np.array([costmap.centre(c) for c in path])
    assert np.hypot(points[:, 0] - 2.0, points[:, 1] - 1.5).min() >= 0.1 + INSCRIBED - 1e-9
    assert costmap.nearest_traversable(0.12, 1.5) is not None  # from inside a lethal band, step out
