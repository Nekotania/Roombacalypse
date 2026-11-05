"""Occupancy grid, likelihood field and pose graph: the map-side building blocks of SLAM."""

from __future__ import annotations

import math

import numpy as np
import pytest

from roombacalypse.core.config import MappingConfig
from roombacalypse.core.geometry import Pose2D, transform_points
from roombacalypse.core.rng import RngStreams
from roombacalypse.environment.rooms import build_world
from roombacalypse.mapping.likelihood_field import LikelihoodField
from roombacalypse.mapping.occupancy_grid import OccupancyGrid
from roombacalypse.mapping.pose_graph import PoseGraph, relative_errors
from roombacalypse.sensors.lidar import Lidar
from tests.helpers import IDEAL, box_room

CFG = MappingConfig()
RES = CFG.resolution


def _cell_value(grid: OccupancyGrid, x: float, y: float) -> float:
    r, c = grid.cells(np.array([x]), np.array([y]))
    return float(grid.log_odds[r[0], c[0]])


# --------------------------------------------------------------------------- occupancy grid


def test_one_beam_marks_free_cells_and_one_hit() -> None:
    grid = OccupancyGrid(CFG)
    grid.insert_rays((0.012, 0.012), np.array([[1.012, 0.012]]))  # along a row of cells
    for x in np.arange(0.012, 0.95, RES):
        assert _cell_value(grid, x, 0.012) == pytest.approx(CFG.miss_log_odds), x
    assert _cell_value(grid, 1.012, 0.012) == pytest.approx(CFG.hit_log_odds)
    assert _cell_value(grid, 1.2, 0.012) == 0.0  # beyond the return: unknown
    assert _cell_value(grid, 0.5, 0.3) == 0.0  # beside the beam: unknown
    assert grid.occupied_mask().sum() == 1


def test_a_cell_is_updated_once_per_scan_and_hits_win() -> None:
    grid = OccupancyGrid(CFG)
    angles = np.radians(np.linspace(-10, 10, 21))
    ends = np.column_stack((2.0 * np.cos(angles), 2.0 * np.sin(angles))) + 0.012
    grid.insert_rays((0.012, 0.012), ends)
    # Every beam crosses the sensor's own cell, which still gets a single miss.
    assert _cell_value(grid, 0.03, 0.03) == pytest.approx(CFG.miss_log_odds)
    # A beam that ends where another passes: that cell counts as hit, not free.
    grid2 = OccupancyGrid(CFG)
    grid2.insert_rays((0.012, 0.012), np.array([[1.012, 0.012], [0.512, 0.012]]))
    assert _cell_value(grid2, 0.512, 0.012) == pytest.approx(CFG.hit_log_odds)


def test_log_odds_are_clamped_so_moved_furniture_is_forgotten() -> None:
    grid = OccupancyGrid(CFG)
    origin, wall = (0.012, 0.012), np.array([[1.012, 0.012]])
    for _ in range(50):
        grid.insert_rays(origin, wall)
    assert _cell_value(grid, 1.012, 0.012) == pytest.approx(CFG.max_log_odds)
    # The "wall" was a box that got pushed away: beams now pass through to 2 m.
    scans = 0
    while grid.occupied_mask()[grid.cells(np.array([1.012]), np.array([0.012]))]:
        grid.insert_rays(origin, np.array([[2.012, 0.012]]))
        scans += 1
    assert scans <= math.ceil((CFG.max_log_odds - CFG.occupied_log_odds) / -CFG.miss_log_odds) + 1
    for _ in range(50):
        grid.insert_rays(origin, np.array([[2.012, 0.012]]))
    assert _cell_value(grid, 1.012, 0.012) == pytest.approx(CFG.min_log_odds)


def test_grid_grows_without_moving_what_it_knows() -> None:
    grid = OccupancyGrid(CFG)
    grid.insert_rays((0.012, 0.012), np.array([[1.012, 0.012]]))
    before = grid.log_odds.copy()
    known = grid.cell_centres(grid.known_mask())
    grid.insert_rays((0.012, 0.012), np.array([[-7.0, 9.0]]))  # far outside the current bounds
    min_x, _, _, max_y = grid.bounds
    assert min_x <= -7.0 and max_y > 9.0
    assert grid.origin[0] / RES == pytest.approx(round(grid.origin[0] / RES))  # still on the lattice
    r, c = grid.cells(known[:, 0], known[:, 1])
    assert np.count_nonzero(grid.log_odds[r, c]) == np.count_nonzero(before)


def test_a_perfect_scan_of_a_box_room_maps_its_walls() -> None:
    world = build_world(box_room(), IDEAL.robot, IDEAL.dock)
    lidar = Lidar(IDEAL.lidar, RngStreams(0).get("sensors.lidar"))
    pose = Pose2D(2.0, 1.5, 0.3)
    grid = OccupancyGrid(CFG)
    grid.insert_scan(pose, lidar.scan(world, pose, 0.0))
    assert grid.free_mask().sum() == 0  # one scan is not yet conclusive about free space (p = 0.40)
    for _ in range(2):
        grid.insert_scan(pose, lidar.scan(world, pose, 0.0))
    occupied = grid.cell_centres(grid.occupied_mask())
    # Every occupied cell is on a wall (within a cell), or on the dock at x <= 0.1.
    to_wall = np.minimum.reduce([np.abs(occupied[:, 0]), np.abs(occupied[:, 0] - 4.0),
                                 np.abs(occupied[:, 1]), np.abs(occupied[:, 1] - 3.0)])
    on_dock = (occupied[:, 0] <= 0.1 + RES) & (np.abs(occupied[:, 1] - 1.5) <= 0.16 + RES)
    assert np.all((to_wall <= RES) | on_dock)
    assert occupied.shape[0] > 200  # the whole perimeter, ~14 m at 5 cm
    free = grid.cell_centres(grid.free_mask())
    assert np.all((free[:, 0] > 0.0) & (free[:, 0] < 4.0) & (free[:, 1] > 0.0) & (free[:, 1] < 3.0))
    assert grid.known_area > 0.9 * 4.0 * 3.0


# --------------------------------------------------------------------------- likelihood field


def test_likelihood_field_coarse_values() -> None:
    field = LikelihoodField(np.array([[1.0125, 1.0125]]), 0.025, 0.05, margin=0.5)
    rows, cols = field.base_cells(np.array([1.0125, 1.0625, 1.0125, 3.0]), np.array([1.0125, 1.0125, 0.9125, 3.0]))
    value = field.lookup(rows, cols)
    assert value[0] == pytest.approx(1.0)
    assert value[1] == pytest.approx(math.exp(-0.5))  # two cells = one sigma away
    assert value[2] == pytest.approx(math.exp(-2.0))  # four cells = two sigmas
    assert value[3] == 0.0  # outside the grid


def test_surface_distances_are_exact_point_to_line_and_point_to_point() -> None:
    rng = np.random.default_rng(4)
    wall = np.column_stack((np.linspace(0.0, 3.0, 300), np.full(300, 1.0137)))  # a wall between grid lines
    angles = np.linspace(-1.2, 1.2, 8)
    leg = np.column_stack((1.5 - 0.015 * np.cos(angles), 2.0 - 0.015 * np.sin(angles)))  # a chair leg's near side
    field = LikelihoodField(np.vstack((wall, leg)), 0.025, 0.05, margin=0.5)
    xs = np.array([0.7, 2.2, 1.53])
    ys = np.array([1.0337, 0.9937, 2.04])
    element, inlier = field.correspond(xs, ys)
    residuals, gradients = field.distances(element, xs, ys)
    assert inlier.all()
    assert residuals[0] == pytest.approx([0.02, 0.0], abs=1e-6)  # 2 cm in front of the wall, along its normal
    assert residuals[1] == pytest.approx([-0.02, 0.0], abs=1e-6)  # signed: behind it
    assert abs(gradients[0, 0, 1]) == pytest.approx(1.0)  # the wall's normal is y
    centre = leg.mean(axis=0)
    assert residuals[2] == pytest.approx([xs[2] - centre[0], ys[2] - centre[1]], abs=1e-9)  # point-to-point vector
    # A single return is not a feature (it may be a sparse sample of a far wall): it constrains nothing.
    sparse = LikelihoodField(np.array([[1.5, 2.3]]), 0.025, 0.05, margin=0.5)
    assert not sparse.correspond(np.array([1.51]), np.array([2.3]))[1][0]
    _, far_inlier = field.correspond(np.array([1.5]), np.array([1.5]))  # 0.49 m from everything
    assert not far_inlier[0] and field.likelihood(np.array([1.5]), np.array([1.5]))[0] == 0.0
    noisy = wall + rng.normal(0.0, 0.01, wall.shape)  # a noisy wall still has a well-defined normal
    noisy_field = LikelihoodField(noisy, 0.025, 0.05, margin=0.5)
    element, _ = noisy_field.correspond(np.array([1.0]), np.array([1.05]))
    _, gradients = noisy_field.distances(element, np.array([1.0]), np.array([1.05]))
    assert abs(gradients[0, 0, 1]) > math.cos(math.radians(10))


def test_block_maximum_bounds_every_offset_in_its_block() -> None:
    rng = np.random.default_rng(2)
    field = LikelihoodField(rng.uniform(0, 2, (40, 2)), 0.025, 0.05, margin=0.2)
    for size in (2, 4, 8):
        bound = field.padded_block_maximum(size)
        rows, cols = field.shape
        r, c = rng.integers(-3, rows + 3, 300), rng.integers(-3, cols + 3, 300)
        top = field.lookup(r, c, bound, pad=size)
        for dr in range(size):
            for dc in range(size):
                assert np.all(top >= field.lookup(r + dr, c + dc) - 1e-12)


# --------------------------------------------------------------------------- pose graph


def test_pose_graph_jacobians_match_finite_differences() -> None:
    rng = np.random.default_rng(3)
    poses = rng.uniform(-2, 2, (2, 3))
    z = rng.uniform(-1, 1, (1, 3))
    i, j = np.array([0]), np.array([1])
    _, a, b = relative_errors(poses, i, j, z)
    h = 1e-7
    for node, jac in ((0, a), (1, b)):
        for k in range(3):
            plus, minus = poses.copy(), poses.copy()
            plus[node, k] += h
            minus[node, k] -= h
            numeric = (relative_errors(plus, i, j, z)[0] - relative_errors(minus, i, j, z)[0]) / (2 * h)
            assert jac[0, :, k] == pytest.approx(numeric[0], abs=1e-6)


def _square_loop(bias: float) -> tuple[PoseGraph, list[Pose2D]]:
    """A robot drives a 2 m square (4 legs of 4 steps) with a heading bias per step, then closes the loop."""
    truth = [Pose2D(0.0, 0.0, 0.0)]
    graph = PoseGraph()
    graph.add_node(truth[0])
    estimate = truth[0]
    info = np.diag([1e4, 1e4, 1e4])
    for k in range(16):
        turn = math.pi / 2 if k % 4 == 3 else 0.0
        step = Pose2D(0.5, 0.0, turn)
        truth.append(truth[-1].compose(step))
        measured = Pose2D(step.x, step.y, step.theta + bias)
        estimate = estimate.compose(measured)
        graph.add_node(estimate)
        graph.add_edge(k, k + 1, measured, info)
    return graph, truth


def test_loop_closure_pulls_a_drifted_square_back_into_shape() -> None:
    graph, truth = _square_loop(bias=math.radians(1.5))
    end_error = graph.pose(16).distance_to(truth[16])
    assert end_error > 0.15  # drifted
    graph.add_edge(0, 16, Pose2D(0.0, 0.0, 2 * math.pi), np.diag([1e4, 1e4, 1e4]), kind="loop")
    report = graph.optimize(iterations=20)
    assert report.final_cost < report.initial_cost
    assert graph.pose(0) == Pose2D(0.0, 0.0, 0.0)  # node 0 anchors the map frame
    errors = [graph.pose(k).distance_to(truth[k]) for k in range(17)]
    assert max(errors) < 0.25 * end_error
    assert graph.pose(16).distance_to(truth[16]) < 0.02


def test_cauchy_kernel_shrugs_off_a_wrong_loop_closure() -> None:
    def build() -> PoseGraph:
        graph, _ = _square_loop(bias=0.0)
        graph.add_edge(0, 16, Pose2D(0.0, 0.0, 0.0), np.diag([1e4, 1e4, 1e4]), robust=True, kind="loop")
        graph.add_edge(4, 12, Pose2D(1.5, 0.3, 0.0), np.diag([1e4, 1e4, 1e4]), robust=True, kind="loop")  # bogus
        return graph

    robust, plain = build(), build()
    robust.optimize(iterations=30, robust_scale=2.0)
    plain.optimize(iterations=30, robust_scale=1e6)  # a huge scale makes the kernel quadratic
    _, truth = _square_loop(bias=0.0)
    worst_robust = max(robust.pose(k).distance_to(truth[k]) for k in range(17))
    worst_plain = max(plain.pose(k).distance_to(truth[k]) for k in range(17))
    assert worst_plain > 0.3
    assert worst_robust < 0.05


def test_pose_graph_rejects_bad_edges() -> None:
    graph = PoseGraph()
    graph.add_node(Pose2D(0, 0, 0))
    graph.add_node(Pose2D(1, 0, 0))
    with pytest.raises(ValueError):
        graph.add_edge(0, 1, Pose2D(1, 0, 0), np.diag([1.0, 1.0, 0.0]))
    with pytest.raises(ValueError):
        graph.add_edge(0, 2, Pose2D(1, 0, 0), np.eye(3))
    graph.add_edge(0, 1, Pose2D(1, 0, 0), np.eye(3))
    assert graph.edge_count == 1 and graph.edges()[0].kind == "sequential"
    graph.clear()
    assert graph.node_count == 0 and graph.edge_count == 0


def test_transform_points_matches_compose() -> None:  # the SLAM front end relies on both agreeing
    pose = Pose2D(1.0, -2.0, 0.7)
    point = Pose2D(0.3, 0.4, 0.0)
    assert transform_points(pose, np.array([[0.3, 0.4]]))[0] == pytest.approx(pose.compose(point).position)
