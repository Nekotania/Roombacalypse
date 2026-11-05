"""Draws the world view: floor, walls, obstacles, dock, robot and trail.

Layering follows height, so the picture reads like a top-down camera:

1. floor and walls (cached; rebuilt only when the camera moves);
2. the trail and obstacles the robot can collide with;
3. the robot;
4. *overhead* obstacles (table tops, seats, the bed), drawn translucent so the
   robot stays visible when it drives underneath them.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass, field

import numpy as np
import pygame
from pygame import gfxdraw

from roombacalypse.core.config import RobotSpec
from roombacalypse.core.geometry import Point, Pose2D, cross, transform_points
from roombacalypse.core.messages import LidarScan
from roombacalypse.environment.shapes import Circle, Shape
from roombacalypse.environment.world import Obstacle, ObstacleKind, World
from roombacalypse.mapping.occupancy_grid import OccupancyGrid
from roombacalypse.navigation.dwa import DwaPlan
from roombacalypse.robot.body import RobotBody
from roombacalypse.visualization import palette
from roombacalypse.visualization.camera import Camera

WALL_THICKNESS = 0.08
"""Visual wall thickness [m], drawn outside the floor (physics walls are lines)."""

_BOLT = ((0.2, 1.0), (-0.55, -0.1), (-0.02, -0.1), (-0.25, -1.0), (0.55, 0.12), (0.02, 0.12))


@dataclass
class Overlays:
    """Optional sensor and belief layers, all drawn on top of the scene."""

    scan: LidarScan | None = None
    scan_pose: Pose2D | None = None
    """True pose at the moment ``scan`` was taken."""
    plan: DwaPlan | None = None
    """Latest DWA decision; its arcs are in the body frame and drawn from the current true pose."""
    goal: Point | None = None
    """Exploration goal in world coordinates (as the robot believes it)."""
    belief: Pose2D | None = None
    """Where the robot's odometry thinks it is, in world coordinates."""
    slam_belief: Pose2D | None = None
    """Where SLAM thinks the robot is, in world coordinates."""
    grid: OccupancyGrid | None = None
    """The robot's occupancy grid (map frame), drawn through ``map_pose``."""
    map_pose: Pose2D | None = None
    """Where the robot's map frame really is in the world (its boot pose): shows map errors as they are."""
    graph_nodes: np.ndarray | None = None
    """Keyframe positions ``(n, 2)`` in world coordinates."""
    graph_loops: list[tuple[int, int]] = field(default_factory=list)
    """Loop-closure constraints as pairs of keyframe indices."""


class WorldRenderer:
    """Renders a :class:`World` and a :class:`RobotBody` through a :class:`Camera`."""

    def __init__(self, spec: RobotSpec) -> None:
        self.spec = spec
        self._floor: pygame.Surface | None = None
        self._floor_key: object = None
        self._overlay: pygame.Surface | None = None
        self._map: tuple[pygame.Surface, tuple[float, float]] | None = None
        self._map_key: object = None

    def draw(
        self,
        surface: pygame.Surface,
        world: World,
        robot: RobotBody,
        camera: Camera,
        *,
        trail: Sequence[Point] = (),
        hover: Sequence[Obstacle] = (),
        ghost: tuple[Shape, bool] | None = None,
        overlays: Overlays | None = None,
    ) -> None:
        """Draw the whole world view into ``camera.viewport``."""
        surface.set_clip(pygame.Rect(camera.viewport))
        surface.blit(self._floor_layer(surface.get_size(), world, camera), (0, 0))
        if len(trail) >= 2:
            pygame.draw.lines(surface, palette.TRAIL, False, [camera.to_screen(x, y) for x, y in trail], 2)

        ground = [o for o in world.obstacles if o.z_min < self.spec.height]
        overhead = sorted((o for o in world.obstacles if o.z_min >= self.spec.height), key=lambda o: o.z_min)
        for obstacle in ground:
            self._draw_solid(surface, camera, obstacle)
        self._draw_robot(surface, camera, robot)

        overlay = self._overlay_layer(surface.get_size())
        for obstacle in overhead:
            alpha = 120 if obstacle.kind is ObstacleKind.SOFT else 75
            self._draw_translucent(overlay, camera, obstacle.shape, palette.KIND_COLORS[obstacle.kind], alpha)
        if ghost is not None:
            shape, ok = ghost
            self._draw_translucent(overlay, camera, shape, palette.GHOST_OK if ok else palette.GHOST_BAD, 110)
        if overlays is not None:
            self._draw_overlays(overlay, camera, robot.pose, overlays)
        surface.blit(overlay, (0, 0))

        for part in hover:
            self._outline(surface, camera, part.shape, palette.HOVER_OUTLINE, 2)
        surface.set_clip(None)

    # ------------------------------------------------------------------ static layer

    def _floor_layer(self, size: tuple[int, int], world: World, camera: Camera) -> pygame.Surface:
        key = (size, camera.key, world.outline)
        if self._floor is not None and self._floor_key == key:
            return self._floor
        floor = pygame.Surface(size)
        floor.fill(palette.BACKGROUND)
        outline = [camera.to_screen(x, y) for x, y in world.outline]
        pygame.draw.polygon(floor, palette.FLOOR, outline)

        # Half-metre grid, clipped to the (possibly concave) floor with an alpha mask.
        grid = pygame.Surface(size, pygame.SRCALPHA)
        b = world.bounds
        for i in range(math.floor(b.min_x * 2), math.ceil(b.max_x * 2) + 1):
            color = palette.FLOOR_GRID_MAJOR if i % 2 == 0 else palette.FLOOR_GRID_MINOR
            pygame.draw.line(grid, color, camera.to_screen(i / 2, b.min_y), camera.to_screen(i / 2, b.max_y))
        for j in range(math.floor(b.min_y * 2), math.ceil(b.max_y * 2) + 1):
            color = palette.FLOOR_GRID_MAJOR if j % 2 == 0 else palette.FLOOR_GRID_MINOR
            pygame.draw.line(grid, color, camera.to_screen(b.min_x, j / 2), camera.to_screen(b.max_x, j / 2))
        mask = pygame.Surface(size, pygame.SRCALPHA)
        pygame.draw.polygon(mask, (255, 255, 255, 255), outline)
        grid.blit(mask, (0, 0), special_flags=pygame.BLEND_RGBA_MULT)
        floor.blit(grid, (0, 0))

        self._draw_walls(floor, camera, world)
        self._floor, self._floor_key = floor, key
        return floor

    @staticmethod
    def _draw_walls(surface: pygame.Surface, camera: Camera, world: World) -> None:
        t = WALL_THICKNESS
        normals: list[Point] = []
        for ax, ay, bx, by in world.walls:
            length = math.hypot(bx - ax, by - ay)
            normals.append(((by - ay) / length, -(bx - ax) / length))  # outward for CCW outlines
        for (ax, ay, bx, by), (nx, ny) in zip(world.walls, normals, strict=True):
            quad = ((ax, ay), (bx, by), (bx + nx * t, by + ny * t), (ax + nx * t, ay + ny * t))
            pygame.draw.polygon(surface, palette.WALL, [camera.to_screen(*p) for p in quad])
        n = len(world.outline)
        for i in range(n):
            vx, vy = world.outline[i]
            px, py = world.outline[i - 1]
            qx, qy = world.outline[(i + 1) % n]
            if cross(vx - px, vy - py, qx - vx, qy - vy) > 0.0:  # convex corner: fill the outer gap
                (n1x, n1y), (n2x, n2y) = normals[i - 1], normals[i]
                corner = ((vx, vy), (vx + n1x * t, vy + n1y * t),
                          (vx + (n1x + n2x) * t, vy + (n1y + n2y) * t), (vx + n2x * t, vy + n2y * t))
                pygame.draw.polygon(surface, palette.WALL, [camera.to_screen(*p) for p in corner])
        for ax, ay, bx, by in world.walls:
            pygame.draw.line(surface, palette.WALL_FACE, camera.to_screen(ax, ay), camera.to_screen(bx, by), 2)

    def _overlay_layer(self, size: tuple[int, int]) -> pygame.Surface:
        if self._overlay is None or self._overlay.get_size() != size:
            self._overlay = pygame.Surface(size, pygame.SRCALPHA)
        self._overlay.fill((0, 0, 0, 0))
        return self._overlay

    # ------------------------------------------------------------------ obstacles

    def _draw_solid(self, surface: pygame.Surface, camera: Camera, obstacle: Obstacle) -> None:
        color = palette.KIND_COLORS[obstacle.kind]
        edge = palette.MOVABLE_OUTLINE if obstacle.movable else palette.darker(color)
        width = 2 if obstacle.movable else 1
        shape = obstacle.shape
        if isinstance(shape, Circle):
            cx, cy = camera.to_screen_int(shape.cx, shape.cy)
            r = max(1, round(camera.length(shape.radius)))
            gfxdraw.filled_circle(surface, cx, cy, r, color)
            gfxdraw.aacircle(surface, cx, cy, r, edge)
            if width > 1:
                pygame.draw.circle(surface, edge, (cx, cy), r, width)
        else:
            points = [camera.to_screen(x, y) for x, y in shape.vertices]
            pygame.draw.polygon(surface, color, points)
            pygame.draw.polygon(surface, edge, points, width)
        if obstacle.kind is ObstacleKind.DOCK:
            self._draw_bolt(surface, camera, shape.centroid)

    @staticmethod
    def _draw_bolt(surface: pygame.Surface, camera: Camera, center: Point) -> None:
        size = 0.045
        points = [camera.to_screen(center[0] + x * size, center[1] + y * size) for x, y in _BOLT]
        pygame.draw.polygon(surface, (255, 214, 90), points)

    @staticmethod
    def _draw_translucent(
        overlay: pygame.Surface, camera: Camera, shape: Shape, color: palette.Color, alpha: int
    ) -> None:
        fill = (*color, alpha)
        edge = (*palette.darker(color, 0.8), min(255, alpha * 3))
        if isinstance(shape, Circle):
            center = camera.to_screen(shape.cx, shape.cy)
            r = max(1.0, camera.length(shape.radius))
            pygame.draw.circle(overlay, fill, center, r)
            pygame.draw.circle(overlay, edge, center, r, 1)
        else:
            points = [camera.to_screen(x, y) for x, y in shape.vertices]
            pygame.draw.polygon(overlay, fill, points)
            pygame.draw.polygon(overlay, edge, points, 1)

    @staticmethod
    def _outline(surface: pygame.Surface, camera: Camera, shape: Shape, color: palette.Color, width: int) -> None:
        if isinstance(shape, Circle):
            r = max(2.0, camera.length(shape.radius))
            pygame.draw.circle(surface, color, camera.to_screen(shape.cx, shape.cy), r, width)
        else:
            pygame.draw.polygon(surface, color, [camera.to_screen(x, y) for x, y in shape.vertices], width)

    # ------------------------------------------------------------------ overlays

    def _draw_overlays(self, overlay: pygame.Surface, camera: Camera, pose: Pose2D, layers: Overlays) -> None:
        if layers.grid is not None and layers.map_pose is not None:
            self._draw_map(overlay, camera, layers.grid, layers.map_pose)
        if layers.graph_nodes is not None:
            self._draw_graph(overlay, camera, layers.graph_nodes, layers.graph_loops)
        if layers.plan is not None:
            self._draw_plan(overlay, camera, pose, layers.plan)
        if layers.goal is not None:
            self._draw_goal(overlay, camera, layers.goal)
        if layers.scan is not None and layers.scan_pose is not None:
            self._draw_scan(overlay, camera, layers.scan, layers.scan_pose)
        if layers.belief is not None:
            self._draw_belief(overlay, camera, pose, layers.belief, palette.BELIEF)
        if layers.slam_belief is not None:
            self._draw_belief(overlay, camera, pose, layers.slam_belief, palette.SLAM_BELIEF)

    def _draw_map(self, overlay: pygame.Surface, camera: Camera, grid: OccupancyGrid, map_pose: Pose2D) -> None:
        """The occupancy grid, cropped to the view, scaled and rotated into the world through ``map_pose``."""
        key = (id(grid), grid.version, camera.key, map_pose, overlay.get_size())
        if self._map_key != key:
            self._map = self._render_map(camera, grid, map_pose)
            self._map_key = key
        if self._map is not None:
            image, (x, y) = self._map
            overlay.blit(image, (x, y))

    @staticmethod
    def _render_map(camera: Camera, grid: OccupancyGrid, map_pose: Pose2D
                    ) -> tuple[pygame.Surface, tuple[float, float]] | None:
        rows, cols = grid.shape
        if rows == 0 or cols == 0:
            return None
        left, top, width, height = camera.viewport
        corners = [map_pose.inverse_transform_point(*camera.to_world(sx, sy))
                   for sx, sy in ((left, top), (left + width, top), (left, top + height), (left + width, top + height))]
        res, (ox, oy) = grid.resolution, grid.origin
        c0 = max(0, math.floor((min(c[0] for c in corners) - ox) / res) - 1)
        c1 = min(cols, math.ceil((max(c[0] for c in corners) - ox) / res) + 1)
        r0 = max(0, math.floor((min(c[1] for c in corners) - oy) / res) - 1)
        r1 = min(rows, math.ceil((max(c[1] for c in corners) - oy) / res) + 1)
        if c0 >= c1 or r0 >= r1:
            return None
        cells = grid.log_odds[r0:r1, c0:c1]
        rgba = np.zeros((*cells.shape, 4), dtype=np.uint8)
        rgba[cells < grid.config.free_log_odds] = palette.MAP_FREE
        rgba[cells > grid.config.occupied_log_odds] = palette.MAP_OCCUPIED
        image = pygame.image.frombuffer(np.ascontiguousarray(rgba[::-1]).tobytes(), (c1 - c0, r1 - r0), "RGBA")
        size = (max(1, round((c1 - c0) * res * camera.scale)), max(1, round((r1 - r0) * res * camera.scale)))
        image = pygame.transform.rotate(pygame.transform.scale(image, size), math.degrees(map_pose.theta))
        centre = camera.to_screen(*map_pose.transform_point(ox + 0.5 * (c0 + c1) * res, oy + 0.5 * (r0 + r1) * res))
        return image, (centre[0] - 0.5 * image.get_width(), centre[1] - 0.5 * image.get_height())

    @staticmethod
    def _draw_graph(overlay: pygame.Surface, camera: Camera, nodes: np.ndarray, loops: list[tuple[int, int]]) -> None:
        points = [camera.to_screen(float(x), float(y)) for x, y in nodes]
        if len(points) >= 2:
            pygame.draw.lines(overlay, palette.GRAPH_EDGE, False, points, 1)
        for i, j in loops:
            pygame.draw.line(overlay, palette.GRAPH_LOOP, points[i], points[j], 2)
        for p in points:
            pygame.draw.circle(overlay, palette.GRAPH_NODE, p, 2)

    @staticmethod
    def _draw_scan(overlay: pygame.Surface, camera: Camera, scan: LidarScan, pose: Pose2D) -> None:
        origin = camera.to_screen(*pose.transform_point(scan.origin_x, scan.origin_y))
        points = transform_points(pose, scan.points())
        screen = [camera.to_screen(float(x), float(y)) for x, y in points]
        for p in screen:
            pygame.draw.line(overlay, palette.LIDAR_RAY, origin, p, 1)
        for p in screen:
            pygame.draw.circle(overlay, palette.LIDAR_HIT, p, 2)

    @staticmethod
    def _draw_plan(overlay: pygame.Surface, camera: Camera, pose: Pose2D, plan: DwaPlan) -> None:
        for index, arc in enumerate(plan.trajectories):
            if index == plan.chosen:
                continue
            color = palette.DWA_ADMISSIBLE if plan.admissible[index] else palette.DWA_REJECTED
            points = [camera.to_screen(float(x), float(y)) for x, y in transform_points(pose, arc)]
            pygame.draw.lines(overlay, color, False, points, 1)
        if plan.chosen >= 0:
            arc = plan.trajectories[plan.chosen]
            points = [camera.to_screen(float(x), float(y)) for x, y in transform_points(pose, arc)]
            pygame.draw.lines(overlay, palette.DWA_CHOSEN, False, points, 3)

    @staticmethod
    def _draw_goal(overlay: pygame.Surface, camera: Camera, goal: Point) -> None:
        cx, cy = camera.to_screen(*goal)
        r = max(4.0, camera.length(0.08))
        color = (*palette.GOAL, 230)
        pygame.draw.circle(overlay, color, (cx, cy), r, 2)
        pygame.draw.line(overlay, color, (cx - 1.6 * r, cy), (cx + 1.6 * r, cy), 2)
        pygame.draw.line(overlay, color, (cx, cy - 1.6 * r), (cx, cy + 1.6 * r), 2)

    def _draw_belief(self, overlay: pygame.Surface, camera: Camera, truth: Pose2D, belief: Pose2D,
                     tint: palette.Color) -> None:
        color = (*tint, 220)
        centre = camera.to_screen(belief.x, belief.y)
        pygame.draw.line(overlay, (*tint, 140), camera.to_screen(truth.x, truth.y), centre, 1)
        pygame.draw.circle(overlay, color, centre, max(4.0, camera.length(self.spec.radius)), 2)
        tip = camera.to_screen(*belief.transform_point(self.spec.radius, 0.0))
        pygame.draw.line(overlay, color, centre, tip, 2)

    # ------------------------------------------------------------------ robot

    def _draw_robot(self, surface: pygame.Surface, camera: Camera, robot: RobotBody) -> None:
        spec = self.spec
        pose = robot.pose
        cx, cy = camera.to_screen_int(pose.x, pose.y)
        r = max(4, round(camera.length(spec.radius)))
        gfxdraw.filled_circle(surface, cx, cy, r, palette.ROBOT_BODY)
        gfxdraw.aacircle(surface, cx, cy, r, palette.ROBOT_RIM)
        gfxdraw.aacircle(surface, cx, cy, max(2, round(r * 0.8)), palette.ROBOT_INNER)
        for side in (1.0, -1.0):
            self._local_box(surface, camera, pose, 0.0, side * 0.5 * spec.wheel_base,
                            2.0 * spec.wheel_radius, 0.022, palette.ROBOT_WHEEL)

        tx, ty = camera.to_screen_int(*pose.transform_point(-0.02, 0.0))
        turret = max(2, round(camera.length(0.038)))
        gfxdraw.filled_circle(surface, tx, ty, turret, palette.ROBOT_TURRET)
        gfxdraw.aacircle(surface, tx, ty, turret, palette.ROBOT_RIM)

        front_hit = any(abs(c.bearing(pose)) <= 0.5 * math.pi + 1e-9 for c in robot.contacts)
        arc = [
            camera.to_screen(*pose.transform_point(spec.radius * math.cos(a), spec.radius * math.sin(a)))
            for a in (-0.5 * math.pi + math.pi * k / 18 for k in range(19))
        ]
        pygame.draw.lines(surface, palette.BUMPER_HIT if front_hit else palette.BUMPER, False, arc, max(2, r // 7))

        tip = pose.transform_point(0.92 * spec.radius, 0.0)
        left = pose.transform_point(0.52 * spec.radius, 0.22 * spec.radius)
        right = pose.transform_point(0.52 * spec.radius, -0.22 * spec.radius)
        pygame.draw.polygon(surface, palette.ROBOT_HEADING, [camera.to_screen(*p) for p in (tip, left, right)])

        for contact in robot.contacts:
            px, py = camera.to_screen_int(*contact.point)
            gfxdraw.filled_circle(surface, px, py, 3, palette.BUMPER_HIT)

    @staticmethod
    def _local_box(
        surface: pygame.Surface, camera: Camera, pose: Pose2D,
        lx: float, ly: float, length: float, width: float, color: palette.Color,
    ) -> None:
        hl, hw = 0.5 * length, 0.5 * width
        corners = ((lx - hl, ly - hw), (lx + hl, ly - hw), (lx + hl, ly + hw), (lx - hl, ly + hw))
        pygame.draw.polygon(surface, color, [camera.to_screen(*pose.transform_point(x, y)) for x, y in corners])
