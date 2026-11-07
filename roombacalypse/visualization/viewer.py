"""The interactive pygame window: real-time loop, input handling, drawing.

The loop uses a fixed-timestep accumulator ("fix your timestep"): rendering
runs at the display rate while the simulation advances in exact ``dt`` steps,
``speed`` times faster than wall-clock time. When the machine cannot keep up,
the loop drops time instead of spiralling, and the panel shows the speed
actually achieved.
"""

from __future__ import annotations

import logging
import math
import os
from collections import deque
from datetime import datetime
from pathlib import Path

import pygame

from roombacalypse.autonomy.brain import Mode, RobotBrain
from roombacalypse.core.config import ViewerConfig
from roombacalypse.core.events import EventKind
from roombacalypse.core.geometry import Point, transform_points
from roombacalypse.environment.catalog import PLACEABLE
from roombacalypse.environment.shapes import Shape
from roombacalypse.environment.world import Obstacle
from roombacalypse.evaluation.coverage import FootprintCoverage
from roombacalypse.mapping.occupancy_grid import OccupancyGrid
from roombacalypse.simulation.engine import Simulation
from roombacalypse.visualization import hud
from roombacalypse.visualization.camera import Camera
from roombacalypse.visualization.renderer import Overlays, WorldRenderer
from roombacalypse.visualization.sound import CollisionSound

logger = logging.getLogger(__name__)

SPEED_STEPS = (0.125, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0)
TRAIL_SAMPLE_PERIOD = 0.1  # [s] of simulation time between trail points
TOAST_SECONDS = 3.5
MAP_REFRESH_SECONDS = 4.0  # [s] wall time between full map re-renders after loop closures (one takes ~1 s)

HELP_LINES: tuple[tuple[str, str], ...] = (
    ("Tab", "manual <-> autonomous"),
    ("Arrows / WASD", "drive (manual mode)"),
    ("Shift (held)", "boost to full wheel speed"),
    ("Space", "pause / resume"),
    ("N", "single step while paused"),
    ("+ / -", "faster / slower simulation"),
    ("R / Shift+R", "reset (same seed / next seed)"),
    ("L", "LiDAR rays"),
    ("G", "navigation: goal and DWA arcs"),
    ("E", "belief: odometry (purple) and SLAM (green)"),
    ("M", "the robot's map (occupancy grid)"),
    ("P", "pose graph: keyframes and loop closures"),
    ("T", "trail"),
    ("V", "collision sound on/off"),
    ("Left click", "place obstacle"),
    ("O", "cycle obstacle type"),
    ("X", "remove obstacle under cursor"),
    ("Wheel", "zoom at cursor"),
    ("Middle / Alt+left drag", "pan"),
    ("0", "fit room to window"),
    ("F12", "save screenshot to out/"),
    ("H / F1", "toggle this help"),
    ("Esc / Q", "quit"),
)


class Viewer:
    """Interactive front-end for a :class:`Simulation` and the robot's :class:`RobotBrain`."""

    def __init__(
        self,
        sim: Simulation,
        brain: RobotBrain,
        config: ViewerConfig,
        *,
        offscreen: bool = False,
    ) -> None:
        if offscreen:
            os.environ["SDL_VIDEODRIVER"] = "dummy"
            os.environ["SDL_AUDIODRIVER"] = "dummy"  # sounds "play" silently, and no device is needed
        pygame.init()
        pygame.display.set_caption("ROOMBACALYPSE")
        self.screen = pygame.display.set_mode((config.width, config.height))
        self.sim = sim
        self.brain = brain
        self.config = config
        self.read_keyboard = not offscreen
        self.world_rect = pygame.Rect(0, 0, config.width - config.panel_width, config.height)
        self.panel_rect = pygame.Rect(self.world_rect.right, 0, config.panel_width, config.height)
        self.camera = self._fit_camera()
        self.renderer = WorldRenderer(sim.spec)
        self.fonts = hud.Fonts()
        self.sound = CollisionSound(config.collision_sound, config.sound_volume, config.sound_cooldown)
        self._event_cursor = 0

        self.speed_index = min(range(len(SPEED_STEPS)), key=lambda i: abs(SPEED_STEPS[i] - config.speed_factor))
        self.paused = False
        self.running = True
        self.show_help = False
        self.show_trail = True
        self.show_lidar = True
        self.show_nav = True
        self.show_belief = False
        self.show_map = True
        self.show_graph = False
        self.template_index = 0
        self.trail: deque[Point] = deque(maxlen=max(2, int(config.trail_seconds / TRAIL_SAMPLE_PERIOD)))
        self.coverage = self._new_coverage()
        self._last_trail_time = -1.0
        self._accumulator = 0.0
        self._achieved_speed = 0.0
        self._lagging = False
        self._fps = 0.0
        self._clock_seconds = 0.0
        self._toasts: list[tuple[str, float]] = []
        self._panning = False
        self._step_once = False
        self._map_refreshed = -math.inf
        self.toast("Welcome. Tab hands control to the robot; H for help.")

    # ------------------------------------------------------------------ public API

    @property
    def speed(self) -> float:
        return SPEED_STEPS[self.speed_index]

    def run(self) -> None:
        """Run until the window is closed."""
        clock = pygame.time.Clock()
        while self.running:
            frame_seconds = clock.tick(self.config.fps) / 1000.0
            self._fps = clock.get_fps()
            self.frame(frame_seconds)
            pygame.display.flip()
        pygame.quit()

    def frame(self, frame_seconds: float) -> None:
        """Process one frame: input, simulation, drawing."""
        self._clock_seconds += frame_seconds
        self._handle_events()
        if self.read_keyboard:
            self._update_teleop()
        self._advance(frame_seconds)
        self._draw()

    def run_frames(self, count: int) -> None:
        """Run ``count`` frames with a fixed frame time (offscreen/scripted use)."""
        for _ in range(count):
            self.frame(1.0 / self.config.fps)

    def save_screenshot(self, path: Path) -> Path:
        path.parent.mkdir(parents=True, exist_ok=True)
        pygame.image.save(self.screen, str(path))
        logger.info("screenshot saved to %s", path)
        return path

    def toast(self, text: str) -> None:
        self._toasts.append((text, self._clock_seconds + TOAST_SECONDS))
        self._toasts = self._toasts[-5:]

    def close(self) -> None:
        pygame.quit()

    # ------------------------------------------------------------------ simulation

    def _advance(self, frame_seconds: float) -> None:
        sim = self.sim
        if self.paused:
            self._achieved_speed = 0.0
            if self._step_once:
                self._step()
                self._step_once = False
                self._listen()
            return
        self._accumulator += min(frame_seconds, 0.1) * self.speed
        steps = 0
        while self._accumulator >= sim.dt and steps < self.config.max_steps_per_frame:
            self._step()
            self._accumulator -= sim.dt
            steps += 1
        self._lagging = self._accumulator >= sim.dt
        if self._lagging:
            self._accumulator = 0.0  # cannot keep up: drop time rather than spiral
        if frame_seconds > 0.0:
            achieved = steps * sim.dt / frame_seconds
            self._achieved_speed += 0.1 * (achieved - self._achieved_speed)
        self._listen()

    def _listen(self) -> None:
        """Play the collision sound if the simulator reported a new contact since the last frame."""
        events, self._event_cursor = self.sim.events.since(self._event_cursor)
        if any(event.kind is EventKind.COLLISION for event in events):
            self.sound.play(self._clock_seconds)

    def _step(self) -> None:
        self.sim.step()
        pose = self.sim.robot.pose
        self.coverage.mark(pose.x, pose.y)
        if self.sim.time - self._last_trail_time >= TRAIL_SAMPLE_PERIOD:
            self.trail.append(pose.position)
            self._last_trail_time = self.sim.time

    def _new_coverage(self) -> FootprintCoverage:
        return FootprintCoverage(self.sim.world, self.sim.spec, self.sim.robot.pose.position)

    def _reset(self, seed: int | None = None) -> None:
        self.sim.reset(seed)
        self._event_cursor = 0
        self.trail.clear()
        self.coverage = self._new_coverage()
        self._last_trail_time = -1.0
        self._accumulator = 0.0
        self.toast(f"Reset: {self.sim.room.name}, seed {self.sim.seed}")

    # ------------------------------------------------------------------ input

    def _update_teleop(self) -> None:
        if self.brain.mode is not Mode.MANUAL:
            return
        keys = pygame.key.get_pressed()
        throttle = int(keys[pygame.K_UP] or keys[pygame.K_w]) - int(keys[pygame.K_DOWN] or keys[pygame.K_s])
        turn = int(keys[pygame.K_LEFT] or keys[pygame.K_a]) - int(keys[pygame.K_RIGHT] or keys[pygame.K_d])
        boost = bool(pygame.key.get_mods() & pygame.KMOD_SHIFT)
        self.brain.teleop.set_intent(throttle, turn, boost)

    def _handle_events(self) -> None:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                self._on_key(event.key, event.mod)
            elif event.type == pygame.MOUSEBUTTONDOWN:
                alt = bool(pygame.key.get_mods() & pygame.KMOD_ALT)
                if event.button == 2 or (event.button == 1 and alt):
                    self._panning = True
                elif event.button == 1 and self.world_rect.collidepoint(event.pos):
                    self._place_at(event.pos)
            elif event.type == pygame.MOUSEBUTTONUP and event.button in (1, 2):
                self._panning = False
            elif event.type == pygame.MOUSEMOTION and self._panning:
                self.camera.pan(*event.rel)
            elif event.type == pygame.MOUSEWHEEL:
                mouse = pygame.mouse.get_pos()
                if self.world_rect.collidepoint(mouse):
                    self.camera.zoom_at(1.15 ** event.y, *mouse)

    def _on_key(self, key: int, mods: int) -> None:
        shift = bool(mods & pygame.KMOD_SHIFT)
        if key in (pygame.K_ESCAPE, pygame.K_q):
            self.running = False
        elif key == pygame.K_TAB:
            mode = self.brain.toggle_mode()
            self.toast("Autonomous: the robot is driving." if mode is Mode.AUTONOMOUS else "Manual: you are driving.")
        elif key == pygame.K_SPACE:
            self.paused = not self.paused
            self.toast("Paused" if self.paused else "Resumed")
        elif key == pygame.K_n:
            if self.paused:
                self._step_once = True
        elif key in (pygame.K_EQUALS, pygame.K_PLUS, pygame.K_KP_PLUS, pygame.K_RIGHTBRACKET):
            self.speed_index = min(self.speed_index + 1, len(SPEED_STEPS) - 1)
            self.toast(f"Speed x{self.speed:g}")
        elif key in (pygame.K_MINUS, pygame.K_KP_MINUS, pygame.K_LEFTBRACKET):
            self.speed_index = max(self.speed_index - 1, 0)
            self.toast(f"Speed x{self.speed:g}")
        elif key == pygame.K_r:
            self._reset(self.sim.seed + 1 if shift else None)
        elif key == pygame.K_l:
            self.show_lidar = not self.show_lidar
        elif key == pygame.K_g:
            self.show_nav = not self.show_nav
        elif key == pygame.K_e:
            self.show_belief = not self.show_belief
            if self.show_belief:
                self.toast("Purple: dead reckoning. Green: SLAM.")
        elif key == pygame.K_t:
            self.show_trail = not self.show_trail
        elif key == pygame.K_v:
            if not self.sound.available:
                self.toast("No collision sound: start with --sound FILE (or --sound bonk).")
            else:
                self.toast("Collision sound on." if self.sound.toggle() else "Collision sound off.")
        elif key == pygame.K_m:
            self.show_map = not self.show_map
            if self.show_map:
                self.toast("Blue: the robot's own map, drawn where it really is.")
        elif key == pygame.K_p:
            self.show_graph = not self.show_graph
            if self.show_graph:
                self.toast("Green: keyframes. Yellow: loop closures.")
        elif key == pygame.K_o:
            self.template_index = (self.template_index + 1) % len(PLACEABLE)
            self.toast(f"Placing: {PLACEABLE[self.template_index].description}")
        elif key == pygame.K_x:
            self._remove_at(pygame.mouse.get_pos())
        elif key in (pygame.K_h, pygame.K_F1):
            self.show_help = not self.show_help
        elif key == pygame.K_0:
            self.camera = self._fit_camera()
        elif key == pygame.K_F12:
            stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
            path = self.save_screenshot(Path("out") / f"screenshot-{stamp}.png")
            self.toast(f"Saved {path}")

    def _place_at(self, screen_pos: tuple[int, int]) -> None:
        template = PLACEABLE[self.template_index]
        x, y = self.camera.to_world(*screen_pos)
        result = self.sim.place_obstacle(template, x, y)
        self.toast(f"Placed {template.label}" if result.ok else f"Can't place {template.label}: {result.reason}")

    def _remove_at(self, screen_pos: tuple[int, int]) -> None:
        if not self.world_rect.collidepoint(screen_pos):
            return
        removed = self.sim.remove_obstacle_at(*self.camera.to_world(*screen_pos))
        if removed:
            self.toast(f"Removed {removed[-1].name}")

    def _fit_camera(self) -> Camera:
        return Camera.fit(self.sim.world.bounds, (0, 0, self.world_rect.width, self.world_rect.height))

    # ------------------------------------------------------------------ drawing

    def _overlays(self) -> Overlays:
        layers = Overlays()
        if self.show_lidar:
            layers.scan, layers.scan_pose = self.sim.last_scan, self.sim.last_scan_pose
        autonomous = self.brain.mode is Mode.AUTONOMOUS
        if self.show_nav and autonomous:
            layers.plan = self.brain.wander.plan
            goal = self.brain.wander.goal
            if goal is not None:
                # The goal lives in the map frame. Draw it *relative to the robot*, like the DWA
                # arcs, so it shows where the robot is actually heading; belief errors show on the E layer.
                relative = self.brain.pose.inverse_transform_point(*goal)
                layers.goal = self.sim.robot.pose.transform_point(*relative)
        slam = self.brain.slam
        if self.show_belief:
            layers.belief = self.sim.boot_pose.compose(self.brain.odometry.pose)
            layers.slam_belief = self.sim.boot_pose.compose(self.brain.pose)
        if self.show_map:
            layers.grid, layers.map_pose = self._map_grid(), self.sim.boot_pose
        if self.show_graph and slam.keyframes:
            nodes = slam.node_poses()
            layers.graph_nodes = transform_points(self.sim.boot_pose, nodes[:, :2])
            layers.graph_loops = [(e.i, e.j) for e in slam.graph.edges() if e.kind == "loop"]
        return layers

    def _map_grid(self) -> OccupancyGrid:
        """The robot's grid; after a loop closure it is re-rendered at most every few seconds (it takes a while)."""
        slam = self.brain.slam
        if slam.grid_is_stale and self._clock_seconds - self._map_refreshed >= MAP_REFRESH_SECONDS:
            self._map_refreshed = self._clock_seconds
            return slam.grid
        return slam.grid_snapshot

    def _draw(self) -> None:
        mouse = pygame.mouse.get_pos()
        in_world = self.read_keyboard and self.world_rect.collidepoint(mouse) and pygame.mouse.get_focused()
        hovered: Obstacle | None = None
        hover: list[Obstacle] = []
        ghost: tuple[Shape, bool] | None = None
        if in_world and not self._panning:
            wx, wy = self.camera.to_world(*mouse)
            hits = self.sim.world.obstacles_at(wx, wy)
            if hits:
                hovered = hits[0]
                hover = self.sim.world.group_members(hovered)
            else:
                template = PLACEABLE[self.template_index]
                ghost = (template.make_shape(wx, wy), self.sim.can_place(template, wx, wy).ok)

        self.screen.fill((0, 0, 0))
        self.renderer.draw(
            self.screen, self.sim.world, self.sim.robot, self.camera,
            trail=[*self.trail, self.sim.robot.pose.position] if self.show_trail else (),
            hover=hover, ghost=ghost, overlays=self._overlays(),
        )
        toggles = (("lidar", self.show_lidar), ("nav", self.show_nav), ("belief", self.show_belief),
                   ("map", self.show_map), ("graph", self.show_graph))
        layers = " · ".join(name for name, on in toggles if on)
        ui = hud.UiState(
            speed=self.speed,
            achieved_speed=self._achieved_speed,
            paused=self.paused,
            lagging=self._lagging,
            fps=self._fps,
            placing=PLACEABLE[self.template_index].label,
            coverage=self.coverage.fraction,
            layers=layers or "none",
        )
        hud.draw_panel(self.screen, self.panel_rect, self.fonts, self.sim, self.brain, ui)
        self._toasts = [(text, until) for text, until in self._toasts if until > self._clock_seconds]
        hud.draw_toasts(self.screen, self.world_rect, self.fonts, [text for text, _ in self._toasts])
        if hovered is not None:
            traits = "movable" if hovered.movable else "fixed"
            hud.draw_tooltip(
                self.screen, mouse, self.fonts,
                f"{hovered.name} · {hovered.kind.value} · z {hovered.z_min:.2f}–{hovered.z_max:.2f} m · {traits}",
            )
        if self.show_help:
            hud.draw_help(self.screen, self.world_rect, self.fonts, HELP_LINES)
