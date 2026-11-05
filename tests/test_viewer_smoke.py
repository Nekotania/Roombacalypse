"""Smoke tests of the pygame front-end and the CLI, rendered offscreen (SDL dummy video driver)."""

from __future__ import annotations

from pathlib import Path

import pygame
import pytest

from roombacalypse.app import main
from roombacalypse.autonomy.brain import Mode, RobotBrain
from roombacalypse.core.config import AppConfig
from roombacalypse.core.events import EventLog
from roombacalypse.core.paths import resolve_room_path
from roombacalypse.environment.rooms import load_room
from roombacalypse.simulation.engine import Simulation
from roombacalypse.visualization.viewer import Viewer


def _viewer(mode: Mode = Mode.MANUAL) -> tuple[Viewer, Simulation, RobotBrain]:
    config = AppConfig()
    events = EventLog()
    brain = RobotBrain(config, publish=events.publisher("robot"), mode=mode)
    sim = Simulation(config, load_room(resolve_room_path("studio")), brain, events=events)
    return Viewer(sim, brain, config.viewer, offscreen=True), sim, brain


def test_viewer_runs_drives_and_saves_screenshot(tmp_path: Path) -> None:
    viewer, sim, brain = _viewer()
    try:
        brain.teleop.set_intent(-1.0, 0.0)  # back out of the dock
        viewer.run_frames(60)  # one second of frames at x1 speed
        assert 1.0 - 2 * sim.dt <= sim.time <= 1.0 + 1e-9  # the accumulator may hold back one step
        assert sim.robot.odometer > 0.2
        viewer.show_help = True
        viewer.run_frames(1)
        path = viewer.save_screenshot(tmp_path / "frame.png")
        assert path.stat().st_size > 10_000
    finally:
        viewer.close()


def test_autonomous_rendering_with_all_layers(tmp_path: Path) -> None:
    viewer, sim, brain = _viewer(Mode.AUTONOMOUS)
    try:
        viewer.show_lidar = viewer.show_nav = viewer.show_belief = viewer.show_map = viewer.show_graph = True
        viewer.speed_index = 7  # x16
        viewer.run_frames(60)  # ~16 simulated seconds
        assert sim.time > 10.0
        assert sim.robot.odometer > 1.0  # it drove itself
        assert brain.wander.goals_chosen > 0 and sim.last_scan is not None
        assert viewer.coverage.fraction > 0.0
        assert len(brain.slam.keyframes) > 5 and brain.slam.grid.known_area > 5.0
        overlays = viewer._overlays()
        assert overlays.grid is not None and overlays.graph_nodes is not None and overlays.slam_belief is not None
        assert viewer.save_screenshot(tmp_path / "auto.png").stat().st_size > 10_000
    finally:
        viewer.close()


def _post(event_type: int, **attributes: object) -> None:
    pygame.event.post(pygame.event.Event(event_type, **attributes))


def test_interactive_controls(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """Drive the mouse/keyboard code paths that offscreen rendering normally skips."""
    monkeypatch.chdir(tmp_path)  # F12 writes screenshots into ./out
    viewer, sim, brain = _viewer()
    try:
        viewer.read_keyboard = True
        monkeypatch.setattr(pygame.mouse, "get_focused", lambda: True)

        # Hovering furniture: group highlight + tooltip.
        sofa = next(o for o in sim.world.obstacles if o.name == "sofa")
        sofa_px = viewer.camera.to_screen_int(*sofa.shape.centroid)
        monkeypatch.setattr(pygame.mouse, "get_pos", lambda: sofa_px)
        viewer.run_frames(1)

        # Hovering free floor: placement preview; click places, X removes.
        free_px = viewer.camera.to_screen_int(6.5, 2.4)
        monkeypatch.setattr(pygame.mouse, "get_pos", lambda: free_px)
        before = len(sim.world.obstacles)
        _post(pygame.MOUSEBUTTONDOWN, button=1, pos=free_px)
        viewer.run_frames(1)
        assert len(sim.world.obstacles) == before + 1
        _post(pygame.KEYDOWN, key=pygame.K_x, mod=0)
        viewer.run_frames(1)
        assert len(sim.world.obstacles) == before

        # Keyboard: pause + single step, speed up/down, cycle tool, layers, refit, screenshot.
        tick = sim.tick
        speed_index = viewer.speed_index
        layers = (viewer.show_lidar, viewer.show_nav, viewer.show_belief, viewer.show_map, viewer.show_graph)
        for key in (pygame.K_SPACE, pygame.K_n, pygame.K_EQUALS, pygame.K_MINUS, pygame.K_o, pygame.K_t,
                    pygame.K_h, pygame.K_0, pygame.K_l, pygame.K_g, pygame.K_e, pygame.K_m, pygame.K_p, pygame.K_F12):
            _post(pygame.KEYDOWN, key=key, mod=0)
        viewer.run_frames(1)
        assert viewer.paused and sim.tick == tick + 1
        assert viewer.speed_index == speed_index
        assert viewer.template_index == 1 and not viewer.show_trail and viewer.show_help
        assert (viewer.show_lidar, viewer.show_nav, viewer.show_belief, viewer.show_map, viewer.show_graph) == tuple(
            not v for v in layers
        )
        assert list((tmp_path / "out").glob("screenshot-*.png"))

        # Tab hands control to the robot and back; the robot's odometry survives the switch.
        _post(pygame.KEYDOWN, key=pygame.K_SPACE, mod=0)
        _post(pygame.KEYDOWN, key=pygame.K_TAB, mod=0)
        viewer.run_frames(30)
        assert brain.mode is Mode.AUTONOMOUS
        odometry_before = brain.odometry.distance
        _post(pygame.KEYDOWN, key=pygame.K_TAB, mod=0)
        viewer.run_frames(1)
        assert brain.mode is Mode.MANUAL and brain.odometry.distance >= odometry_before

        scale = viewer.camera.scale
        _post(pygame.MOUSEWHEEL, x=0, y=2)
        viewer.run_frames(1)
        assert viewer.camera.scale > scale

        seed = sim.seed
        _post(pygame.KEYDOWN, key=pygame.K_r, mod=pygame.KMOD_SHIFT)
        viewer.run_frames(1)
        assert sim.seed == seed + 1 and sim.tick <= 1 and viewer.coverage.fraction < 0.05

        _post(pygame.KEYDOWN, key=pygame.K_ESCAPE, mod=0)
        viewer.run_frames(1)
        assert not viewer.running
    finally:
        viewer.close()


def test_cli_screenshot_mode(tmp_path: Path) -> None:
    target = tmp_path / "cli.png"
    assert main(["--room", "test_chamber", "--mode", "auto", "--screenshot", str(target), "--frames", "3"]) == 0
    assert target.is_file()


def test_cli_headless_run(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["--headless", "--duration", "20", "--room", "test_chamber", "--seed", "3"]) == 0
    output = capsys.readouterr().out
    assert "coverage" in output and "penetration     none" in output


def test_cli_reports_bad_config() -> None:
    assert main(["--set", "robot.radius=-1", "--screenshot", "unused.png"]) == 2
    assert main(["--room", "no_such_room", "--screenshot", "unused.png"]) == 2
    assert main(["--headless", "--duration", "0"]) == 2
