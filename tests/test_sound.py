"""The optional collision sound: synthesis, loading, cooldown, and the viewer playing it on contact."""

from __future__ import annotations

import dataclasses
import logging
import os
from pathlib import Path

import numpy as np
import pygame
import pytest

from roombacalypse.app import build_parser, load_app_config
from roombacalypse.autonomy.brain import RobotBrain
from roombacalypse.core.config import AppConfig
from roombacalypse.core.events import EventLog
from roombacalypse.core.geometry import Pose2D
from roombacalypse.simulation.engine import Simulation
from roombacalypse.visualization.sound import BUILTIN, CollisionSound, synthesize_bonk
from roombacalypse.visualization.viewer import Viewer
from tests.helpers import IDEAL, box_room


@pytest.fixture(autouse=True)
def silent_audio(monkeypatch: pytest.MonkeyPatch) -> None:
    """A dummy audio device: sounds load and "play" without hardware."""
    monkeypatch.setitem(os.environ, "SDL_AUDIODRIVER", "dummy")
    if pygame.mixer.get_init():
        pygame.mixer.quit()


def test_the_builtin_bonk_is_a_short_decaying_sweep() -> None:
    wave = synthesize_bonk(22050)
    assert wave.dtype == np.int16 and wave.shape == (int(0.35 * 22050),)
    assert np.abs(wave).max() == pytest.approx(0.8 * 32767, abs=1)
    assert abs(int(wave[0])) < 1000  # a click-free start
    first, last = np.abs(wave[:2000]).max(), np.abs(wave[-2000:]).max()
    assert last < 0.1 * first  # it dies away


def test_no_sound_unless_asked_for() -> None:
    sound = CollisionSound("")
    assert not sound.available and not sound.play(0.0)


def test_builtin_sound_plays_with_a_cooldown_and_can_be_muted() -> None:
    sound = CollisionSound(BUILTIN, volume=0.5, cooldown=0.6)
    assert sound.available
    assert sound.play(10.0)
    assert not sound.play(10.3)  # still cooling down
    assert sound.play(10.7)
    assert not sound.toggle() and not sound.play(20.0)  # muted
    assert sound.toggle() and sound.play(21.0)
    assert sound.plays == 3


def test_your_own_file_is_loaded_and_a_missing_one_only_warns(tmp_path: Path, caplog: pytest.LogCaptureFixture
                                                               ) -> None:
    path = tmp_path / "scream.wav"
    _write_wav(path)
    assert CollisionSound(str(path)).available
    with caplog.at_level(logging.WARNING):
        missing = CollisionSound(str(tmp_path / "nope.wav"))
    assert not missing.available and "disabled" in caplog.text


def _write_wav(path: Path) -> None:
    import wave

    with wave.open(str(path), "wb") as handle:
        handle.setnchannels(1)
        handle.setsampwidth(2)
        handle.setframerate(22050)
        handle.writeframes(synthesize_bonk(22050).tobytes())


def test_the_viewer_plays_it_when_the_robot_hits_a_wall() -> None:
    config = dataclasses.replace(IDEAL, viewer=dataclasses.replace(IDEAL.viewer, collision_sound=BUILTIN))
    events = EventLog()
    brain = RobotBrain(config, publish=events.publisher("robot"))
    sim = Simulation(config, box_room(), brain, events=events)
    sim.robot.pose = Pose2D(3.5, 1.5, 0.0)  # 33 cm from the right-hand wall, facing it
    viewer = Viewer(sim, brain, config.viewer, offscreen=True)
    try:
        assert viewer.sound.available
        brain.teleop.set_intent(1.0, 0.0)  # a human drives it straight into the wall
        viewer.run_frames(120)
        assert any(c.obstacle_id is None for c in sim.robot.contacts)
        assert viewer.sound.plays == 1  # one contact, one sound (the cooldown swallows scraping)
    finally:
        viewer.close()


def test_the_command_line_sets_the_sound() -> None:
    args = build_parser().parse_args(["--sound", "sounds/scream.wav"])
    assert load_app_config(args).viewer.collision_sound == "sounds/scream.wav"
    assert AppConfig().viewer.collision_sound == ""  # off by default
