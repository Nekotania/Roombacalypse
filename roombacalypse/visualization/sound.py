"""Sound effects: a noise whenever the robot hits something (off by default).

The viewer plays one sound per new contact the *simulator* reports
(``COLLISION`` events), so it also sounds for things the bumper cannot feel,
such as reversing into a chair, and a cooldown keeps a scraping robot from
machine-gunning it.

``viewer.collision_sound`` (or ``--sound``) selects it:

* ``""``: silence (the default);
* ``"bonk"``: a cartoon bonk synthesised here, so nothing has to be downloaded;
* a path to any WAV or OGG file of your own. Keep personal sound files in
  ``sounds/``, which git ignores: clips from films and cartoons are usually
  copyrighted and must not be committed to the repository.

Without a working audio device (a server, a CI runner) the sound quietly
disables itself with one log line; the simulation never depends on it.
"""

from __future__ import annotations

import logging
import math
from pathlib import Path

import numpy as np
import pygame

logger = logging.getLogger(__name__)

BUILTIN = "bonk"


def synthesize_bonk(sample_rate: int, duration: float = 0.35) -> np.ndarray:
    """A cartoon "bonk": a tone sweeping down from about 900 Hz with a percussive envelope (mono int16)."""
    t = np.arange(int(duration * sample_rate)) / sample_rate
    frequency = 140.0 + 760.0 * np.exp(-t / 0.05)
    phase = 2.0 * math.pi * np.cumsum(frequency) / sample_rate
    tone = np.sin(phase) + 0.35 * np.sin(2.0 * phase) + 0.15 * np.sin(3.0 * phase)
    envelope = np.exp(-t / 0.09) * np.minimum(1.0, t / 0.003)
    wave = tone * envelope
    return np.round(0.8 * 32767.0 * wave / np.max(np.abs(wave))).astype(np.int16)


class CollisionSound:
    """Loads (or synthesises) the collision sound and plays it at most once per ``cooldown`` seconds."""

    def __init__(self, source: str, volume: float = 0.8, cooldown: float = 0.6) -> None:
        self.source = source
        self.cooldown = cooldown
        self.muted = False
        self.plays = 0
        self._sound: pygame.mixer.Sound | None = None
        self._last_play = -math.inf
        if source:
            try:
                self._sound = self._load(source)
                self._sound.set_volume(volume)
            except (pygame.error, OSError) as exc:
                logger.warning("collision sound %r disabled: %s", source, exc)
                self._sound = None

    @staticmethod
    def _load(source: str) -> pygame.mixer.Sound:
        if not pygame.mixer.get_init():
            pygame.mixer.init()
        if source == BUILTIN:
            mixer = pygame.mixer.get_init()
            if mixer is None:
                raise pygame.error("the audio mixer did not start")
            frequency, _, channels = mixer
            mono = synthesize_bonk(frequency)
            samples = mono if channels == 1 else np.repeat(mono[:, None], channels, axis=1)
            return pygame.sndarray.make_sound(np.ascontiguousarray(samples))
        path = Path(source).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"no such sound file: {path}")
        return pygame.mixer.Sound(str(path))

    @property
    def available(self) -> bool:
        """A sound is loaded and an audio device is open."""
        return self._sound is not None

    @property
    def active(self) -> bool:
        return self.available and not self.muted

    def toggle(self) -> bool:
        """Mute or unmute; returns whether the sound is now active."""
        self.muted = not self.muted
        return self.active

    def play(self, now: float) -> bool:
        """Play unless muted or still cooling down (``now`` in seconds, any clock); True if it played."""
        if not self.active or now - self._last_play < self.cooldown:
            return False
        assert self._sound is not None
        self._sound.play()
        self._last_play = now
        self.plays += 1
        return True
