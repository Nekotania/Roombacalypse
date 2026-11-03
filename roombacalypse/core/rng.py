"""Reproducible, independent random streams derived from a single root seed."""

from __future__ import annotations

import zlib

import numpy as np


class RngStreams:
    """Factory for named random generators.

    Every noisy subsystem asks for its own stream, e.g.
    ``streams.get("sensors.lidar")``. A stream is seeded from the root seed
    *and* a CRC32 hash of its name (NumPy ``SeedSequence`` spawn keys), so:

    * the same root seed always reproduces the same numbers, and
    * adding or removing a subsystem never shifts the numbers another
      subsystem sees, which keeps regression scenarios stable as the
      project grows.
    """

    def __init__(self, seed: int) -> None:
        if seed < 0:
            raise ValueError("seed must be non-negative")
        self.seed = seed
        self._streams: dict[str, np.random.Generator] = {}

    def get(self, name: str) -> np.random.Generator:
        """Return the generator for ``name``, creating it on first use."""
        generator = self._streams.get(name)
        if generator is None:
            key = zlib.crc32(name.encode("utf-8"))
            sequence = np.random.SeedSequence(self.seed, spawn_key=(key,))
            generator = np.random.Generator(np.random.PCG64(sequence))
            self._streams[name] = generator
        return generator
