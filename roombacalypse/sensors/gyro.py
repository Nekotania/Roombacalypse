"""MEMS yaw-rate gyroscope (truth side): the IMU every robot vacuum carries.

Each physics step the sensor measures the body's *true* yaw rate (after
collisions, so a robot pinned against a wall with spinning wheels reads no
rotation) and corrupts it the way a MEMS gyro does:

    measured = (1 + s) · ω_true + b(t) + n,      n ~ N(0, ARW² / dt)

* ``s``: scale-factor error, drawn once per power-up;
* ``b``: bias, drawn once per power-up (``bias_std``), then a random walk
  (``bias_walk``): the reason gyro headings drift slowly;
* ``n``: white rate noise with angle-random-walk density ``noise_density``;
* quantisation to ``resolution``.

The measured rate is integrated at the physics rate into a cumulative angle
(:class:`GyroReading`), as an IMU's on-board integrator does. Two random
numbers are drawn per step whatever happens, so the stream stays
comparable across scenes. Not modelled: temperature effects, g-sensitivity,
cross-axis coupling.
"""

from __future__ import annotations

import math

import numpy as np

from roombacalypse.core.config import GyroConfig
from roombacalypse.core.messages import GyroReading


class Gyro:
    """One gyro, with its own hidden bias and scale error."""

    def __init__(self, config: GyroConfig, rng: np.random.Generator) -> None:
        self.config = config
        self._rng = rng
        self.scale_error = float(rng.normal(0.0, config.scale_error_std))
        """Hidden: this unit's scale-factor error."""
        self.bias = float(rng.normal(0.0, math.radians(config.bias_std_deg)))
        """Hidden: this unit's current bias [rad/s]."""
        self.angle = 0.0
        self._noise_density = math.radians(config.noise_density_deg)
        self._bias_walk = math.radians(config.bias_walk_deg)
        self._resolution = math.radians(config.resolution_deg)

    def observe(self, true_rate: float, dt: float) -> None:
        """Advance by one physics step during which the body turned at ``true_rate`` [rad/s]."""
        noise, walk = self._rng.standard_normal(2)
        self.bias += self._bias_walk * math.sqrt(dt) * float(walk)
        rate = (1.0 + self.scale_error) * true_rate + self.bias + self._noise_density / math.sqrt(dt) * float(noise)
        if self._resolution > 0.0:
            rate = round(rate / self._resolution) * self._resolution
        self.angle += rate * dt

    def read(self) -> GyroReading:
        return GyroReading(self.angle)
