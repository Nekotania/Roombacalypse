"""2D scanning LiDAR: exact ray casting plus a realistic error model.

The scan plane sits at ``mount_height``. Only obstacles whose vertical band
contains that height reflect beams, so the robot sees table legs but not
table tops, and sees neither socks below the plane nor the underside of a
bed above it. Walls reflect at every height.

Error model, per beam:

* Gaussian range noise with ``sigma = range_noise_std + range_noise_per_metre * range``;
* dropouts: with ``dropout_probability`` the beam returns nothing (``inf``);
* no return beyond ``max_range``.

Not modelled: motion distortion during the sweep (the scan is an instant
snapshot), beam divergence, mixed pixels at edges, surface reflectivity.
See docs/algorithms/lidar.md.
"""

from __future__ import annotations

import math

import numpy as np

from roombacalypse.core.config import LidarConfig
from roombacalypse.core.geometry import TAU, Pose2D
from roombacalypse.core.messages import LidarScan
from roombacalypse.environment.shapes import Circle
from roombacalypse.environment.world import World
from roombacalypse.sensors.raycast import cast_rays


def beam_angles(config: LidarConfig) -> np.ndarray:
    """Beam directions relative to the robot heading [rad], read-only.

    A full 360° scan spreads beams evenly (no duplicate beam at ±180°); a
    partial field of view includes both edges.
    """
    resolution = math.radians(config.angular_resolution_deg)
    if config.fov_deg >= 360.0 - 1e-9:
        count = max(1, round(TAU / resolution))
        angles = -math.pi + np.arange(count) * (TAU / count)
    else:
        fov = math.radians(config.fov_deg)
        count = math.floor(fov / resolution + 1e-9) + 1
        angles = -0.5 * fov + np.arange(count) * resolution
    angles.flags.writeable = False
    return angles


class Lidar:
    """Produces :class:`LidarScan` readings from the ground-truth world."""

    def __init__(self, config: LidarConfig, rng: np.random.Generator) -> None:
        self.config = config
        self.angles = beam_angles(config)
        self._cos = np.cos(self.angles)
        self._sin = np.sin(self.angles)
        self._rng = rng
        self._world: World | None = None
        self._version = -1
        self._seg_start = np.empty((0, 2))
        self._seg_delta = np.empty((0, 2))
        self._circle_centres = np.empty((0, 2))
        self._circle_radii = np.empty(0)

    @property
    def beam_count(self) -> int:
        return int(self.angles.shape[0])

    def _refresh(self, world: World) -> None:
        """Rebuild the visible geometry when the world changed (cached by ``World.version``)."""
        if world is self._world and world.version == self._version:
            return
        h = self.config.mount_height
        segments = list(world.walls)
        centres: list[tuple[float, float]] = []
        radii: list[float] = []
        for obstacle in world.obstacles_in_band(h, h + 1e-9):
            if isinstance(obstacle.shape, Circle):
                centres.append(obstacle.shape.center)
                radii.append(obstacle.shape.radius)
            else:
                segments.extend(obstacle.shape.edges())
        seg = np.asarray(segments, dtype=float).reshape(-1, 4)
        self._seg_start = seg[:, :2].copy()
        self._seg_delta = seg[:, 2:] - seg[:, :2]
        self._circle_centres = np.asarray(centres, dtype=float).reshape(-1, 2)
        self._circle_radii = np.asarray(radii, dtype=float)
        self._world = world
        self._version = world.version

    def true_ranges(self, world: World, pose: Pose2D) -> np.ndarray:
        """Noise-free distance along every beam (``inf`` where nothing is hit within any range)."""
        self._refresh(world)
        ox, oy = pose.transform_point(self.config.mount_x, 0.0)
        c, s = math.cos(pose.theta), math.sin(pose.theta)
        dx = c * self._cos - s * self._sin
        dy = s * self._cos + c * self._sin
        return cast_rays(ox, oy, dx, dy, self._seg_start, self._seg_delta, self._circle_centres, self._circle_radii)

    def scan(self, world: World, pose: Pose2D, time: float) -> LidarScan:
        """One noisy sweep taken from ``pose`` at simulation time ``time``."""
        cfg = self.config
        ranges = self.true_ranges(world, pose)
        # Draw a fixed number of random numbers per scan regardless of what is
        # hit, so the stream (and every later scan) is independent of the scene.
        noise = self._rng.standard_normal(ranges.shape[0])
        drop = self._rng.random(ranges.shape[0]) < cfg.dropout_probability
        finite = np.isfinite(ranges)
        sigma = cfg.range_noise_std + cfg.range_noise_per_metre * np.where(finite, ranges, 0.0)
        measured = np.where(finite, ranges + sigma * noise, np.inf)
        measured = np.maximum(measured, 0.0)
        measured[(measured > cfg.max_range) | drop] = np.inf
        measured.flags.writeable = False
        return LidarScan(time, self.angles, measured, cfg.max_range, cfg.mount_x, 0.0)
