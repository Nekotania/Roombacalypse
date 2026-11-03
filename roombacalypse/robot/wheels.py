"""How far the robot *really* moves for a given wheel rotation (ground truth only).

The drivetrain and the encoders work in "nominal metres": wheel rotation
multiplied by the *nominal* wheel radius from the spec sheet. This model
turns that into true ground travel, introducing the two classic sources of
odometry error (Borenstein & Feng's UMBmark):

* **systematic:** each wheel's true radius and the true wheel base differ
  from nominal by a small amount drawn once per reset (manufacturing
  tolerance). Unequal radii make a "straight" drive curve; a wrong wheel base
  makes every turn slightly too large or too small;
* **random:** a slowly varying slip per wheel, modelled as an
  Ornstein-Uhlenbeck process (think tiles versus rug). Independent per-step
  noise would average out within centimetres; correlated slip does not.

The robot's software never sees these values: it integrates its encoders
with the nominal geometry and has to live with the drift.
"""

from __future__ import annotations

import math

import numpy as np

from roombacalypse.core.config import DriveErrorConfig, RobotSpec

_MAX_SLIP = 0.5


class WheelGroundModel:
    """Calibration errors plus correlated slip for one robot instance."""

    def __init__(
        self,
        spec: RobotSpec,
        errors: DriveErrorConfig,
        calibration_rng: np.random.Generator,
        slip_rng: np.random.Generator,
    ) -> None:
        self.left_scale = 1.0 + float(calibration_rng.normal(0.0, errors.wheel_radius_std))
        """True left radius / nominal radius."""
        self.right_scale = 1.0 + float(calibration_rng.normal(0.0, errors.wheel_radius_std))
        """True right radius / nominal radius."""
        self.wheel_base = spec.wheel_base * (1.0 + float(calibration_rng.normal(0.0, errors.wheel_base_std)))
        """True distance between the wheel contact points [m]."""
        self._slip_std = errors.slip_std
        self._tau = errors.slip_time_constant
        self._rng = slip_rng
        self.slip_left = 0.0
        """Current relative slip of the left wheel (positive = ground travel exceeds rotation)."""
        self.slip_right = 0.0

    def ground_travel(self, d_left: float, d_right: float, dt: float) -> tuple[float, float]:
        """True distances each wheel moves over the floor for nominal rotations ``d_left``/``d_right``.

        Advances the slip process by ``dt``. Random numbers are drawn every
        call, moving or not, so the stream consumption never depends on the
        robot's motion and runs stay reproducible.
        """
        decay = math.exp(-dt / self._tau)
        kick = self._slip_std * math.sqrt(1.0 - decay * decay)  # exact OU discretisation
        z_left, z_right = self._rng.standard_normal(2)
        self.slip_left = max(-_MAX_SLIP, min(_MAX_SLIP, self.slip_left * decay + kick * float(z_left)))
        self.slip_right = max(-_MAX_SLIP, min(_MAX_SLIP, self.slip_right * decay + kick * float(z_right)))
        return (
            d_left * self.left_scale * (1.0 + self.slip_left),
            d_right * self.right_scale * (1.0 + self.slip_right),
        )
