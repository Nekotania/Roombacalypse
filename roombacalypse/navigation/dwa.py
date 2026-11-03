"""Dynamic Window Approach local planner (Fox, Burgard & Thrun, 1997).

Every control tick:

1. **Dynamic window.** Enumerate wheel-speed pairs the motors can reach
   within one control period from the current (encoder-measured) wheel
   speeds, given the per-wheel acceleration and speed limits. Sampling in
   wheel space makes the window exact for this drivetrain. Reversing is not
   considered (the robot has no rear bumper); turning on the spot is.
2. **Predict.** Each pair is simulated for ``horizon`` seconds in the robot
   frame: during the first control period the twist ramps linearly from the
   current one (as the drivetrain's acceleration limit makes it), then stays
   constant. Assuming the new speed is reached instantly would underestimate
   how far a braking robot travels and let it creep into the safety margin.
3. **Check.** Find how far along each arc the body stays outside the
   *hard* radius around every obstacle point. An arc is *admissible* if the
   robot could still brake to a stop within that free distance
   (v² / 2a ≤ free distance).
4. **Score** admissible arcs and pick the best. Progress towards the goal
   dominates; clearance rewards the *average* distance to obstacles along
   the arc (so moving away from a pillar beats lingering beside it); an
   intrusion penalty grows with how deep an arc dips into the safety margin;
   speed rewards moving at all; heading at the end of the arc mostly breaks
   ties, e.g. the direction to turn on the spot when the goal is behind.

The hard radius is the body plus ``safety_margin``. If the robot is already
inside it (it can slip a few millimetres in while passing an obstacle), the
hard radius relaxes to the body plus 1.5 cm so the robot can manoeuvre out;
if it is closer still (right after a bump), it may not get any closer at
all. A single all-or-nothing radius deadlocks: once inside, every forward
arc "approaches" something. See docs/algorithms/dwa.md.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.spatial import KDTree

from roombacalypse.core.config import DwaConfig, RobotSpec
from roombacalypse.core.geometry import Point, wrap_angles
from roombacalypse.core.kinematics import arc_displacement_array, wheel_speeds_to_twist
from roombacalypse.core.messages import STOP, DriveCommand

_MEANINGFUL_SPEED = 0.02  # [m/s]
_MEANINGFUL_FREE_DISTANCE = 0.05  # [m] creeping a centimetre towards a wall is not progress
_MIN_CLEARANCE = 0.015  # [m] hard limit beyond the body once inside the safety margin


@dataclass(frozen=True, eq=False)
class DwaPlan:
    """One planning cycle's outcome (also used for visualisation)."""

    command: DriveCommand
    trajectories: np.ndarray
    """Candidate arcs, shape ``(S, K + 1, 2)``, body frame at decision time (index 0 = start)."""
    admissible: np.ndarray
    """Shape ``(S,)``: can the robot brake in time on this arc?"""
    chosen: int
    """Index of the chosen arc, or -1 when nothing was admissible (command is STOP)."""
    free_distance: float
    """Free arc length ahead on the chosen arc [m] (``inf`` if clear for the whole horizon)."""
    blocked: bool
    """True if no admissible arc makes meaningful forward progress (only turning, stopping or creeping)."""


class DynamicWindowPlanner:
    """Samples, predicts, checks and scores arcs; returns the best :class:`DriveCommand`."""

    def __init__(self, spec: RobotSpec, config: DwaConfig, control_period: float) -> None:
        self.spec = spec
        self.config = config
        self.control_period = control_period
        steps = max(1, round(config.horizon / config.prediction_step))
        self.times = np.arange(1, steps + 1) * config.prediction_step
        self.collision_radius = spec.radius + config.safety_margin
        self.min_radius = spec.radius + min(_MIN_CLEARANCE, config.safety_margin)

    def window(self, wheel_speeds: tuple[float, float]) -> tuple[np.ndarray, np.ndarray]:
        """Candidate (v, ω) pairs reachable within one control period."""
        spec, cfg = self.spec, self.config
        reach = spec.max_wheel_accel * self.control_period
        limit = spec.max_wheel_speed
        axes = []
        for current in wheel_speeds:
            lo, hi = max(current - reach, -limit), min(current + reach, limit)
            if lo > hi:  # already beyond a limit: only the clamped speed is reachable
                lo = hi = min(max(current, -limit), limit)
            axes.append(np.linspace(lo, hi, cfg.wheel_samples))
        v_left, v_right = (a.ravel() for a in np.meshgrid(axes[0], axes[1], indexing="ij"))
        v = 0.5 * (v_left + v_right)
        w = (v_right - v_left) / spec.wheel_base
        violation = np.maximum.reduce(
            [-v, v - cfg.max_speed, np.abs(w) - cfg.max_angular_speed, np.zeros_like(v)]
        )
        keep = violation <= 1e-9
        if not keep.any():
            # Outside the allowed envelope (still reversing, or faster than the
            # cruise cap): take the reachable options closest to it, i.e. brake.
            keep = violation <= violation.min() + 1e-9
        return np.clip(v[keep], 0.0, cfg.max_speed), w[keep]

    def predict(
        self, v: np.ndarray, w: np.ndarray, v0: float = 0.0, w0: float = 0.0
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Positions and headings at the prediction times, each shape ``(S, K)``.

        The twist ramps linearly from (v0, w0) to (v, w) over the first control
        period (each ramp segment approximated by a constant-curvature arc,
        exact for straight lines), then holds.
        """
        t = self.times[None, :]
        period = self.control_period
        ramp_t = np.minimum(t, period)
        vv, ww = v[:, None], w[:, None]
        ramp_distance = v0 * ramp_t + (vv - v0) * ramp_t**2 / (2.0 * period)
        ramp_turn = w0 * ramp_t + (ww - w0) * ramp_t**2 / (2.0 * period)
        x1, y1 = arc_displacement_array(ramp_distance, ramp_turn)
        hold_t = np.maximum(t - period, 0.0)
        dx2, dy2 = arc_displacement_array(vv * hold_t, ww * hold_t)
        c, s = np.cos(ramp_turn), np.sin(ramp_turn)
        x = x1 + c * dx2 - s * dy2
        y = y1 + s * dx2 + c * dy2
        return x, y, ramp_turn + ww * hold_t

    def hard_radius(self, nearest_now: float) -> float:
        """Distance from the centre that arcs must not violate, given the current nearest obstacle."""
        if nearest_now >= self.collision_radius:
            return self.collision_radius
        if nearest_now >= self.min_radius:
            return self.min_radius  # slipped into the margin: room to manoeuvre out
        return max(nearest_now - 1e-6, 1e-6)  # closer still: never get any closer

    def clearance(
        self, x: np.ndarray, y: np.ndarray, obstacles: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """For each arc: the time it stays outside the hard radius (``inf`` for the whole horizon),
        its mean clearance from the body along the way (normalised to [0, 1]) and how deep it goes
        into the safety margin (0 = never, 1 = touching the body)."""
        count = x.shape[0]
        margin = self.config.safety_margin
        reach = margin + self.config.clearance_cap  # beyond this, distance stops mattering
        if obstacles.shape[0] == 0:
            return np.full(count, np.inf), np.ones(count), np.zeros(count)
        tree = KDTree(obstacles)
        nearest_now, _ = tree.query((0.0, 0.0))
        radius = self.hard_radius(float(nearest_now))
        distances, _ = tree.query(
            np.column_stack((x.ravel(), y.ravel())), distance_upper_bound=self.spec.radius + reach
        )
        distances = distances.reshape(x.shape)
        collide = distances < radius
        first = collide.argmax(axis=1)
        safe = np.where(first > 0, self.times[np.maximum(first - 1, 0)], 0.0)
        free_time = np.where(collide.any(axis=1), safe, np.inf)
        gap = np.clip(distances - self.spec.radius, 0.0, reach)  # inf (nothing near) -> reach
        intrusion = np.clip((margin - gap.min(axis=1)) / margin, 0.0, 1.0) if margin > 0 else np.zeros(count)
        return free_time, gap.mean(axis=1) / reach, intrusion

    def plan(self, wheel_speeds: tuple[float, float], goal: Point, obstacles: np.ndarray) -> DwaPlan:
        """Choose a command that heads for ``goal`` (body frame) while avoiding ``obstacles`` (body frame)."""
        cfg = self.config
        v, w = self.window(wheel_speeds)
        v0, w0 = wheel_speeds_to_twist(wheel_speeds[0], wheel_speeds[1], self.spec.wheel_base)
        x, y, theta = self.predict(v, w, v0, w0)
        free_time, clearance, intrusion = self.clearance(x, y, obstacles)
        clear = np.isinf(free_time)
        free_distance = np.where(clear, np.inf, v * np.where(clear, 0.0, free_time))
        admissible = v * v / (2.0 * self.spec.max_wheel_accel) <= free_distance + 1e-9

        gx, gy = goal
        end_x, end_y, end_theta = x[:, -1], y[:, -1], theta[:, -1]
        heading = 1.0 - np.abs(wrap_angles(np.arctan2(gy - end_y, gx - end_x) - end_theta)) / math.pi
        reach = max(cfg.max_speed * float(self.times[-1]), 1e-6)
        progress = np.clip((math.hypot(gx, gy) - np.hypot(gx - end_x, gy - end_y)) / reach, -1.0, 1.0)
        speed = v / cfg.max_speed
        score = (
            cfg.heading_weight * heading
            + cfg.progress_weight * progress
            + cfg.clearance_weight * clearance
            - cfg.intrusion_weight * intrusion
            + cfg.speed_weight * speed
        )
        score = np.where(admissible, score, -np.inf)

        start = np.zeros((v.shape[0], 1, 2))
        trajectories = np.concatenate((start, np.stack((x, y), axis=-1)), axis=1)
        progressing = admissible & (v > _MEANINGFUL_SPEED) & (free_distance > _MEANINGFUL_FREE_DISTANCE)
        blocked = not bool(np.any(progressing))
        if not np.isfinite(score).any():
            return DwaPlan(STOP, trajectories, admissible, -1, 0.0, True)
        best = int(np.argmax(score))
        command = DriveCommand(float(v[best]), float(w[best]))
        return DwaPlan(command, trajectories, admissible, best, float(free_distance[best]), blocked)
