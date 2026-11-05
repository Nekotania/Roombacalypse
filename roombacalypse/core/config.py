"""Typed configuration: frozen dataclasses with defaults, loaded from layered TOML.

Layering, later layers win:

1. defaults declared in this module (the single source of truth in code);
2. ``config/default.toml``, a commented copy of those defaults (a unit test
   keeps the two in sync);
3. an optional user file (``--config my.toml``);
4. command-line overrides (``--set robot.radius=0.2``).

Unknown keys and wrong types raise :class:`ConfigError`, so a typo fails
loudly instead of silently falling back to a default.

Units are SI (m, s, rad, W, J) unless a key name ends in ``_deg``.
"""

from __future__ import annotations

import dataclasses
import tomllib
import typing
from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


class ConfigError(ValueError):
    """Invalid configuration file or value."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ConfigError(message)


def is_multiple_of(value: float, base: float, tolerance: float = 1e-6) -> bool:
    """True if ``value`` is (numerically) an integer multiple of ``base``."""
    ratio = value / base
    return abs(ratio - round(ratio)) <= tolerance * max(1.0, abs(ratio))


# --------------------------------------------------------------------------- schema


@dataclass(frozen=True)
class SimulationConfig:
    """Global timing and randomness."""

    dt: float = 0.02
    """Physics timestep [s]."""
    seed: int = 42
    """Root seed for every random stream (see ``core/rng.py``)."""
    control_period: float = 0.1
    """Period of the robot software's control loop [s]; a multiple of ``dt``."""

    def __post_init__(self) -> None:
        _require(self.dt > 0.0, "simulation.dt must be positive")
        _require(self.seed >= 0, "simulation.seed must be non-negative")
        _require(
            self.control_period >= self.dt and is_multiple_of(self.control_period, self.dt),
            "simulation.control_period must be a positive multiple of simulation.dt",
        )


@dataclass(frozen=True)
class RobotSpec:
    """Nominal physical parameters of the robot: its spec sheet.

    Both the simulator and the robot's software read these. Milestone 2 adds
    *true* calibration errors that only the simulator knows about.
    """

    radius: float = 0.17
    """Body radius [m] (round body, Roomba-sized)."""
    height: float = 0.092
    """Body height [m]; the robot drives under anything higher than this."""
    wheel_base: float = 0.235
    """Distance between the two drive wheels' contact points [m]."""
    wheel_radius: float = 0.036
    """Drive wheel radius [m] (used by wheel encoders from milestone 2)."""
    max_wheel_speed: float = 0.5
    """Maximum wheel surface speed [m/s]."""
    max_wheel_accel: float = 1.0
    """Maximum wheel surface acceleration [m/s²]."""

    def __post_init__(self) -> None:
        for name in ("radius", "height", "wheel_base", "wheel_radius", "max_wheel_speed", "max_wheel_accel"):
            _require(getattr(self, name) > 0.0, f"robot.{name} must be positive")
        _require(self.wheel_base < 2.0 * self.radius, "robot.wheel_base must fit inside the body (< 2 * radius)")

    @property
    def max_angular_speed(self) -> float:
        """Yaw rate when spinning in place at full wheel speed [rad/s]."""
        return 2.0 * self.max_wheel_speed / self.wheel_base


@dataclass(frozen=True)
class DriveErrorConfig:
    """Ground-truth imperfections of the drive. Only the simulator knows the drawn values.

    They make wheel odometry wrong in the two classic ways (Borenstein & Feng,
    UMBmark): *systematic* errors from unequal wheel radii and an uncertain
    wheel base, and *random* errors from slip. All zeros gives ideal wheels.
    """

    wheel_radius_std: float = 0.002
    """Relative std of each wheel's true radius (manufacturing tolerance), drawn per reset."""
    wheel_base_std: float = 0.01
    """Relative std of the true wheel base, drawn per reset."""
    slip_std: float = 0.01
    """Stationary relative std of the slowly varying per-wheel slip."""
    slip_time_constant: float = 0.5
    """Correlation time of the slip (an Ornstein-Uhlenbeck process) [s]."""

    def __post_init__(self) -> None:
        for name in ("wheel_radius_std", "wheel_base_std", "slip_std"):
            _require(0.0 <= getattr(self, name) < 0.2, f"drive_errors.{name} must be in [0, 0.2)")
        _require(self.slip_time_constant > 0.0, "drive_errors.slip_time_constant must be positive")


@dataclass(frozen=True)
class LidarConfig:
    """2D scanning LiDAR on top of the robot (an LDS-style turret)."""

    mount_height: float = 0.095
    """Height of the scan plane [m]. Obstacles whose band does not contain it are invisible."""
    mount_x: float = -0.02
    """Turret position along the body x axis [m]."""
    angular_resolution_deg: float = 1.0
    """Angle between beams [deg]."""
    fov_deg: float = 360.0
    """Field of view [deg]; 360 means a full rotation."""
    max_range: float = 6.0
    """Beyond this distance there is no return [m]."""
    range_noise_std: float = 0.008
    """Constant part of the Gaussian range noise [m]."""
    range_noise_per_metre: float = 0.004
    """Range-proportional part of the noise: sigma = std + per_metre * range."""
    dropout_probability: float = 0.01
    """Chance that a beam returns nothing."""
    scan_period: float = 0.2
    """Time per scan [s] (5 Hz); a multiple of simulation.dt."""

    def __post_init__(self) -> None:
        _require(self.mount_height > 0.0, "lidar.mount_height must be positive")
        _require(0.05 <= self.angular_resolution_deg <= 10.0, "lidar.angular_resolution_deg must be in [0.05, 10]")
        _require(0.0 < self.fov_deg <= 360.0, "lidar.fov_deg must be in (0, 360]")
        _require(self.max_range > 0.0, "lidar.max_range must be positive")
        _require(self.range_noise_std >= 0.0 and self.range_noise_per_metre >= 0.0, "lidar noise must be >= 0")
        _require(0.0 <= self.dropout_probability < 1.0, "lidar.dropout_probability must be in [0, 1)")
        _require(self.scan_period > 0.0, "lidar.scan_period must be positive")


@dataclass(frozen=True)
class BumperConfig:
    """Front bumper with a left and a right switch, like a Roomba."""

    arc_deg: float = 180.0
    """Front arc covered by the bumper, centred on the heading [deg]."""
    center_deadband_deg: float = 15.0
    """Hits within +-this of straight ahead close both switches [deg]."""

    def __post_init__(self) -> None:
        _require(0.0 < self.arc_deg <= 360.0, "bumper.arc_deg must be in (0, 360]")
        _require(0.0 <= self.center_deadband_deg < 0.5 * self.arc_deg, "bumper.center_deadband_deg out of range")


@dataclass(frozen=True)
class EncoderConfig:
    """Wheel encoders."""

    ticks_per_revolution: float = 508.8
    """Encoder counts per wheel revolution (the iRobot Create 2 figure)."""

    def __post_init__(self) -> None:
        _require(self.ticks_per_revolution > 0.0, "encoders.ticks_per_revolution must be positive")


@dataclass(frozen=True)
class DwaConfig:
    """Dynamic Window Approach local planner (see docs/algorithms/dwa.md)."""

    max_speed: float = 0.3
    """Cruise speed cap [m/s] (robot vacuums clean at about 0.3 m/s)."""
    max_angular_speed: float = 2.0
    """Yaw rate cap [rad/s]."""
    horizon: float = 1.5
    """How far ahead each candidate arc is simulated [s]."""
    prediction_step: float = 0.1
    """Time between predicted points on an arc [s]."""
    wheel_samples: int = 9
    """Samples per wheel across the dynamic window (wheel_samples² candidates)."""
    safety_margin: float = 0.03
    """Clearance required beyond the body radius [m]."""
    clearance_cap: float = 0.3
    """Closest approach to obstacles (beyond the safety margin) above which an arc scores no better [m]."""
    heading_weight: float = 0.2
    """Weight of facing the goal at the end of the arc (mostly a tie-breaker)."""
    progress_weight: float = 1.0
    """Weight of getting closer to the goal over the horizon."""
    clearance_weight: float = 0.4
    """Weight of keeping away from obstacles (mean distance along the arc)."""
    intrusion_weight: float = 1.0
    """Penalty for entering the safety margin, scaled by how deep the arc goes into it."""
    speed_weight: float = 0.2
    """Weight of moving at all."""
    obstacle_range: float = 2.5
    """Obstacle points farther than this are ignored [m]."""

    def __post_init__(self) -> None:
        _require(self.max_speed > 0.0 and self.max_angular_speed > 0.0, "dwa speed caps must be positive")
        _require(0.0 < self.prediction_step <= self.horizon, "dwa needs 0 < prediction_step <= horizon")
        _require(self.wheel_samples >= 3 and self.wheel_samples % 2 == 1, "dwa.wheel_samples must be odd and >= 3")
        _require(self.safety_margin >= 0.0 and self.clearance_cap > 0.0, "dwa margins must be positive")
        for name in ("heading_weight", "progress_weight", "clearance_weight", "intrusion_weight", "speed_weight"):
            _require(getattr(self, name) >= 0.0, f"dwa.{name} must be >= 0")
        _require(self.obstacle_range > 0.0, "dwa.obstacle_range must be positive")


@dataclass(frozen=True)
class WanderConfig:
    """Milestone-2 autonomous behaviour: explore without a map (see docs/algorithms/dwa.md)."""

    goal_min_distance: float = 0.6
    """Shortest exploration goal worth driving to [m]."""
    goal_max_distance: float = 2.5
    """Longest exploration goal [m]."""
    goal_reached_tolerance: float = 0.25
    """Distance at which a goal counts as reached [m]."""
    goal_timeout: float = 15.0
    """Give up on a goal after this long [s]."""
    stall_time: float = 4.0
    """Give up on a goal if the distance to it has not shrunk for this long [s]."""
    candidate_step_deg: float = 15.0
    """Angular spacing of candidate exploration directions [deg]."""
    visit_cell_size: float = 0.5
    """Cell size of the dead-reckoned visit counter used to prefer new areas [m]."""
    backup_distance: float = 0.12
    """How far to reverse after a bump [m]."""
    backup_speed: float = 0.15
    """Reversing speed [m/s]."""
    turn_min_deg: float = 60.0
    """Smallest escape turn after a bump [deg]."""
    turn_max_deg: float = 150.0
    """Largest escape turn after a bump [deg]."""
    turn_speed: float = 1.5
    """Escape turn rate [rad/s]."""
    bump_memory_seconds: float = 20.0
    """How long a bump location is remembered as an obstacle [s]."""

    def __post_init__(self) -> None:
        _require(0.0 < self.goal_min_distance < self.goal_max_distance, "wander goal distances out of order")
        _require(self.goal_reached_tolerance > 0.0, "wander.goal_reached_tolerance must be positive")
        _require(self.goal_timeout > 0.0 and self.stall_time > 0.0, "wander timeouts must be positive")
        _require(0.0 < self.candidate_step_deg <= 90.0, "wander.candidate_step_deg must be in (0, 90]")
        _require(self.visit_cell_size > 0.0, "wander.visit_cell_size must be positive")
        _require(self.backup_distance >= 0.0 and self.backup_speed > 0.0, "wander backup values out of range")
        _require(0.0 < self.turn_min_deg <= self.turn_max_deg < 180.0, "wander turn angles must be in (0, 180)")
        _require(self.turn_speed > 0.0 and self.bump_memory_seconds >= 0.0, "wander turn/memory values out of range")


@dataclass(frozen=True)
class MappingConfig:
    """Log-odds occupancy grid the robot builds from its scans (see docs/algorithms/slam.md)."""

    resolution: float = 0.05
    """Cell size [m]."""
    hit_log_odds: float = 0.85
    """Added to a cell a beam ended in (p = 0.70 for a single hit)."""
    miss_log_odds: float = -0.4
    """Added to a cell a beam passed through (p = 0.40 for a single miss)."""
    min_log_odds: float = -2.0
    """Clamp: a cell never becomes more certainly free than this, so moved furniture can reappear."""
    max_log_odds: float = 3.5
    """Clamp: a cell never becomes more certainly occupied than this, so moved furniture can be forgotten."""
    occupied_log_odds: float = 0.6
    """Cells above this count as occupied (p > 0.65)."""
    free_log_odds: float = -0.6
    """Cells below this count as free (p < 0.35)."""

    def __post_init__(self) -> None:
        _require(0.005 <= self.resolution <= 0.5, "mapping.resolution must be in [0.005, 0.5]")
        _require(self.hit_log_odds > 0.0 > self.miss_log_odds, "mapping needs hit_log_odds > 0 > miss_log_odds")
        _require(self.min_log_odds < 0.0 < self.max_log_odds, "mapping needs min_log_odds < 0 < max_log_odds")
        _require(
            self.min_log_odds < self.free_log_odds < 0.0 < self.occupied_log_odds < self.max_log_odds,
            "mapping needs min_log_odds < free_log_odds < 0 < occupied_log_odds < max_log_odds",
        )


@dataclass(frozen=True)
class SlamConfig:
    """Graph SLAM: scan matching, keyframes, loop closure, pose-graph optimisation (docs/algorithms/slam.md).

    Odometry noise figures are the robot's *own* model of its wheels, the kind
    of number a robot is tuned with; they are not the simulator's hidden errors.
    """

    node_distance: float = 0.25
    """A new keyframe is added after this much travel [m]..."""
    node_angle_deg: float = 15.0
    """...or this much rotation [deg]."""
    local_map_nodes: int = 20
    """Recent keyframes whose scans form the local map that new scans are matched against."""
    scan_voxel: float = 0.04
    """Scans are thinned to one point per voxel of this size before matching [m]."""
    field_resolution: float = 0.025
    """Cell size of the likelihood fields used for scan matching [m]."""
    field_sigma: float = 0.05
    """Width of the likelihood field around mapped points [m]."""
    track_min_score: float = 0.5
    """Mean likelihood a tracking match needs; below it the robot falls back to a wider search."""
    track_window: float = 0.25
    """Half-width of the fallback search window, translation [m]."""
    track_window_deg: float = 8.0
    """Half-width of the fallback search window, rotation [deg]."""
    prior_translation_weight: float = 0.5
    """Pull of the odometry prediction on the match [1/m²]; matters only where the scan is ambiguous (corridors)."""
    prior_rotation_weight: float = 0.5
    """Pull of the odometry prediction on the match heading [1/rad²]."""
    match_point_std: float = 0.01
    """How far the points of an aligned scan typically lie from the mapped surfaces [m]; the information of a
    scan-matching constraint is the points' normal structure divided by this squared."""
    odometry_translation_noise: float = 0.03
    """Odometry translation std per metre travelled [m/m]."""
    odometry_rotation_noise: float = 0.03
    """Odometry heading std per radian turned [rad/rad]."""
    odometry_heading_drift: float = 0.015
    """Odometry heading std per metre travelled (unequal wheels) [rad/m]."""
    loop_search_radius: float = 2.5
    """Old keyframes within this distance of the current estimate are loop-closure candidates [m]."""
    loop_min_travel: float = 5.0
    """Candidates must lie at least this much travel back along the trajectory [m]."""
    loop_interval: float = 2.0
    """Travel between loop-closure attempts [m]."""
    loop_chain_nodes: int = 10
    """Keyframes on each side of the nearest candidate that form the reference map for a loop closure."""
    loop_window: float = 0.5
    """Half-width of the loop-closure search window, translation [m]."""
    loop_window_deg: float = 12.0
    """Half-width of the loop-closure search window, rotation [deg]."""
    loop_min_score: float = 0.6
    """Mean likelihood a loop closure needs."""
    loop_max_ambiguity: float = 0.9
    """Reject a loop closure if a clearly different pose scores this fraction of the best one (corridors,
    symmetric rooms)."""
    loop_robust_scale: float = 2.0
    """Cauchy kernel scale on loop-closure constraints, in standard deviations."""
    optimizer_iterations: int = 15
    """Maximum Levenberg-Marquardt iterations per pose-graph optimisation."""

    def __post_init__(self) -> None:
        _require(self.node_distance > 0.0 and self.node_angle_deg > 0.0, "slam node thresholds must be positive")
        _require(self.local_map_nodes >= 1 and self.loop_chain_nodes >= 1, "slam node counts must be >= 1")
        _require(0.0 < self.scan_voxel < 0.5, "slam.scan_voxel must be in (0, 0.5)")
        _require(0.0 < self.field_resolution <= self.field_sigma, "slam needs 0 < field_resolution <= field_sigma")
        for name in ("track_min_score", "loop_min_score", "loop_max_ambiguity"):
            _require(0.0 < getattr(self, name) < 1.0, f"slam.{name} must be in (0, 1)")
        for name in ("track_window", "track_window_deg", "loop_window", "loop_window_deg", "match_point_std",
                     "loop_search_radius", "loop_interval", "loop_robust_scale"):
            _require(getattr(self, name) > 0.0, f"slam.{name} must be positive")
        for name in ("prior_translation_weight", "prior_rotation_weight", "odometry_translation_noise",
                     "odometry_rotation_noise", "odometry_heading_drift", "loop_min_travel"):
            _require(getattr(self, name) >= 0.0, f"slam.{name} must be >= 0")
        _require(self.optimizer_iterations >= 1, "slam.optimizer_iterations must be >= 1")


@dataclass(frozen=True)
class DockConfig:
    """Geometry of the charging station."""

    width: float = 0.32
    """Width along the wall [m]."""
    depth: float = 0.10
    """Depth from the wall into the room [m]."""
    height: float = 0.10
    """Height [m]; tall enough for the LiDAR (milestone 2) to see it."""
    gap: float = 0.005
    """Clearance between the docked robot's bumper and the dock face [m]."""

    def __post_init__(self) -> None:
        for name in ("width", "depth", "height"):
            _require(getattr(self, name) > 0.0, f"dock.{name} must be positive")
        _require(self.gap >= 0.0, "dock.gap must be non-negative")


@dataclass(frozen=True)
class PhysicsConfig:
    """Contact solver settings."""

    contact_skin: float = 0.001
    """Bodies closer than this count as touching [m] (keeps bumpers pressed at rest)."""
    max_solver_iterations: int = 8
    """Penetration-resolution passes per step before reverting the move."""

    def __post_init__(self) -> None:
        _require(self.contact_skin >= 0.0, "physics.contact_skin must be non-negative")
        _require(self.max_solver_iterations >= 1, "physics.max_solver_iterations must be >= 1")


@dataclass(frozen=True)
class TeleopConfig:
    """Manual driving."""

    linear_speed: float = 0.3
    """Forward/backward speed with the arrow keys [m/s]."""
    boost_linear_speed: float = 0.5
    """Speed while Shift is held [m/s]."""
    angular_speed: float = 2.0
    """Turn rate [rad/s]."""

    def __post_init__(self) -> None:
        for name in ("linear_speed", "boost_linear_speed", "angular_speed"):
            _require(getattr(self, name) > 0.0, f"teleop.{name} must be positive")


@dataclass(frozen=True)
class ViewerConfig:
    """Pygame window and real-time loop."""

    width: int = 1280
    """Window width [px]."""
    height: int = 800
    """Window height [px]."""
    panel_width: int = 340
    """Width of the status panel on the right [px]."""
    fps: int = 60
    """Target frame rate."""
    speed_factor: float = 1.0
    """Initial simulation speed relative to wall-clock time."""
    max_steps_per_frame: int = 400
    """Physics steps allowed per rendered frame before the loop drops time."""
    trail_seconds: float = 60.0
    """Length of the ground-truth trail drawn behind the robot [s]."""

    def __post_init__(self) -> None:
        _require(self.width >= 640 and self.height >= 480, "viewer window must be at least 640x480")
        _require(0 < self.panel_width < self.width // 2, "viewer.panel_width must be < half the width")
        _require(self.fps > 0, "viewer.fps must be positive")
        _require(self.speed_factor > 0.0, "viewer.speed_factor must be positive")
        _require(self.max_steps_per_frame >= 1, "viewer.max_steps_per_frame must be >= 1")
        _require(self.trail_seconds >= 0.0, "viewer.trail_seconds must be non-negative")


@dataclass(frozen=True)
class AppConfig:
    """Root of the configuration tree."""

    room: str = "studio"
    """Built-in room name (``config/rooms/<name>.toml``) or a path to a room file."""
    simulation: SimulationConfig = field(default_factory=SimulationConfig)
    robot: RobotSpec = field(default_factory=RobotSpec)
    drive_errors: DriveErrorConfig = field(default_factory=DriveErrorConfig)
    lidar: LidarConfig = field(default_factory=LidarConfig)
    bumper: BumperConfig = field(default_factory=BumperConfig)
    encoders: EncoderConfig = field(default_factory=EncoderConfig)
    dock: DockConfig = field(default_factory=DockConfig)
    physics: PhysicsConfig = field(default_factory=PhysicsConfig)
    teleop: TeleopConfig = field(default_factory=TeleopConfig)
    dwa: DwaConfig = field(default_factory=DwaConfig)
    wander: WanderConfig = field(default_factory=WanderConfig)
    mapping: MappingConfig = field(default_factory=MappingConfig)
    slam: SlamConfig = field(default_factory=SlamConfig)
    viewer: ViewerConfig = field(default_factory=ViewerConfig)

    def __post_init__(self) -> None:
        _require(
            self.lidar.scan_period >= self.simulation.dt and is_multiple_of(self.lidar.scan_period, self.simulation.dt),
            "lidar.scan_period must be a positive multiple of simulation.dt",
        )
        _require(
            self.lidar.mount_height >= self.robot.height,
            "lidar.mount_height must be >= robot.height (the turret sits on top; self-occlusion is not modelled)",
        )
        _require(abs(self.lidar.mount_x) < self.robot.radius, "lidar.mount_x must lie inside the body")
        _require(
            self.dwa.max_speed <= self.robot.max_wheel_speed, "dwa.max_speed must not exceed robot.max_wheel_speed"
        )
        _require(self.wander.backup_speed <= self.robot.max_wheel_speed, "wander.backup_speed exceeds the wheel limit")
        # A step must be much shorter than the robot radius, otherwise thin
        # obstacles (chair legs) could be tunnelled through between two steps.
        step = self.robot.max_wheel_speed * self.simulation.dt
        _require(
            step < 0.25 * self.robot.radius,
            f"robot.max_wheel_speed * simulation.dt = {step:.3f} m per step is too large "
            f"(must be < radius / 4 = {self.robot.radius / 4:.3f} m)",
        )
        _require(
            self.teleop.boost_linear_speed <= self.robot.max_wheel_speed,
            "teleop.boost_linear_speed must not exceed robot.max_wheel_speed",
        )


# --------------------------------------------------------------------------- loading


def build_dataclass[T](cls: type[T], data: Mapping[str, Any], where: str = "") -> T:
    """Construct the frozen dataclass ``cls`` from a (TOML) mapping.

    Missing keys keep their defaults, nested tables build nested dataclasses,
    and ``int`` is accepted where ``float`` is expected.

    Raises:
        ConfigError: on unknown keys, wrong types or failed validation.
    """
    if not isinstance(data, Mapping):
        raise ConfigError(f"[{where}] must be a table, got {type(data).__name__}")
    hints = typing.get_type_hints(cls)
    allowed = {f.name for f in dataclasses.fields(cls) if f.init}  # type: ignore[arg-type]
    unknown = sorted(set(data) - allowed)
    if unknown:
        location = f"[{where}]" if where else "the top level"
        raise ConfigError(f"unknown key(s) {', '.join(unknown)} in {location}")
    kwargs: dict[str, Any] = {}
    for name, raw in data.items():
        expected = hints[name]
        key = f"{where}.{name}" if where else name
        if dataclasses.is_dataclass(expected) and isinstance(expected, type):
            kwargs[name] = build_dataclass(expected, raw, key)
        else:
            kwargs[name] = _coerce(raw, expected, key)
    try:
        return cls(**kwargs)
    except ConfigError:
        raise
    except (TypeError, ValueError) as exc:  # pragma: no cover - defensive
        raise ConfigError(f"{where or 'config'}: {exc}") from exc


def _coerce(value: Any, expected: Any, key: str) -> Any:
    if expected is float:
        if isinstance(value, bool) or not isinstance(value, int | float):
            raise ConfigError(f"{key} must be a number, got {value!r}")
        return float(value)
    if expected is int:
        if isinstance(value, bool) or not isinstance(value, int):
            raise ConfigError(f"{key} must be an integer, got {value!r}")
        return value
    if expected is bool:
        if not isinstance(value, bool):
            raise ConfigError(f"{key} must be true or false, got {value!r}")
        return value
    if expected is str:
        if not isinstance(value, str):
            raise ConfigError(f"{key} must be a string, got {value!r}")
        return value
    raise ConfigError(f"{key}: unsupported config field type {expected!r}")


def deep_merge(base: Mapping[str, Any], update: Mapping[str, Any]) -> dict[str, Any]:
    """Recursively merge ``update`` into a copy of ``base`` (tables merge, values replace)."""
    merged: dict[str, Any] = dict(base)
    for key, value in update.items():
        if isinstance(value, Mapping) and isinstance(merged.get(key), Mapping):
            merged[key] = deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def read_toml(path: Path) -> dict[str, Any]:
    """Read a TOML file, wrapping parse errors in :class:`ConfigError`."""
    try:
        with path.open("rb") as handle:
            return tomllib.load(handle)
    except tomllib.TOMLDecodeError as exc:
        raise ConfigError(f"{path}: {exc}") from exc


def parse_override(text: str) -> dict[str, Any]:
    """Parse ``section.key=value`` into a nested mapping.

    The value is parsed as a TOML value (numbers, booleans, quoted strings,
    arrays); anything that is not valid TOML is taken as a bare string, so
    ``room=test_chamber`` works without quotes.
    """
    key, separator, raw = text.partition("=")
    key = key.strip()
    if not separator or not key:
        raise ConfigError(f"override {text!r} must look like section.key=value")
    raw = raw.strip()
    try:
        value: Any = tomllib.loads(f"v = {raw}")["v"]
    except tomllib.TOMLDecodeError:
        value = raw
    nested: dict[str, Any] = {}
    cursor = nested
    parts = key.split(".")
    for part in parts[:-1]:
        cursor = cursor.setdefault(part, {})
    cursor[parts[-1]] = value
    return nested


def load_config(paths: Iterable[Path] = (), overrides: Iterable[str] = ()) -> AppConfig:
    """Build an :class:`AppConfig` from TOML files and ``key=value`` overrides."""
    data: dict[str, Any] = {}
    for path in paths:
        data = deep_merge(data, read_toml(path))
    for override in overrides:
        data = deep_merge(data, parse_override(override))
    return build_dataclass(AppConfig, data)


def config_to_dict(config: Any) -> dict[str, Any]:
    """Plain-dict view of a config dataclass (useful for logging and tests)."""
    return dataclasses.asdict(config)
