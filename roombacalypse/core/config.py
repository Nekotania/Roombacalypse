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
    dock: DockConfig = field(default_factory=DockConfig)
    physics: PhysicsConfig = field(default_factory=PhysicsConfig)
    teleop: TeleopConfig = field(default_factory=TeleopConfig)
    viewer: ViewerConfig = field(default_factory=ViewerConfig)

    def __post_init__(self) -> None:
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
