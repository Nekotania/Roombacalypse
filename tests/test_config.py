"""Configuration schema, TOML layering and validation."""

from __future__ import annotations

from pathlib import Path

import pytest

from roombacalypse.core.config import (
    AppConfig,
    ConfigError,
    RobotSpec,
    SimulationConfig,
    build_dataclass,
    config_to_dict,
    load_config,
    parse_override,
)
from roombacalypse.core.paths import default_config_path


def test_defaults_are_valid() -> None:
    config = AppConfig()
    assert config.simulation.dt == 0.02
    assert config.robot.max_angular_speed == pytest.approx(2 * 0.5 / 0.235)


def test_default_toml_matches_code_defaults() -> None:
    """config/default.toml documents the defaults; it must not drift from the code."""
    assert config_to_dict(load_config([default_config_path()])) == config_to_dict(AppConfig())


def test_partial_override_keeps_other_defaults(tmp_path: Path) -> None:
    path = tmp_path / "mine.toml"
    path.write_text("[robot]\nradius = 0.2\n\n[simulation]\nseed = 7\n")
    config = load_config([path])
    assert config.robot.radius == 0.2
    assert config.robot.wheel_base == RobotSpec().wheel_base
    assert config.simulation.seed == 7
    assert config.simulation.dt == SimulationConfig().dt


def test_integers_are_accepted_for_floats() -> None:
    assert build_dataclass(RobotSpec, {"max_wheel_speed": 1}).max_wheel_speed == 1.0


@pytest.mark.parametrize(
    ("data", "message"),
    [
        ({"robot": {"radiuss": 0.2}}, "unknown key"),
        ({"robto": {}}, "unknown key"),
        ({"robot": {"radius": "big"}}, "must be a number"),
        ({"simulation": {"seed": 1.5}}, "must be an integer"),
        ({"robot": {"radius": True}}, "must be a number"),
        ({"robot": 3}, "must be a table"),
        ({"robot": {"radius": -1.0}}, "must be positive"),
        ({"simulation": {"control_period": 0.05, "dt": 0.03}}, "multiple"),
        ({"simulation": {"dt": 0.1}}, "too large"),
        ({"robot": {"wheel_base": 0.5}}, "fit inside"),
    ],
)
def test_invalid_configs_are_rejected(data: dict, message: str) -> None:
    with pytest.raises(ConfigError, match=message):
        build_dataclass(AppConfig, data)


def test_layers_apply_in_order(tmp_path: Path) -> None:
    first = tmp_path / "a.toml"
    second = tmp_path / "b.toml"
    first.write_text("[robot]\nradius = 0.2\nheight = 0.1\n")
    second.write_text("[robot]\nradius = 0.18\n")
    config = load_config([first, second], ["robot.height=0.08"])
    assert (config.robot.radius, config.robot.height) == (0.18, 0.08)


def test_parse_override_types() -> None:
    assert parse_override("robot.radius=0.2") == {"robot": {"radius": 0.2}}
    assert parse_override("simulation.seed = 9") == {"simulation": {"seed": 9}}
    assert parse_override("room=test_chamber") == {"room": "test_chamber"}  # bare string
    assert parse_override('room="studio"') == {"room": "studio"}
    with pytest.raises(ConfigError):
        parse_override("no-equals-sign")


def test_malformed_toml_is_a_config_error(tmp_path: Path) -> None:
    path = tmp_path / "broken.toml"
    path.write_text("[robot\nradius = 0.2\n")
    with pytest.raises(ConfigError):
        load_config([path])
