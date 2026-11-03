"""Locations of configuration and room files.

The project is meant to be run from a source checkout, where ``config/`` sits
next to the package. ``ROOMBACALYPSE_CONFIG_DIR`` overrides the location.
"""

from __future__ import annotations

import os
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]


def config_dir() -> Path:
    """Directory holding ``default.toml`` and the ``rooms/`` folder."""
    override = os.environ.get("ROOMBACALYPSE_CONFIG_DIR")
    return Path(override) if override else PROJECT_ROOT / "config"


def default_config_path() -> Path:
    """Path of the documented default configuration file."""
    return config_dir() / "default.toml"


def rooms_dir() -> Path:
    """Directory holding the built-in room files."""
    return config_dir() / "rooms"


def available_rooms() -> list[str]:
    """Names of built-in rooms (file stems in :func:`rooms_dir`)."""
    directory = rooms_dir()
    if not directory.is_dir():
        return []
    return sorted(p.stem for p in directory.glob("*.toml"))


def resolve_room_path(name_or_path: str) -> Path:
    """Turn a built-in room name (``"studio"``) or a file path into a path.

    Raises:
        FileNotFoundError: if neither a file nor a built-in room matches.
    """
    candidate = Path(name_or_path)
    if candidate.suffix == ".toml" or candidate.parent != Path("."):
        if candidate.is_file():
            return candidate
        raise FileNotFoundError(f"room file not found: {candidate}")
    builtin = rooms_dir() / f"{name_or_path}.toml"
    if builtin.is_file():
        return builtin
    rooms = ", ".join(available_rooms()) or "(none found)"
    raise FileNotFoundError(f"unknown room {name_or_path!r}; built-in rooms: {rooms}")
