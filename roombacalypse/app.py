"""Command-line entry point and composition root.

This is the only place that knows every concrete class: it loads the
configuration and room, builds the simulation and the robot's software, and
hands them to the viewer.
"""

from __future__ import annotations

import argparse
import dataclasses
import logging
import sys
from collections.abc import Sequence
from pathlib import Path

from roombacalypse import __version__
from roombacalypse.autonomy.teleop import TeleopController
from roombacalypse.core.config import AppConfig, ConfigError, load_config
from roombacalypse.core.paths import available_rooms, default_config_path, resolve_room_path
from roombacalypse.environment.rooms import RoomError, load_room
from roombacalypse.simulation.engine import Simulation

logger = logging.getLogger("roombacalypse")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="roombacalypse",
        description="ROOMBACALYPSE: an unnecessarily overengineered robot vacuum simulator.",
    )
    parser.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    parser.add_argument("--room", help="built-in room name or path to a room .toml (see --list-rooms)")
    parser.add_argument("--seed", type=int, help="root random seed (overrides simulation.seed)")
    parser.add_argument("--config", type=Path, action="append", default=[],
                        help="extra TOML config layered over config/default.toml (repeatable)")
    parser.add_argument("--set", dest="overrides", action="append", default=[], metavar="KEY=VALUE",
                        help="override one config value, e.g. --set robot.max_wheel_speed=0.3 (repeatable)")
    parser.add_argument("--list-rooms", action="store_true", help="list built-in rooms and exit")
    parser.add_argument("--screenshot", type=Path, metavar="PNG",
                        help="render offscreen for --frames frames, save a screenshot and exit")
    parser.add_argument("--frames", type=int, default=60, help="frames to render with --screenshot (default 60)")
    parser.add_argument("--log-level", default="INFO", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    return parser


def load_app_config(args: argparse.Namespace) -> AppConfig:
    """Layer config/default.toml, --config files, --set overrides, then --room/--seed."""
    paths = [default_config_path()] if default_config_path().is_file() else []
    paths.extend(args.config)
    config = load_config(paths, args.overrides)
    if args.room:
        config = dataclasses.replace(config, room=args.room)
    if args.seed is not None:  # replace() re-runs validation
        config = dataclasses.replace(config, simulation=dataclasses.replace(config.simulation, seed=args.seed))
    return config


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(level=args.log_level, format="%(asctime)s %(levelname)-7s %(name)s: %(message)s")

    if args.list_rooms:
        for name in available_rooms():
            room = load_room(resolve_room_path(name))
            print(f"{name:16s} {room.name}: {room.description}")
        return 0

    try:
        config = load_app_config(args)
        room = load_room(resolve_room_path(config.room))
    except (ConfigError, RoomError, FileNotFoundError) as exc:
        logger.error("%s", exc)
        return 2

    teleop = TeleopController(config.teleop)
    try:
        sim = Simulation(config, room, teleop)
    except RoomError as exc:
        logger.error("%s", exc)
        return 2

    # Imported here so that pygame is only needed when a window is opened.
    from roombacalypse.visualization.viewer import Viewer

    viewer = Viewer(sim, teleop, config.viewer, offscreen=args.screenshot is not None)
    if args.screenshot is not None:
        viewer.run_frames(max(1, args.frames))
        viewer.save_screenshot(args.screenshot)
        viewer.close()
        return 0
    try:
        viewer.run()
    except KeyboardInterrupt:
        viewer.close()
    return 0


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
