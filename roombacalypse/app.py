"""Command-line entry point and composition root.

This is the only place that knows every concrete class: it loads the
configuration and room, builds the simulation and the robot's software,
wires the shared event log, and hands everything to the viewer or the
headless scenario runner.
"""

from __future__ import annotations

import argparse
import dataclasses
import logging
import sys
from collections.abc import Sequence
from pathlib import Path

from roombacalypse import __version__
from roombacalypse.autonomy.brain import Mode, RobotBrain
from roombacalypse.core.config import AppConfig, ConfigError, load_config
from roombacalypse.core.events import EventLog
from roombacalypse.core.paths import available_rooms, default_config_path, resolve_room_path
from roombacalypse.environment.rooms import RoomError, RoomSpec, load_room
from roombacalypse.simulation.engine import Simulation

logger = logging.getLogger("roombacalypse")

MODES = {"manual": Mode.MANUAL, "auto": Mode.AUTONOMOUS}


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
    parser.add_argument("--mode", choices=sorted(MODES),
                        help="initial control mode (default: manual with a window, auto when headless)")
    parser.add_argument("--list-rooms", action="store_true", help="list built-in rooms and exit")
    parser.add_argument("--headless", action="store_true",
                        help="simulate without a window for --duration seconds and print a scorecard")
    parser.add_argument("--duration", type=float, default=600.0,
                        help="simulated seconds for --headless (default 600)")
    parser.add_argument("--screenshot", type=Path, metavar="PNG",
                        help="render offscreen for --frames frames, save a screenshot and exit")
    parser.add_argument("--frames", type=int, default=60, help="frames to render with --screenshot (default 60)")
    parser.add_argument("--sound", metavar="FILE",
                        help="play FILE (WAV/OGG) whenever the robot hits something; 'bonk' for the built-in sound")
    parser.add_argument("--log-level", default="INFO", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    return parser


def load_app_config(args: argparse.Namespace) -> AppConfig:
    """Layer config/default.toml, --config files, --set overrides, then --room/--seed."""
    paths = [default_config_path()] if default_config_path().is_file() else []
    paths.extend(args.config)
    config = load_config(paths, args.overrides)
    if args.room:
        config = dataclasses.replace(config, room=args.room)
    if args.sound is not None:
        config = dataclasses.replace(config, viewer=dataclasses.replace(config.viewer, collision_sound=args.sound))
    if args.seed is not None:  # replace() re-runs validation
        config = dataclasses.replace(config, simulation=dataclasses.replace(config.simulation, seed=args.seed))
    return config


def run_headless(config: AppConfig, room: RoomSpec, mode: Mode, duration: float) -> int:
    from roombacalypse.evaluation.scenario import run_scenario

    logger.info("headless run: %s, %.0f s simulated, mode %s", room.name, duration, mode.value)
    result = run_scenario(config, room, duration=duration, mode=mode)
    print(result.summary())
    return 0


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

    mode = MODES[args.mode] if args.mode else (Mode.AUTONOMOUS if args.headless else Mode.MANUAL)
    if args.headless:
        if args.duration <= 0:
            logger.error("--duration must be positive")
            return 2
        try:
            return run_headless(config, room, mode, args.duration)
        except RoomError as exc:
            logger.error("%s", exc)
            return 2

    events = EventLog()
    brain = RobotBrain(config, publish=events.publisher("robot"), mode=mode)
    try:
        sim = Simulation(config, room, brain, events=events)
    except RoomError as exc:
        logger.error("%s", exc)
        return 2

    # Imported here so that pygame is only needed when a window is opened.
    from roombacalypse.visualization.viewer import Viewer

    viewer = Viewer(sim, brain, config.viewer, offscreen=args.screenshot is not None)
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
