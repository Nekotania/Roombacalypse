"""Status panel, toasts, help overlay and tooltips."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import pygame

from roombacalypse import __version__
from roombacalypse.autonomy.brain import Mode, RobotBrain
from roombacalypse.core.events import EventKind
from roombacalypse.robot.body import Contact
from roombacalypse.simulation.engine import Simulation
from roombacalypse.visualization import palette

_MONO = "menlo,sfmono,monaco,dejavusansmono,consolas,couriernew,monospace"


class Fonts:
    """The handful of fonts the UI uses (system monospace, pygame default as fallback)."""

    def __init__(self) -> None:
        pygame.font.init()
        self.title = pygame.font.SysFont(_MONO, 22, bold=True)
        self.heading = pygame.font.SysFont(_MONO, 13, bold=True)
        self.body = pygame.font.SysFont(_MONO, 14)
        self.small = pygame.font.SysFont(_MONO, 12)


@dataclass(frozen=True)
class UiState:
    """Viewer state the panel displays."""

    speed: float
    achieved_speed: float
    paused: bool
    lagging: bool
    fps: float
    placing: str
    coverage: float
    """Fraction of the accessible floor swept so far (ground truth)."""
    layers: str
    """Which overlays are on, e.g. ``"lidar · nav · belief"``."""


class TextColumn:
    """Top-to-bottom text layout inside a rectangle."""

    LABEL_WIDTH = 92

    def __init__(self, surface: pygame.Surface, rect: pygame.Rect, fonts: Fonts) -> None:
        self.surface = surface
        self.fonts = fonts
        self.x = rect.x + 16
        self.y = rect.y + 14
        self.right = rect.right - 16

    def _blit(self, font: pygame.font.Font, text: str, color: palette.Color, x: int | None = None) -> None:
        self.surface.blit(font.render(text, True, color), (self.x if x is None else x, self.y))

    def title(self, text: str, color: palette.Color = palette.TEXT) -> None:
        self._blit(self.fonts.title, text, color)
        self.y += self.fonts.title.get_linesize()

    def line(self, text: str, color: palette.Color = palette.TEXT, font: pygame.font.Font | None = None) -> None:
        font = font or self.fonts.body
        self._blit(font, text, color)
        self.y += font.get_linesize()

    def section(self, text: str) -> None:
        self.y += 10
        self._blit(self.fonts.heading, text.upper(), palette.ACCENT)
        self.y += self.fonts.heading.get_linesize() + 2
        pygame.draw.line(self.surface, palette.PANEL_RULE, (self.x, self.y), (self.right, self.y))
        self.y += 5

    def row(self, label: str, value: str, color: palette.Color = palette.TEXT) -> None:
        self._blit(self.fonts.body, label, palette.TEXT_DIM)
        self._blit(self.fonts.body, value, color, self.x + self.LABEL_WIDTH)
        self.y += self.fonts.body.get_linesize()


def format_time(seconds: float) -> str:
    """``HH:MM:SS.s``."""
    hours, rest = divmod(seconds, 3600.0)
    minutes, secs = divmod(rest, 60.0)
    return f"{int(hours):02d}:{int(minutes):02d}:{secs:04.1f}"


def bearing_word(angle: float) -> str:
    """Name the direction of a body-frame bearing (0 = front, +left)."""
    names = ("front", "front-left", "left", "rear-left", "rear", "rear-right", "right", "front-right")
    index = round(math.degrees(angle) / 45.0) % 8
    return names[index]


def describe_contact(sim: Simulation, contact: Contact) -> str:
    name = "wall" if contact.obstacle_id is None else sim.world.obstacle(contact.obstacle_id).name
    return f"{name} ({bearing_word(contact.bearing(sim.robot.pose))})"


_SOURCE_COLORS = {"robot": palette.ACCENT, "sim": palette.TEXT_DIM, "user": palette.WARN}


def draw_panel(
    surface: pygame.Surface, rect: pygame.Rect, fonts: Fonts, sim: Simulation, brain: RobotBrain, ui: UiState
) -> None:
    """Draw the status panel on the right: truth, sensors, belief, statistics, events."""
    pygame.draw.rect(surface, palette.PANEL_BG, rect)
    pygame.draw.line(surface, palette.PANEL_RULE, rect.topleft, rect.bottomleft, 2)
    col = TextColumn(surface, rect, fonts)
    col.title("ROOMBACALYPSE")
    col.line(f"v{__version__} · milestone 3: it knows where it is", palette.TEXT_DIM, fonts.small)

    col.section("simulation")
    col.row("time", format_time(sim.time))
    if ui.paused:
        col.row("speed", "PAUSED", palette.WARN)
    else:
        color = palette.WARN if ui.lagging else palette.TEXT
        col.row("speed", f"x{ui.speed:g}  (actual x{ui.achieved_speed:.1f})", color)
    col.row("room", f"{sim.room.name} · seed {sim.seed}")

    col.section("control")
    if brain.mode is Mode.AUTONOMOUS:
        col.row("mode", f"AUTONOMOUS · {brain.wander.state.value}", palette.ACCENT)
        goal = brain.wander.goal
        if goal is not None:
            belief_pose = brain.pose
            col.row("goal", f"{math.hypot(goal[0] - belief_pose.x, goal[1] - belief_pose.y):.2f} m away (belief)")
        else:
            col.row("goal", "none")
    else:
        col.row("mode", "MANUAL · Tab for autonomy", palette.ACCENT)
        col.row("goal", "you are the goal")
    col.row("command", f"v {sim.command.linear:+.2f} m/s  ω {sim.command.angular:+.2f}")

    body = sim.robot
    pose = body.pose
    col.section("robot · ground truth")
    col.row("pose", f"x {pose.x:4.2f} y {pose.y:4.2f} θ {math.degrees(pose.theta):+4.0f}°")
    col.row("velocity", f"v {body.linear_velocity:+.2f} m/s  ω {body.angular_velocity:+.2f}")
    if body.contacts:
        col.row("contact", describe_contact(sim, body.contacts[0]), palette.BAD)
    else:
        col.row("contact", "none", palette.GOOD)

    col.section("sensors · what the robot gets")
    scan = brain.last_scan
    if scan is None:
        col.row("lidar", "no scan yet", palette.TEXT_DIM)
    else:
        valid = int(scan.valid.sum())
        nearest = float(scan.ranges[scan.valid].min()) if valid else math.inf
        col.row("lidar", f"{valid}/{scan.ranges.shape[0]} · nearest {nearest:.2f} m")
    bumper = brain.last_bumper
    col.row("bumper", f"L {'■' if bumper.left else '□'}   R {'■' if bumper.right else '□'}",
            palette.BAD if bumper.pressed else palette.TEXT)
    col.row("odometry", f"{brain.odometry.distance:.2f} m travelled")

    col.section("belief · SLAM vs dead reckoning")
    slam = brain.slam
    located = sim.boot_pose.compose(brain.pose)
    error = located.distance_to(pose)
    heading_error = abs(math.degrees(math.remainder(located.theta - pose.theta, 2 * math.pi)))
    col.row("SLAM", f"{100 * error:.1f} cm · {heading_error:.1f}° from truth",
            palette.WARN if error > 0.05 else palette.GOOD)
    belief = sim.boot_pose.compose(brain.odometry.pose)
    drift = belief.distance_to(pose)
    heading_error = abs(math.degrees(math.remainder(belief.theta - pose.theta, 2 * math.pi)))
    col.row("odometry", f"{drift:.2f} m · {heading_error:.1f}° from truth",
            palette.WARN if drift > 0.5 else palette.TEXT)
    if slam.lost:
        col.row("tracking", f"LOST · scan fit {slam.tracking_score:.2f}", palette.BAD)
    else:
        col.row("tracking", f"scan fit {slam.tracking_score:.2f}")
    col.row("map", f"{len(slam.keyframes)} keyframes · {len(slam.loop_closures)} loops")

    col.section("statistics")
    col.row("distance", f"{body.odometer:.1f} m · swept {100 * ui.coverage:.0f}% of floor")
    col.row("bumps", f"{brain.bumps} felt · {sim.events.count(EventKind.COLLISION)} contacts")

    col.section("events")
    for event in reversed(sim.events.recent(5)):
        text = f"{format_time(event.time)[3:]}  {event.message}"
        col.line(text[:44], _SOURCE_COLORS.get(event.source, palette.TEXT), fonts.small)

    col.y = rect.bottom - 2 * fonts.small.get_linesize() - 10
    col.line("H help · Tab mode · L G E M P layers", palette.TEXT_DIM, fonts.small)
    col.line(f"{ui.fps:3.0f} fps · layers: {ui.layers}", palette.TEXT_DIM, fonts.small)


def draw_toasts(surface: pygame.Surface, area: pygame.Rect, fonts: Fonts, messages: Sequence[str]) -> None:
    """Recent messages, bottom-left of the world view, newest last."""
    y = area.bottom - 12
    for text in reversed(messages):
        image = fonts.body.render(text, True, palette.TEXT)
        y -= image.get_height() + 8
        box = pygame.Rect(area.x + 12, y - 4, image.get_width() + 16, image.get_height() + 8)
        backdrop = pygame.Surface(box.size, pygame.SRCALPHA)
        backdrop.fill((16, 17, 22, 200))
        surface.blit(backdrop, box.topleft)
        surface.blit(image, (box.x + 8, box.y + 4))


def draw_help(surface: pygame.Surface, area: pygame.Rect, fonts: Fonts, lines: Sequence[tuple[str, str]]) -> None:
    """Centered overlay listing the controls."""
    row_h = fonts.body.get_linesize() + 2
    width, height = 560, 70 + row_h * len(lines)
    box = pygame.Rect(0, 0, width, height)
    box.center = area.center
    backdrop = pygame.Surface(box.size, pygame.SRCALPHA)
    backdrop.fill((16, 17, 22, 228))
    surface.blit(backdrop, box.topleft)
    pygame.draw.rect(surface, palette.PANEL_RULE, box, 1)
    surface.blit(fonts.heading.render("CONTROLS", True, palette.ACCENT), (box.x + 20, box.y + 18))
    y = box.y + 46
    for keys, action in lines:
        surface.blit(fonts.body.render(keys, True, palette.TEXT), (box.x + 20, y))
        surface.blit(fonts.body.render(action, True, palette.TEXT_DIM), (box.x + 230, y))
        y += row_h


def draw_tooltip(surface: pygame.Surface, position: tuple[int, int], fonts: Fonts, text: str) -> None:
    image = fonts.small.render(text, True, palette.TEXT)
    box = pygame.Rect(position[0] + 14, position[1] + 14, image.get_width() + 12, image.get_height() + 8)
    bounds = surface.get_rect()
    if box.right > bounds.right:
        box.x = position[0] - box.width - 6
    if box.bottom > bounds.bottom:
        box.y = position[1] - box.height - 6
    pygame.draw.rect(surface, (16, 17, 22), box)
    pygame.draw.rect(surface, palette.PANEL_RULE, box, 1)
    surface.blit(image, (box.x + 6, box.y + 4))
