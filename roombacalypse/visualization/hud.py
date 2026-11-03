"""Status panel, toasts, help overlay and tooltips."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import pygame

from roombacalypse import __version__
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


def draw_panel(surface: pygame.Surface, rect: pygame.Rect, fonts: Fonts, sim: Simulation, ui: UiState) -> None:
    """Draw the status panel on the right."""
    pygame.draw.rect(surface, palette.PANEL_BG, rect)
    pygame.draw.line(surface, palette.PANEL_RULE, rect.topleft, rect.bottomleft, 2)
    col = TextColumn(surface, rect, fonts)
    col.title("ROOMBACALYPSE")
    col.line(f"v{__version__} · milestone 1: it moves", palette.TEXT_DIM, fonts.small)

    col.section("simulation")
    col.row("time", format_time(sim.time))
    if ui.paused:
        col.row("speed", "PAUSED", palette.WARN)
    else:
        lag_color = palette.WARN if ui.lagging else palette.TEXT
        col.row("speed", f"x{ui.speed:g}  (actual x{ui.achieved_speed:.1f})", lag_color)
    col.row("room", sim.room.name)
    col.row("seed", str(sim.seed))

    col.section("control")
    col.row("mode", sim.controller.name.upper(), palette.ACCENT)
    col.row("command", f"v {sim.command.linear:+.2f} m/s")
    col.row("", f"ω {sim.command.angular:+.2f} rad/s")

    body = sim.robot
    pose = body.pose
    col.section("robot · ground truth")
    col.row("position", f"x {pose.x:5.2f}  y {pose.y:5.2f} m")
    col.row("heading", f"{math.degrees(pose.theta):+6.1f}°")
    col.row("velocity", f"v {body.linear_velocity:+.2f} m/s")
    col.row("", f"ω {body.angular_velocity:+.2f} rad/s")
    col.row("wheels", f"L {body.v_left:+.2f}  R {body.v_right:+.2f} m/s")
    col.row("odometer", f"{body.odometer:.2f} m")
    if body.contacts:
        col.row("contact", describe_contact(sim, body.contacts[0]), palette.BAD)
        for contact in body.contacts[1:3]:
            col.row("", describe_contact(sim, contact), palette.BAD)
    else:
        col.row("contact", "none", palette.GOOD)
    if body.blocked:
        col.row("", "pinned (move cancelled)", palette.WARN)

    col.section("world")
    col.row("obstacles", str(len(sim.world.obstacles)))
    col.row("floor", f"{sim.world.floor_area:.1f} m²")
    col.row("placing", ui.placing)

    col.y = rect.bottom - 2 * fonts.small.get_linesize() - 10
    col.line("H help · Space pause · R reset · Esc quit", palette.TEXT_DIM, fonts.small)
    col.line(f"{ui.fps:4.0f} fps", palette.TEXT_DIM, fonts.small)


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
