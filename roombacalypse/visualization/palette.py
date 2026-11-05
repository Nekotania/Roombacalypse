"""Colours. One place to restyle the whole UI."""

from __future__ import annotations

from roombacalypse.environment.world import ObstacleKind

Color = tuple[int, int, int]

BACKGROUND: Color = (22, 23, 29)
FLOOR: Color = (228, 221, 206)
FLOOR_GRID_MAJOR: Color = (200, 191, 172)
FLOOR_GRID_MINOR: Color = (216, 208, 191)
WALL: Color = (62, 64, 78)
WALL_FACE: Color = (98, 101, 118)

ROBOT_BODY: Color = (42, 45, 54)
ROBOT_RIM: Color = (156, 164, 182)
ROBOT_INNER: Color = (70, 74, 88)
ROBOT_WHEEL: Color = (18, 18, 22)
ROBOT_HEADING: Color = (53, 208, 255)
ROBOT_TURRET: Color = (92, 98, 116)
BUMPER: Color = (110, 116, 134)
BUMPER_HIT: Color = (255, 77, 77)
TRAIL: Color = (60, 140, 220)

LIDAR_RAY = (255, 80, 80, 45)
LIDAR_HIT = (255, 64, 64, 230)
DWA_ADMISSIBLE = (120, 200, 255, 55)
DWA_REJECTED = (255, 96, 96, 45)
DWA_CHOSEN = (53, 208, 255, 235)
GOAL: Color = (255, 196, 48)
BELIEF: Color = (214, 92, 255)
SLAM_BELIEF: Color = (40, 230, 160)

MAP_FREE = (80, 190, 255, 38)
MAP_OCCUPIED = (16, 92, 196, 225)
GRAPH_EDGE = (40, 230, 160, 120)
GRAPH_NODE = (40, 230, 160, 200)
GRAPH_LOOP = (255, 214, 64, 230)

MOVABLE_OUTLINE: Color = (240, 186, 40)
HOVER_OUTLINE: Color = (255, 255, 255)
GHOST_OK: Color = (80, 220, 140)
GHOST_BAD: Color = (255, 90, 90)

PANEL_BG: Color = (29, 31, 39)
PANEL_RULE: Color = (52, 55, 68)
TEXT: Color = (222, 226, 234)
TEXT_DIM: Color = (138, 144, 160)
ACCENT: Color = (53, 208, 255)
GOOD: Color = (90, 220, 140)
WARN: Color = (255, 176, 64)
BAD: Color = (255, 92, 92)

KIND_COLORS: dict[ObstacleKind, Color] = {
    ObstacleKind.FURNITURE: (139, 108, 82),
    ObstacleKind.SOFT: (104, 124, 160),
    ObstacleKind.LEG: (88, 66, 50),
    ObstacleKind.SURFACE: (156, 124, 92),
    ObstacleKind.WALL: WALL,
    ObstacleKind.APPLIANCE: (168, 175, 188),
    ObstacleKind.PLANT: (72, 140, 84),
    ObstacleKind.PILLAR: (118, 118, 130),
    ObstacleKind.BOX: (198, 160, 98),
    ObstacleKind.CLUTTER: (218, 80, 150),
    ObstacleKind.DOCK: (44, 172, 120),
}


def darker(color: Color, factor: float = 0.65) -> Color:
    """Scale a colour towards black."""
    return (int(color[0] * factor), int(color[1] * factor), int(color[2] * factor))
