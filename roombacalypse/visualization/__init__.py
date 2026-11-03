"""Pygame front-end. The only package allowed to import pygame (enforced by tests).

Everything here *reads* simulation state and turns user input into calls on
public simulation/controller methods; nothing here contains robotics logic.
"""

import os

# Must be set before pygame is first imported anywhere.
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
