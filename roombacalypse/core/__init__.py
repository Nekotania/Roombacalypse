"""Shared kernel used by every other package.

``core`` imports nothing else from this project. It holds the vocabulary both
sides of the "wall of truth" (simulator and robot software) need to share:
planar geometry, differential-drive math, message types, configuration and
random-stream management.
"""
