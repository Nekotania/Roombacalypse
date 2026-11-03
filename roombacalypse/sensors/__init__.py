"""Simulated sensors (truth side): they read ground truth and add realistic errors.

Each sensor is a small, independently testable class whose output is an
immutable reading type from :mod:`roombacalypse.core.messages`, the only
thing the robot's software ever sees.
"""
