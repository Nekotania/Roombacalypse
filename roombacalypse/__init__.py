"""ROOMBACALYPSE: an unnecessarily overengineered autonomous robotic vacuum simulator.

Package map (see ``docs/ARCHITECTURE.md`` for the full picture):

* ``core``          shared kernel: geometry, kinematics math, messages, config, RNG
* ``environment``   ground-truth room geometry, obstacles, room files
* ``robot``         ground-truth physical robot: drivetrain and body state
* ``simulation``    contact physics, clock, the simulation engine
* ``autonomy``      the robot's software (milestone 1: manual teleoperation)
* ``visualization`` pygame front-end (the only package allowed to import pygame)
"""

__version__ = "0.1.0"
