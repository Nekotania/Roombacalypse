"""ROOMBACALYPSE: an unnecessarily overengineered autonomous robotic vacuum simulator.

Package map (see ``docs/ARCHITECTURE.md`` for the full picture):

* ``core``          shared kernel: geometry, kinematics math, messages, events, config, RNG
* ``environment``   ground truth: room geometry, obstacles, room files
* ``robot``         ground truth: drivetrain, wheel/ground errors, body state
* ``sensors``       ground truth -> readings: LiDAR, bumper, wheel encoders
* ``simulation``    contact physics, clock, the simulation engine
* ``localization``  belief: odometry (M2), particle filter (M3)
* ``navigation``    belief: obstacle memory, Dynamic Window Approach planner
* ``autonomy``      belief: the robot's brain, teleoperation, exploration behaviour
* ``evaluation``    harness: ground-truth scoring and headless scenarios
* ``visualization`` harness: pygame front-end (the only package allowed to import pygame)
"""

__version__ = "0.2.0"
