"""The robot's software: one long-lived instance that owns perception state and picks a behaviour.

Real robots keep their odometry, map and recent obstacle memory regardless
of who is steering, so the brain owns those and delegates only *command
generation*: to the human (manual teleoperation) or to the autonomous
behaviour. Switching modes therefore never loses the robot's sense of where
it is, and the map keeps growing while a human drives.

Per control tick: odometry integrates the encoders; SLAM folds in the scan
(if one arrived) and returns the map-frame pose; the obstacle memory keeps
the latest scan and recent bumps in the odometry frame for the local
planner; then the active mode produces a command.
"""

from __future__ import annotations

from enum import StrEnum

from roombacalypse.autonomy.teleop import TeleopController
from roombacalypse.autonomy.wander import WanderBehaviour
from roombacalypse.core.config import AppConfig
from roombacalypse.core.events import EventKind, EventPublisher
from roombacalypse.core.geometry import Pose2D
from roombacalypse.core.messages import BumperReading, DriveCommand, LidarScan, SensorFrame
from roombacalypse.core.rng import RngStreams
from roombacalypse.localization.odometry import OdometryTracker
from roombacalypse.localization.slam import GraphSlam
from roombacalypse.navigation.local_obstacles import ObstacleMemory


class Mode(StrEnum):
    MANUAL = "manual"
    AUTONOMOUS = "autonomous"


def bump_side(reading: BumperReading) -> str:
    if reading.left and not reading.right:
        return "left"
    if reading.right and not reading.left:
        return "right"
    return "front"


_BUMP_BEARINGS = {"left": 0.6, "right": -0.6, "front": 0.0}  # [rad], where a switch's obstacle probably is


class RobotBrain:
    """Implements :class:`~roombacalypse.core.interfaces.Controller`."""

    def __init__(self, config: AppConfig, *, publish: EventPublisher | None = None, mode: Mode = Mode.MANUAL) -> None:
        self.config = config
        self.spec = config.robot
        self.teleop = TeleopController(config.teleop)
        self.odometry = OdometryTracker(config.robot, config.encoders, config.odometry)
        self.slam = GraphSlam(config.mapping, config.slam, gyro=config.gyro.enabled and config.odometry.use_gyro)
        self.memory = ObstacleMemory(config.wander.bump_memory_seconds)
        self.wander = WanderBehaviour(
            config.robot, config.wander, config.dwa, config.simulation.control_period, publish
        )
        self.initial_mode = mode
        self.mode = mode
        self.obstacle_range = max(config.dwa.obstacle_range, config.wander.goal_max_distance + 0.5)
        self._publish = publish
        self.time = 0.0
        self.last_scan: LidarScan | None = None
        self.last_bumper = BumperReading()
        self.bumps = 0
        self._loops_reported = 0
        self._was_lost = False

    @property
    def name(self) -> str:
        return self.mode.value

    @property
    def pose(self) -> Pose2D:
        """Where the robot believes it is, in the map frame (SLAM)."""
        return self.slam.pose

    def reset(self, seed: int) -> None:
        streams = RngStreams(seed)
        self.teleop.reset(seed)
        self.odometry.reset()
        self.slam.reset()
        self._loops_reported = 0
        self._was_lost = False
        self.memory.reset()
        self.wander.reset(streams.get("robot.wander"), docked=True)  # it boots on its dock, and knows it
        self.mode = self.initial_mode
        self.time = 0.0
        self.last_scan = None
        self.last_bumper = BumperReading()
        self.bumps = 0

    def set_mode(self, mode: Mode) -> None:
        if mode is self.mode:
            return
        self.mode = mode
        self.teleop.set_intent(0.0, 0.0)
        if mode is Mode.AUTONOMOUS:
            self.wander.activate()
        self._emit(EventKind.MODE, f"mode: {mode.value}", mode=mode.value)

    def toggle_mode(self) -> Mode:
        self.set_mode(Mode.MANUAL if self.mode is Mode.AUTONOMOUS else Mode.AUTONOMOUS)
        return self.mode

    def _emit(self, kind: EventKind, message: str, **data: object) -> None:
        if self._publish is not None:
            self._publish(self.time, kind, message, **data)

    def update(self, frame: SensorFrame, dt: float) -> DriveCommand:
        self.time = frame.time
        pose = self.odometry.update(frame.encoders, frame.time, frame.gyro)
        map_pose = self.slam.update(frame.time, pose, frame.lidar)
        if len(self.slam.loop_closures) > self._loops_reported:
            closure = self.slam.loop_closures[-1]
            self._loops_reported = len(self.slam.loop_closures)
            self._emit(EventKind.MAP, f"loop closed: map corrected by {100 * closure.correction:.1f} cm",
                       correction=closure.correction)
        lost = self.slam.lost
        if lost and not self._was_lost:
            self._emit(EventKind.MAP, "scan does not fit the map: dead reckoning", score=self.slam.tracking_score)
        self._was_lost = lost
        if frame.lidar is not None:
            self.last_scan = frame.lidar
            self.memory.update_scan(frame.lidar, pose)
        new_bump: BumperReading | None = None
        if frame.bumper.pressed and not self.last_bumper.pressed:
            new_bump = frame.bumper
            self.bumps += 1
            side = bump_side(frame.bumper)
            self.memory.add_bump(frame.time, pose, _BUMP_BEARINGS[side], self.spec.radius)
            self._emit(EventKind.BUMP, f"bump ({side})", side=side)
        self.last_bumper = frame.bumper
        self.memory.expire(frame.time)

        if self.mode is Mode.MANUAL:
            return self.teleop.update(frame, dt)
        obstacles = self.memory.points_in_body(pose, self.obstacle_range)
        ready = self.last_scan is not None and not self.odometry.calibrating  # sit still on the dock until then
        return self.wander.update(frame.time, self.odometry, obstacles, new_bump, ready, map_pose)
