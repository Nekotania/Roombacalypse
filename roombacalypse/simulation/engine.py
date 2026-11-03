"""The simulation engine: one robot, one room, fixed timestep, no rendering.

Step order (every ``dt``):

1. if a LiDAR sweep is due, scan from the current true pose;
2. if a control tick is due, build a :class:`SensorFrame` (encoders, latched
   bumper, the scan if a new one completed) and ask the controller for a
   :class:`DriveCommand` (held until the next tick);
3. the drivetrain moves the wheel speeds towards the command within limits;
4. the wheel/ground model turns wheel rotation into true ground travel
   (calibration errors and slip); the pose is integrated exactly;
5. the contact solver resolves collisions, possibly pushing obstacles;
6. body state, bumper latches, collision events and the clock advance.

Deterministic: the same room, config, seed and controller give bit-identical
runs (``tests/test_simulation.py``). Every random number comes from a named
stream of :class:`RngStreams`.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass

from roombacalypse.core.config import AppConfig
from roombacalypse.core.events import EventKind, EventLog
from roombacalypse.core.geometry import Pose2D, wrap_angle
from roombacalypse.core.interfaces import Controller
from roombacalypse.core.kinematics import integrate_wheel_travel
from roombacalypse.core.messages import STOP, DriveCommand, LidarScan, SensorFrame
from roombacalypse.core.rng import RngStreams
from roombacalypse.environment.catalog import ObstacleTemplate
from roombacalypse.environment.rooms import RoomSpec, build_world
from roombacalypse.environment.shapes import Circle, shapes_overlap
from roombacalypse.environment.world import Obstacle, World
from roombacalypse.robot.body import Contact, RobotBody
from roombacalypse.robot.drivetrain import Drivetrain
from roombacalypse.robot.wheels import WheelGroundModel
from roombacalypse.sensors.bumper import Bumper
from roombacalypse.sensors.encoders import WheelEncoders
from roombacalypse.sensors.lidar import Lidar
from roombacalypse.simulation.clock import SimClock, steps_per_period
from roombacalypse.simulation.physics import ContactSolver

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class Placement:
    """Result of trying to place an obstacle."""

    ok: bool
    reason: str
    obstacle: Obstacle | None = None


class Simulation:
    """Ground-truth simulation of one robot in one room."""

    def __init__(
        self,
        config: AppConfig,
        room: RoomSpec,
        controller: Controller,
        *,
        seed: int | None = None,
        events: EventLog | None = None,
    ) -> None:
        self.config = config
        self.room = room
        self.spec = config.robot
        self.dt = config.simulation.dt
        self.control_period = config.simulation.control_period
        self._control_every = steps_per_period(self.control_period, self.dt)
        self._lidar_every = steps_per_period(config.lidar.scan_period, self.dt)
        self.drivetrain = Drivetrain(self.spec)
        self.solver = ContactSolver(self.spec, config.physics)
        self.clock = SimClock(self.dt)
        self.controller = controller
        self.events = events if events is not None else EventLog()
        self.seed = config.simulation.seed if seed is None else seed
        # Assigned by reset():
        self.rng: RngStreams
        self.world: World
        self.robot: RobotBody
        self.boot_pose: Pose2D
        self.ground: WheelGroundModel
        self.lidar: Lidar
        self.bumper: Bumper
        self.encoders: WheelEncoders
        self.last_scan: LidarScan | None = None
        self.last_scan_pose: Pose2D | None = None
        self.command: DriveCommand = STOP
        self._pending_scan: LidarScan | None = None
        self._touching: frozenset[object] = frozenset()
        self.reset()

    # ------------------------------------------------------------------ lifecycle

    @property
    def time(self) -> float:
        """Simulation time [s]."""
        return self.clock.time

    @property
    def tick(self) -> int:
        return self.clock.tick

    def reset(self, seed: int | None = None) -> None:
        """Rebuild the room, draw a new robot instance and put it back on the dock."""
        if seed is not None:
            self.seed = seed
        self.rng = RngStreams(self.seed)
        self.world = build_world(self.room, self.spec, self.config.dock)
        assert self.world.dock is not None
        self.boot_pose = self.world.dock.pose
        self.robot = RobotBody(pose=self.boot_pose)
        self.ground = WheelGroundModel(
            self.spec, self.config.drive_errors, self.rng.get("truth.calibration"), self.rng.get("truth.slip")
        )
        self.lidar = Lidar(self.config.lidar, self.rng.get("sensors.lidar"))
        self.bumper = Bumper(self.config.bumper)
        self.encoders = WheelEncoders(self.spec, self.config.encoders)
        self.last_scan = None
        self.last_scan_pose = None
        self._pending_scan = None
        self._touching = frozenset()
        self.clock.reset()
        self.command = STOP
        self.events.clear()
        self.controller.reset(self.seed)
        self.events.emit(0.0, "sim", EventKind.RESET, f"reset: {self.room.name}, seed {self.seed}", seed=self.seed)
        logger.info("reset: room=%r seed=%d controller=%s", self.room.name, self.seed, self.controller.name)

    def set_controller(self, controller: Controller) -> None:
        """Swap the robot's software entirely (it starts from a clean state)."""
        self.controller = controller
        controller.reset(self.seed)
        self.command = STOP
        logger.info("controller -> %s", controller.name)

    # ------------------------------------------------------------------ stepping

    def sense(self) -> SensorFrame:
        """Build the frame handed to the controller; consumes the pending scan."""
        frame = SensorFrame(self.time, self.encoders.read(self.robot), self.bumper.read(), self._pending_scan)
        self._pending_scan = None
        return frame

    def step(self) -> None:
        """Advance the simulation by one ``dt``."""
        body = self.robot
        if self.clock.tick % self._lidar_every == 0:
            scan = self.lidar.scan(self.world, body.pose, self.time)
            self._pending_scan = scan
            self.last_scan, self.last_scan_pose = scan, body.pose
        if self.clock.tick % self._control_every == 0:
            self.command = self._sanitize(self.controller.update(self.sense(), self.control_period))

        dt = self.dt
        target = self.drivetrain.target_wheel_speeds(self.command)
        v_left, v_right = self.drivetrain.step((body.v_left, body.v_right), target, dt)
        # Wheel speeds change linearly within the step, so the trapezoid rule
        # gives each wheel's exact rotation (in nominal metres).
        d_left = 0.5 * (body.v_left + v_left) * dt
        d_right = 0.5 * (body.v_right + v_right) * dt
        ground_left, ground_right = self.ground.ground_travel(d_left, d_right, dt)
        start = body.pose
        proposed = integrate_wheel_travel(start, ground_left, ground_right, self.ground.wheel_base)
        result = self.solver.resolve(self.world, start, proposed)

        end = result.pose
        dx, dy = end.x - start.x, end.y - start.y
        mid_heading = start.theta + 0.5 * wrap_angle(end.theta - start.theta)
        body.linear_velocity = (dx * math.cos(mid_heading) + dy * math.sin(mid_heading)) / dt
        body.angular_velocity = wrap_angle(end.theta - start.theta) / dt
        body.odometer += math.hypot(dx, dy)
        body.pose = end
        body.v_left, body.v_right = v_left, v_right
        body.travel_left += d_left  # wheels keep turning when the body is blocked: they slip
        body.travel_right += d_right
        body.contacts = result.contacts
        body.blocked = result.blocked
        self.bumper.observe(body)
        self._record_collisions(result.contacts)
        self.clock.advance()

    def run(self, duration: float) -> None:
        """Step for ``duration`` seconds of simulation time."""
        for _ in range(round(duration / self.dt)):
            self.step()

    def _sanitize(self, command: DriveCommand) -> DriveCommand:
        if math.isfinite(command.linear) and math.isfinite(command.angular):
            return command
        logger.error("controller %s returned a non-finite command %s; stopping", self.controller.name, command)
        return STOP

    def _record_collisions(self, contacts: tuple[Contact, ...]) -> None:
        """Emit one COLLISION event whenever the body starts touching something new."""
        touching = frozenset("wall" if c.obstacle_id is None else c.obstacle_id for c in contacts)
        for key in sorted(touching - self._touching, key=str):
            obstacle_id: int | None = None
            if key == "wall":
                name = "wall"
            else:
                assert isinstance(key, int)
                obstacle_id = key
                name = self.world.obstacle(key).name
            self.events.emit(
                self.time, "sim", EventKind.COLLISION, f"collision with {name}", obstacle=name, obstacle_id=obstacle_id
            )
        self._touching = touching

    # ------------------------------------------------------------------ world editing

    def can_place(self, template: ObstacleTemplate, x: float, y: float) -> Placement:
        """Check whether ``template`` fits at (x, y) without touching anything."""
        shape = template.make_shape(x, y)
        if not self.world.is_shape_free(shape, template.z_min, template.z_max, tolerance=1e-4):
            return Placement(False, "no room there")
        if template.z_min < self.spec.height:
            pose = self.robot.pose
            if shapes_overlap(shape, Circle(pose.x, pose.y, self.spec.radius + 0.01)):
                return Placement(False, "that is where the robot is")
        return Placement(True, "ok")

    def place_obstacle(self, template: ObstacleTemplate, x: float, y: float) -> Placement:
        """Add a ``template`` obstacle at (x, y) if it fits."""
        check = self.can_place(template, x, y)
        if not check.ok:
            return check
        obstacle = self.world.add_obstacle(
            template.label, template.kind, template.make_shape(x, y),
            z_min=template.z_min, z_max=template.z_max, mobility=template.mobility,
        )
        self.events.emit(self.time, "user", EventKind.WORLD_EDIT, f"placed {obstacle.name}", obstacle=obstacle.name)
        logger.info("placed %s #%d at (%.2f, %.2f)", obstacle.name, obstacle.id, x, y)
        return Placement(True, "placed", obstacle)

    def remove_obstacle_at(self, x: float, y: float) -> list[Obstacle]:
        """Remove the topmost obstacle at (x, y) together with its furniture group."""
        hits = self.world.obstacles_at(x, y)
        dock_id = self.world.dock.obstacle_id if self.world.dock else None
        for hit in hits:
            if hit.id == dock_id:
                continue
            removed = [self.world.remove_obstacle(o.id) for o in self.world.group_members(hit)]
            self._touching = frozenset(k for k in self._touching if k not in {o.id for o in removed})
            self.events.emit(self.time, "user", EventKind.WORLD_EDIT, f"removed {hit.name}", obstacle=hit.name)
            logger.info("removed %s", ", ".join(o.name for o in removed))
            return removed
        return []
