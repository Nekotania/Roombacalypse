"""The simulation engine: one robot, one room, fixed timestep, no rendering.

Step order (every ``dt``):

1. if a control tick is due, build a :class:`SensorFrame` and ask the
   controller for a new :class:`DriveCommand` (held until the next tick);
2. the drivetrain moves the wheel speeds towards the command within limits;
3. the pose is integrated exactly from each wheel's rolling distance;
4. the contact solver resolves collisions, possibly pushing obstacles;
5. the body state and clock advance.

The engine is deterministic: the same room, config, seed and command sequence
give bit-identical trajectories (enforced by ``tests/test_simulation.py``).
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass

from roombacalypse.core.config import AppConfig
from roombacalypse.core.geometry import wrap_angle
from roombacalypse.core.interfaces import Controller
from roombacalypse.core.kinematics import integrate_wheel_travel
from roombacalypse.core.messages import STOP, DriveCommand, SensorFrame
from roombacalypse.core.rng import RngStreams
from roombacalypse.environment.catalog import ObstacleTemplate
from roombacalypse.environment.rooms import RoomSpec, build_world
from roombacalypse.environment.shapes import Circle, shapes_overlap
from roombacalypse.environment.world import Obstacle, World
from roombacalypse.robot.body import RobotBody
from roombacalypse.robot.drivetrain import Drivetrain
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

    def __init__(self, config: AppConfig, room: RoomSpec, controller: Controller, *, seed: int | None = None) -> None:
        self.config = config
        self.room = room
        self.spec = config.robot
        self.dt = config.simulation.dt
        self.control_period = config.simulation.control_period
        self._control_every = steps_per_period(self.control_period, self.dt)
        self.drivetrain = Drivetrain(self.spec)
        self.solver = ContactSolver(self.spec, config.physics)
        self.clock = SimClock(self.dt)
        self.controller = controller
        self.seed = config.simulation.seed if seed is None else seed
        # Assigned by reset():
        self.rng: RngStreams
        self.world: World
        self.robot: RobotBody
        self.command: DriveCommand = STOP
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
        """Rebuild the room from its description and put the robot back on the dock."""
        if seed is not None:
            self.seed = seed
        self.rng = RngStreams(self.seed)
        self.world = build_world(self.room, self.spec, self.config.dock)
        assert self.world.dock is not None
        self.robot = RobotBody(pose=self.world.dock.pose)
        self.clock.reset()
        self.command = STOP
        self.controller.reset()
        logger.info("reset: room=%r seed=%d controller=%s", self.room.name, self.seed, self.controller.name)

    def set_controller(self, controller: Controller) -> None:
        """Swap the robot's software (e.g. manual <-> autonomous)."""
        self.controller = controller
        controller.reset()
        self.command = STOP
        logger.info("controller -> %s", controller.name)

    # ------------------------------------------------------------------ stepping

    def sense(self) -> SensorFrame:
        """Build the frame handed to the controller. Sensors join in milestone 2."""
        return SensorFrame(time=self.time)

    def step(self) -> None:
        """Advance the simulation by one ``dt``."""
        if self.clock.tick % self._control_every == 0:
            self.command = self._sanitize(self.controller.update(self.sense(), self.control_period))

        body = self.robot
        dt = self.dt
        target = self.drivetrain.target_wheel_speeds(self.command)
        v_left, v_right = self.drivetrain.step((body.v_left, body.v_right), target, dt)
        # Wheel speeds change linearly within the step, so the trapezoid rule
        # gives each wheel's exact rolling distance.
        d_left = 0.5 * (body.v_left + v_left) * dt
        d_right = 0.5 * (body.v_right + v_right) * dt
        start = body.pose
        proposed = integrate_wheel_travel(start, d_left, d_right, self.spec.wheel_base)
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
            logger.info("removed %s", ", ".join(o.name for o in removed))
            return removed
        return []
