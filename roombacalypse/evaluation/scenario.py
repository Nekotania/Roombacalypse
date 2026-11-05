"""Headless scenario runs: simulate, then score the robot against ground truth.

Used by ``main.py --headless``, by the acceptance tests and (M7) by benchmarks.
"""

from __future__ import annotations

import math
import time as wallclock
from dataclasses import dataclass

from roombacalypse.autonomy.brain import Mode, RobotBrain
from roombacalypse.core.config import AppConfig
from roombacalypse.core.events import EventKind, EventLog
from roombacalypse.environment.rooms import RoomSpec
from roombacalypse.evaluation.coverage import FootprintCoverage
from roombacalypse.evaluation.metrics import min_separation, pose_error
from roombacalypse.evaluation.slam_metrics import MapAccuracy, MapScore, TrajectoryError, TrajectoryRecorder
from roombacalypse.simulation.engine import Simulation


@dataclass(frozen=True)
class ScenarioResult:
    """Ground-truth scorecard of one run."""

    room: str
    seed: int
    mode: str
    duration: float
    """Simulated time [s]."""
    distance: float
    """True distance travelled [m]."""
    bumps: int
    """Bumper presses the robot noticed."""
    collisions: int
    """Contacts the simulator recorded (includes ones the bumper cannot feel, e.g. reversing)."""
    visible_collisions: int
    """Contacts with walls or obstacles the LiDAR can see. Avoidance should make this zero."""
    coverage: float
    """Fraction of the accessible floor swept by the footprint."""
    accessible_area: float
    """[m²]"""
    min_separation: float
    """Most negative body gap seen [m]; >= 0 (up to rounding) means nothing was ever penetrated."""
    odometry_position_error: float
    """Final dead-reckoning position error [m]."""
    odometry_heading_error: float
    """Final dead-reckoning heading error [rad]."""
    online: TrajectoryError
    """SLAM pose the robot used for control, at every control tick, against the truth."""
    keyframes: TrajectoryError
    """Final (optimised) keyframe poses against the truth."""
    map: MapScore
    """The robot's occupancy grid against the true scan-plane geometry."""
    keyframe_count: int
    loop_closures: int
    loop_attempts: int
    goals: int
    escapes: int
    abandoned: int
    wall_time: float
    """Real seconds the run took."""

    @property
    def bumps_per_minute(self) -> float:
        return self.bumps / (self.duration / 60.0) if self.duration > 0 else 0.0

    @property
    def realtime_factor(self) -> float:
        return self.duration / self.wall_time if self.wall_time > 0 else math.inf

    def summary(self) -> str:
        return "\n".join(
            [
                f"room            {self.room} (seed {self.seed}, {self.mode})",
                f"simulated       {self.duration:.0f} s in {self.wall_time:.1f} s wall time "
                f"(x{self.realtime_factor:.0f} real time)",
                f"distance        {self.distance:.1f} m",
                f"coverage        {100 * self.coverage:.1f} % of {self.accessible_area:.1f} m² accessible floor",
                f"bumps           {self.bumps} ({self.bumps_per_minute:.2f}/min); simulator saw {self.collisions} "
                f"contacts, {self.visible_collisions} with things the LiDAR can see",
                f"penetration     {'none' if self.min_separation > -1e-9 else f'{-self.min_separation:.2e} m'}",
                f"odometry drift  {self.odometry_position_error:.2f} m, "
                f"{math.degrees(self.odometry_heading_error):.1f}° after {self.duration:.0f} s",
                f"SLAM pose error {100 * self.online.rms_position:.1f} cm RMS, "
                f"{100 * self.online.max_position:.1f} cm max; "
                f"heading {math.degrees(self.online.rms_heading):.2f}° RMS, "
                f"{math.degrees(self.online.max_heading):.1f}° max",
                f"SLAM keyframes  {self.keyframe_count}, ATE {100 * self.keyframes.rms_position:.1f} cm RMS "
                f"({100 * self.keyframes.aligned_rms_position:.1f} cm aligned); "
                f"{self.loop_closures} loop closures in {self.loop_attempts} attempts",
                f"map             precision {100 * self.map.precision:.1f} % (surface only "
                f"{100 * self.map.surface_precision:.1f} %), recall {100 * self.map.recall:.1f} %, "
                f"{self.map.occupied_cells} occupied cells",
                f"behaviour       {self.goals} goals, {self.abandoned} abandoned, {self.escapes} bump escapes",
            ]
        )


def run_scenario(
    config: AppConfig,
    room: RoomSpec,
    *,
    duration: float,
    seed: int | None = None,
    mode: Mode = Mode.AUTONOMOUS,
    check_penetration: bool = True,
) -> ScenarioResult:
    """Simulate ``duration`` seconds without a window and score the run."""
    events = EventLog(capacity=100_000)
    brain = RobotBrain(config, publish=events.publisher("robot"), mode=mode)
    sim = Simulation(config, room, brain, seed=seed, events=events)
    coverage = FootprintCoverage(sim.world, config.robot, sim.robot.pose.position)
    recorder = TrajectoryRecorder(sim.boot_pose)
    map_accuracy = MapAccuracy(sim.world, config.lidar.mount_height, sim.boot_pose)
    control_every = round(sim.control_period / sim.dt)
    worst = math.inf
    started = wallclock.perf_counter()
    for _ in range(round(duration / sim.dt)):
        tick, time, before, scan = sim.tick, sim.time, sim.robot.pose, sim.last_scan
        sim.step()
        if tick % control_every == 0:  # the brain just updated from a frame taken at `time`
            recorder.record(time, before, brain.pose)
        if sim.last_scan is not scan and sim.last_scan is not None and sim.last_scan_pose is not None:
            map_accuracy.observe(sim.last_scan, sim.last_scan_pose)
        pose = sim.robot.pose
        coverage.mark(pose.x, pose.y)
        if check_penetration:
            worst = min(worst, min_separation(sim.world, pose, config.robot))
    elapsed = wallclock.perf_counter() - started
    slam = brain.slam
    scan_height = config.lidar.mount_height
    visible_collisions = 0
    for event in events.recent(events.total):
        if event.kind is EventKind.COLLISION:
            obstacle_id = event.data.get("obstacle_id")
            if obstacle_id is None:
                visible_collisions += 1  # walls span every height
            else:
                obstacle = sim.world.obstacle(obstacle_id)
                visible_collisions += obstacle.z_min <= scan_height < obstacle.z_max
    belief = sim.boot_pose.compose(brain.odometry.pose)
    position_error, heading_error = pose_error(belief, sim.robot.pose)
    return ScenarioResult(
        room=room.name,
        seed=sim.seed,
        mode=mode.value,
        duration=sim.time,
        distance=sim.robot.odometer,
        bumps=brain.bumps,
        collisions=events.count(EventKind.COLLISION),
        visible_collisions=visible_collisions,
        coverage=coverage.fraction,
        accessible_area=coverage.accessible_area,
        min_separation=worst,
        odometry_position_error=position_error,
        odometry_heading_error=heading_error,
        online=recorder.online_error(),
        keyframes=recorder.keyframe_error([k.time for k in slam.keyframes], slam.node_poses()),
        map=map_accuracy.score(slam.grid),
        keyframe_count=len(slam.keyframes),
        loop_closures=len(slam.loop_closures),
        loop_attempts=slam.loop_attempts,
        goals=brain.wander.goals_chosen,
        escapes=brain.wander.escapes,
        abandoned=brain.wander.abandoned,
        wall_time=elapsed,
    )
