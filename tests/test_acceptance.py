"""Milestone acceptance scenarios: long, deterministic, headless runs scored against ground truth.

Marked ``slow`` (about half an hour in total) and skipped by default; run with ``pytest -m slow``.
Thresholds come from measured runs with a safety margin; the measurements are in docs/ROADMAP.md.
Each run is scored once against the criteria of every milestone so far.
"""

from __future__ import annotations

import dataclasses

import pytest

from roombacalypse.autonomy.brain import Mode, RobotBrain
from roombacalypse.core.config import AppConfig, AutonomyConfig
from roombacalypse.core.events import EventKind, EventLog
from roombacalypse.core.paths import resolve_room_path
from roombacalypse.environment.rooms import load_room
from roombacalypse.evaluation.scenario import ScenarioResult, run_scenario
from roombacalypse.simulation.engine import Simulation

WANDER = dataclasses.replace(AppConfig(), autonomy=AutonomyConfig(behaviour="wander"))
"""M2's map-free wandering: the M2 and M3 criteria were set on it, and keep meaning the same."""

# Milestone 2: 10 minutes of autonomous wandering, 5 seeds per room.
M2_THRESHOLDS = {
    #               min coverage, max bumps/min, min distance [m]
    "test_chamber": (0.70, 0.2, 120.0),
    "studio": (0.60, 1.2, 120.0),
    "apartment": (0.35, 1.2, 120.0),
}

# Milestone 3: SLAM against ground truth. Lengths in metres. Also applied to the M4 coverage runs.
M3_THRESHOLDS = {
    #               online RMS, online max, keyframe ATE, aligned ATE, map precision, map recall
    "test_chamber": (0.015, 0.04, 0.015, 0.01, 0.98, 0.90),
    "studio": (0.025, 0.06, 0.03, 0.015, 0.98, 0.85),
    "apartment": (0.08, 0.15, 0.08, 0.04, 0.93, 0.82),  # the stress test: corridor, doorways, a 13 m loop
}

# Milestone 4: 30 minutes of the coverage executive, 5 seeds per room.
M4_THRESHOLDS = {
    #               min coverage of the accessible floor, max bumps/min, must declare the job done
    "test_chamber": (0.87, 0.2, True),
    "studio": (0.90, 0.5, True),  # the brief's criterion: >= 90 % of the studio in 30 minutes
    "apartment": (0.75, 0.6, False),  # about 60 m² in six rooms: still sweeping at 30 minutes
}
M4_RECALL = {"apartment": 0.77}
"""Map recall in the coverage runs, where it differs from M3's: measured 80.4-87.1 % in the apartment (wandering:
86.8-92 %). Lanes along walls see more surface at grazing angles, which the map represents less completely."""


def _check_slam(result: ScenarioResult, room_name: str, recall_floor: float | None = None) -> None:
    summary = result.summary()
    online_rms, online_max, ate, aligned, precision, recall = M3_THRESHOLDS[room_name]
    recall = recall if recall_floor is None else recall_floor
    assert result.online.rms_position <= online_rms, summary
    assert result.online.max_position <= online_max, summary
    assert result.keyframes.rms_position <= ate, summary
    assert result.keyframes.aligned_rms_position <= aligned, summary
    assert result.map.precision >= precision, summary
    assert result.map.recall >= recall, summary
    assert result.loop_closures > 0, summary
    assert result.relocalizations == 0, f"lost without being kidnapped\n{summary}"


@pytest.mark.slow
@pytest.mark.parametrize("seed", [1, 2, 3, 4, 5])
@pytest.mark.parametrize("room_name", sorted(M2_THRESHOLDS))
def test_ten_minutes_of_autonomous_wandering(room_name: str, seed: int) -> None:
    result = run_scenario(WANDER, load_room(resolve_room_path(room_name)), duration=600.0, seed=seed)
    summary = result.summary()

    min_coverage, max_bump_rate, min_distance = M2_THRESHOLDS[room_name]
    assert result.min_separation > -1e-9, f"the robot penetrated an obstacle\n{summary}"
    assert result.visible_collisions == 0, f"the robot hit something its LiDAR can see\n{summary}"
    assert result.distance >= min_distance, f"the robot got stuck\n{summary}"
    assert result.bumps_per_minute <= max_bump_rate, summary
    assert result.coverage >= min_coverage, summary
    _check_slam(result, room_name)


@pytest.mark.slow
@pytest.mark.parametrize("seed", [1, 2, 3, 4, 5])
@pytest.mark.parametrize("room_name", sorted(M4_THRESHOLDS))
def test_thirty_minutes_of_coverage(room_name: str, seed: int) -> None:
    result = run_scenario(AppConfig(), load_room(resolve_room_path(room_name)), duration=1800.0, seed=seed)
    summary = result.summary()

    min_coverage, max_bump_rate, must_finish = M4_THRESHOLDS[room_name]
    assert result.min_separation > -1e-9, f"the robot penetrated an obstacle\n{summary}"
    assert result.visible_collisions == 0, f"the robot hit something its LiDAR can see\n{summary}"
    assert result.coverage >= min_coverage, summary
    assert result.bumps_per_minute <= max_bump_rate, summary
    assert result.final_state not in ("trapped", "lost"), summary
    if must_finish:
        assert result.final_state == "done" and result.finished_at is not None, summary
    _check_slam(result, room_name, M4_RECALL.get(room_name))


# Milestone 4: the kidnapped robot. Five kidnappings per run, to random open floor, every 120 s from t = 300 s.
KIDNAP_RUNS = [("test_chamber", 1), ("test_chamber", 2), ("test_chamber", 3), ("studio", 1), ("studio", 2),
               ("studio", 3), ("apartment", 1), ("apartment", 2)]
KIDNAP_ERROR = {"test_chamber": 0.03, "studio": 0.05, "apartment": 0.15}  # [m] when found, and at the end


@pytest.mark.slow
@pytest.mark.parametrize(("room_name", "seed"), KIDNAP_RUNS)
def test_a_robot_kidnapped_five_times_always_finds_itself(room_name: str, seed: int) -> None:
    config = AppConfig()
    events = EventLog(100_000)
    brain = RobotBrain(config, publish=events.publisher("robot"), mode=Mode.AUTONOMOUS)
    sim = Simulation(config, load_room(resolve_room_path(room_name)), brain, seed=seed, events=events)
    kidnaps = [300.0 + 120.0 * k for k in range(5)]
    tolerance = KIDNAP_ERROR[room_name]
    found = 0
    while sim.time < 1000.0:
        if kidnaps and sim.time >= kidnaps[0]:
            kidnaps.pop(0)
            sim.kidnap()
        sim.step()
        if len(brain.slam.relocalizations) > found:
            found = len(brain.slam.relocalizations)
            error = sim.boot_pose.compose(brain.pose).distance_to(sim.robot.pose)
            assert error < tolerance, f"relocalised {100 * error:.0f} cm off at t = {sim.time:.1f} s"
    assert found >= 3  # some kidnappings land while still lost, or where the old pose still fits
    assert not brain.slam.lost
    assert sim.boot_pose.compose(brain.pose).distance_to(sim.robot.pose) < tolerance
    visible = [e for e in events.recent(100_000) if e.kind is EventKind.COLLISION
               and (e.data.get("obstacle_id") is None
                    or sim.world.obstacle(e.data["obstacle_id"]).overlaps_band(0.094, 0.096))]
    assert not visible
