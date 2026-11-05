"""Milestone acceptance scenarios: long, deterministic, headless runs scored against ground truth.

Marked ``slow`` (about eight minutes in total) and skipped by default; run with ``pytest -m slow``.
Thresholds come from measured runs with a safety margin; the measurements are in docs/ROADMAP.md.
Each run is scored once against the criteria of every milestone so far.
"""

from __future__ import annotations

import pytest

from roombacalypse.core.config import AppConfig
from roombacalypse.core.paths import resolve_room_path
from roombacalypse.environment.rooms import load_room
from roombacalypse.evaluation.scenario import run_scenario

# Milestone 2: 10 minutes of autonomous wandering, 5 seeds per room.
M2_THRESHOLDS = {
    #               min coverage, max bumps/min, min distance [m]
    "test_chamber": (0.70, 0.2, 120.0),
    "studio": (0.60, 1.2, 120.0),
    "apartment": (0.35, 1.2, 120.0),
}

# Milestone 3: the same runs, SLAM against ground truth. Lengths in metres.
M3_THRESHOLDS = {
    #               online RMS, online max, keyframe ATE, aligned ATE, map precision, map recall
    "test_chamber": (0.015, 0.04, 0.015, 0.01, 0.98, 0.90),
    "studio": (0.025, 0.06, 0.03, 0.015, 0.98, 0.85),
    "apartment": (0.08, 0.15, 0.08, 0.04, 0.93, 0.82),  # the stress test: corridor, doorways, a 13 m loop
}


@pytest.mark.slow
@pytest.mark.parametrize("seed", [1, 2, 3, 4, 5])
@pytest.mark.parametrize("room_name", sorted(M2_THRESHOLDS))
def test_ten_minutes_of_autonomous_wandering(room_name: str, seed: int) -> None:
    result = run_scenario(AppConfig(), load_room(resolve_room_path(room_name)), duration=600.0, seed=seed)
    summary = result.summary()

    min_coverage, max_bump_rate, min_distance = M2_THRESHOLDS[room_name]
    assert result.min_separation > -1e-9, f"the robot penetrated an obstacle\n{summary}"
    assert result.visible_collisions == 0, f"the robot hit something its LiDAR can see\n{summary}"
    assert result.distance >= min_distance, f"the robot got stuck\n{summary}"
    assert result.bumps_per_minute <= max_bump_rate, summary
    assert result.coverage >= min_coverage, summary

    online_rms, online_max, ate, aligned, precision, recall = M3_THRESHOLDS[room_name]
    assert result.online.rms_position <= online_rms, summary
    assert result.online.max_position <= online_max, summary
    assert result.keyframes.rms_position <= ate, summary
    assert result.keyframes.aligned_rms_position <= aligned, summary
    assert result.map.precision >= precision, summary
    assert result.map.recall >= recall, summary
    assert result.loop_closures > 0, summary
