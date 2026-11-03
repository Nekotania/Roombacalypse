"""Milestone acceptance scenarios: long, deterministic, headless runs scored against ground truth.

Marked ``slow`` (about a minute in total) and skipped by default; run with ``pytest -m slow``.
Thresholds come from measured runs with a safety margin; the measurements are in docs/ROADMAP.md.
"""

from __future__ import annotations

import pytest

from roombacalypse.core.config import AppConfig
from roombacalypse.core.paths import resolve_room_path
from roombacalypse.environment.rooms import load_room
from roombacalypse.evaluation.scenario import run_scenario

# Milestone 2: 10 minutes of autonomous wandering, 5 seeds per built-in room.
M2_THRESHOLDS = {
    #               min coverage, max bumps/min, min distance [m]
    "test_chamber": (0.70, 0.2, 120.0),
    "studio": (0.60, 1.2, 120.0),
}


@pytest.mark.slow
@pytest.mark.parametrize("seed", [1, 2, 3, 4, 5])
@pytest.mark.parametrize("room_name", sorted(M2_THRESHOLDS))
def test_m2_ten_minutes_of_autonomous_wandering(room_name: str, seed: int) -> None:
    min_coverage, max_bump_rate, min_distance = M2_THRESHOLDS[room_name]
    result = run_scenario(AppConfig(), load_room(resolve_room_path(room_name)), duration=600.0, seed=seed)
    summary = result.summary()
    assert result.min_separation > -1e-9, f"the robot penetrated an obstacle\n{summary}"
    assert result.visible_collisions == 0, f"the robot hit something its LiDAR can see\n{summary}"
    assert result.distance >= min_distance, f"the robot got stuck\n{summary}"
    assert result.bumps_per_minute <= max_bump_rate, summary
    assert result.coverage >= min_coverage, summary
