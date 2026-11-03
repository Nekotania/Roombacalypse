"""World <-> screen mapping."""

from __future__ import annotations

import pytest

from roombacalypse.environment.shapes import AABB
from roombacalypse.visualization.camera import Camera

ROOM = AABB(0.0, 0.0, 8.0, 6.0)


def test_fit_centres_room_and_flips_y() -> None:
    camera = Camera.fit(ROOM, (0, 0, 900, 800), margin=0.0)
    assert camera.scale == pytest.approx(min(900 / 8, 800 / 6))
    assert camera.to_screen(4.0, 3.0) == pytest.approx((450.0, 400.0))
    _, top_y = camera.to_screen(4.0, 6.0)
    _, bottom_y = camera.to_screen(4.0, 0.0)
    assert top_y < bottom_y  # world y up -> screen y down


def test_round_trip() -> None:
    camera = Camera.fit(ROOM, (20, 10, 900, 800))
    for point in [(0.0, 0.0), (8.0, 6.0), (3.3, 1.7)]:
        assert camera.to_world(*camera.to_screen(*point)) == pytest.approx(point)


def test_zoom_keeps_point_under_cursor() -> None:
    camera = Camera.fit(ROOM, (0, 0, 900, 800))
    anchor = camera.to_world(200.0, 300.0)
    camera.zoom_at(1.7, 200.0, 300.0)
    assert camera.to_world(200.0, 300.0) == pytest.approx(anchor)


def test_pan_follows_drag() -> None:
    camera = Camera.fit(ROOM, (0, 0, 900, 800))
    before = camera.to_screen(2.0, 2.0)
    camera.pan(30.0, -20.0)
    after = camera.to_screen(2.0, 2.0)
    assert (after[0] - before[0], after[1] - before[1]) == pytest.approx((30.0, -20.0))
