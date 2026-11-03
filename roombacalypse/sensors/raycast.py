"""Vectorised 2D ray casting against line segments and circles.

All rays share one origin O and have unit directions d_i. For a segment
A + u·e (u in [0, 1]) the intersection O + t·d = A + u·e solves, with
w = A − O and the 2D cross product a × b = a.x·b.y − a.y·b.x,

    t = (w × e) / (d × e),    u = (w × d) / (d × e).

For a circle with centre C and radius r, |O + t·d − C|² = r² gives
t = −b − √(b² − c) with b = d·(O − C) and c = |O − C|² − r² (the near root).
Everything is computed for all ray/primitive pairs at once with NumPy; a
360-beam scan of a furnished room is well under a millisecond.
"""

from __future__ import annotations

import numpy as np

_PARALLEL_EPS = 1e-12


def cast_rays(
    ox: float,
    oy: float,
    dx: np.ndarray,
    dy: np.ndarray,
    seg_start: np.ndarray,
    seg_delta: np.ndarray,
    circle_centres: np.ndarray,
    circle_radii: np.ndarray,
) -> np.ndarray:
    """Distance from (ox, oy) along each unit direction to the first surface hit (``inf`` if none).

    Args:
        dx, dy: ray direction components, shape ``(N,)``, unit length.
        seg_start: segment start points, shape ``(M, 2)``.
        seg_delta: segment end minus start, shape ``(M, 2)``.
        circle_centres: shape ``(K, 2)``.
        circle_radii: shape ``(K,)``.
    """
    best = np.full(dx.shape[0], np.inf)
    if seg_start.shape[0]:
        wx = seg_start[:, 0] - ox
        wy = seg_start[:, 1] - oy
        ex = seg_delta[:, 0]
        ey = seg_delta[:, 1]
        denom = dx[:, None] * ey[None, :] - dy[:, None] * ex[None, :]
        parallel = np.abs(denom) < _PARALLEL_EPS
        safe = np.where(parallel, 1.0, denom)
        t = (wx * ey - wy * ex)[None, :] / safe
        u = (wx[None, :] * dy[:, None] - wy[None, :] * dx[:, None]) / safe
        hit = ~parallel & (t >= 0.0) & (u >= 0.0) & (u <= 1.0)
        best = np.minimum(best, np.where(hit, t, np.inf).min(axis=1))
    if circle_centres.shape[0]:
        fx = ox - circle_centres[:, 0]
        fy = oy - circle_centres[:, 1]
        b = dx[:, None] * fx[None, :] + dy[:, None] * fy[None, :]
        c = fx * fx + fy * fy - circle_radii * circle_radii
        disc = b * b - c[None, :]
        t = -b - np.sqrt(np.maximum(disc, 0.0))
        hit = (disc >= 0.0) & (t >= 0.0)
        best = np.minimum(best, np.where(hit, t, np.inf).min(axis=1))
    return best
