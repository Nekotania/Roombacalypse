"""Pose-graph optimisation: the SLAM back end (belief side).

Nodes are robot poses ``x_i = (x, y, θ)`` in the map frame (one per keyframe).
An edge ``(i, j, z, Ω)`` says "seen from node i, node j is at relative pose
z", with information matrix Ω (inverse covariance). The optimiser finds the
poses minimising

    F(x) = Σ_edges ρ(e_ijᵀ Ω e_ij),    e_ij = t2v(Z⁻¹ · (X_i⁻¹ · X_j)),

where ``t2v`` turns a relative transform into ``(dx, dy, dθ)`` and ρ is the
identity for trusted edges and the Cauchy kernel ``c² log(1 + s/c²)`` for
loop closures, so a single wrong loop closure cannot wreck the map. This is
the formulation of Grisetti, Kümmerle, Stachniss & Burgard, "A Tutorial on
Graph-Based SLAM" (2010), solved with Levenberg-Marquardt on a sparse
system:

* Jacobians for all edges are computed at once (NumPy), their 3×3 blocks are
  assembled into a sparse ``H = Jᵀ Ω J`` (SciPy, duplicates summed) and
  ``(H + μ·diag H) Δ = −b`` is solved with SuperLU;
* the robust kernel is applied by iteratively reweighted least squares:
  ``Ω ← w·Ω`` with ``w = 1 / (1 + s/c²)``;
* node 0 (the boot pose on the dock) is held fixed: it *defines* the map frame.

Loop closures in an apartment-sized graph (about a thousand nodes after ten
minutes) optimise in tens of milliseconds.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy import sparse
from scipy.sparse import linalg as sparse_linalg

from roombacalypse.core.geometry import Pose2D, wrap_angles

_INITIAL_DAMPING = 1e-4
_MIN_STEP = 1e-5  # [m or rad] converged when no node moves more than this...
_MIN_IMPROVEMENT = 1e-6  # ...or the cost improves by less than this fraction


@dataclass(frozen=True)
class Edge:
    """One relative-pose constraint (read-only view for visualisation and tests)."""

    i: int
    j: int
    measurement: Pose2D
    information: np.ndarray
    robust: bool
    kind: str
    """``"sequential"`` (consecutive keyframes) or ``"loop"`` (loop closure)."""


@dataclass(frozen=True)
class OptimizationReport:
    iterations: int
    initial_cost: float
    final_cost: float
    max_correction: float
    """Largest change of any node's position during this optimisation [m]."""


def relative_errors(
    poses: np.ndarray, i: np.ndarray, j: np.ndarray, z: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Errors ``(E, 3)`` and Jacobians ``A = ∂e/∂x_i``, ``B = ∂e/∂x_j`` (each ``(E, 3, 3)``) of every edge."""
    xi, yi, ti = poses[i, 0], poses[i, 1], poses[i, 2]
    dx, dy = poses[j, 0] - xi, poses[j, 1] - yi
    ci, si = np.cos(ti), np.sin(ti)
    u = ci * dx + si * dy  # node j's position in node i's frame
    v = -si * dx + ci * dy
    cz, sz = np.cos(z[:, 2]), np.sin(z[:, 2])
    eu, ev = u - z[:, 0], v - z[:, 1]
    error = np.column_stack((cz * eu + sz * ev, -sz * eu + cz * ev, wrap_angles(poses[j, 2] - ti - z[:, 2])))

    count = i.shape[0]
    phi = ti + z[:, 2]
    cp, sp = np.cos(phi), np.sin(phi)
    a = np.zeros((count, 3, 3))
    b = np.zeros((count, 3, 3))
    # Q = R(z_θ)ᵀ R(θ_i)ᵀ = R(−φ); ∂e_xy/∂t_j = Q, ∂e_xy/∂t_i = −Q.
    b[:, 0, 0], b[:, 0, 1], b[:, 1, 0], b[:, 1, 1] = cp, sp, -sp, cp
    a[:, :2, :2] = -b[:, :2, :2]
    # ∂e_xy/∂θ_i = R(z_θ)ᵀ (v, −u).
    a[:, 0, 2] = cz * v - sz * u
    a[:, 1, 2] = -sz * v - cz * u
    a[:, 2, 2] = -1.0
    b[:, 2, 2] = 1.0
    return error, a, b


class PoseGraph:
    """Keyframe poses plus relative-pose constraints, optimised in place."""

    def __init__(self) -> None:
        self._poses = np.zeros((64, 3))
        self._count = 0
        self._i: list[int] = []
        self._j: list[int] = []
        self._z: list[tuple[float, float, float]] = []
        self._omega: list[np.ndarray] = []
        self._robust: list[bool] = []
        self._kind: list[str] = []

    # ------------------------------------------------------------------ building

    def clear(self) -> None:
        self._count = 0
        for store in (self._i, self._j, self._z, self._omega, self._robust, self._kind):
            store.clear()

    @property
    def node_count(self) -> int:
        return self._count

    @property
    def edge_count(self) -> int:
        return len(self._i)

    @property
    def poses(self) -> np.ndarray:
        """All node poses, shape ``(n, 3)`` (a view; do not modify)."""
        return self._poses[: self._count]

    def pose(self, index: int) -> Pose2D:
        x, y, theta = self._poses[index]
        return Pose2D(float(x), float(y), float(theta))

    def add_node(self, pose: Pose2D) -> int:
        if self._count == self._poses.shape[0]:
            self._poses = np.vstack((self._poses, np.zeros_like(self._poses)))
        self._poses[self._count] = (pose.x, pose.y, pose.theta)
        self._count += 1
        return self._count - 1

    def set_pose(self, index: int, pose: Pose2D) -> None:
        self._poses[index] = (pose.x, pose.y, pose.theta)

    def add_edge(
        self, i: int, j: int, measurement: Pose2D, information: np.ndarray, *, robust: bool = False,
        kind: str = "sequential",
    ) -> None:
        """Constrain node ``j`` to sit at ``measurement`` in node ``i``'s frame."""
        if not (0 <= i < self._count and 0 <= j < self._count and i != j):
            raise ValueError(f"bad edge {i} -> {j} for {self._count} nodes")
        omega = np.asarray(information, dtype=float)
        if omega.shape != (3, 3) or not np.allclose(omega, omega.T) or np.linalg.eigvalsh(omega).min() <= 0.0:
            raise ValueError("edge information must be a symmetric positive-definite 3x3 matrix")
        self._i.append(i)
        self._j.append(j)
        self._z.append((measurement.x, measurement.y, measurement.theta))
        self._omega.append(omega)
        self._robust.append(robust)
        self._kind.append(kind)

    def edges(self) -> list[Edge]:
        return [
            Edge(i, j, Pose2D(*z), omega, robust, kind)
            for i, j, z, omega, robust, kind in zip(
                self._i, self._j, self._z, self._omega, self._robust, self._kind, strict=True
            )
        ]

    # ------------------------------------------------------------------ optimisation

    def _arrays(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        return (
            np.asarray(self._i, dtype=np.intp),
            np.asarray(self._j, dtype=np.intp),
            np.asarray(self._z, dtype=float).reshape(-1, 3),
            np.asarray(self._omega, dtype=float).reshape(-1, 3, 3),
            np.asarray(self._robust, dtype=bool),
        )

    @staticmethod
    def _cost(error: np.ndarray, omega: np.ndarray, robust: np.ndarray, scale: float) -> tuple[float, np.ndarray]:
        """Total (robust) cost and the IRLS weight of every edge."""
        s = np.einsum("ei,eij,ej->e", error, omega, error)
        c2 = scale * scale
        weights = np.where(robust, 1.0 / (1.0 + s / c2), 1.0)
        costs = np.where(robust, c2 * np.log1p(s / c2), s)
        return float(costs.sum()), weights

    def cost(self, robust_scale: float = 2.0) -> float:
        """Current value of the objective."""
        if not self._i:
            return 0.0
        i, j, z, omega, robust = self._arrays()
        error, _, _ = relative_errors(self.poses, i, j, z)
        return self._cost(error, omega, robust, robust_scale)[0]

    def optimize(self, iterations: int = 15, robust_scale: float = 2.0) -> OptimizationReport:
        """Levenberg-Marquardt with node 0 fixed; returns what happened."""
        n = self._count
        if n < 2 or not self._i:
            return OptimizationReport(0, 0.0, 0.0, 0.0)
        i, j, z, omega, robust = self._arrays()
        poses = self.poses.copy()
        start = poses.copy()
        error, a, b = relative_errors(poses, i, j, z)
        cost, weights = self._cost(error, omega, robust, robust_scale)
        initial = cost
        damping = _INITIAL_DAMPING
        done = 0
        # Block (row, col) index patterns for the 3x3 blocks of every edge.
        offsets = np.arange(3)
        block_r = offsets[:, None].repeat(3, axis=1).ravel()
        block_c = offsets[None, :].repeat(3, axis=0).ravel()
        for iteration in range(1, iterations + 1):
            done = iteration
            w_omega = omega * weights[:, None, None]
            at_w = np.einsum("eki,ekl->eil", a, w_omega)  # Aᵀ Ω
            bt_w = np.einsum("eki,ekl->eil", b, w_omega)  # Bᵀ Ω
            blocks = (
                (i, i, np.einsum("eil,elj->eij", at_w, a)),
                (i, j, np.einsum("eil,elj->eij", at_w, b)),
                (j, i, np.einsum("eil,elj->eij", bt_w, a)),
                (j, j, np.einsum("eil,elj->eij", bt_w, b)),
            )
            # Node 0 is fixed, so its rows and columns are dropped (indices shift by 3).
            rows = np.concatenate([(3 * r - 3)[:, None] + block_r[None, :] for r, _, _ in blocks]).ravel()
            cols = np.concatenate([(3 * c - 3)[:, None] + block_c[None, :] for _, c, _ in blocks]).ravel()
            data = np.concatenate([m.reshape(-1, 9) for _, _, m in blocks]).ravel()
            keep = (rows >= 0) & (cols >= 0)
            free_h = sparse.coo_matrix((data[keep], (rows[keep], cols[keep])), shape=(3 * n - 3, 3 * n - 3)).tocsc()
            gradient = np.zeros(3 * n)
            np.add.at(gradient, (3 * i)[:, None] + offsets, np.einsum("eil,el->ei", at_w, error))
            np.add.at(gradient, (3 * j)[:, None] + offsets, np.einsum("eil,el->ei", bt_w, error))
            free_g = gradient[3:]
            diagonal = free_h.diagonal()
            improved = False
            previous = cost
            while damping < 1e8:
                system = (free_h + sparse.diags(damping * np.maximum(diagonal, 1e-9))).tocsc()
                step = sparse_linalg.spsolve(system, -free_g)
                trial = poses.copy()
                trial[1:] += step.reshape(-1, 3)
                trial[1:, 2] = wrap_angles(trial[1:, 2])
                t_error, t_a, t_b = relative_errors(trial, i, j, z)
                t_cost, t_weights = self._cost(t_error, omega, robust, robust_scale)
                if np.isfinite(t_cost) and t_cost <= cost:
                    poses, error, a, b, cost, weights = trial, t_error, t_a, t_b, t_cost, t_weights
                    damping = max(damping * 0.2, 1e-9)
                    improved = True
                    break
                damping *= 10.0
            if not improved or float(np.abs(step).max()) < _MIN_STEP or previous - cost < _MIN_IMPROVEMENT * previous:
                break
        self._poses[:n] = poses
        moved = float(np.hypot(*(poses[:, :2] - start[:, :2]).T).max())
        return OptimizationReport(done, initial, cost, moved)
