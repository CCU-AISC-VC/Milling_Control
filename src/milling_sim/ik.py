"""Tool-axis IK for milling: position (3) + cutter-axis direction (2) = 5-DOF task.

Rotation about the cutter axis is free (symmetric tool), so the sixth joint-space direction
is used by a null-space term that keeps joints near the middle of their ranges. Each path
sample is solved with Levenberg-Marquardt seeded by the previous sample's solution, which
gives joint-space continuity for densely sampled paths.

Units: m, rad. The direction error is the rotation vector that aligns the current cutter
axis with the desired one; the two task rows are its components in the plane orthogonal
to the current axis.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .urdf_kinematics import SerialChain


@dataclass
class IKParams:
    pos_tol: float = 2e-6  # m
    dir_tol: float = 2e-5  # rad
    max_iters: int = 60
    lm_lambda: float = 1e-3
    w_pos: float = 1e3  # 1 mm -> 1
    w_dir: float = 180.0 / np.pi  # 1 deg -> 1
    step_max: float = 0.25  # rad per iteration
    k_null: float = 0.2  # null-space gain (toward the reference posture), per iteration
    limit_margin_frac: float = 0.02  # soft margin as a fraction of the range
    min_seed_margin: float = 0.1  # seeds whose solution is closer to a limit than this are rejected


@dataclass
class IKResult:
    q: np.ndarray  # (N, dof)
    pos_err: np.ndarray  # (N,) m
    dir_err: np.ndarray  # (N,) rad
    iters: np.ndarray  # (N,)
    converged: np.ndarray  # (N,) bool
    manipulability: np.ndarray  # (N,) sqrt(det(J J^T)) of the 6x6 Jacobian
    limit_margin: np.ndarray  # (N,) min over joints of distance to limit / half-range (1 = centre)
    info: dict = field(default_factory=dict)

    @property
    def all_converged(self) -> bool:
        return bool(np.all(self.converged))


def _perp_basis(a: np.ndarray) -> np.ndarray:
    t = np.array([1.0, 0.0, 0.0]) if abs(a[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    b1 = np.cross(a, t)
    b1 /= np.linalg.norm(b1)
    b2 = np.cross(a, b1)
    return np.stack([b1, b2], axis=1)  # 3x2


def _align_rotvec(a: np.ndarray, d: np.ndarray) -> tuple[np.ndarray, float]:
    """Rotation vector that rotates unit vector a onto unit vector d, and the angle."""
    c = np.cross(a, d)
    s = np.linalg.norm(c)
    ang = float(np.arctan2(s, float(np.dot(a, d))))
    if s < 1e-12:
        if ang > 1.0:  # anti-parallel: pick any perpendicular axis
            return _perp_basis(a)[:, 0] * ang, ang
        return np.zeros(3), ang
    return c / s * ang, ang


class ToolAxisIK:
    def __init__(self, chain: SerialChain, tool_axis_in_tcp: np.ndarray, params: IKParams | None = None, q_ref: np.ndarray | None = None):
        """``q_ref``: reference posture the free (null-space) motion is pulled toward and that
        seed selection prefers; defaults to the middle of the joint ranges."""
        self.chain = chain
        self.a_t = np.asarray(tool_axis_in_tcp, dtype=float)
        self.a_t /= np.linalg.norm(self.a_t)
        self.prm = params or IKParams()
        self.lower = chain.lower
        self.upper = chain.upper
        self.mid = (self.lower + self.upper) / 2
        self.half = (self.upper - self.lower) / 2
        self.dof = chain.dof
        self.q_ref = self.mid.copy() if q_ref is None else np.asarray(q_ref, dtype=float)

    # ---------------------------------------------------------------- helpers
    def tool_pose(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        T = self.chain.fk(q)
        return T[:3, 3], T[:3, :3] @ self.a_t, T

    def errors(self, q: np.ndarray, p_des: np.ndarray, d_des: np.ndarray) -> tuple[float, float]:
        p, a, _ = self.tool_pose(q)
        _, ang = _align_rotvec(a, d_des)
        return float(np.linalg.norm(p_des - p)), ang

    def limit_margin(self, q: np.ndarray) -> float:
        return float(np.min(1.0 - np.abs(q - self.mid) / self.half))

    def manipulability(self, q: np.ndarray) -> float:
        J = self.chain.jacobian(q)
        return float(np.sqrt(max(np.linalg.det(J @ J.T), 0.0)))

    # ---------------------------------------------------------------- single point
    def solve_point(self, p_des: np.ndarray, d_des: np.ndarray, q0: np.ndarray) -> tuple[np.ndarray, float, float, int, bool]:
        prm = self.prm
        q = np.array(q0, dtype=float)
        d_des = np.asarray(d_des, dtype=float) / np.linalg.norm(d_des)
        for it in range(1, prm.max_iters + 1):
            T = self.chain.fk(q)
            p = T[:3, 3]
            a = T[:3, :3] @ self.a_t
            e_p = p_des - p
            e_w, ang = _align_rotvec(a, d_des)
            if np.linalg.norm(e_p) < prm.pos_tol and ang < prm.dir_tol:
                return q, float(np.linalg.norm(e_p)), ang, it - 1, True
            J = self.chain.jacobian(q)
            B = _perp_basis(a)
            Jt = np.vstack([prm.w_pos * J[:3], prm.w_dir * (B.T @ J[3:])])  # 5 x dof
            et = np.concatenate([prm.w_pos * e_p, prm.w_dir * (B.T @ e_w)])
            A = Jt.T @ Jt
            A += prm.lm_lambda * (np.diag(np.diag(A)) + np.eye(self.dof))
            dq = np.linalg.solve(A, Jt.T @ et)
            # null-space: pull toward the reference posture (rotation about the cutter axis is free)
            Jp = np.linalg.solve(Jt @ Jt.T + 1e-6 * np.eye(Jt.shape[0]), Jt).T  # dof x 5 damped pseudo-inverse
            N = np.eye(self.dof) - Jp @ Jt
            dq += N @ (prm.k_null * (self.q_ref - q) / self.half)
            n = np.linalg.norm(dq, ord=np.inf)
            if n > prm.step_max:
                dq *= prm.step_max / n
            q = np.clip(q + dq, self.lower + prm.limit_margin_frac * self.half, self.upper - prm.limit_margin_frac * self.half)
        pe, ang = self.errors(q, p_des, d_des)
        return q, pe, ang, prm.max_iters, False

    # ---------------------------------------------------------------- seeds
    def choose_seed(self, p_des: np.ndarray, d_des: np.ndarray, seeds: list[np.ndarray], n_random: int = 32, rng=None) -> tuple[np.ndarray, dict]:
        """Solve the first point from several seeds; among converged solutions with a limit
        margin >= ``min_seed_margin``, keep the one closest (joint-space 2-norm) to ``q_ref``.

        Seeds: the given ones, a structured grid over wrist configurations around ``q_ref``
        (J4/J6 in {0, +-90, 180} deg, J5 in {-60, 0, +60} deg), then random ones."""
        rng = rng or np.random.default_rng(0)
        cands = list(seeds)
        if self.dof == 6:
            for j4 in (0.0, np.pi / 2, np.pi, -np.pi / 2):
                for j5 in (-1.0, 0.0, 1.0):
                    for j6 in (0.0, np.pi / 2, np.pi, -np.pi / 2):
                        q = self.q_ref.copy()
                        q[3:6] = [j4, j5, j6]
                        cands.append(np.clip(q, self.lower, self.upper))
        cands += [rng.uniform(self.lower, self.upper) for _ in range(n_random)]
        best, best_dist, n_ok, n_margin = None, np.inf, 0, 0
        for q0 in cands:
            q, pe, ang, it, ok = self.solve_point(p_des, d_des, q0)
            if not ok:
                continue
            n_ok += 1
            if self.limit_margin(q) < self.prm.min_seed_margin:
                continue
            n_margin += 1
            dist = float(np.linalg.norm(q - self.q_ref))
            if dist < best_dist:
                best, best_dist = q, dist
        if best is None:
            raise RuntimeError("no seed converged with enough limit margin for the first path point (unreachable pose?)")
        return best, {"n_candidates": len(cands), "n_converged": n_ok, "n_with_margin": n_margin, "dist_to_ref_deg": float(np.degrees(best_dist)), "limit_margin": self.limit_margin(best), "manipulability": self.manipulability(best)}

    # ---------------------------------------------------------------- path
    def solve_path(self, P: np.ndarray, D: np.ndarray, q0: np.ndarray) -> IKResult:
        P = np.asarray(P, dtype=float)
        D = np.asarray(D, dtype=float)
        n = len(P)
        Q = np.zeros((n, self.dof))
        pe = np.zeros(n)
        de = np.zeros(n)
        its = np.zeros(n, dtype=int)
        ok = np.zeros(n, dtype=bool)
        man = np.zeros(n)
        lm = np.zeros(n)
        q = np.array(q0, dtype=float)
        for i in range(n):
            q, pe[i], de[i], its[i], ok[i] = self.solve_point(P[i], D[i], q)
            Q[i] = q
            man[i] = self.manipulability(q)
            lm[i] = self.limit_margin(q)
        return IKResult(Q, pe, de, its, ok, man, lm)
