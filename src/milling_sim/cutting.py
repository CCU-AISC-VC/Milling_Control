"""First-order cutting-force and static-deflection models (docs/SPEC.md §8).

Force: specific cutting energy k_c [N/mm^2 = J/mm^3] times the material removal rate gives the
cutting power; divided by the cutting speed v_c = pi * D * n this is the tangential force F_t.
Radial and axial components are fixed ratios of F_t. Directions in the program/world frame:
tangential along -feed (resists the feed), radial = z x feed, axial = +z (pushes the tool up).

Deflection: the robot is a rigid chain with a torsional spring in each joint, K = diag(k_j).
The tool-tip compliance is C = J K^-1 J^T (6x6); the static tip deviation is C[:3,:] @ wrench.
Joint stiffness values are placeholders until measured (configs/robots/*.yaml, UNVERIFIED).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import yaml

from . import REPO_ROOT


@dataclass
class Material:
    name: str
    k_c: float  # N/mm^2
    radial_ratio: float
    axial_ratio: float
    density: float


def load_materials(path=REPO_ROOT / "configs/materials.yaml") -> tuple[dict[str, Material], str]:
    d = yaml.safe_load(open(path))
    mats = {k: Material(k, float(v["k_c_N_mm2"]), float(v["radial_ratio"]), float(v["axial_ratio"]), float(v.get("density_kg_m3", 0))) for k, v in d["materials"].items()}
    return mats, d.get("default", next(iter(mats)))


def cutting_force(mrr_mm3_s: np.ndarray, rpm: np.ndarray, feed_dir: np.ndarray, tool_diameter_m: float, mat: Material, up: np.ndarray = (0.0, 0.0, 1.0)) -> tuple[np.ndarray, np.ndarray]:
    """Per-sample force vector on the TOOL [N] (N x 3, world/program frame) and its magnitude.

    ``feed_dir``: unit feed direction per sample (N x 3). Samples with rpm <= 0 or MRR 0 get 0.
    """
    D_mm = tool_diameter_m * 1e3
    v_c = np.pi * D_mm * np.maximum(rpm, 0.0) / 60.0  # mm/s
    P = mat.k_c * np.maximum(mrr_mm3_s, 0.0)  # W  (N/mm^2 * mm^3/s = N*mm/s = mW... see note)
    # k_c [N/mm^2] * MRR [mm^3/s] = N*mm/s = 1e-3 W; keep everything in N and mm/s:
    F_t = np.where(v_c > 0, P / np.maximum(v_c, 1e-9), 0.0)  # N  (N*mm/s / (mm/s) = N)
    up = np.asarray(up, dtype=float)
    f = np.asarray(feed_dir, dtype=float)
    n = np.linalg.norm(f, axis=1, keepdims=True)
    f = np.where(n > 1e-12, f / np.maximum(n, 1e-12), 0.0)
    radial = np.cross(up, f)
    F = (-f) * F_t[:, None] + radial * (mat.radial_ratio * F_t)[:, None] + up[None, :] * (mat.axial_ratio * F_t)[:, None]
    return F, np.linalg.norm(F, axis=1)


def tip_jacobian(chain, q: np.ndarray, tool_axis_in_tcp: np.ndarray, tool_length: float) -> tuple[np.ndarray, np.ndarray]:
    """Geometric Jacobian at the tool tip and the tip position (base frame)."""
    T = chain.fk(q)
    J = chain.jacobian(q)
    a = T[:3, :3] @ tool_axis_in_tcp
    r = a * tool_length
    Jv = J[:3] - np.cross(r[:, None], J[3:], axis=0)
    return np.vstack([Jv, J[3:]]), T[:3, 3] + r


def static_deflection(chain, Q: np.ndarray, F_base: np.ndarray, k_joint: np.ndarray, tool_axis_in_tcp, tool_length: float) -> tuple[np.ndarray, np.ndarray]:
    """Static tip deviation [m] (N x 3) and joint torques [N m] (N x dof) for forces F (N x 3, base frame)."""
    Kinv = np.diag(1.0 / np.asarray(k_joint, dtype=float))
    dev = np.zeros((len(Q), 3))
    tau = np.zeros((len(Q), chain.dof))
    for i, (q, F) in enumerate(zip(Q, F_base)):
        J, _ = tip_jacobian(chain, q, tool_axis_in_tcp, tool_length)
        w = np.concatenate([F, np.zeros(3)])
        tau[i] = J.T @ w
        dev[i] = (J[:3] @ Kinv @ J.T) @ w
    return dev, tau
