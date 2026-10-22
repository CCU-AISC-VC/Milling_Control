"""Independent URDF kinematics in NumPy (forward kinematics + geometric Jacobian).

This is deliberately *not* built on Newton: it is the reference implementation that
Newton's URDF import is checked against (M0), and it is what the IK in ``ik.py`` uses.
Only serial chains of revolute / fixed joints are supported, which covers the FANUC arm.

Conventions: quaternions are ``(x, y, z, w)`` (SciPy / Newton order); transforms are
4x4 homogeneous matrices; lengths in m, angles in rad.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R


def rpy_to_matrix(rpy) -> np.ndarray:
    """URDF roll-pitch-yaw (fixed-axis XYZ) to rotation matrix."""
    return R.from_euler("xyz", rpy).as_matrix()


def make_transform(xyz, rpy) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rpy_to_matrix(rpy)
    T[:3, 3] = xyz
    return T


def axis_angle_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R.from_rotvec(axis * angle).as_matrix()
    return T


@dataclass
class UrdfJoint:
    name: str
    type: str  # "revolute" | "fixed" | "continuous"
    parent: str
    child: str
    origin: np.ndarray  # 4x4
    axis: np.ndarray  # unit vector in the joint frame
    lower: float = -np.inf
    upper: float = np.inf
    velocity: float = np.inf


@dataclass
class SerialChain:
    """Ordered joints from ``base`` to ``tip`` (fixed joints included)."""

    base: str
    tip: str
    joints: list[UrdfJoint]
    link_names: list[str] = field(default_factory=list)

    @property
    def actuated(self) -> list[UrdfJoint]:
        return [j for j in self.joints if j.type in ("revolute", "continuous")]

    @property
    def dof(self) -> int:
        return len(self.actuated)

    @property
    def joint_names(self) -> list[str]:
        return [j.name for j in self.actuated]

    @property
    def lower(self) -> np.ndarray:
        return np.array([j.lower for j in self.actuated])

    @property
    def upper(self) -> np.ndarray:
        return np.array([j.upper for j in self.actuated])

    @property
    def velocity_limits(self) -> np.ndarray:
        return np.array([j.velocity for j in self.actuated])

    # ------------------------------------------------------------------ FK
    def link_transforms(self, q: np.ndarray) -> dict[str, np.ndarray]:
        """World transform of every link in the chain (base link at identity)."""
        q = np.asarray(q, dtype=float)
        assert q.shape == (self.dof,), f"expected {self.dof} joint values, got {q.shape}"
        T = np.eye(4)
        out = {self.base: T.copy()}
        k = 0
        for j in self.joints:
            T = T @ j.origin
            if j.type in ("revolute", "continuous"):
                T = T @ axis_angle_matrix(j.axis, q[k])
                k += 1
            out[j.child] = T.copy()
        return out

    def fk(self, q: np.ndarray, link: str | None = None) -> np.ndarray:
        """4x4 transform of ``link`` (default: tip) in the base frame."""
        return self.link_transforms(q)[link or self.tip]

    def fk_batch(self, Q: np.ndarray, link: str | None = None) -> np.ndarray:
        return np.stack([self.fk(q, link) for q in Q])

    # ------------------------------------------------------------ Jacobian
    def jacobian(self, q: np.ndarray, link: str | None = None) -> np.ndarray:
        """Geometric Jacobian (6 x dof) of ``link`` origin, expressed in the base frame.

        Rows 0-2: linear velocity, rows 3-5: angular velocity.
        """
        link = link or self.tip
        q = np.asarray(q, dtype=float)
        T = np.eye(4)
        axes, origins = [], []
        k = 0
        T_link = None
        for j in self.joints:
            T = T @ j.origin
            if j.type in ("revolute", "continuous"):
                axes.append(T[:3, :3] @ j.axis)
                origins.append(T[:3, 3].copy())
                T = T @ axis_angle_matrix(j.axis, q[k])
                k += 1
            if j.child == link:
                T_link = T.copy()
                break
        assert T_link is not None, f"link {link} not in chain"
        p = T_link[:3, 3]
        J = np.zeros((6, self.dof))
        for i, (z, o) in enumerate(zip(axes, origins)):
            J[:3, i] = np.cross(z, p - o)
            J[3:, i] = z
        return J


def _parse_vec(s: str | None, default="0 0 0") -> np.ndarray:
    return np.array([float(v) for v in (s or default).split()])


def load_chain(urdf_path: str | Path, base: str, tip: str) -> SerialChain:
    root = ET.parse(str(urdf_path)).getroot()
    joints_by_child: dict[str, UrdfJoint] = {}
    for jel in root.findall("joint"):
        o = jel.find("origin")
        xyz = _parse_vec(o.get("xyz") if o is not None else None)
        rpy = _parse_vec(o.get("rpy") if o is not None else None)
        a = jel.find("axis")
        axis = _parse_vec(a.get("xyz") if a is not None else None, "1 0 0")
        lim = jel.find("limit")
        j = UrdfJoint(
            name=jel.get("name"),
            type=jel.get("type"),
            parent=jel.find("parent").get("link"),
            child=jel.find("child").get("link"),
            origin=make_transform(xyz, rpy),
            axis=axis / np.linalg.norm(axis),
            lower=float(lim.get("lower", "-inf")) if lim is not None else -np.inf,
            upper=float(lim.get("upper", "inf")) if lim is not None else np.inf,
            velocity=float(lim.get("velocity", "inf")) if lim is not None else np.inf,
        )
        joints_by_child[j.child] = j
    # walk from tip up to base
    chain: list[UrdfJoint] = []
    link = tip
    while link != base:
        if link not in joints_by_child:
            raise ValueError(f"link {link!r} has no parent joint; cannot reach {base!r}")
        j = joints_by_child[link]
        chain.append(j)
        link = j.parent
    chain.reverse()
    return SerialChain(base=base, tip=tip, joints=chain, link_names=[base] + [j.child for j in chain])


def quat_xyzw_from_matrix(T: np.ndarray) -> np.ndarray:
    return R.from_matrix(T[:3, :3]).as_quat()


def pose_error(T_a: np.ndarray, T_b: np.ndarray) -> tuple[float, float]:
    """(position error [m], orientation error [rad]) between two 4x4 transforms."""
    dp = float(np.linalg.norm(T_a[:3, 3] - T_b[:3, 3]))
    dR = R.from_matrix(T_a[:3, :3].T @ T_b[:3, :3])
    return dp, float(np.linalg.norm(dR.as_rotvec()))
