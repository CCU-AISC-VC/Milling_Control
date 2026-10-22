"""Load robots from the flattened URDF into a Newton ``ModelBuilder`` and query FK.

Several robots can be added to the same builder (M6 two-robot cell); each one is tracked
by a :class:`RobotHandle` that remembers its body indices and joint-coordinate offsets so
code can address links by URDF name after ``builder.finalize()``.

Conventions: Newton ``body_q`` is ``(px, py, pz, qx, qy, qz, qw)``; we convert to 4x4.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import warp as wp
from scipy.spatial.transform import Rotation as R

import newton


@dataclass
class RobotHandle:
    name: str
    urdf: str
    body_start: int
    body_count: int
    q_start: int
    dof: int
    joint_names: list[str]
    link_body: dict[str, int] = field(default_factory=dict)
    base_xform: np.ndarray = field(default_factory=lambda: np.eye(4))

    def body(self, link: str) -> int:
        return self.link_body[link]

    @property
    def q_slice(self) -> slice:
        return slice(self.q_start, self.q_start + self.dof)


def xform_from_matrix(T: np.ndarray) -> wp.transform:
    q = R.from_matrix(T[:3, :3]).as_quat()  # xyzw
    return wp.transform(wp.vec3(*T[:3, 3].tolist()), wp.quat(*q.tolist()))


def matrix_from_body_q(bq: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R.from_quat(bq[3:7]).as_matrix()
    T[:3, 3] = bq[:3]
    return T


def add_robot(
    builder: newton.ModelBuilder,
    urdf_path: str | Path,
    name: str,
    base_xform: np.ndarray | None = None,
    joint_names: tuple[str, ...] = ("J1", "J2", "J3", "J4", "J5", "J6"),
    enable_self_collisions: bool = False,
) -> RobotHandle:
    """Add one fixed-base robot. Returns a handle valid after ``builder.finalize()``."""
    base_xform = np.eye(4) if base_xform is None else np.asarray(base_xform, dtype=float)
    body_start = builder.body_count
    q_start = len(builder.joint_q)
    builder.add_urdf(
        str(urdf_path),
        xform=xform_from_matrix(base_xform),
        floating=False,
        enable_self_collisions=enable_self_collisions,
    )
    body_count = builder.body_count - body_start
    dof = len(builder.joint_q) - q_start
    assert dof == len(joint_names), f"URDF has {dof} coordinates, expected {len(joint_names)}"
    link_body = {}
    for i in range(body_start, body_start + body_count):
        label = builder.body_label[i]  # e.g. "m710id_70_spindle/J1_link"
        link_body[label.split("/")[-1]] = i
    return RobotHandle(
        name=name,
        urdf=str(urdf_path),
        body_start=body_start,
        body_count=body_count,
        q_start=q_start,
        dof=dof,
        joint_names=list(joint_names),
        link_body=link_body,
        base_xform=base_xform,
    )


class KinematicScene:
    """A finalized Newton model holding one or more kinematically driven robots."""

    def __init__(self, model: newton.Model, robots: list[RobotHandle]):
        self.model = model
        self.robots = {r.name: r for r in robots}
        self.state = model.state()
        self._q = np.zeros(model.joint_coord_count, dtype=np.float32)
        self._q_wp = wp.array(self._q, dtype=wp.float32)

    def set_q(self, robot: str, q: np.ndarray) -> None:
        h = self.robots[robot]
        self._q[h.q_slice] = np.asarray(q, dtype=np.float32)

    def eval_fk(self) -> None:
        self._q_wp.assign(self._q)
        newton.eval_fk(self.model, self._q_wp, self.model.joint_qd, self.state)

    def link_pose(self, robot: str, link: str) -> np.ndarray:
        """4x4 world pose of ``link`` from the last ``eval_fk``."""
        h = self.robots[robot]
        bq = self.state.body_q.numpy()[h.body(link)]
        return matrix_from_body_q(bq)

    def fk(self, robot: str, q: np.ndarray, links: list[str]) -> dict[str, np.ndarray]:
        self.set_q(robot, q)
        self.eval_fk()
        return {l: self.link_pose(robot, l) for l in links}


def build_single_robot(urdf_path: str | Path, name: str = "robot", base_xform=None) -> KinematicScene:
    builder = newton.ModelBuilder(up_axis=newton.Axis.Z)
    h = add_robot(builder, urdf_path, name, base_xform=base_xform)
    model = builder.finalize()
    return KinematicScene(model, [h])
