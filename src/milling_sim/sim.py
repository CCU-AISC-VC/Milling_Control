"""Newton cell scene: robots (kinematic replay), table and stock boxes, collision queries.

The robot is *not* simulated dynamically: each frame we set the joint coordinates from the
trajectory file and run ``eval_fk``; the collision pipeline then reports contact candidates
between the robot's collision shapes and the static boxes. From each contact we compute a
signed distance along the contact normal (negative = penetration), aggregated per shape pair.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import warp as wp
from scipy.spatial.transform import Rotation as R

import newton

from .cell import CellConfig
from .newton_robot import RobotHandle, add_robot, matrix_from_body_q, xform_from_matrix
from .tool import rotation_z_to
from .urdf_kinematics import load_chain


@dataclass
class ContactSummary:
    pairs: dict[tuple[str, str], float]  # (role0, role1) -> min signed distance [m]
    n_contacts: int


@dataclass
class CellScene:
    model: newton.Model
    cell: CellConfig
    robots: dict[str, RobotHandle]
    shape_role: dict[int, str]  # shape index -> "robot/link" | "table" | "stock"
    static_shapes: dict[str, int]
    pipeline: newton.CollisionPipeline
    contacts: newton.Contacts
    state: newton.State
    _q: np.ndarray = field(default=None)
    _q_wp: wp.array = field(default=None)

    def __post_init__(self):
        self._q = np.zeros(self.model.joint_coord_count, dtype=np.float32)
        self._q_wp = wp.array(self._q, dtype=wp.float32)

    def set_q(self, robot: str, q: np.ndarray) -> None:
        self._q[self.robots[robot].q_slice] = np.asarray(q, dtype=np.float32)

    def eval_fk(self) -> None:
        self._q_wp.assign(self._q)
        newton.eval_fk(self.model, self._q_wp, self.model.joint_qd, self.state)

    def link_pose(self, robot: str, link: str) -> np.ndarray:
        return matrix_from_body_q(self.state.body_q.numpy()[self.robots[robot].body(link)])

    def body_poses(self) -> np.ndarray:
        bq = self.state.body_q.numpy()
        out = np.tile(np.eye(4), (len(bq), 1, 1))
        out[:, :3, :3] = R.from_quat(bq[:, 3:7]).as_matrix()
        out[:, :3, 3] = bq[:, :3]
        return out

    def collide(self) -> ContactSummary:
        """Run the collision pipeline on the current state and summarise signed distances."""
        self.pipeline.collide(self.state, self.contacts)
        c = self.contacts
        n = int(c.rigid_contact_count.numpy()[0])
        if n == 0:
            return ContactSummary({}, 0)
        s0 = c.rigid_contact_shape0.numpy()[:n]
        s1 = c.rigid_contact_shape1.numpy()[:n]
        p0 = c.rigid_contact_point0.numpy()[:n]
        p1 = c.rigid_contact_point1.numpy()[:n]
        nrm = c.rigid_contact_normal.numpy()[:n]
        sb = self.model.shape_body.numpy()
        T = self.body_poses()
        out: dict[tuple[str, str], float] = {}
        for k in range(n):
            b0, b1 = int(sb[s0[k]]), int(sb[s1[k]])
            w0 = (T[b0] @ np.append(p0[k], 1.0))[:3] if b0 >= 0 else p0[k]
            w1 = (T[b1] @ np.append(p1[k], 1.0))[:3] if b1 >= 0 else p1[k]
            d = float(np.dot(w1 - w0, nrm[k]))
            key = (self.shape_role.get(int(s0[k]), f"shape_{s0[k]}"), self.shape_role.get(int(s1[k]), f"shape_{s1[k]}"))
            out[key] = min(out.get(key, np.inf), d)
        return ContactSummary(out, n)


def _add_box(builder: newton.ModelBuilder, b) -> int:
    return builder.add_shape_box(
        -1,
        xform=wp.transform(wp.vec3(*[float(v) for v in b.center]), wp.quat_identity()),
        hx=float(b.size[0]) / 2,
        hy=float(b.size[1]) / 2,
        hz=float(b.size[2]) / 2,
        label=b.name,
        color=tuple(float(v) for v in b.color),
        opacity=float(b.opacity),
    )


def build_cell_scene(cell: CellConfig, rigid_contact_max: int = 8192, include_environment: bool = True) -> CellScene:
    """Robots + table + stock(s) (+ floor / fence / cabinets when ``include_environment``)."""
    builder = newton.ModelBuilder(up_axis=newton.Axis.Z)
    robots: dict[str, RobotHandle] = {}
    shape_role: dict[int, str] = {}
    for rp in cell.robots:
        s_start = builder.shape_count
        h = add_robot(builder, rp.robot.urdf, rp.name, base_xform=rp.base_xform, joint_names=tuple(rp.robot.joint_names))
        robots[rp.name] = h
        body_link = {v: k for k, v in h.link_body.items()}
        for s in range(s_start, builder.shape_count):
            shape_role[s] = f"{rp.name}/{body_link[builder.shape_body[s]]}"
        if rp.tool is not None:
            # cylinder from the spindle nose (tcp frame origin) to the tool tip, attached to the
            # link that carries the tcp frame; Newton cylinders extend along local Z
            tcp_link = rp.robot.tcp_link
            parent = None
            for link in reversed(load_chain(rp.robot.urdf, rp.robot.base_link, tcp_link).link_names):
                if link in h.link_body and link != tcp_link and builder.body_mass[h.link_body[link]] > 0:
                    parent = link
                    break
            parent = parent or "spindle_link"
            T_pl = load_chain(rp.robot.urdf, parent, tcp_link).fk(np.zeros(0))  # parent link <- tcp
            axis_pl = T_pl[:3, :3] @ rp.robot.tool_axis_in_tcp
            T_cyl = np.eye(4)
            T_cyl[:3, :3] = rotation_z_to(axis_pl)
            T_cyl[:3, 3] = T_pl[:3, 3] + axis_pl * rp.tool.length_from_nose / 2
            s = builder.add_shape_cylinder(h.link_body[parent], xform=xform_from_matrix(T_cyl), radius=rp.tool.radius, half_height=rp.tool.length_from_nose / 2, label=f"{rp.name}_tool", color=(0.25, 0.25, 0.28))
            shape_role[s] = f"{rp.name}/tool"
    static: dict[str, int] = {}
    tb = cell.table_box()
    if tb is not None:
        s = _add_box(builder, tb)
        static["table"] = s
        shape_role[s] = "table"
    for b in cell.stock_boxes_world():
        s = _add_box(builder, b)
        static[b.name] = s
        shape_role[s] = b.name  # "stock", "stock_2", ...
    if include_environment:
        for b in cell.environment_boxes():
            s = _add_box(builder, b)
            static[b.name] = s
            shape_role[s] = f"env/{b.name}"
    model = builder.finalize()
    pipeline = newton.CollisionPipeline(model, rigid_contact_max=rigid_contact_max, include_static_kinematic_pairs=True)
    contacts = pipeline.contacts()
    return CellScene(model, cell, robots, shape_role, static, pipeline, contacts, model.state())


def classify_contact(role0: str, role1: str, in_stock_allowed: bool, spindle_link: str = "spindle_link") -> str:
    """'expected' = spindle vs stock while the tool is allowed inside the stock (cutting moves and
    pure retracts along the tool axis); everything else that penetrates is 'unexpected'."""
    roles = {role0, role1}
    is_spindle = any(r.endswith("/" + spindle_link) for r in roles)
    if is_spindle and "stock" in roles and in_stock_allowed:
        return "expected"
    return "unexpected"


def in_stock_allowed_mask(motion: np.ndarray, tcp_prog: np.ndarray, segment: np.ndarray, tool_dir_prog=(0.0, 0.0, -1.0)) -> np.ndarray:
    """Per trajectory sample: may the tool be inside the stock? True for cutting samples and for
    the samples of a rapid *segment* that is a pure retract along the tool axis (G0 Z+ right
    after a cut). Samples are grouped by G-code segment, not by motion type, so a retract
    followed by a traverse is judged on its own."""
    n = len(motion)
    allowed = motion != "rapid"
    a = -np.asarray(tool_dir_prog, dtype=float)  # retract direction
    i = 0
    while i < n:
        if motion[i] == "rapid":
            j = i
            while j + 1 < n and segment[j + 1] == segment[i]:
                j += 1
            d = tcp_prog[j] - tcp_prog[i]
            L = np.linalg.norm(d)
            if L > 0 and np.dot(d / L, a) > 0.999 and i > 0 and allowed[i - 1]:
                allowed[i : j + 1] = True
            i = j + 1
        else:
            i += 1
    return allowed
