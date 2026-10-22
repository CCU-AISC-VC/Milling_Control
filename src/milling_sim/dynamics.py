"""Newton dynamics of the robot: joint PD position actuators, gravity, optional external
tool-tip force (M9). Solver: ``SolverMuJoCo`` (MuJoCo-Warp), contacts disabled (the cell's
collisions are checked kinematically in replay.py).

Link masses / inertias come from the URDF import (arm links have no <inertial>, so Newton derives
them from the collision meshes at default density; spindle 2.1 kg from the xacro): UNVERIFIED.
Gravity feed-forward: the generalized gravity torque tau_g(q) = sum_links J_com^T (m g) is
computed from the same masses/COMs with the NumPy chain; it is applied either as a joint
feed-forward torque (``control.joint_f``) or as an equivalent target offset (-tau_g / kp).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import warp as wp
import yaml

import newton
from newton import JointTargetMode

from .cell import CellConfig
from .newton_robot import add_robot
from .urdf_kinematics import SerialChain, load_chain


@dataclass
class DynConfig:
    kp: np.ndarray
    kd: np.ndarray
    sim_dt: float = 1e-3
    control_dt: float = 4e-3
    gravity: bool = True
    gravity_ff: str = "none"  # "none" | "torque" | "target_offset"
    integrator: str | None = "implicitfast"
    effort_limit: float = 1.0e5  # N m, placeholder (UNVERIFIED)


class RobotDynamics:
    def __init__(self, cell: CellConfig, robot: str, cfg: DynConfig):
        rp = cell.robot(robot)
        self.rc = rp.robot
        self.cfg = cfg
        self.chain: SerialChain = load_chain(self.rc.urdf, self.rc.base_link, self.rc.tcp_link)
        builder = newton.ModelBuilder(up_axis=newton.Axis.Z)
        if not cfg.gravity:
            builder.gravity = 0.0
        self.handle = add_robot(builder, self.rc.urdf, robot, base_xform=rp.base_xform, joint_names=tuple(self.rc.joint_names))
        dof = self.handle.dof
        for i in range(dof):
            builder.joint_target_ke[self.handle.q_start + i] = float(cfg.kp[i])
            builder.joint_target_kd[self.handle.q_start + i] = float(cfg.kd[i])
            builder.joint_target_mode[self.handle.q_start + i] = int(JointTargetMode.POSITION_VELOCITY)
            builder.joint_effort_limit[self.handle.q_start + i] = float(cfg.effort_limit)  # URDF has effort=0 -> MuJoCo rejects an empty force range
        self.model = builder.finalize()
        kw = {"disable_contacts": True}
        if cfg.integrator:
            kw["integrator"] = cfg.integrator
        self.solver = newton.solvers.SolverMuJoCo(self.model, **kw)
        self.state_0 = self.model.state()
        self.state_1 = self.model.state()
        self.control = self.model.control()
        self.contacts = None
        self.dof = dof
        self.q_slice = self.handle.q_slice
        # gravity model from the Newton masses / COMs (body frame) for the chain links
        self.body_mass = self.model.body_mass.numpy()
        self.body_com = self.model.body_com.numpy()
        self.g_vec = np.array([0.0, 0.0, -9.81 if cfg.gravity else 0.0])
        self.base_xform = rp.base_xform
        self._tau_ff = np.zeros(self.model.joint_dof_count, dtype=np.float32)

    # ------------------------------------------------------------------ gravity
    def gravity_torque(self, q: np.ndarray) -> np.ndarray:
        """Generalized gravity torque tau_g(q) [N m] (what gravity applies on each joint)."""
        Ts = self.chain.link_transforms(q)
        tau = np.zeros(self.dof)
        # joint axes/origins in base frame
        T = np.eye(4)
        axes, origins = [], []
        for j in self.chain.joints:
            T = T @ j.origin
            if j.type in ("revolute", "continuous"):
                axes.append(T[:3, :3] @ j.axis)
                origins.append(T[:3, 3].copy())
                from .urdf_kinematics import axis_angle_matrix

                T = T @ axis_angle_matrix(j.axis, q[len(axes) - 1])
        g_base = self.base_xform[:3, :3].T @ self.g_vec
        for link, Tl in Ts.items():
            if link not in self.handle.link_body:
                continue
            b = self.handle.link_body[link]
            m = self.body_mass[b]
            if m <= 0:
                continue
            p_com = Tl[:3, :3] @ self.body_com[b] + Tl[:3, 3]
            # joints that move this link: those before it in the chain
            k_link = self.chain.link_names.index(link)
            n_act = sum(1 for j in self.chain.joints[:k_link] if j.type in ("revolute", "continuous"))
            for i in range(n_act):
                tau[i] += np.dot(np.cross(axes[i], p_com - origins[i]), m * g_base)
        return tau

    # ------------------------------------------------------------------ stepping
    def reset(self, q: np.ndarray) -> None:
        qfull = self.model.joint_q.numpy()
        qfull[self.q_slice] = q
        self.model.joint_q.assign(qfull)
        self.state_0.joint_q.assign(qfull)
        self.state_0.joint_qd.zero_()
        newton.eval_fk(self.model, self.state_0.joint_q, self.state_0.joint_qd, self.state_0)
        self.set_target(q, np.zeros(self.dof))

    def set_target(self, q_t: np.ndarray, qd_t: np.ndarray | None = None, tau_ff: np.ndarray | None = None) -> None:
        qt = self.control.joint_target_q.numpy()
        qt[self.q_slice] = q_t
        self.control.joint_target_q.assign(qt)
        if self.control.joint_target_qd is not None:
            vt = self.control.joint_target_qd.numpy()
            vt[self.q_slice] = qd_t if qd_t is not None else 0.0
            self.control.joint_target_qd.assign(vt)
        if tau_ff is not None and self.control.joint_f is not None:
            self._tau_ff[:] = 0.0
            self._tau_ff[self.q_slice] = tau_ff
            self.control.joint_f.assign(self._tau_ff)

    def step(self, n_substeps: int = 1, body_force: tuple[int, np.ndarray, np.ndarray] | None = None) -> None:
        """Advance n_substeps of sim_dt. ``body_force``: (body index, force [N] world, point [m] world)."""
        for _ in range(n_substeps):
            self.state_0.clear_forces()
            if body_force is not None:
                b, F, p = body_force
                bq = self.state_0.body_q.numpy()[b]
                com_w = bq[:3] + _rot(bq[3:7]) @ self.body_com[b]
                tau = np.cross(p - com_w, F)
                f = self.state_0.body_f.numpy()
                f[b] = np.concatenate([F, tau]).astype(np.float32)
                self.state_0.body_f.assign(f)
            self.solver.step(self.state_0, self.state_1, self.control, self.contacts, self.cfg.sim_dt)
            self.state_0, self.state_1 = self.state_1, self.state_0

    def joint_state(self) -> tuple[np.ndarray, np.ndarray]:
        return self.state_0.joint_q.numpy()[self.q_slice].astype(float), self.state_0.joint_qd.numpy()[self.q_slice].astype(float)

    def pd_torque(self, q_t, qd_t, q, qd) -> np.ndarray:
        return self.cfg.kp * (q_t - q) + self.cfg.kd * (qd_t - qd)


def _rot(q_xyzw):
    from scipy.spatial.transform import Rotation as R

    return R.from_quat(q_xyzw).as_matrix()


def load_dyn_config(rc_path, **overrides) -> DynConfig:
    raw = yaml.safe_load(open(rc_path))
    g = raw.get("pd_gains", {})
    cfg = DynConfig(kp=np.asarray(g.get("kp", [1e5] * 6), dtype=float), kd=np.asarray(g.get("kd", [1e3] * 6), dtype=float))
    for k, v in overrides.items():
        setattr(cfg, k, v)
    return cfg
