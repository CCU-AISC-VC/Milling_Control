"""End-to-end pipeline functions used by the student UI (M8): G-code -> toolpath -> IK ->
Newton replay check -> material removal / force / deflection. Same algorithms as the stage
scripts (m1/m2/m3/m7); the scripts remain the reference runs with full outputs.
"""
from __future__ import annotations

import hashlib
import platform
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

import numpy as np
import yaml

from . import REPO_ROOT
from .cell import CellConfig
from .cutting import cutting_force, load_materials, static_deflection
from .gcode import Program, parse_gcode
from .ik import IKParams, ToolAxisIK
from .removal import Cutter, HeightMap, make_heightmap
from .tool import ToolConfig
from .toolpath import SamplingParams, Toolpath, sample_program
from .traj_io import JointTrajectory
from .urdf_kinematics import load_chain, quat_xyzw_from_matrix

Progress = Callable[[float, str], None]


def _noop(_p: float, _s: str) -> None:
    pass


@dataclass
class PlanResult:
    program: Program
    toolpath: Toolpath
    traj: JointTrajectory
    checks: dict
    P_base: np.ndarray
    seed_info: dict


def plan_trajectory(cell: CellConfig, robot: str, gcode_text: str, tool: ToolConfig | None, sampling: SamplingParams | None = None, rapid_feed_mm_min: float = 3000.0, gcode_name: str = "<ui>", seed: int = 0, progress: Progress = _noop) -> PlanResult:
    rp = cell.robot(robot)
    rc = rp.robot
    chain = load_chain(rc.urdf, rc.base_link, rc.tcp_link)
    tool_length = tool.length_from_nose if tool else 0.0
    progress(0.05, "解析 G-code")
    prog = parse_gcode(gcode_text, rapid_feed_mm_min=rapid_feed_mm_min, source=gcode_name)
    prm = sampling or SamplingParams()
    tp = sample_program(prog, prm)
    progress(0.15, f"取樣 {tp.n} 點，IK 中")
    T_bp = cell.program_in_base(robot)
    P = (T_bp[:3, :3] @ tp.p.T).T + T_bp[:3, 3]
    D = (T_bp[:3, :3] @ tp.tool_dir.T).T
    ik = ToolAxisIK(chain, rc.tool_axis_in_tcp, IKParams(), q_ref=rc.q_ref, tool_length=tool_length)
    q0, seed_info = ik.choose_seed(P[0], D[0], list(rc.named_states.values()), rng=np.random.default_rng(seed))
    res = ik.solve_path(P, D, q0)
    progress(0.55, "IK 完成，檢查中")
    Q = res.q
    Tfk = chain.fk_batch(Q)
    axis_fk = np.einsum("nij,j->ni", Tfk[:, :3, :3], ik.a_t)
    tcp_pos = Tfk[:, :3, 3] + axis_fk * tool_length
    tcp_quat = np.array([quat_xyzw_from_matrix(T) for T in Tfk])
    pos_err = np.linalg.norm(tcp_pos - P, axis=1)
    dir_err = np.arccos(np.clip(np.einsum("ni,ni->n", axis_fk, D), -1, 1))
    qd = np.gradient(Q, tp.t, axis=0)
    vel_ratio = np.abs(qd) / (chain.velocity_limits * rc.velocity_scale)
    dq_step = np.abs(np.diff(Q, axis=0))
    checks = {
        "fk_pos_err_max_mm": float(pos_err.max() * 1e3),
        "fk_dir_err_max_deg": float(np.degrees(dir_err.max())),
        "n_not_converged": int((~res.converged).sum()),
        "max_step_deg": float(np.degrees(dq_step.max())) if len(dq_step) else 0.0,
        "vel_ratio_max": float(vel_ratio.max()),
        "limit_margin_min": float(res.limit_margin.min()),
        "manipulability_min": float(res.manipulability.min()),
        "q_range_deg": {"min": np.degrees(Q.min(axis=0)).round(2).tolist(), "max": np.degrees(Q.max(axis=0)).round(2).tolist()},
    }
    checks["pass"] = bool(checks["fk_pos_err_max_mm"] < 0.05 and checks["fk_dir_err_max_deg"] < 0.05 and checks["n_not_converged"] == 0 and checks["max_step_deg"] < 5.0 and checks["vel_ratio_max"] <= 1.0)
    meta = {
        "gcode": gcode_name,
        "gcode_md5": hashlib.md5(gcode_text.encode()).hexdigest(),
        "cell": str(cell.path.relative_to(REPO_ROOT)) if cell.path and cell.path.is_relative_to(REPO_ROOT) else str(cell.path),
        "cell_overrides": {"programs": cell.programs, "stocks": cell.stocks, "workpiece": cell.workpiece},
        "robot": robot,
        "robot_config": str(rc.path.relative_to(REPO_ROOT)),
        "urdf": str(rc.urdf.relative_to(REPO_ROOT)),
        "tool_axis_in_tcp": rc.tool_axis_in_tcp.tolist(),
        "tool": tool.to_meta() if tool else None,
        "T_base_program": T_bp.round(9).tolist(),
        "sampling": {"max_spacing_mm": prm.max_spacing * 1e3, "chord_tol_mm": prm.chord_tol * 1e3, "accel_m_s2": prm.accel, "rapid_feed_mm_min": rapid_feed_mm_min},
        "ik": ik.prm.__dict__,
        "seed": {"seed": seed, "preferred_state": rc.preferred_state, **seed_info, "q0_deg": np.degrees(q0).round(3).tolist()},
        "checks": checks,
        "velocity_scale": rc.velocity_scale,
        "source": "milling_sim.pipeline.plan_trajectory",
        "env": {"python": platform.python_version(), "numpy": np.__version__},
    }
    traj = JointTrajectory(t=tp.t, q=Q, joint_names=rc.joint_names, tcp_pos=tcp_pos, tcp_quat_xyzw=tcp_quat, feed_mm_min=tp.feed * 6e4, spindle_rpm=tp.spindle_rpm, segment=tp.segment, motion=tp.motion, meta=meta)
    progress(0.6, "軌跡完成")
    return PlanResult(prog, tp, traj, checks, P, seed_info)


def check_replay(cell: CellConfig, trajs: dict[str, JointTrajectory], sim_hz: float = 30.0, progress: Progress = _noop) -> dict:
    """Newton kinematic replay with collision classification (no media). Returns the checks dict."""
    from .replay import replay
    from .sim import build_cell_scene

    progress(0.62, "建 Newton 場景")
    scene = build_cell_scene(cell, include_environment=True)
    tip_offsets = {}
    for n in trajs:
        rc = cell.robot(n).robot
        tip_offsets[n] = float(yaml.safe_load(rc.path.read_text()).get("mesh_tip_offset_mm", 0.0)) * 1e-3
    progress(0.65, f"Newton 重播 {sim_hz:g} Hz，碰撞檢查")
    res = replay(scene, cell, trajs, sim_hz=sim_hz, tip_offsets=tip_offsets)
    checks = {
        "n_frames": int(len(res.times)),
        "per_robot": res.per_robot,
        "n_unexpected_collision_frames": len(res.unexpected_frames),
        "unexpected_first": res.unexpected_frames[:5],
        "robot_robot_min_mm": float(np.nanmin(res.df["robot_robot_mm"])) if res.df["robot_robot_mm"].notna().any() else None,
        "robot_env_min_mm": float(np.nanmin(res.df["robot_env_mm"])) if res.df["robot_env_mm"].notna().any() else None,
        "other_min_mm": float(np.nanmin(res.df["other_mm"])) if res.df["other_mm"].notna().any() else None,
        "replay_time_s": res.replay_time_s,
    }
    ok = checks["n_unexpected_collision_frames"] == 0
    for pr in res.per_robot.values():
        ok &= pr["tcp_err_max_mm"] < 0.1 and abs(pr["spindle_stock_min_mm"] - pr["expected_deepest_mm"]) < 1.0
    if checks["robot_robot_min_mm"] is not None:
        ok &= checks["robot_robot_min_mm"] > res.extras.get("robot_robot_tol_mm", 5.0)
    checks["pass"] = bool(ok)
    progress(0.85, "重播完成")
    return checks


@dataclass
class RemovalResult:
    heightmap: HeightMap
    per_robot: dict
    total_volume_mm3: float
    max_depth_mm: float
    series: dict = field(default_factory=dict)  # robot -> dict of arrays (t, mrr, force, deflection)


def material_removal(cell: CellConfig, trajs: dict[str, JointTrajectory], tools: dict[str, ToolConfig], material: str | None = None, resolution_mm: float = 0.5, progress: Progress = _noop) -> RemovalResult:
    mats, default_mat = load_materials()
    mat = mats[material or default_mat]
    stock = cell.stock_boxes_world()[0]
    hm = make_heightmap(stock.center[:2], stock.size[:2], z_top=stock.center[2] + stock.size[2] / 2, height=stock.size[2], resolution=resolution_mm * 1e-3)
    cutters = {n: Cutter(hm, tools[n].radius, ball=(tools[n].type == "ball")) for n in trajs}
    events = []
    tip_world = {}
    dir_world = {}
    for n, tr in trajs.items():
        T_wb = cell.robot(n).base_xform
        tip_world[n] = (T_wb[:3, :3] @ tr.tcp_pos.T).T + T_wb[:3, 3]
        dir_world[n] = (T_wb[:3, :3] @ tr.tool_dirs(cell.robot(n).robot.tool_axis_in_tcp).T).T
        events += [(tr.t[i], n, i) for i in range(1, tr.n)]
    events.sort()
    vols = {n: np.zeros(trajs[n].n) for n in trajs}
    progress(0.87, "材料移除")
    for k, (t, n, i) in enumerate(events):
        tr = trajs[n]
        if tr.motion[i] != "rapid":
            vols[n][i] = cutters[n].sweep(tip_world[n][i - 1], tip_world[n][i], dir_world[n][i - 1], dir_world[n][i])
    per_robot, series = {}, {}
    for n, tr in trajs.items():
        rc = cell.robot(n).robot
        chain = load_chain(rc.urdf, rc.base_link, rc.tcp_link)
        k_joint = np.asarray(yaml.safe_load(rc.path.read_text()).get("joint_stiffness_Nm_rad", [1e6] * 6), dtype=float)
        dt = np.diff(tr.t, prepend=tr.t[0] - 1e-9)
        dt[0] = 1.0
        mrr = vols[n] * 1e9 / np.maximum(dt, 1e-9)
        feed_dir = np.diff(tip_world[n], axis=0, prepend=tip_world[n][:1])
        F_w, F_mag = cutting_force(mrr, tr.spindle_rpm, feed_dir, tools[n].diameter, mat)
        T_wb = cell.robot(n).base_xform
        F_b = (T_wb[:3, :3].T @ F_w.T).T
        dev, tau = static_deflection(chain, tr.q, F_b, k_joint, rc.tool_axis_in_tcp, tools[n].length_from_nose)
        cutting = vols[n] > 0
        per_robot[n] = {
            "tool": tools[n].name,
            "removed_volume_mm3": float(vols[n].sum() * 1e9),
            "cutting_time_s": float(dt[cutting].sum()),
            "mrr_max_mm3_s": float(mrr.max()),
            "mrr_mean_cutting_mm3_s": float(mrr[cutting].mean()) if cutting.any() else 0.0,
            "force_max_N": float(F_mag.max()),
            "deflection_max_mm": float(np.linalg.norm(dev, axis=1).max() * 1e3),
            "joint_torque_max_Nm": np.abs(tau).max(axis=0).round(2).tolist(),
        }
        series[n] = {"t": tr.t, "mrr": mrr, "force": F_mag, "force_vec_world": F_w, "deflection_mm": np.linalg.norm(dev, axis=1) * 1e3}
    Z = hm.numpy()
    progress(0.98, "材料移除完成")
    return RemovalResult(hm, per_robot, float(hm.removed_volume() * 1e9), float((hm.z_top - Z).max() * 1e3), series)
