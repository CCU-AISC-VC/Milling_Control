#!/usr/bin/env python3
"""M2: G-code -> sampled TCP path -> cell placement -> tool-axis IK -> joint trajectory file.

Checks: IK residuals, joint-space continuity, velocity vs URDF limits x velocity_scale,
limit margin, manipulability, and a cross-check of the result with Newton's FK.

Run:  ~/envs/aisc_sim/bin/python scripts/m2_ik_trajectory.py [--gcode ...] [--cell ...]
Outputs: outputs/m2_ik/<stem>/ (plots, summary.json, trajectory/)
"""
from __future__ import annotations

import argparse
import json
import platform
import sys
import time
from pathlib import Path

import _bootstrap  # noqa: F401
import numpy as np

from milling_sim import ASSETS_DIR, OUTPUTS_DIR, REPO_ROOT
from milling_sim.cell import load_cell
from milling_sim.gcode import load_gcode
from milling_sim.ik import IKParams, ToolAxisIK
from milling_sim.toolpath import SamplingParams, sample_program
from milling_sim.traj_io import JointTrajectory, write_trajectory
from milling_sim.urdf_kinematics import load_chain, quat_xyzw_from_matrix
from milling_sim.viz import finish_figure, new_figure


def rel(p: Path) -> str:
    p = Path(p)
    try:
        return str(p.resolve().relative_to(REPO_ROOT))
    except ValueError:
        return str(p)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--gcode", type=Path, default=ASSETS_DIR / "gcode/sample_pocket_contour.nc")
    ap.add_argument("--cell", type=Path, default=REPO_ROOT / "configs/cells/single_robot_table.yaml")
    ap.add_argument("--robot", default="fanuc_1")
    ap.add_argument("--max-spacing-mm", type=float, default=1.0)
    ap.add_argument("--chord-tol-mm", type=float, default=0.02)
    ap.add_argument("--accel", type=float, default=1.0)
    ap.add_argument("--rapid-feed", type=float, default=3000.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--no-newton-check", action="store_true")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()
    out = args.out or (OUTPUTS_DIR / "m2_ik" / args.gcode.stem)
    out.mkdir(parents=True, exist_ok=True)

    cell = load_cell(args.cell)
    rp = cell.robot(args.robot)
    rc = rp.robot
    chain = load_chain(rc.urdf, rc.base_link, rc.tcp_link)

    # --- M1: parse + sample (program frame)
    prog = load_gcode(args.gcode, rapid_feed_mm_min=args.rapid_feed)
    prm = SamplingParams(max_spacing=args.max_spacing_mm * 1e-3, chord_tol=args.chord_tol_mm * 1e-3, accel=args.accel)
    tp = sample_program(prog, prm)

    # --- program -> base
    T_bp = cell.program_in_base(args.robot)
    P = (T_bp[:3, :3] @ tp.p.T).T + T_bp[:3, 3]
    D = (T_bp[:3, :3] @ tp.tool_dir.T).T

    # --- IK
    ik = ToolAxisIK(chain, rc.tool_axis_in_tcp, IKParams(), q_ref=rc.q_ref)
    seeds = list(rc.named_states.values())
    t0 = time.perf_counter()
    q0, seed_info = ik.choose_seed(P[0], D[0], seeds, rng=np.random.default_rng(args.seed))
    res = ik.solve_path(P, D, q0)
    t_ik = time.perf_counter() - t0

    # --- derived quantities and checks
    Q = res.q
    dq_step = np.abs(np.diff(Q, axis=0))
    qd = np.gradient(Q, tp.t, axis=0)
    qdd = np.gradient(qd, tp.t, axis=0)
    vlim = chain.velocity_limits * rc.velocity_scale
    vel_ratio = np.abs(qd) / vlim
    acc_ratio = np.abs(qdd) / rc.acceleration_limits
    # FK of the solution (independent chain) -> TCP pose columns of the trajectory file
    Tfk = chain.fk_batch(Q)
    tcp_pos = Tfk[:, :3, 3]
    tcp_quat = np.array([quat_xyzw_from_matrix(T) for T in Tfk])
    pos_err_fk = np.linalg.norm(tcp_pos - P, axis=1)
    axis_fk = np.einsum("nij,j->ni", Tfk[:, :3, :3], ik.a_t)
    dir_err_fk = np.arccos(np.clip(np.einsum("ni,ni->n", axis_fk, D), -1, 1))

    newton_check = None
    if not args.no_newton_check:
        from milling_sim.newton_robot import build_single_robot

        scene = build_single_robot(rc.urdf, name=args.robot)
        idx = np.arange(0, len(Q), max(1, len(Q) // 200))
        errs = []
        for i in idx:
            T = scene.fk(args.robot, Q[i], [rc.tcp_link])[rc.tcp_link]
            errs.append(np.linalg.norm(T[:3, 3] - P[i]))
        newton_check = {"n_checked": int(len(idx)), "pos_err_max_mm": float(np.max(errs) * 1e3)}

    checks = {
        "ik_pos_err_max_mm": float(res.pos_err.max() * 1e3),
        "ik_dir_err_max_deg": float(np.degrees(res.dir_err.max())),
        "fk_pos_err_max_mm": float(pos_err_fk.max() * 1e3),
        "fk_dir_err_max_deg": float(np.degrees(dir_err_fk.max())),
        "n_not_converged": int((~res.converged).sum()),
        "iters_mean": float(res.iters.mean()),
        "max_step_deg_per_joint": np.degrees(dq_step.max(axis=0)).round(4).tolist(),
        "max_step_deg": float(np.degrees(dq_step.max())),
        "vel_ratio_max_per_joint": vel_ratio.max(axis=0).round(4).tolist(),
        "vel_ratio_max": float(vel_ratio.max()),
        "acc_ratio_max_per_joint": acc_ratio.max(axis=0).round(4).tolist(),
        "limit_margin_min": float(res.limit_margin.min()),
        "manipulability_min": float(res.manipulability.min()),
        "q_range_deg": {"min": np.degrees(Q.min(axis=0)).round(2).tolist(), "max": np.degrees(Q.max(axis=0)).round(2).tolist()},
        "newton_fk_check": newton_check,
    }
    gate = {"pos_mm": 0.05, "dir_deg": 0.05, "max_step_deg": 5.0, "vel_ratio": 1.0}
    checks["pass"] = bool(
        checks["fk_pos_err_max_mm"] < gate["pos_mm"]
        and checks["fk_dir_err_max_deg"] < gate["dir_deg"]
        and checks["n_not_converged"] == 0
        and checks["max_step_deg"] < gate["max_step_deg"]
        and checks["vel_ratio_max"] <= gate["vel_ratio"]
    )
    checks["gate"] = gate

    # --- trajectory file (SPEC §5)
    meta = {
        "gcode": rel(args.gcode),
        "gcode_md5": prog.md5,
        "cell": rel(args.cell),
        "robot": args.robot,
        "robot_config": rel(rc.path),
        "urdf": rel(rc.urdf),
        "tool_axis_in_tcp": rc.tool_axis_in_tcp.tolist(),
        "T_base_program": T_bp.round(9).tolist(),
        "sampling": {"max_spacing_mm": args.max_spacing_mm, "chord_tol_mm": args.chord_tol_mm, "accel_m_s2": args.accel, "rapid_feed_mm_min": args.rapid_feed},
        "ik": ik.prm.__dict__,
        "seed": {"seed": args.seed, "preferred_state": rc.preferred_state, **seed_info, "q0_deg": np.degrees(q0).round(3).tolist()},
        "checks": checks,
        "velocity_scale": rc.velocity_scale,
        "ik_time_s": t_ik,
        "env": {"python": platform.python_version(), "numpy": np.__version__},
    }
    traj = JointTrajectory(
        t=tp.t,
        q=Q,
        joint_names=rc.joint_names,
        tcp_pos=tcp_pos,
        tcp_quat_xyzw=tcp_quat,
        feed_mm_min=tp.feed * 6e4,
        spindle_rpm=tp.spindle_rpm,
        segment=tp.segment,
        motion=tp.motion,
        meta=meta,
    )
    write_trajectory(out / "trajectory", traj)

    # --- plots
    t = tp.t
    fig, ax = new_figure(3, 2, size=(12, 8))
    for j in range(6):
        a = ax[j // 2, j % 2]
        a.plot(t, np.degrees(Q[:, j]), lw=0.8)
        a.axhline(np.degrees(chain.lower[j]), color="r", lw=0.6, ls=":")
        a.axhline(np.degrees(chain.upper[j]), color="r", lw=0.6, ls=":")
        a.set_ylabel(f"{rc.joint_names[j]} [deg]")
        lo, hi = np.degrees(Q[:, j].min()), np.degrees(Q[:, j].max())
        pad = max(5.0, 0.2 * (hi - lo))
        a.set_ylim(lo - pad, hi + pad)
    ax[2, 0].set_xlabel("t [s]")
    ax[2, 1].set_xlabel("t [s]")
    fig.suptitle(f"joint trajectory, {args.gcode.name} (dotted red = URDF limits, if within view)")
    finish_figure(fig, out / "joints.png")

    fig, ax = new_figure(2, 1, size=(12, 6))
    for j in range(6):
        ax[0].plot(t, vel_ratio[:, j], lw=0.7, label=rc.joint_names[j])
    ax[0].axhline(1.0, color="r", lw=0.8, ls="--")
    ax[0].set_ylabel(f"|qd| / (limit x {rc.velocity_scale})")
    ax[0].legend(ncol=6, fontsize=8, loc="upper right")
    ax[0].set_title("joint velocity relative to the allowed limit")
    for j in range(6):
        ax[1].plot(t, acc_ratio[:, j], lw=0.7)
    ax[1].axhline(1.0, color="r", lw=0.8, ls="--")
    ax[1].set_ylabel("|qdd| / acc limit (UNVERIFIED)")
    ax[1].set_xlabel("t [s]")
    finish_figure(fig, out / "joint_velocity.png")

    fig, ax = new_figure(4, 1, size=(12, 9))
    ax[0].plot(t, pos_err_fk * 1e6, lw=0.7)
    ax[0].set_ylabel("TCP pos err [µm]")
    ax[1].plot(t, np.degrees(dir_err_fk) * 1e3, lw=0.7)
    ax[1].set_ylabel("tool axis err [mdeg]")
    ax[2].plot(t, res.limit_margin, lw=0.7)
    ax[2].set_ylabel("min limit margin [-]")
    ax[2].set_ylim(0, 1)
    ax[3].plot(t, res.manipulability, lw=0.7)
    ax[3].set_ylabel("manipulability")
    ax[3].set_xlabel("t [s]")
    fig.suptitle("IK quality along the path")
    finish_figure(fig, out / "ik_quality.png")

    # TCP path in base frame, top + side
    fig, ax = new_figure(1, 2, size=(11, 5))
    cut = tp.is_cutting()
    ax[0].plot(P[:, 0], P[:, 1], color="0.7", lw=0.6)
    ax[0].scatter(P[cut, 0], P[cut, 1], c=P[cut, 2], s=1)
    ax[0].scatter([0], [0], marker="s", color="k", label="robot base")
    ax[0].set_aspect("equal")
    ax[0].set_xlabel("x_base [m]")
    ax[0].set_ylabel("y_base [m]")
    ax[0].legend(fontsize=8)
    ax[0].set_title("TCP path in base frame (top)")
    ax[1].plot(P[:, 0], P[:, 2], color="0.7", lw=0.6)
    ax[1].scatter(P[cut, 0], P[cut, 2], c=P[cut, 2], s=1)
    ax[1].set_xlabel("x_base [m]")
    ax[1].set_ylabel("z_base [m]")
    ax[1].set_title("side")
    finish_figure(fig, out / "tcp_path_base.png")

    summary = {k: meta[k] for k in ("gcode", "gcode_md5", "cell", "robot", "seed", "checks", "ik_time_s", "env")}
    summary["n_samples"] = int(tp.n)
    summary["duration_s"] = tp.duration
    summary["trajectory_dir"] = rel(out / "trajectory")
    (out / "summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps({"checks": checks, "seed": summary["seed"], "ik_time_s": round(t_ik, 2), "n_samples": summary["n_samples"]}, indent=2))
    return 0 if checks["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
