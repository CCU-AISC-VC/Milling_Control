#!/usr/bin/env python3
"""M9: Newton dynamics (MuJoCo-Warp) of the robot tracking a trajectory with joint PD actuators
under gravity, optionally with gravity feed-forward and the M7 cutting force applied at the
tool tip. Reports joint and tool-tip tracking errors and actuator torques.

Modes: pd (PD only) | pd_gff (PD + gravity feed-forward torque) | pd_gff_cut (+ cutting force)

Run:  ~/envs/aisc_sim/bin/python scripts/m9_dynamics.py [--traj ...] [--modes pd,pd_gff,pd_gff_cut] [--duration 60]
Outputs: outputs/m9_dynamics/<stem>/
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
import pandas as pd

from milling_sim import OUTPUTS_DIR, REPO_ROOT
from milling_sim.cell import load_cell
from milling_sim.dynamics import RobotDynamics, load_dyn_config
from milling_sim.tool import ToolConfig
from milling_sim.traj_io import read_trajectory
from milling_sim.viz import finish_figure, new_figure


def rel(p) -> str:
    try:
        return str(Path(p).resolve().relative_to(REPO_ROOT))
    except ValueError:
        return str(p)


def run_mode(mode: str, cell, robot: str, traj, cfg, tool_length: float, F_world: np.ndarray | None, settle: float, duration: float, log) -> tuple[pd.DataFrame, dict]:
    dyn = RobotDynamics(cell, robot, cfg)
    rp = cell.robot(robot)
    chain = dyn.chain
    a_t = rp.robot.tool_axis_in_tcp
    body_spindle = dyn.handle.body("spindle_link")
    T_wb = rp.base_xform
    n_sub = max(1, int(round(cfg.control_dt / cfg.sim_dt)))
    qd_ref_all = traj.qd()
    T_end = min(traj.t[-1], duration) if duration > 0 else traj.t[-1]
    ticks = np.arange(0.0, T_end + 1e-9, cfg.control_dt)
    q0 = traj.q[0]
    dyn.reset(q0)
    gff = mode in ("pd_gff", "pd_gff_cut")
    # settle at the first point
    for k in range(int(settle / cfg.control_dt)):
        tau_ff = -dyn.gravity_torque(q0) if gff else None
        dyn.set_target(q0, np.zeros(dyn.dof), tau_ff=tau_ff)
        dyn.step(n_sub)
    q_s, _ = dyn.joint_state()
    sag = chain.fk(q_s)[:3, 3] + (chain.fk(q_s)[:3, :3] @ a_t) * tool_length - (chain.fk(q0)[:3, 3] + (chain.fk(q0)[:3, :3] @ a_t) * tool_length)
    rows = []
    t0 = time.perf_counter()
    for i, t in enumerate(ticks):
        q_t = traj.interp_q(np.array([t]))[0]
        qd_t = np.array([np.interp(t, traj.t, qd_ref_all[:, j]) for j in range(dyn.dof)])
        tau_ff = -dyn.gravity_torque(q_t) if gff else None
        dyn.set_target(q_t, qd_t, tau_ff=tau_ff)
        body_force = None
        if mode == "pd_gff_cut" and F_world is not None:
            j = min(int(np.searchsorted(traj.t, t, side="right")), traj.n - 1)
            F = F_world[j]
            if np.linalg.norm(F) > 0:
                q_now, _ = dyn.joint_state()
                T_nose = T_wb @ chain.fk(q_now)
                p_tip = T_nose[:3, 3] + (T_nose[:3, :3] @ a_t) * tool_length
                body_force = (body_spindle, F, p_tip)
        dyn.step(n_sub, body_force=body_force)
        q, qd = dyn.joint_state()
        tau = dyn.pd_torque(q_t, qd_t, q, qd) + (tau_ff if tau_ff is not None else 0.0)
        T_sim = chain.fk(q)
        tip_sim = T_sim[:3, 3] + (T_sim[:3, :3] @ a_t) * tool_length
        tip_ref = np.array([np.interp(t, traj.t, traj.tcp_pos[:, k]) for k in range(3)])
        err = tip_sim - tip_ref
        row = {"t": t, "tip_err_mm": np.linalg.norm(err) * 1e3, "tip_err_z_mm": err[2] * 1e3}
        for j in range(dyn.dof):
            row[f"q{j+1}"] = q[j]
            row[f"e{j+1}_mdeg"] = np.degrees(q[j] - q_t[j]) * 1e3
            row[f"tau{j+1}"] = tau[j]
        rows.append(row)
        if i % 2500 == 0:
            log(f"  [{mode}] t={t:6.1f}/{T_end:.1f} s  tip err {row['tip_err_mm']:.3f} mm  ({time.perf_counter()-t0:.0f} s)")
        if not np.isfinite(q).all():
            log(f"  [{mode}] diverged at t={t:.3f} s")
            break
    df = pd.DataFrame(rows)
    info = {
        "mode": mode,
        "finite": bool(np.isfinite(df[[f"q{j+1}" for j in range(dyn.dof)]].to_numpy()).all()),
        "settle_sag_mm": (sag * 1e3).round(4).tolist(),
        "tip_err_max_mm": float(df.tip_err_mm.max()),
        "tip_err_mean_mm": float(df.tip_err_mm.mean()),
        "tip_err_z_min_mm": float(df.tip_err_z_mm.min()),
        "joint_err_max_mdeg": [float(df[f"e{j+1}_mdeg"].abs().max()) for j in range(dyn.dof)],
        "tau_max_Nm": [float(df[f"tau{j+1}"].abs().max()) for j in range(dyn.dof)],
        "wall_time_s": time.perf_counter() - t0,
        "n_ticks": int(len(df)),
    }
    return df, info


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--traj", type=Path, default=OUTPUTS_DIR / "m2_ik/sample_pocket_contour/trajectory")
    ap.add_argument("--cell", type=Path, default=None)
    ap.add_argument("--robot", default=None)
    ap.add_argument("--modes", default="pd,pd_gff,pd_gff_cut")
    ap.add_argument("--sim-dt", type=float, default=2e-3)
    ap.add_argument("--control-hz", type=float, default=250.0)
    ap.add_argument("--settle", type=float, default=1.0)
    ap.add_argument("--duration", type=float, default=0.0, help="simulate only the first N seconds (0 = whole trajectory)")
    ap.add_argument("--material", default=None)
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()

    traj = read_trajectory(args.traj)
    meta = traj.meta
    cell = load_cell(args.cell or (REPO_ROOT / meta["cell"]))
    robot = args.robot or meta["robot"]
    rc = cell.robot(robot).robot
    tool = ToolConfig.from_meta(meta.get("tool")) or cell.robot(robot).tool
    tool_length = tool.length_from_nose if tool else 0.0
    stem = Path(meta["gcode"]).stem
    out = args.out or (OUTPUTS_DIR / "m9_dynamics" / stem)
    out.mkdir(parents=True, exist_ok=True)
    modes = [m.strip() for m in args.modes.split(",") if m.strip()]

    F_world = None
    if "pd_gff_cut" in modes:
        from milling_sim.pipeline import material_removal

        rem = material_removal(cell, {robot: traj}, {robot: tool}, material=args.material, resolution_mm=1.0)
        F_world = rem.series[robot]["force_vec_world"]
        print(f"cutting force series: max {np.linalg.norm(F_world, axis=1).max():.1f} N", flush=True)

    cfg = load_dyn_config(rc.path, sim_dt=args.sim_dt, control_dt=1.0 / args.control_hz)
    results, dfs = {}, {}
    for mode in modes:
        print(f"=== {mode} ===", flush=True)
        df, info = run_mode(mode, cell, robot, traj, cfg, tool_length, F_world, args.settle, args.duration, lambda s: print(s, flush=True))
        df.to_csv(out / f"dyn_{mode}.csv", index=False)
        results[mode] = info
        dfs[mode] = df
        print(json.dumps({k: (round(v, 4) if isinstance(v, float) else v) for k, v in info.items()}), flush=True)

    gate = {"finite": True, "pd_gff_tip_err_max_mm": 1.0}
    ok = all(r["finite"] for r in results.values())
    if "pd_gff" in results:
        ok &= results["pd_gff"]["tip_err_max_mm"] < gate["pd_gff_tip_err_max_mm"]
    summary = {
        "trajectory": rel(args.traj),
        "gcode": meta.get("gcode"),
        "cell": rel(cell.path),
        "robot": robot,
        "tool": meta.get("tool"),
        "dynamics": {"solver": "SolverMuJoCo (implicitfast)", "sim_dt": cfg.sim_dt, "control_hz": args.control_hz, "kp": cfg.kp.tolist(), "kd": cfg.kd.tolist(), "effort_limit_Nm": cfg.effort_limit, "gravity": cfg.gravity, "masses_kg_from_urdf_import": True, "note": "masses/inertias derived from collision meshes (UNVERIFIED); gains tuned for simulation only"},
        "modes": results,
        "pass": bool(ok),
        "gate": gate,
        "env": {"python": platform.python_version(), "newton": __import__("newton").__version__},
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=2))

    # --- plots
    fig, ax = new_figure(3, 1, size=(12, 9))
    for mode, df in dfs.items():
        ax[0].plot(df.t, df.tip_err_mm, lw=0.7, label=mode)
        ax[1].plot(df.t, df.tip_err_z_mm, lw=0.7, label=mode)
    ax[0].set_ylabel("tool-tip error [mm]")
    ax[0].set_title("Newton dynamics (MuJoCo-Warp): joint PD under gravity, tool-tip tracking error vs the planned trajectory")
    ax[0].legend(fontsize=8)
    ax[1].set_ylabel("tip error z [mm] (negative = sag)")
    ax[1].axhline(0, color="k", lw=0.5)
    ax[1].legend(fontsize=8)
    ref = dfs[modes[-1]]
    for j in range(6):
        ax[2].plot(ref.t, ref[f"tau{j+1}"], lw=0.6, label=f"J{j+1}")
    ax[2].set_ylabel(f"actuator torque [N m] ({modes[-1]})")
    ax[2].set_xlabel("t [s]")
    ax[2].legend(fontsize=7, ncol=6)
    finish_figure(fig, out / "tip_error_torque.png")

    fig, ax = new_figure(3, 2, size=(12, 8))
    for j in range(6):
        a = ax[j // 2, j % 2]
        for mode, df in dfs.items():
            a.plot(df.t, df[f"e{j+1}_mdeg"], lw=0.6, label=mode)
        a.set_ylabel(f"J{j+1} err [mdeg]")
    ax[2, 0].set_xlabel("t [s]")
    ax[2, 1].set_xlabel("t [s]")
    ax[0, 0].legend(fontsize=7)
    fig.suptitle("joint tracking error (sim - reference)")
    finish_figure(fig, out / "joint_errors.png")
    print(json.dumps({"pass": summary["pass"], "modes": {m: {k: r[k] for k in ("tip_err_max_mm", "tip_err_mean_mm", "settle_sag_mm", "wall_time_s")} for m, r in results.items()}}, indent=2))
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
