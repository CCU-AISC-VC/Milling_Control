#!/usr/bin/env python3
"""M3: replay a trajectory file in a Newton cell scene; check TCP tracking and collisions;
record USD (for Isaac Sim RTX) and head-less GL videos.

Run:  ~/envs/aisc_sim/bin/python scripts/m3_sim_replay.py [--traj outputs/m2_ik/sample_pocket_contour/trajectory]
Outputs: outputs/m3_sim/<stem>/
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
from milling_sim.sim import build_cell_scene, classify_contact, in_stock_allowed_mask
from milling_sim.traj_io import read_trajectory
from milling_sim.viz import HeadlessGL, finish_figure, new_figure, save_gif, save_mp4


PENETRATION_TOL = 5e-5  # m; touching surfaces report ~1e-10 m "penetration" from float noise


def rel(p) -> str:
    try:
        return str(Path(p).resolve().relative_to(REPO_ROOT))
    except ValueError:
        return str(p)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--traj", type=Path, default=OUTPUTS_DIR / "m2_ik/sample_pocket_contour/trajectory")
    ap.add_argument("--cell", type=Path, default=None, help="defaults to the cell recorded in meta.json")
    ap.add_argument("--robot", default=None)
    ap.add_argument("--sim-hz", type=float, default=60.0)
    ap.add_argument("--video-speed", type=float, default=10.0, help="time-lapse factor for the videos/USD")
    ap.add_argument("--video-fps", type=int, default=30)
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--no-usd", action="store_true")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()

    traj = read_trajectory(args.traj)
    meta = traj.meta
    cell_path = args.cell or (REPO_ROOT / meta["cell"])
    robot = args.robot or meta["robot"]
    stem = Path(meta["gcode"]).stem
    out = args.out or (OUTPUTS_DIR / "m3_sim" / stem)
    out.mkdir(parents=True, exist_ok=True)

    cell = load_cell(cell_path)
    rc = cell.robot(robot).robot
    scene = build_cell_scene(cell)
    tip_offset = float(getattr(rc, "mesh_tip_offset_m", 0.0))
    if "mesh_tip_offset_mm" in (yaml_raw := __import__("yaml").safe_load(rc.path.read_text())):
        tip_offset = float(yaml_raw["mesh_tip_offset_mm"]) * 1e-3

    # ---------------------------------------------------------------- replay
    dt = 1.0 / args.sim_hz
    T_end = traj.t[-1]
    times = np.arange(0.0, T_end + 1e-9, dt)
    Q = traj.interp_q(times)
    P_ref = np.stack([np.interp(times, traj.t, traj.tcp_pos[:, k]) for k in range(3)], axis=1)
    idx_sample = np.clip(np.searchsorted(traj.t, times, side="right") - 1, 0, traj.n - 1)
    motion = traj.motion[idx_sample]
    cutting = motion != "rapid"
    T_bp0 = np.asarray(meta["T_base_program"])
    tcp_prog_samples = (np.linalg.inv(T_bp0) @ np.c_[traj.tcp_pos, np.ones(traj.n)].T).T[:, :3]
    allowed_samples = in_stock_allowed_mask(traj.motion, tcp_prog_samples, traj.segment)
    # a frame between samples i and i+1 is allowed if either neighbour is (retract starts inside the stock)
    allowed = allowed_samples[idx_sample] | allowed_samples[np.clip(idx_sample + 1, 0, traj.n - 1)]
    T_wb = cell.robot(robot).base_xform
    P_ref_world = (T_wb[:3, :3] @ P_ref.T).T + T_wb[:3, 3]

    rows = []
    pair_min: dict[tuple[str, str], float] = {}
    unexpected_frames = []
    t0 = time.perf_counter()
    for i, t in enumerate(times):
        scene.set_q(robot, Q[i])
        scene.eval_fk()
        tcp = scene.link_pose(robot, rc.tcp_link)[:3, 3]
        err = float(np.linalg.norm(tcp - P_ref_world[i]))
        cs = scene.collide()
        d_spindle_stock = np.inf
        d_other = np.inf
        unexpected = []
        for (r0, r1), d in cs.pairs.items():
            pair_min[(r0, r1)] = min(pair_min.get((r0, r1), np.inf), d)
            roles = {r0, r1}
            if any(r.endswith("/spindle_link") for r in roles) and "stock" in roles:
                d_spindle_stock = min(d_spindle_stock, d)
            else:
                d_other = min(d_other, d)
            if d < -PENETRATION_TOL and classify_contact(r0, r1, bool(allowed[i])) == "unexpected":
                unexpected.append((r0, r1, d))
        if unexpected:
            unexpected_frames.append({"frame": i, "t": float(t), "pairs": [(a, b, float(d)) for a, b, d in unexpected]})
        rows.append({"t": t, "tcp_err_mm": err * 1e3, "n_contacts": cs.n_contacts, "d_spindle_stock_mm": d_spindle_stock * 1e3, "d_other_min_mm": d_other * 1e3, "cutting": bool(cutting[i]), "z_prog_mm": float(np.interp(t, traj.t, traj.tcp_pos[:, 2])) * 1e3})
    t_replay = time.perf_counter() - t0
    df = pd.DataFrame(rows)
    df.to_csv(out / "replay.csv", index=False)

    # programmed depth below the stock top (program z<0 while cutting) -> expected deepest penetration
    T_bp = np.asarray(meta["T_base_program"])
    z_prog = (np.linalg.inv(T_bp) @ np.c_[traj.tcp_pos, np.ones(traj.n)].T).T[:, 2]
    max_depth = float(-z_prog.min())
    expected_deepest = -(max_depth - tip_offset)
    d_ss = df.d_spindle_stock_mm.replace(np.inf, np.nan)
    checks = {
        "n_frames": int(len(times)),
        "sim_hz": args.sim_hz,
        "replay_time_s": t_replay,
        "tcp_err_max_mm": float(df.tcp_err_mm.max()),
        "tcp_err_mean_mm": float(df.tcp_err_mm.mean()),
        "n_unexpected_collision_frames": len(unexpected_frames),
        "unexpected_first": unexpected_frames[:5],
        "spindle_stock_min_mm": float(d_ss.min()),
        "programmed_max_depth_mm": max_depth * 1e3,
        "mesh_tip_offset_mm": tip_offset * 1e3,
        "expected_deepest_mm": expected_deepest * 1e3,
        "other_pairs_min_mm": float(df.d_other_min_mm.replace(np.inf, np.nan).min()),
        "pair_min_mm": {f"{a} | {b}": round(d * 1e3, 3) for (a, b), d in sorted(pair_min.items())},
    }
    gate = {"tcp_err_mm": 0.1, "unexpected": 0, "depth_tol_mm": 1.0, "penetration_tol_mm": PENETRATION_TOL * 1e3}
    checks["pass"] = bool(
        checks["tcp_err_max_mm"] < gate["tcp_err_mm"]
        and checks["n_unexpected_collision_frames"] == gate["unexpected"]
        and abs(checks["spindle_stock_min_mm"] - checks["expected_deepest_mm"]) < gate["depth_tol_mm"]
    )
    checks["gate"] = gate

    # ---------------------------------------------------------------- plots
    fig, ax = new_figure(3, 1, size=(12, 8))
    ax[0].plot(df.t, df.tcp_err_mm * 1e3, lw=0.7)
    ax[0].set_ylabel("TCP tracking err [µm]")
    ax[0].set_title("Newton replay vs trajectory file")
    ax[1].plot(df.t, df.d_spindle_stock_mm, lw=0.7, label="spindle - stock")
    ax[1].plot(df.t, df.z_prog_mm, lw=0.7, ls="--", label="programmed Z [mm]")
    ax[1].axhline(0, color="k", lw=0.5)
    ax[1].set_ylabel("[mm]")
    ax[1].legend(fontsize=8, loc="upper right")
    ax[2].plot(df.t, df.d_other_min_mm, lw=0.7)
    ax[2].set_ylabel("min clearance, other pairs [mm]")
    ax[2].set_xlabel("t [s]")
    finish_figure(fig, out / "tracking_clearance.png")

    # ---------------------------------------------------------------- videos + USD (time-lapse)
    n_vid = int(np.ceil(T_end / args.video_speed * args.video_fps)) + 1
    t_vid = np.linspace(0.0, T_end, n_vid)
    Qv = traj.interp_q(t_vid)
    world_path = (T_wb[:3, :3] @ traj.tcp_pos.T).T + T_wb[:3, 3]
    cut_mask = traj.motion != "rapid"
    media = {}
    if not args.no_usd:
        import newton.viewer as nv

        usd_path = out / "scene.usd"
        usd_tmp = out / "scene_tmp.usd"  # written here, then atomically moved over scene.usd
        usd = nv.ViewerUSD(str(usd_tmp), fps=args.video_fps, up_axis="Z", num_frames=n_vid)
        usd.set_model(scene.model)
        for k in range(n_vid):
            scene.set_q(robot, Qv[k])
            scene.eval_fk()
            usd.begin_frame(k / args.video_fps)
            usd.log_state(scene.state)
            usd.end_frame()
        usd.close()
        import os

        os.replace(usd_tmp, usd_path)
        media["usd"] = rel(usd_path)
        media["usd_frames"] = n_vid
    if not args.no_video:
        gl = HeadlessGL(scene.model, 1280, 720)
        views = {"overview": ((3.6, -3.4, 2.4), (1.0, 0.0, 0.9)), "closeup": ((2.3, -1.1, 1.35), (1.42, -0.02, 0.72))}
        for name, (pos, tgt) in views.items():
            frames = []
            gl.set_camera(pos, tgt)
            for k in range(n_vid):
                scene.set_q(robot, Qv[k])
                scene.eval_fk()
                j = int(np.searchsorted(traj.t, t_vid[k], side="right"))
                lines, points = [], []
                if j >= 2:
                    seg_cut = cut_mask[1:j] & cut_mask[: j - 1]
                    st = world_path[: j - 1][seg_cut]
                    en = world_path[1:j][seg_cut]
                    if len(st):
                        lines.append(("trail", st, en, (1.0, 0.45, 0.05)))
                tcp = scene.link_pose(robot, rc.tcp_link)[:3, 3]
                points.append(("tcp", [tcp], 0.006, (1.0, 0.9, 0.1)))
                rgb = gl.render(scene.state, points=points, lines=lines, dt=1 / args.video_fps)
                frames.append(rgb.copy())
            save_mp4(out / f"replay_{name}.mp4", frames, fps=args.video_fps)
            small = [f[::3, ::3] for f in frames[:: max(1, n_vid // 150)]]
            save_gif(out / f"replay_{name}.gif", small, fps=12)
            media[name] = {"mp4": rel(out / f"replay_{name}.mp4"), "gif": rel(out / f"replay_{name}.gif"), "frames": len(frames)}
            from milling_sim.viz import save_png

            save_png(out / f"frame_{name}_last.png", frames[-1])
            save_png(out / f"frame_{name}_mid.png", frames[len(frames) // 2])
        gl.close()

    summary = {
        "trajectory": rel(args.traj),
        "gcode": meta["gcode"],
        "cell": rel(cell_path),
        "robot": robot,
        "checks": checks,
        "media": media,
        "video_speed": args.video_speed,
        "env": {"python": platform.python_version(), "newton": __import__("newton").__version__},
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps({"checks": {k: v for k, v in checks.items() if k != "pair_min_mm"}, "pair_min_mm": checks["pair_min_mm"], "media": media}, indent=2))
    return 0 if checks["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
