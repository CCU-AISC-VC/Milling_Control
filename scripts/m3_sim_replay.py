#!/usr/bin/env python3
"""M3: replay one trajectory file in the Newton cell scene (tool-aware since M7); check TCP
tracking and collisions; record USD (for Isaac Sim RTX) and head-less GL videos.

Uses the shared multi-robot replay module (src/milling_sim/replay.py); the M6 script is the
same pipeline with several trajectories.

Run:  ~/envs/aisc_sim/bin/python scripts/m3_sim_replay.py [--traj outputs/m2_ik/sample_pocket_contour/trajectory]
Outputs: outputs/m3_sim/<stem>/
"""
from __future__ import annotations

import argparse
import json
import platform
import sys
from pathlib import Path

import _bootstrap  # noqa: F401
import numpy as np
import yaml

from milling_sim import OUTPUTS_DIR, REPO_ROOT
from milling_sim.cell import load_cell
from milling_sim.replay import PENETRATION_TOL, record_usd, render_videos, replay
from milling_sim.sim import build_cell_scene
from milling_sim.traj_io import read_trajectory
from milling_sim.viz import finish_figure, new_figure

DEFAULT_VIEWS = {"overview": ((3.6, -3.4, 2.4), (1.0, 0.0, 0.9)), "closeup": ((2.3, -1.1, 1.35), (1.42, -0.02, 0.76))}


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
    ap.add_argument("--video-speed", type=float, default=10.0)
    ap.add_argument("--video-fps", type=int, default=30)
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--no-usd", action="store_true")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()

    traj = read_trajectory(args.traj)
    meta = traj.meta
    cell = load_cell(args.cell or (REPO_ROOT / meta["cell"]))
    robot = args.robot or meta["robot"]
    stem = Path(meta["gcode"]).stem
    out = args.out or (OUTPUTS_DIR / "m3_sim" / stem)
    out.mkdir(parents=True, exist_ok=True)
    rc = cell.robot(robot).robot
    tip_offset = float(yaml.safe_load(rc.path.read_text()).get("mesh_tip_offset_mm", 0.0)) * 1e-3

    scene = build_cell_scene(cell, include_environment=True)
    res = replay(scene, cell, {robot: traj}, sim_hz=args.sim_hz, tip_offsets={robot: tip_offset})
    res.df.to_csv(out / "replay.csv", index=False)
    pr = res.per_robot[robot]
    df = res.df
    checks = {
        "n_frames": int(len(res.times)),
        "sim_hz": args.sim_hz,
        "replay_time_s": res.replay_time_s,
        "tcp_err_max_mm": pr["tcp_err_max_mm"],
        "tcp_err_mean_mm": pr["tcp_err_mean_mm"],
        "n_unexpected_collision_frames": len(res.unexpected_frames),
        "unexpected_first": res.unexpected_frames[:5],
        "spindle_stock_min_mm": pr["spindle_stock_min_mm"],
        "programmed_max_depth_mm": pr["programmed_max_depth_mm"],
        "expected_deepest_mm": pr["expected_deepest_mm"],
        "tool": meta.get("tool"),
        "other_pairs_min_mm": float(np.nanmin(df["other_mm"])) if df["other_mm"].notna().any() else None,
        "robot_env_min_mm": float(np.nanmin(df["robot_env_mm"])) if df["robot_env_mm"].notna().any() else None,
        "pair_min_mm": {f"{a} | {b}": round(d * 1e3, 3) for (a, b), d in sorted(res.pair_min.items())},
        "penetration_tol_mm": PENETRATION_TOL * 1e3,
    }
    gate = {"tcp_err_mm": 0.1, "unexpected": 0, "depth_tol_mm": 1.0}
    checks["pass"] = bool(checks["tcp_err_max_mm"] < gate["tcp_err_mm"] and checks["n_unexpected_collision_frames"] == 0 and abs(checks["spindle_stock_min_mm"] - checks["expected_deepest_mm"]) < gate["depth_tol_mm"])
    checks["gate"] = gate

    fig, ax = new_figure(3, 1, size=(12, 8))
    ax[0].plot(df.t, df[f"{robot}_tcp_err_mm"] * 1e3, lw=0.7)
    ax[0].set_ylabel("tool-tip tracking err [µm]")
    ax[0].set_title("Newton replay vs trajectory file")
    ax[1].plot(df.t, df[f"{robot}_spindle_stock_mm"], lw=0.7, label="tool/spindle - stock")
    ax[1].plot(df.t, df[f"{robot}_z_prog_mm"], lw=0.7, ls="--", label="programmed Z [mm]")
    ax[1].axhline(0, color="k", lw=0.5)
    ax[1].set_ylabel("[mm]")
    ax[1].legend(fontsize=8, loc="upper right")
    ax[2].plot(df.t, df["other_mm"], lw=0.7, label="other pairs")
    ax[2].plot(df.t, df["robot_env_mm"], lw=0.7, label="robot - environment")
    ax[2].set_ylabel("min clearance [mm]")
    ax[2].set_xlabel("t [s]")
    ax[2].legend(fontsize=8)
    finish_figure(fig, out / "tracking_clearance.png")

    T_end = res.times[-1]
    n_vid = int(np.ceil(T_end / args.video_speed * args.video_fps)) + 1
    t_vid = np.linspace(0.0, T_end, n_vid)
    media = {}
    if not args.no_usd:
        core = build_cell_scene(cell, include_environment=False)
        record_usd(core, {robot: traj}, t_vid, out / "scene.usd", args.video_fps)
        media["usd"] = rel(out / "scene.usd")
        media["usd_frames"] = n_vid
    if not args.no_video:
        views = {k: (tuple(v["pos"]), tuple(v["target"])) for k, v in cell.cameras.items()} or DEFAULT_VIEWS
        media.update(render_videos(scene, cell, {robot: traj}, t_vid, out, views, args.video_fps))

    summary = {"trajectory": rel(args.traj), "gcode": meta["gcode"], "cell": rel(cell.path), "robot": robot, "checks": checks, "media": media, "video_speed": args.video_speed, "env": {"python": platform.python_version(), "newton": __import__("newton").__version__}}
    (out / "summary.json").write_text(json.dumps(summary, indent=2, default=str))
    print(json.dumps({"checks": {k: v for k, v in checks.items() if k != "pair_min_mm"}, "pair_min_mm": checks["pair_min_mm"], "media": media}, indent=2, default=str))
    return 0 if checks["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
