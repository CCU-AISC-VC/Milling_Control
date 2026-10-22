#!/usr/bin/env python3
"""M6: replay two robots' trajectory files together in the dual cell (shared time base),
check tracking, robot-robot / robot-environment clearance and collisions, record USD and
GL videos.

Run:  ~/envs/aisc_sim/bin/python scripts/m6_dual_replay.py \
        --traj fanuc_1=outputs/m6_dual/traj_fanuc_1/trajectory --traj fanuc_2=outputs/m6_dual/traj_fanuc_2/trajectory
Outputs: outputs/m6_dual/
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


def rel(p) -> str:
    try:
        return str(Path(p).resolve().relative_to(REPO_ROOT))
    except ValueError:
        return str(p)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cell", type=Path, default=REPO_ROOT / "configs/cells/dual_robot_table.yaml")
    ap.add_argument("--traj", action="append", required=True, help="robot=path/to/trajectory (repeatable)")
    ap.add_argument("--sim-hz", type=float, default=60.0)
    ap.add_argument("--video-speed", type=float, default=10.0)
    ap.add_argument("--video-fps", type=int, default=30)
    ap.add_argument("--views", default="overview,closeup,side")
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--no-usd", action="store_true")
    ap.add_argument("--out", type=Path, default=OUTPUTS_DIR / "m6_dual")
    args = ap.parse_args()
    out: Path = args.out
    out.mkdir(parents=True, exist_ok=True)

    cell = load_cell(args.cell)
    trajs = {}
    for item in args.traj:
        name, path = item.split("=", 1)
        trajs[name] = read_trajectory(path)
    tip_offsets = {}
    for n in trajs:
        rc = cell.robot(n).robot
        tip_offsets[n] = float(yaml.safe_load(rc.path.read_text()).get("mesh_tip_offset_mm", 0.0)) * 1e-3

    scene = build_cell_scene(cell, include_environment=True)
    res = replay(scene, cell, trajs, sim_hz=args.sim_hz, tip_offsets=tip_offsets)
    res.df.to_csv(out / "replay.csv", index=False)

    checks = {
        "n_frames": int(len(res.times)),
        "sim_hz": args.sim_hz,
        "replay_time_s": res.replay_time_s,
        "per_robot": res.per_robot,
        "n_unexpected_collision_frames": len(res.unexpected_frames),
        "unexpected_first": res.unexpected_frames[:5],
        "robot_robot_min_mm": float(np.nanmin(res.df["robot_robot_mm"])) if res.df["robot_robot_mm"].notna().any() else None,
        "robot_env_min_mm": float(np.nanmin(res.df["robot_env_mm"])) if res.df["robot_env_mm"].notna().any() else None,
        "other_min_mm": float(np.nanmin(res.df["other_mm"])) if res.df["other_mm"].notna().any() else None,
        "pair_min_mm": {f"{a} | {b}": round(d * 1e3, 3) for (a, b), d in sorted(res.pair_min.items()) if d < 0.5},
        "penetration_tol_mm": PENETRATION_TOL * 1e3,
        **res.extras,
    }
    gate = {"tcp_err_mm": 0.1, "unexpected": 0, "depth_tol_mm": 1.0, "robot_robot_min_mm": res.extras.get("robot_robot_tol_mm", 5.0)}
    ok = checks["n_unexpected_collision_frames"] == 0
    for n, pr in res.per_robot.items():
        ok &= pr["tcp_err_max_mm"] < gate["tcp_err_mm"]
        ok &= abs(pr["spindle_stock_min_mm"] - pr["expected_deepest_mm"]) < gate["depth_tol_mm"]
    if checks["robot_robot_min_mm"] is not None:
        ok &= checks["robot_robot_min_mm"] > gate["robot_robot_min_mm"]
    checks["pass"] = bool(ok)
    checks["gate"] = gate

    # ---- plots
    df = res.df
    fig, ax = new_figure(3, 1, size=(12, 9))
    for n in trajs:
        ax[0].plot(df.t, df[f"{n}_tcp_err_mm"] * 1e3, lw=0.7, label=n)
    ax[0].set_ylabel("TCP tracking err [µm]")
    ax[0].legend(fontsize=8)
    ax[0].set_title("dual-robot replay: tracking, spindle depth, clearance")
    for n in trajs:
        ax[1].plot(df.t, df[f"{n}_spindle_stock_mm"], lw=0.7, label=f"{n} spindle-stock")
        ax[1].plot(df.t, df[f"{n}_z_prog_mm"], lw=0.6, ls="--", label=f"{n} programmed Z")
    ax[1].axhline(0, color="k", lw=0.5)
    ax[1].set_ylabel("[mm]")
    ax[1].legend(fontsize=7, ncol=2, loc="upper right")
    ax[2].plot(df.t, df["robot_robot_mm"], lw=0.8, label="robot-robot")
    ax[2].plot(df.t, df["robot_env_mm"], lw=0.8, label="robot-fence/floor/cabinet")
    ax[2].plot(df.t, df["other_mm"], lw=0.8, label="other (table, non-spindle vs stock)")
    ax[2].axhline(0, color="r", lw=0.6, ls="--")
    ax[2].set_ylabel("min clearance [mm]")
    ax[2].set_xlabel("t [s]")
    ax[2].legend(fontsize=8)
    finish_figure(fig, out / "tracking_clearance.png")

    # ---- media (time-lapse)
    T_end = res.times[-1]
    n_vid = int(np.ceil(T_end / args.video_speed * args.video_fps)) + 1
    t_vid = np.linspace(0.0, T_end, n_vid)
    media = {}
    if not args.no_usd:
        core = build_cell_scene(cell, include_environment=False)  # robots + table + stock only; RTX re-authors the environment
        record_usd(core, trajs, t_vid, out / "scene.usd", args.video_fps)
        media["usd"] = rel(out / "scene.usd")
        media["usd_frames"] = n_vid
    if not args.no_video:
        views = {k: (tuple(v["pos"]), tuple(v["target"])) for k, v in cell.cameras.items() if k in args.views.split(",")}
        media.update(render_videos(scene, cell, trajs, t_vid, out, views, args.video_fps))

    summary = {
        "cell": rel(args.cell),
        "trajectories": {n: {"dir": rel(p.split("=", 1)[1]), "gcode": trajs[n].meta.get("gcode"), "duration_s": float(trajs[n].t[-1])} for n, p in zip(trajs, args.traj)},
        "checks": checks,
        "media": media,
        "video_speed": args.video_speed,
        "env": {"python": platform.python_version(), "newton": __import__("newton").__version__},
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=2, default=str))
    print(json.dumps({k: v for k, v in checks.items() if k != "pair_min_mm"}, indent=2, default=str))
    print("pair_min_mm (<500 mm):", json.dumps(checks["pair_min_mm"], indent=1))
    return 0 if checks["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
