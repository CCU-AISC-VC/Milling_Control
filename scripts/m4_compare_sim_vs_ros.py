#!/usr/bin/env python3
"""M4: compare joint states recorded by milling_bridge (mock hardware or real robot) with the
trajectory file they were played from.

Alignment: the player publishes a ``main_sent`` event with the ROS stamp when the main goal was
sent; trajectory time = (t_js - stamp - lead_in) * speed_scale.

Run:  ~/envs/aisc_sim/bin/python scripts/m4_compare_sim_vs_ros.py --session outputs/m4_bridge/mock_unit_square --traj outputs/m2_ik/unit_square_100mm/trajectory
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import _bootstrap  # noqa: F401
import numpy as np
import pandas as pd

from milling_sim import OUTPUTS_DIR, REPO_ROOT
from milling_sim.traj_io import read_trajectory
from milling_sim.viz import finish_figure, new_figure


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", type=Path, required=True, help="folder with joint_states.csv + events.json")
    ap.add_argument("--traj", type=Path, required=True)
    ap.add_argument("--lead-in", type=float, default=None, help="override the player's lead_in [s]")
    ap.add_argument("--label", default="mock")
    args = ap.parse_args()

    traj = read_trajectory(args.traj)
    js = pd.read_csv(args.session / "joint_states.csv")
    events = json.loads((args.session / "events.json").read_text())
    main_ev = [e for e in events if e.get("event") == "main_sent"]
    if not main_ev:
        print("no main_sent event in events.json", file=sys.stderr)
        return 2
    ev = main_ev[-1]
    scale = float(ev.get("speed_scale", 1.0))
    lead = args.lead_in if args.lead_in is not None else 0.5
    t_rel = (js["t"].to_numpy() - ev["stamp"] - lead) * scale  # trajectory time
    m = (t_rel >= 0) & (t_rel <= traj.duration)
    q_rec = js[traj.joint_names].to_numpy()[m]
    q_ref = traj.interp_q(t_rel[m])
    err = q_rec - q_ref
    max_abs = np.abs(err).max(axis=0)
    rms = np.sqrt((err**2).mean(axis=0))
    # timing-insensitive error: best time shift within +-0.1 s (controller latency)
    best = (None, np.inf)
    for shift in np.linspace(-0.1, 0.1, 41):
        e = np.abs(q_rec - traj.interp_q(np.clip(t_rel[m] + shift, 0, traj.duration))).max()
        if e < best[1]:
            best = (float(shift), float(e))
    summary = {
        "session": str(args.session),
        "trajectory": str(args.traj),
        "label": args.label,
        "speed_scale": scale,
        "n_recorded_in_window": int(m.sum()),
        "record_rate_hz": float(1.0 / np.median(np.diff(js["t"].to_numpy()))) if len(js) > 2 else None,
        "max_abs_err_rad_per_joint": max_abs.round(6).tolist(),
        "max_abs_err_rad": float(max_abs.max()),
        "max_abs_err_deg": float(np.degrees(max_abs.max())),
        "rms_err_rad_per_joint": rms.round(6).tolist(),
        "best_time_shift_s": best[0],
        "max_abs_err_rad_after_shift": best[1],
        "gate_rad": 1e-3,
        "pass": bool(best[1] < 1e-3),
    }
    out = args.session
    (out / "compare_summary.json").write_text(json.dumps(summary, indent=2))

    fig, ax = new_figure(2, 1, size=(12, 7))
    for j, name in enumerate(traj.joint_names):
        ax[0].plot(t_rel[m], np.degrees(q_rec[:, j]), lw=0.8, label=f"{name} recorded")
        ax[0].plot(traj.t, np.degrees(traj.q[:, j]), lw=0.8, ls="--", color="k", alpha=0.5)
    ax[0].set_ylabel("joint [deg]")
    ax[0].set_title(f"{args.label}: recorded joint states (solid) vs trajectory file (dashed)")
    ax[0].legend(ncol=6, fontsize=7, loc="upper right")
    for j, name in enumerate(traj.joint_names):
        ax[1].plot(t_rel[m], np.degrees(err[:, j]) * 1e3, lw=0.8, label=name)
    ax[1].set_ylabel("error [mdeg]")
    ax[1].set_xlabel("trajectory time [s]")
    ax[1].legend(ncol=6, fontsize=7, loc="upper right")
    finish_figure(fig, out / "compare.png")
    print(json.dumps(summary, indent=2))
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
