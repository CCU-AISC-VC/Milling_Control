#!/usr/bin/env python3
"""M0: check that Newton's import of the FANUC URDF matches an independent NumPy FK.

Samples joint angles uniformly inside the URDF limits, evaluates the pose of ``flange``,
``spindle_link`` and ``spindle_tcp`` with both implementations and reports the differences.
Also renders the robot at the SRDF ``home`` and ``ready`` poses with the TCP axes drawn.

Run:  ~/envs/aisc_sim/bin/python scripts/m0_fk_check.py [--n 1000] [--seed 0]
Outputs: outputs/m0_assets/
"""
from __future__ import annotations

import argparse
import json
import platform
import sys
import traceback
from pathlib import Path

import _bootstrap  # noqa: F401
import numpy as np
import pandas as pd

from milling_sim import ASSETS_DIR, OUTPUTS_DIR
from milling_sim.newton_robot import build_single_robot
from milling_sim.urdf_kinematics import load_chain, pose_error
from milling_sim.viz import HeadlessGL, finish_figure, new_figure, save_png

URDF = ASSETS_DIR / "fanuc_m710id70_spindle/m710id_70_spindle.urdf"
LINKS = ["flange", "spindle_link", "spindle_tcp"]
POSES = {
    "home": np.zeros(6),
    "ready": np.array([0.0, -0.785, 0.785, 0.0, -1.5708, 0.0]),  # SRDF group_state "ready"
}
PASS_POS_MM = 0.01
PASS_ROT_DEG = 0.001


def env_versions() -> dict:
    import newton
    import warp

    return {"python": platform.python_version(), "newton": newton.__version__, "warp": warp.__version__}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=1000)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", type=Path, default=OUTPUTS_DIR / "m0_assets")
    args = ap.parse_args()
    out: Path = args.out
    out.mkdir(parents=True, exist_ok=True)

    chain = load_chain(URDF, "base_link", "spindle_tcp")
    scene = build_single_robot(URDF, name="fanuc_1")
    rng = np.random.default_rng(args.seed)
    Q = rng.uniform(chain.lower, chain.upper, size=(args.n, chain.dof))

    rows = []
    for i, q in enumerate(Q):
        ref = chain.link_transforms(q)
        nt = scene.fk("fanuc_1", q, LINKS)
        row = {f"q{k+1}": q[k] for k in range(chain.dof)}
        for link in LINKS:
            dp, dr = pose_error(ref[link], nt[link])
            row[f"{link}_pos_err_mm"] = dp * 1e3
            row[f"{link}_rot_err_deg"] = np.degrees(dr)
        tcp = ref["spindle_tcp"][:3, 3]
        row["tcp_x"], row["tcp_y"], row["tcp_z"] = tcp
        row["tcp_radius"] = float(np.hypot(tcp[0], tcp[1]))
        rows.append(row)
    df = pd.DataFrame(rows)
    df.to_csv(out / "fk_check.csv", index=False)

    summary = {
        "n_samples": args.n,
        "seed": args.seed,
        "urdf": str(URDF.relative_to(ASSETS_DIR.parent)),
        "joint_names": chain.joint_names,
        "joint_limits_deg": {"lower": np.degrees(chain.lower).tolist(), "upper": np.degrees(chain.upper).tolist()},
        "joint_velocity_limits_deg_s": np.degrees(chain.velocity_limits).tolist(),
        "errors": {
            link: {
                "pos_max_mm": float(df[f"{link}_pos_err_mm"].max()),
                "pos_mean_mm": float(df[f"{link}_pos_err_mm"].mean()),
                "rot_max_deg": float(df[f"{link}_rot_err_deg"].max()),
            }
            for link in LINKS
        },
        "tcp_workspace_m": {
            "radius_min": float(df.tcp_radius.min()),
            "radius_max": float(df.tcp_radius.max()),
            "z_min": float(df.tcp_z.min()),
            "z_max": float(df.tcp_z.max()),
        },
        "named_poses": {},
        "env": env_versions(),
    }
    for name, q in POSES.items():
        T = chain.fk(q)
        summary["named_poses"][name] = {
            "q_rad": q.tolist(),
            "tcp_pos_m": T[:3, 3].round(6).tolist(),
            "tcp_x_axis": T[:3, 0].round(6).tolist(),
            "tcp_z_axis": T[:3, 2].round(6).tolist(),
        }
    summary["pass"] = bool(
        summary["errors"]["spindle_tcp"]["pos_max_mm"] < PASS_POS_MM
        and summary["errors"]["spindle_tcp"]["rot_max_deg"] < PASS_ROT_DEG
    )
    summary["pass_criteria"] = {"pos_max_mm": PASS_POS_MM, "rot_max_deg": PASS_ROT_DEG}

    # --- error histogram
    fig, ax = new_figure(1, 2, size=(10, 4))
    ax[0].hist(df.spindle_tcp_pos_err_mm * 1e3, bins=50)
    ax[0].set_xlabel("spindle_tcp position error [µm]")
    ax[0].set_ylabel("count")
    ax[1].hist(df.spindle_tcp_rot_err_deg * 3600, bins=50)
    ax[1].set_xlabel("spindle_tcp rotation error [arcsec]")
    fig.suptitle(f"Newton add_urdf vs NumPy FK, {args.n} random joint samples (seed {args.seed})")
    finish_figure(fig, out / "fk_error_hist.png")

    # --- reachable TCP cloud (side and top view)
    fig, ax = new_figure(1, 2, size=(10, 4.5))
    ax[0].scatter(df.tcp_x, df.tcp_z, s=2, alpha=0.4)
    ax[0].set_xlabel("x [m]")
    ax[0].set_ylabel("z [m]")
    ax[0].set_title("TCP samples, side view (x-z)")
    ax[0].set_aspect("equal")
    ax[1].scatter(df.tcp_x, df.tcp_y, s=2, alpha=0.4)
    ax[1].set_xlabel("x [m]")
    ax[1].set_ylabel("y [m]")
    ax[1].set_title("TCP samples, top view (x-y)")
    ax[1].set_aspect("equal")
    finish_figure(fig, out / "tcp_samples.png")

    # --- snapshots with TCP axes (red = +X, green = +Y, blue = +Z of spindle_tcp)
    snapshot_ok = True
    try:
        gl = HeadlessGL(scene.model, 1280, 720)
        for name, q in POSES.items():
            scene.set_q("fanuc_1", q)
            scene.eval_fk()
            T = scene.link_pose("fanuc_1", "spindle_tcp")
            p = T[:3, 3]
            L = 0.3
            lines = [
                ("tcp_x", [p], [p + L * T[:3, 0]], (1.0, 0.1, 0.1)),
                ("tcp_y", [p], [p + L * T[:3, 1]], (0.1, 0.8, 0.1)),
                ("tcp_z", [p], [p + L * T[:3, 2]], (0.1, 0.3, 1.0)),
            ]
            points = [("tcp", [p], 0.02, (1.0, 0.9, 0.0))]
            gl.set_camera((3.2, -3.0, 2.2), (0.8, 0.0, 1.0))
            rgb = gl.render(scene.state, points=points, lines=lines)
            rgb = gl.render(scene.state, points=points, lines=lines)  # second frame: buffers settled
            save_png(out / f"snapshot_{name}.png", rgb)
            # close-up of the spindle
            gl.set_camera((p[0] + 1.0, p[1] - 1.0, p[2] + 0.6), p)
            rgb = gl.render(scene.state, points=points, lines=lines)
            rgb = gl.render(scene.state, points=points, lines=lines)
            save_png(out / f"snapshot_{name}_spindle.png", rgb)
        gl.close()
    except Exception:
        snapshot_ok = False
        (out / "snapshot_error.txt").write_text(traceback.format_exc())
    summary["snapshot_ok"] = snapshot_ok

    (out / "summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps({k: summary[k] for k in ("errors", "tcp_workspace_m", "named_poses", "pass", "snapshot_ok")}, indent=2))
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
