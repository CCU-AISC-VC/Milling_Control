#!/usr/bin/env python3
"""M7: height-map material removal, MRR, cutting-force estimate and static tool-tip deflection
for one or more trajectory files on the cell's stock.

Run:  ~/envs/aisc_sim/bin/python scripts/m7_material_removal.py --traj fanuc_1=outputs/m2_ik/sample_pocket_contour/trajectory
      ~/envs/aisc_sim/bin/python scripts/m7_material_removal.py --cell configs/cells/dual_robot_table.yaml \
          --traj fanuc_1=outputs/m6_dual/traj_fanuc_1/trajectory --traj fanuc_2=outputs/m6_dual/traj_fanuc_2/trajectory
Outputs: outputs/m7_removal/<name>/
"""
from __future__ import annotations

import argparse
import json
import platform
import sys
from pathlib import Path

import _bootstrap  # noqa: F401
import numpy as np
import pandas as pd
import yaml

from milling_sim import OUTPUTS_DIR, REPO_ROOT
from milling_sim.cell import load_cell
from milling_sim.cutting import cutting_force, load_materials, static_deflection
from milling_sim.removal import Cutter, make_heightmap, surface_mesh
from milling_sim.tool import ToolConfig, load_tool
from milling_sim.traj_io import read_trajectory
from milling_sim.urdf_kinematics import load_chain
from milling_sim.viz import finish_figure, new_figure


def rel(p) -> str:
    try:
        return str(Path(p).resolve().relative_to(REPO_ROOT))
    except ValueError:
        return str(p)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--traj", action="append", required=True, help="robot=path/to/trajectory (repeatable)")
    ap.add_argument("--cell", type=Path, default=None, help="defaults to the cell in the first trajectory's meta")
    ap.add_argument("--tool", type=Path, default=None, help="tool YAML override when the trajectory has no tool")
    ap.add_argument("--material", default=None)
    ap.add_argument("--resolution-mm", type=float, default=0.5)
    ap.add_argument("--name", default=None)
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()

    trajs = {}
    for item in args.traj:
        n, p = item.split("=", 1)
        trajs[n] = read_trajectory(p)
    first = next(iter(trajs.values()))
    cell = load_cell(args.cell or (REPO_ROOT / first.meta["cell"]))
    mats, default_mat = load_materials()
    mat = mats[args.material or default_mat]
    name = args.name or ("_".join(Path(t.meta["gcode"]).stem for t in trajs.values()) if len(trajs) > 1 else Path(first.meta["gcode"]).stem)
    out = args.out or (OUTPUTS_DIR / "m7_removal" / name)
    out.mkdir(parents=True, exist_ok=True)

    # --- tools per robot
    tools: dict[str, ToolConfig] = {}
    for n, tr in trajs.items():
        t = ToolConfig.from_meta(tr.meta.get("tool"))
        if t is None and args.tool is not None:
            t = load_tool(args.tool)
        if t is None and cell.robot(n).tool is not None:
            t = cell.robot(n).tool
        if t is None:
            t = load_tool("configs/tools/flat_10mm.yaml")
        tools[n] = t

    # --- height map over the first stock (world XY)
    stock = cell.stock_boxes_world()[0]
    hm = make_heightmap(stock.center[:2], stock.size[:2], z_top=stock.center[2] + stock.size[2] / 2, height=stock.size[2], resolution=args.resolution_mm * 1e-3)
    cutters = {n: Cutter(hm, tools[n].radius, ball=(tools[n].type == "ball")) for n in trajs}

    # --- merge all samples in global time order (each robot's samples keep their own index)
    events = []
    tip_world = {}
    for n, tr in trajs.items():
        T_wb = cell.robot(n).base_xform
        tip_world[n] = (T_wb[:3, :3] @ tr.tcp_pos.T).T + T_wb[:3, 3]
        for i in range(1, tr.n):
            events.append((tr.t[i], n, i))
    events.sort()

    rows = []
    vol_total = {n: 0.0 for n in trajs}
    for t, n, i in events:
        tr = trajs[n]
        p0, p1 = tip_world[n][i - 1], tip_world[n][i]
        dt = tr.t[i] - tr.t[i - 1]
        vol = 0.0
        if tr.motion[i] != "rapid":
            vol = cutters[n].sweep(p0, p1)
        vol_total[n] += vol
        mrr = vol * 1e9 / dt if dt > 0 else 0.0  # mm^3/s
        rows.append({"t": t, "robot": n, "i": i, "dt": dt, "vol_mm3": vol * 1e9, "mrr_mm3_s": mrr, "rpm": tr.spindle_rpm[i], "fx": p1[0] - p0[0], "fy": p1[1] - p0[1], "fz": p1[2] - p0[2]})
    df = pd.DataFrame(rows)

    # --- cutting force (world frame) and static deflection per robot sample
    summary_robots = {}
    for n, tr in trajs.items():
        rc = cell.robot(n).robot
        chain = load_chain(rc.urdf, rc.base_link, rc.tcp_link)
        raw = yaml.safe_load(rc.path.read_text())
        k_joint = np.asarray(raw.get("joint_stiffness_Nm_rad", [1e6] * 6), dtype=float)
        sub = df[df.robot == n].sort_values("i")
        feed_dir = sub[["fx", "fy", "fz"]].to_numpy()
        F_w, F_mag = cutting_force(sub.mrr_mm3_s.to_numpy(), sub.rpm.to_numpy(), feed_dir, tools[n].diameter, mat)
        T_wb = cell.robot(n).base_xform
        F_b = (T_wb[:3, :3].T @ F_w.T).T
        Q = tr.q[sub.i.to_numpy()]
        dev, tau = static_deflection(chain, Q, F_b, k_joint, rc.tool_axis_in_tcp, tools[n].length_from_nose)
        df.loc[sub.index, "F_N"] = F_mag
        df.loc[sub.index, "Fx_N"] = F_w[:, 0]
        df.loc[sub.index, "Fy_N"] = F_w[:, 1]
        df.loc[sub.index, "Fz_N"] = F_w[:, 2]
        df.loc[sub.index, "deflection_mm"] = np.linalg.norm(dev, axis=1) * 1e3
        df.loc[sub.index, "deflection_z_mm"] = dev[:, 2] * 1e3
        df.loc[sub.index, "tau_max_Nm"] = np.abs(tau).max(axis=1)
        summary_robots[n] = {
            "tool": tools[n].to_meta(),
            "removed_volume_mm3": vol_total[n] * 1e9,
            "cutting_time_s": float(sub.dt[sub.vol_mm3 > 0].sum()),
            "mrr_max_mm3_s": float(sub.mrr_mm3_s.max()),
            "mrr_mean_cutting_mm3_s": float(sub.mrr_mm3_s[sub.vol_mm3 > 0].mean()) if (sub.vol_mm3 > 0).any() else 0.0,
            "force_max_N": float(F_mag.max()),
            "force_mean_cutting_N": float(F_mag[sub.vol_mm3.to_numpy() > 0].mean()) if (sub.vol_mm3 > 0).any() else 0.0,
            "deflection_max_mm": float(np.linalg.norm(dev, axis=1).max() * 1e3),
            "deflection_z_max_mm": float(np.abs(dev[:, 2]).max() * 1e3),
            "joint_torque_max_Nm": np.abs(tau).max(axis=0).round(3).tolist(),
            "joint_stiffness_Nm_rad": k_joint.tolist(),
        }
    df.to_csv(out / "removal.csv", index=False)

    Z = hm.numpy()
    depth_mm = (hm.z_top - Z) * 1e3
    summary = {
        "name": name,
        "cell": rel(cell.path),
        "material": {"name": mat.name, "k_c_N_mm2": mat.k_c, "radial_ratio": mat.radial_ratio, "axial_ratio": mat.axial_ratio},
        "stock_mm": {"size": (stock.size * 1e3).round(3).tolist(), "center_world": stock.center.round(4).tolist()},
        "heightmap": {"resolution_mm": args.resolution_mm, "nx": hm.nx, "ny": hm.ny},
        "removed_volume_total_mm3": float(hm.removed_volume() * 1e9),
        "max_depth_mm": float(depth_mm.max()),
        "robots": summary_robots,
        "trajectories": {n: rel(p.split("=", 1)[1]) for n, p in zip(trajs, args.traj)},
        "pass": bool(np.isfinite(depth_mm).all() and hm.removed_volume() > 0 and abs(sum(vol_total.values()) - hm.removed_volume()) < 1e-4 * max(hm.removed_volume(), 1e-12)),
        "env": {"python": platform.python_version(), "warp": __import__("warp").__version__},
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=2))
    np.save(out / "heightmap_depth_mm.npy", depth_mm.astype(np.float32))

    # --- plots
    import matplotlib.pyplot as plt

    xs, ys = hm.cell_centers()
    fig, ax = new_figure(1, 1, size=(10, 4 + 4 * stock.size[1] / stock.size[0]))
    im = ax.imshow(depth_mm.T, origin="lower", extent=[xs[0] * 1e3, xs[-1] * 1e3, ys[0] * 1e3, ys[-1] * 1e3], cmap="viridis_r", aspect="equal")
    ax.set_xlabel("world x [mm]")
    ax.set_ylabel("world y [mm]")
    ax.set_title(f"machined depth [mm], {name}, tool(s) " + ", ".join(f"{n}: {t.name}" for n, t in tools.items()))
    fig.colorbar(im, ax=ax, shrink=0.8, label="depth [mm]")
    finish_figure(fig, out / "heightmap.png")

    step = max(1, int(round(2.0 / args.resolution_mm)))  # ~2 mm for the 3D plot
    V, F = surface_mesh(hm, step=step)
    fig = plt.figure(figsize=(11, 6))
    ax3 = fig.add_subplot(1, 1, 1, projection="3d")
    ax3.plot_trisurf(V[:, 0] * 1e3, V[:, 1] * 1e3, V[:, 2] * 1e3, triangles=F, cmap="viridis_r", linewidth=0, antialiased=False)
    ax3.set_xlabel("x [mm]")
    ax3.set_ylabel("y [mm]")
    ax3.set_zlabel("z [mm]")
    ax3.set_box_aspect((stock.size[0], stock.size[1], max(stock.size[0], stock.size[1]) * 0.25))
    ax3.set_title("machined surface (height map, 2.5D)")
    finish_figure(fig, out / "surface_3d.png")

    fig, ax = new_figure(3, 1, size=(12, 8))
    for n in trajs:
        sub = df[df.robot == n]
        ax[0].plot(sub.t, sub.mrr_mm3_s, lw=0.7, label=n)
        ax[1].plot(sub.t, sub.F_N, lw=0.7, label=n)
        ax[2].plot(sub.t, sub.deflection_mm, lw=0.7, label=f"{n} |δ|")
        ax[2].plot(sub.t, sub.deflection_z_mm, lw=0.6, ls="--", label=f"{n} δz")
    ax[0].set_ylabel("MRR [mm³/s]")
    ax[0].set_title(f"material removal rate, cutting force ({mat.name}, k_c {mat.k_c:.0f} N/mm²) and static tip deflection (joint stiffness UNVERIFIED)")
    ax[1].set_ylabel("|F| [N]")
    ax[2].set_ylabel("deflection [mm]")
    ax[2].set_xlabel("t [s]")
    for a in ax:
        a.legend(fontsize=7, loc="upper right")
    finish_figure(fig, out / "mrr_force_deflection.png")

    print(json.dumps({k: summary[k] for k in ("removed_volume_total_mm3", "max_depth_mm", "material", "pass")}, indent=2))
    for n, srr in summary_robots.items():
        print(n, {k: (round(v, 3) if isinstance(v, float) else v) for k, v in srr.items() if k not in ("tool", "joint_stiffness_Nm_rad")})
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
