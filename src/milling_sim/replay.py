"""Multi-robot kinematic replay in a Newton cell: tracking, clearance, collision classes,
head-less GL videos and USD recording. Used by M3-style single-robot runs and the M6 dual cell.

Collision classes per frame (signed distance d < -tol means penetration):
- expected: a robot's spindle vs a stock while that robot's program allows the tool inside
  (cutting samples, or the pure retract right after a cut);
- unexpected: everything else (robot-robot, robot-table, robot-fence/floor/cabinet,
  non-spindle link vs stock, spindle vs stock during traverses).
"""
from __future__ import annotations

import os
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pandas as pd

from .cell import CellConfig
from .clearance import CONTACT_TOL, MeshClearance
from .removal import Cutter, make_heightmap, solid_from_heightmap
from .sim import CellScene, in_stock_allowed_mask
from .tool import ToolConfig
from .traj_io import JointTrajectory

PENETRATION_TOL = 5e-5  # m, box-vs-mesh contacts from the Newton pipeline
ROBOT_ROBOT_TOL = CONTACT_TOL  # m, unsigned mesh-vs-mesh clearance below this = contact


@dataclass
class ReplayResult:
    times: np.ndarray
    Q: dict[str, np.ndarray]  # robot -> (n_frames, dof)
    df: pd.DataFrame
    pair_min: dict[tuple[str, str], float]
    unexpected_frames: list[dict]
    per_robot: dict[str, dict]
    replay_time_s: float
    extras: dict = field(default_factory=dict)


def tool_box_distance(tip: np.ndarray, axis: np.ndarray, radius: float, length: float, ball: bool, box_center: np.ndarray, box_size: np.ndarray) -> float:
    """Signed vertical distance [m] between the lowest point of a tool (cylinder from the tip up
    along -axis, ball or flat end) and an axis-aligned box: negative = the tool is below the box
    top while inside its footprint (penetration depth); positive = clearance (vertical gap when
    inside the footprint, otherwise the lateral gap). Newton's cylinder-vs-box contacts are not a
    usable depth measure for thin tools, so the replay uses this instead."""
    a = np.asarray(axis, dtype=float)  # spindle -> tip (points down when cutting)
    tilt = float(np.arccos(np.clip(-a[2], -1.0, 1.0)))
    low = np.asarray(tip, dtype=float).copy()
    if not ball:
        low[2] -= radius * np.sin(tilt)  # the rim of a tilted flat end dips below the tip
    lo = box_center - box_size / 2
    hi = box_center + box_size / 2
    # lateral gap of the tool's footprint (approximated by the tip disk) to the box footprint
    dx = max(lo[0] - (low[0] + radius), (low[0] - radius) - hi[0], 0.0)
    dy = max(lo[1] - (low[1] + radius), (low[1] - radius) - hi[1], 0.0)
    lateral = float(np.hypot(dx, dy))
    vertical = float(low[2] - hi[2])
    if lateral > 0.0:
        return float(np.hypot(lateral, max(vertical, 0.0)))
    return vertical


def _robot_of(role: str) -> str | None:
    return role.split("/")[0] if "/" in role and not role.startswith("env/") else None


def _is_spindle(role: str, spindle_link: str = "spindle_link") -> bool:
    """Spindle body or the mounted tool cylinder (both are allowed inside the stock while cutting)."""
    return (role.endswith("/" + spindle_link) or role.endswith("/tool")) and not role.startswith("env/")


def frame_tables(trajs: dict[str, JointTrajectory], cell: CellConfig, sim_hz: float):
    """Common time base and per-robot interpolated joints / reference TCP (world) / allow masks."""
    T_end = max(t.t[-1] for t in trajs.values())
    times = np.arange(0.0, T_end + 1e-9, 1.0 / sim_hz)
    Q, P_ref_w, allowed, cutting, z_prog = {}, {}, {}, {}, {}
    for name, tr in trajs.items():
        Q[name] = tr.interp_q(times)  # holds the last point after the trajectory ends
        T_wb = cell.robot(name).base_xform
        P = np.stack([np.interp(times, tr.t, tr.tcp_pos[:, k]) for k in range(3)], axis=1)
        P_ref_w[name] = (T_wb[:3, :3] @ P.T).T + T_wb[:3, 3]
        idx = np.clip(np.searchsorted(tr.t, times, side="right") - 1, 0, tr.n - 1)
        T_bp = np.asarray(tr.meta["T_base_program"])
        prog = (np.linalg.inv(T_bp) @ np.c_[tr.tcp_pos, np.ones(tr.n)].T).T[:, :3]
        allow_s = in_stock_allowed_mask(tr.motion, prog, tr.segment)
        ended = times > tr.t[-1]
        allowed[name] = (allow_s[idx] | allow_s[np.clip(idx + 1, 0, tr.n - 1)]) & ~ended
        cutting[name] = (tr.motion[idx] != "rapid") & ~ended
        z_prog[name] = np.interp(times, tr.t, prog[:, 2])
    return times, Q, P_ref_w, allowed, cutting, z_prog


def replay(scene: CellScene, cell: CellConfig, trajs: dict[str, JointTrajectory], sim_hz: float = 60.0, tip_offsets: dict[str, float] | None = None, tol: float = PENETRATION_TOL, clearance_every: int = 2) -> ReplayResult:
    """``clearance_every``: run the (costly) robot-robot mesh clearance every N frames; in between
    the last value is carried (at 60 Hz and milling feeds the motion between checks is < 1 mm)."""
    tip_offsets = tip_offsets or {}
    times, Q, P_ref_w, allowed, cutting, z_prog = frame_tables(trajs, cell, sim_hz)
    names = list(trajs)
    tcp_links = {n: cell.robot(n).robot.tcp_link for n in names}
    tool_cfg = {n: ToolConfig.from_meta(trajs[n].meta.get("tool")) for n in names}
    tool_len = {n: (tool_cfg[n].length_from_nose if tool_cfg[n] else 0.0) for n in names}
    tool_axis = {n: np.asarray(cell.robot(n).robot.tool_axis_in_tcp, dtype=float) for n in names}
    boxes = [(b.name, b.center, b.size, "stock") for b in cell.stock_boxes_world()]
    tb = cell.table_box()
    if tb is not None:
        boxes.append(("table", tb.center, tb.size, "table"))
    tip_offsets = {n: (0.0 if trajs[n].meta.get("tool") else tip_offsets.get(n, 0.0)) for n in names}  # with a tool shape the tip is exact
    clearance = MeshClearance(scene, names) if len(names) >= 2 else None
    d_rr, pair_rr = np.inf, None
    rows = []
    pair_min: dict[tuple[str, str], float] = {}
    unexpected_frames = []
    t0 = time.perf_counter()
    for i, t in enumerate(times):
        for n in names:
            scene.set_q(n, Q[n][i])
        scene.eval_fk()
        row = {"t": t}
        unexpected = []
        for n in names:
            T_nose = scene.link_pose(n, tcp_links[n])
            a_w = T_nose[:3, :3] @ tool_axis[n]
            tcp = T_nose[:3, 3] + a_w * tool_len[n]
            row[f"{n}_tcp_err_mm"] = float(np.linalg.norm(tcp - P_ref_w[n][i])) * 1e3
            row[f"{n}_spindle_stock_mm"] = np.inf
            row[f"{n}_z_prog_mm"] = z_prog[n][i] * 1e3
            row[f"{n}_tool_table_mm"] = np.inf
            if tool_cfg[n] is not None:
                tc = tool_cfg[n]
                for bname, bc, bs, kind in boxes:
                    d = tool_box_distance(tcp, a_w, tc.radius, tc.length_from_nose, tc.type == "ball", bc, bs)
                    key = (f"{n}/tool", bname)
                    pair_min[key] = min(pair_min.get(key, np.inf), d)
                    if kind == "stock":
                        row[f"{n}_spindle_stock_mm"] = min(row[f"{n}_spindle_stock_mm"], d * 1e3)
                        if d < -tol and not allowed[n][i]:
                            unexpected.append((f"{n}/tool", bname, float(d)))
                    else:
                        row[f"{n}_tool_table_mm"] = min(row[f"{n}_tool_table_mm"], d * 1e3)
                        if d < -tol:
                            unexpected.append((f"{n}/tool", bname, float(d)))
        row["robot_robot_mm"] = np.inf
        row["robot_env_mm"] = np.inf
        row["other_mm"] = np.inf
        cs = scene.collide()
        row["n_contacts"] = cs.n_contacts
        if clearance is not None:
            if i % clearance_every == 0 or i == len(times) - 1:
                d_rr, pair_rr = clearance.min_distance_all()
            row["robot_robot_mm"] = d_rr * 1e3
            if pair_rr is not None:
                pair_min[pair_rr] = min(pair_min.get(pair_rr, np.inf), d_rr)
            if d_rr < ROBOT_ROBOT_TOL:
                unexpected.append((pair_rr[0], pair_rr[1], float(d_rr)))
        for (r0, r1), d in cs.pairs.items():
            a, b = _robot_of(r0), _robot_of(r1)
            if a and b and a != b:
                continue  # robot-robot: Newton mesh-mesh candidates are not a clearance measure (see clearance.py)
            if r0.endswith("/tool") or r1.endswith("/tool"):
                continue  # tool cylinder: Newton's cylinder-vs-box distances are wrong for thin tools; handled analytically above
            pair_min[(r0, r1)] = min(pair_min.get((r0, r1), np.inf), d)
            cls = "unexpected"
            if (a or b) and (r0.startswith("env/") or r1.startswith("env/")):
                row["robot_env_mm"] = min(row["robot_env_mm"], d * 1e3)
            elif (a or b) and ("stock" in (r0 if not a else r1)):
                rob = a or b
                rr = r0 if a else r1
                if _is_spindle(rr):
                    row[f"{rob}_spindle_stock_mm"] = min(row[f"{rob}_spindle_stock_mm"], d * 1e3)
                    if allowed[rob][i]:
                        cls = "expected"
                else:
                    row["other_mm"] = min(row["other_mm"], d * 1e3)
            else:
                row["other_mm"] = min(row["other_mm"], d * 1e3)
            if cls == "unexpected" and d < -tol:
                unexpected.append((r0, r1, float(d)))
        if unexpected:
            unexpected_frames.append({"frame": i, "t": float(t), "pairs": unexpected[:6]})
        rows.append(row)
    df = pd.DataFrame(rows).replace(np.inf, np.nan)
    per_robot = {}
    for n in names:
        depth = float(-min(z_prog[n].min(), 0.0))
        tip = float(tip_offsets.get(n, 0.0))
        per_robot[n] = {
            "tcp_err_max_mm": float(df[f"{n}_tcp_err_mm"].max()),
            "tcp_err_mean_mm": float(df[f"{n}_tcp_err_mm"].mean()),
            "spindle_stock_min_mm": float(df[f"{n}_spindle_stock_mm"].min()),
            "programmed_max_depth_mm": depth * 1e3,
            "expected_deepest_mm": -(depth - tip) * 1e3,
            "duration_s": float(trajs[n].t[-1]),
        }
    extras = {"robot_robot_method": "warp unsigned mesh queries, vertex-to-surface, coarse meshes subdivided to 10 mm, wrist links J4..spindle; clearance < tol = contact", "robot_robot_tol_mm": ROBOT_ROBOT_TOL * 1e3, "n_mesh_queries": clearance.n_queries if clearance else 0, "clearance_every": clearance_every}
    return ReplayResult(times, Q, df, pair_min, unexpected_frames, per_robot, time.perf_counter() - t0, extras)


# ---------------------------------------------------------------------------- machined stock
STOCK_COLOR = (0.82, 0.78, 0.64)


class StockEvolver:
    """Height-map stock that is cut progressively as time advances (for videos / USD)."""

    def __init__(self, cell: CellConfig, trajs: dict[str, JointTrajectory], tools: dict[str, ToolConfig], resolution: float = 1e-3, mesh_step: int = 1):
        stock = cell.stock_boxes_world()[0]
        self.hm = make_heightmap(stock.center[:2], stock.size[:2], z_top=stock.center[2] + stock.size[2] / 2, height=stock.size[2], resolution=resolution)
        self.cutters = {n: Cutter(self.hm, tools[n].radius, ball=(tools[n].type == "ball")) for n in trajs}
        self.mesh_step = mesh_step
        events = []
        self.tip_world = {}
        self.dir_world = {}
        for n, tr in trajs.items():
            T_wb = cell.robot(n).base_xform
            self.tip_world[n] = (T_wb[:3, :3] @ tr.tcp_pos.T).T + T_wb[:3, 3]
            self.dir_world[n] = (T_wb[:3, :3] @ tr.tool_dirs(cell.robot(n).robot.tool_axis_in_tcp).T).T
            events += [(tr.t[i], n, i) for i in range(1, tr.n) if tr.motion[i] != "rapid"]
        events.sort()
        self.events = events
        self.k = 0
        self._wp_points = None
        self._wp_indices = None

    def advance_to(self, t: float) -> None:
        while self.k < len(self.events) and self.events[self.k][0] <= t:
            _, n, i = self.events[self.k]
            self.cutters[n].sweep(self.tip_world[n][i - 1], self.tip_world[n][i], self.dir_world[n][i - 1], self.dir_world[n][i])
            self.k += 1

    def mesh_arrays(self):
        import warp as wp

        V, F = solid_from_heightmap(self.hm, self.mesh_step)
        if self._wp_points is None or len(self._wp_points) != len(V):
            self._wp_points = wp.array(V, dtype=wp.vec3)
            self._wp_indices = wp.array(F.reshape(-1), dtype=wp.int32)
        else:
            self._wp_points.assign(V)
        return self._wp_points, self._wp_indices


def tools_from_trajs(cell: CellConfig, trajs: dict[str, JointTrajectory]) -> dict[str, ToolConfig]:
    out = {}
    for n, tr in trajs.items():
        t = ToolConfig.from_meta(tr.meta.get("tool")) or cell.robot(n).tool
        if t is None:
            from .tool import load_tool

            t = load_tool("configs/tools/flat_10mm.yaml")
        out[n] = t
    return out


# ---------------------------------------------------------------------------- media
def record_usd(scene: CellScene, trajs: dict[str, JointTrajectory], t_vid: np.ndarray, usd_path: Path, fps: int, stock: StockEvolver | None = None, stock_every: int = 5) -> None:
    import newton.viewer as nv

    tmp = usd_path.with_name(usd_path.stem + "_tmp" + usd_path.suffix)
    usd = nv.ViewerUSD(str(tmp), fps=fps, up_axis="Z", num_frames=len(t_vid))
    usd.set_model(scene.model)
    Qv = {n: tr.interp_q(t_vid) for n, tr in trajs.items()}
    for k in range(len(t_vid)):
        for n in trajs:
            scene.set_q(n, Qv[n][k])
        scene.eval_fk()
        usd.begin_frame(k / fps)
        usd.log_state(scene.state)
        if stock is not None and (k % stock_every == 0 or k == len(t_vid) - 1):
            stock.advance_to(float(t_vid[k]))
            pts, idx = stock.mesh_arrays()
            usd.log_mesh("/stock_machined", pts, idx, color=STOCK_COLOR, roughness=0.7, dynamic=True)
        usd.end_frame()
    usd.close()
    os.replace(tmp, usd_path)


TRAIL_COLORS = [(1.0, 0.45, 0.05), (0.1, 0.75, 1.0), (0.3, 1.0, 0.3), (1.0, 0.3, 0.8)]


def render_videos(scene: CellScene, cell: CellConfig, trajs: dict[str, JointTrajectory], t_vid: np.ndarray, out: Path, views: dict[str, tuple], fps: int, size=(1280, 720), gif_frames: int = 150, stock_factory=None) -> dict:
    """``stock_factory``: callable returning a fresh StockEvolver (one per view, since the cut
    progresses with the video time)."""
    from .viz import HeadlessGL, save_gif, save_mp4, save_png

    gl = HeadlessGL(scene.model, *size)
    Qv = {n: tr.interp_q(t_vid) for n, tr in trajs.items()}
    world_path, cut_mask, tcp_links = {}, {}, {}
    for n, tr in trajs.items():
        T_wb = cell.robot(n).base_xform
        world_path[n] = (T_wb[:3, :3] @ tr.tcp_pos.T).T + T_wb[:3, 3]
        cut_mask[n] = tr.motion != "rapid"
        tcp_links[n] = cell.robot(n).robot.tcp_link
    media = {}
    for vname, (pos, tgt) in views.items():
        frames = []
        gl.set_camera(pos, tgt)
        stock = stock_factory() if stock_factory is not None else None
        for k in range(len(t_vid)):
            for n in trajs:
                scene.set_q(n, Qv[n][k])
            scene.eval_fk()
            if stock is not None:
                stock.advance_to(float(t_vid[k]))
                pts, idx = stock.mesh_arrays()
                gl.viewer.log_mesh("/stock_machined", pts, idx, color=STOCK_COLOR, dynamic=True)
            lines, points = [], []
            for ci, (n, tr) in enumerate(trajs.items()):
                j = int(np.searchsorted(tr.t, t_vid[k], side="right"))
                if j >= 2:
                    seg_cut = cut_mask[n][1:j] & cut_mask[n][: j - 1]
                    st, en = world_path[n][: j - 1][seg_cut], world_path[n][1:j][seg_cut]
                    if len(st):
                        lines.append((f"trail_{n}", st, en, TRAIL_COLORS[ci % len(TRAIL_COLORS)]))
                tcp = scene.link_pose(n, tcp_links[n])[:3, 3]
                points.append((f"tcp_{n}", [tcp], 0.008, (1.0, 0.9, 0.1)))
            frames.append(gl.render(scene.state, points=points, lines=lines, dt=1 / fps).copy())
        save_mp4(out / f"replay_{vname}.mp4", frames, fps=fps)
        step = max(1, len(frames) // gif_frames)
        save_gif(out / f"replay_{vname}.gif", [f[::3, ::3] for f in frames[::step]], fps=12)
        save_png(out / f"frame_{vname}_mid.png", frames[len(frames) // 2])
        save_png(out / f"frame_{vname}_last.png", frames[-1])
        media[vname] = {"mp4": str(out / f"replay_{vname}.mp4"), "gif": str(out / f"replay_{vname}.gif"), "frames": len(frames)}
    gl.close()
    return media
