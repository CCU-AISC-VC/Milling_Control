"""Viser web app: pick cell / robot / tool / material, upload or choose G-code, place the
workpiece, run the pipeline (parse -> IK -> Newton collision check -> material removal),
watch the 3D animation and export the trajectory folder for the ROS 2 bridge.

Start:  ~/envs/aisc_sim/bin/python scripts/m8_ui.py --port 8080   (then open http://localhost:8080)
Guide:  docs/UI_GUIDE.md
"""
from __future__ import annotations

import copy
import io
import threading
import time
import traceback
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path

import numpy as np
import trimesh
import viser
import yaml
from scipy.spatial.transform import Rotation as R

from .. import ASSETS_DIR, OUTPUTS_DIR, REPO_ROOT
from ..cell import CellConfig, load_cell
from ..cutting import load_materials
from ..pipeline import PlanResult, RemovalResult, check_replay, material_removal, plan_trajectory
from ..removal import surface_mesh
from ..tool import ToolConfig, list_tools, load_tool
from ..toolpath import SamplingParams
from ..traj_io import JointTrajectory, write_trajectory
from ..urdf_kinematics import load_chain, make_transform

UI_OUT = OUTPUTS_DIR / "ui"
URDF_COLORS = {"FANUC_Yellow": (255, 214, 3), "FANUC_Black": (40, 40, 42), "FANUC_Gray": (62, 64, 68)}


def _wxyz(T: np.ndarray) -> np.ndarray:
    q = R.from_matrix(T[:3, :3]).as_quat()  # xyzw
    return np.array([q[3], q[0], q[1], q[2]])


def load_visual_meshes(urdf_path: Path) -> dict[str, trimesh.Trimesh]:
    """link -> visual mesh in the link frame (URDF visual origin and scale applied, coloured)."""
    root = ET.parse(str(urdf_path)).getroot()
    out = {}
    for lel in root.findall("link"):
        vis = lel.find("visual")
        if vis is None:
            continue
        geo = vis.find("geometry/mesh")
        if geo is None:
            continue
        m = trimesh.load(str(urdf_path.parent / geo.get("filename")), force="mesh")
        m.apply_scale([float(v) for v in geo.get("scale", "1 1 1").split()])
        o = vis.find("origin")
        if o is not None:
            m.apply_transform(make_transform([float(v) for v in o.get("xyz", "0 0 0").split()], [float(v) for v in o.get("rpy", "0 0 0").split()]))
        mat = vis.find("material")
        rgb = URDF_COLORS.get(mat.get("name") if mat is not None else "", (150, 150, 150))
        m.visual.face_colors = np.array([*rgb, 255], dtype=np.uint8)
        out[lel.get("name")] = m
    return out


class MillingApp:
    def __init__(self, host: str = "0.0.0.0", port: int = 8080):
        self.server = viser.ViserServer(host=host, port=port, label="Milling_Control")
        self.server.scene.set_up_direction("+z")
        self.cells = {p.stem: p for p in sorted((REPO_ROOT / "configs/cells").glob("*.yaml"))}
        self.tools = list_tools()
        self.materials, self.default_material = load_materials()
        self.gcodes = {p.name: p for p in sorted((ASSETS_DIR / "gcode").glob("*.nc"))}
        self.uploaded: dict[str, str] = {}
        self.cell: CellConfig | None = None
        self.link_handles: dict[str, dict[str, viser.GlbHandle]] = {}
        self.tool_handles: dict[str, viser.GlbHandle] = {}
        self.chains = {}
        self.meshes_cache: dict[str, dict[str, trimesh.Trimesh]] = {}
        self.plan: PlanResult | None = None
        self.replay_checks: dict | None = None
        self.removal: RemovalResult | None = None
        self.trail_handle = None
        self.surface_handle = None
        self.playing = False
        self.t_play = 0.0
        self._busy = False
        self._build_gui()
        self._load_cell(self.cell_dd.value)
        threading.Thread(target=self._anim_loop, daemon=True).start()

    # ------------------------------------------------------------------ GUI
    def _build_gui(self) -> None:
        g = self.server.gui
        g.add_markdown("## FANUC 銑削模擬平台\n選 cell、刀具、G-code，按 **執行模擬**。說明：`docs/UI_GUIDE.md`")
        with g.add_folder("1. Cell 與機器人"):
            self.cell_dd = g.add_dropdown("Cell", list(self.cells), initial_value="single_robot_table" if "single_robot_table" in self.cells else list(self.cells)[0])
            self.robot_dd = g.add_dropdown("機器人", ["fanuc_1"], initial_value="fanuc_1")
            self.env_cb = g.add_checkbox("顯示圍籬與環境", True)
            self.cell_dd.on_update(lambda _: self._load_cell(self.cell_dd.value))
            self.env_cb.on_update(lambda _: self._draw_static())
        with g.add_folder("2. G-code"):
            self.gcode_dd = g.add_dropdown("程式", list(self.gcodes), initial_value="sample_pocket_contour.nc" if "sample_pocket_contour.nc" in self.gcodes else list(self.gcodes)[0])
            self.upload_btn = g.add_upload_button("上傳 G-code (.nc)", mime_type="*/*")
            self.gcode_info = g.add_markdown("")
            self.upload_btn.on_upload(self._on_upload)
            self.gcode_dd.on_update(lambda _: self._show_gcode_info())
        with g.add_folder("3. 刀具"):
            self.tool_dd = g.add_dropdown("刀具庫", list(self.tools), initial_value=list(self.tools)[0])
            self.tool_type = g.add_dropdown("刀型", ["flat", "ball"], initial_value="flat")
            self.tool_dia = g.add_number("直徑 [mm]", 10.0, min=0.5, max=50.0, step=0.5)
            self.tool_len = g.add_number("鼻端到刀尖 [mm]", 60.0, min=0.0, max=300.0, step=1.0)
            self.tool_rpm = g.add_number("最高轉速 [rpm]", 12000.0, min=0, max=60000, step=500)
            self.tool_name = g.add_text("自訂刀具名稱", "my_tool")
            self.tool_save = g.add_button("儲存為新刀具")
            self.tool_dd.on_update(lambda _: self._fill_tool(self.tool_dd.value))
            self.tool_save.on_click(lambda _: self._save_tool())
        with g.add_folder("4. 工件位置與材料"):
            self.wp_origin = g.add_vector3("程式原點 [mm, base]", (1300.0, -100.0, 750.0), step=1.0)
            self.wp_yaw = g.add_number("程式 X 軸方位 [deg]", 0.0, min=-180, max=180, step=1.0)
            self.wp_size = g.add_vector3("工件尺寸 [mm]", (200.0, 200.0, 50.0), step=1.0)
            self.mat_dd = g.add_dropdown("材料", list(self.materials), initial_value=self.default_material)
        with g.add_folder("5. 取樣與檢查"):
            self.rapid_feed = g.add_number("G0 速度 [mm/min]", 3000.0, min=100, max=20000, step=100)
            self.spacing = g.add_number("取樣間距 [mm]", 1.0, min=0.1, max=5.0, step=0.1)
            self.sim_hz = g.add_number("碰撞檢查 [Hz]", 30.0, min=5, max=120, step=5)
            self.res_mm = g.add_number("高度圖解析度 [mm]", 0.5, min=0.1, max=2.0, step=0.1)
        with g.add_folder("6. 執行"):
            self.run_btn = g.add_button("執行模擬", color="green")
            self.progress = g.add_progress_bar(0.0)
            self.status = g.add_markdown("尚未執行")
            self.result_md = g.add_markdown("")
            self.export_btn = g.add_button("匯出軌跡給 ROS 2 bridge", disabled=True)
            self.run_btn.on_click(lambda _: threading.Thread(target=self._run, daemon=True).start())
            self.export_btn.on_click(lambda _: self._export())
        with g.add_folder("7. 播放"):
            self.play_cb = g.add_checkbox("播放", False)
            self.speed = g.add_slider("倍速", 1.0, 50.0, 1.0, 10.0)
            self.time_sl = g.add_slider("時間 [s]", 0.0, 1.0, 0.01, 0.0)
            self.time_sl.on_update(lambda _: self._on_time_slider())
        self._fill_tool(self.tool_dd.value)
        self._show_gcode_info()

    # ------------------------------------------------------------------ cell / scene
    def _load_cell(self, name: str) -> None:
        self.cell = load_cell(self.cells[name])
        self.robot_dd.options = [r.name for r in self.cell.robots]
        self.robot_dd.value = self.cell.robots[0].name
        rp = self.cell.robots[0]
        if rp.tool is not None and rp.tool.name in self.tools:
            self.tool_dd.value = rp.tool.name
            self._fill_tool(rp.tool.name)
        T = self.cell.program_in_base(rp.name)
        self.wp_origin.value = tuple((T[:3, 3] * 1e3).round(1).tolist())
        self.wp_yaw.value = float(np.degrees(np.arctan2(T[1, 0], T[0, 0])))
        stock = self.cell.stock_boxes_world()[0]
        self.wp_size.value = tuple((stock.size * 1e3).round(1).tolist())
        self._draw_robots()
        self._draw_static()
        self._clear_results()

    def _draw_robots(self) -> None:
        for h in [h for d in self.link_handles.values() for h in d.values()] + list(self.tool_handles.values()):
            h.remove()
        self.link_handles, self.tool_handles, self.chains = {}, {}, {}
        for rp in self.cell.robots:
            key = str(rp.robot.urdf)
            if key not in self.meshes_cache:
                self.meshes_cache[key] = load_visual_meshes(rp.robot.urdf)
            self.chains[rp.name] = load_chain(rp.robot.urdf, rp.robot.base_link, rp.robot.tcp_link)
            self.link_handles[rp.name] = {}
            for link, m in self.meshes_cache[key].items():
                self.link_handles[rp.name][link] = self.server.scene.add_mesh_trimesh(f"/robots/{rp.name}/{link}", m)
            q = rp.robot.named_states.get("home", np.zeros(6))
            self._pose_robot(rp.name, q)

    def _tool_mesh(self, tool: ToolConfig) -> trimesh.Trimesh:
        m = trimesh.creation.cylinder(radius=tool.radius, height=max(tool.length_from_nose, 1e-3), sections=24)
        m.apply_translation([0, 0, tool.length_from_nose / 2])  # base at origin, extends +Z
        m.visual.face_colors = np.array([70, 70, 75, 255], dtype=np.uint8)
        return m

    def _pose_robot(self, name: str, q: np.ndarray) -> None:
        rp = self.cell.robot(name)
        chain = self.chains[name]
        Ts = chain.link_transforms(q)
        for link, h in self.link_handles[name].items():
            if link not in Ts:
                continue
            Tw = rp.base_xform @ Ts[link]
            h.position = Tw[:3, 3]
            h.wxyz = _wxyz(Tw)
        tool = self._current_tool() if name == self.robot_dd.value else rp.tool
        if tool is not None:
            T_nose = rp.base_xform @ Ts[rp.robot.tcp_link]
            a = T_nose[:3, :3] @ rp.robot.tool_axis_in_tcp
            Rz = np.eye(4)
            from ..tool import rotation_z_to

            Rz[:3, :3] = rotation_z_to(a)
            Rz[:3, 3] = T_nose[:3, 3]
            if name not in self.tool_handles:
                self.tool_handles[name] = self.server.scene.add_mesh_trimesh(f"/robots/{name}/tool", self._tool_mesh(tool))
            self.tool_handles[name].position = Rz[:3, 3]
            self.tool_handles[name].wxyz = _wxyz(Rz)

    def _draw_static(self) -> None:
        for name in ("table", "stocks", "env"):
            try:
                self.server.scene.remove_by_name(f"/static/{name}")
            except Exception:
                pass
        cell = self._cell_with_overrides()
        tb = cell.table_box()
        if tb is not None:
            self.server.scene.add_box("/static/table/box", color=tuple(int(c * 255) for c in tb.color), dimensions=tuple(tb.size), position=tuple(tb.center))
        for i, b in enumerate(cell.stock_boxes_world()):
            self.server.scene.add_box(f"/static/stocks/{i}", color=(205, 195, 160), dimensions=tuple(b.size), position=tuple(b.center), opacity=0.6)
        if self.env_cb.value:
            for i, b in enumerate(cell.environment_boxes()):
                self.server.scene.add_box(f"/static/env/{i}", color=tuple(int(c * 255) for c in b.color), dimensions=tuple(b.size), position=tuple(b.center), opacity=(b.opacity if b.opacity < 1 else None))
        self.server.scene.add_grid("/static/env/grid", width=12, height=10, position=(1.5, 0, -0.003))

    # ------------------------------------------------------------------ helpers
    def _current_tool(self) -> ToolConfig:
        return ToolConfig(self.tool_name.value or self.tool_dd.value, self.tool_type.value, self.tool_dia.value * 1e-3, 0.0, self.tool_len.value * 1e-3, self.tool_dia.value * 1e-3, 2, float(self.tool_rpm.value))

    def _fill_tool(self, name: str) -> None:
        t = load_tool(self.tools[name])
        self.tool_type.value = t.type
        self.tool_dia.value = t.diameter * 1e3
        self.tool_len.value = t.length_from_nose * 1e3
        self.tool_rpm.value = t.max_rpm
        self.tool_name.value = t.name
        if self.cell is not None and self.robot_dd.value in self.tool_handles:
            self.tool_handles[self.robot_dd.value].remove()
            del self.tool_handles[self.robot_dd.value]
            self._pose_robot(self.robot_dd.value, self.cell.robot(self.robot_dd.value).robot.named_states.get("home", np.zeros(6)))

    def _save_tool(self) -> None:
        t = self._current_tool()
        name = "".join(ch if ch.isalnum() or ch in "_-" else "_" for ch in t.name) or "custom_tool"
        p = REPO_ROOT / "configs/tools" / f"{name}.yaml"
        p.write_text(yaml.safe_dump({"name": name, "type": t.type, "diameter_mm": t.diameter * 1e3, "flute_length_mm": t.flute_length * 1e3, "length_from_nose_mm": t.length_from_nose * 1e3, "shank_diameter_mm": t.shank_diameter * 1e3, "flutes": t.flutes, "max_rpm": t.max_rpm}, sort_keys=False))
        self.tools = list_tools()
        self.tool_dd.options = list(self.tools)
        self.tool_dd.value = name
        self.status.content = f"已儲存刀具 `configs/tools/{name}.yaml`"

    def _cell_with_overrides(self) -> CellConfig:
        cell = copy.deepcopy(self.cell)
        robot = self.robot_dd.value
        rp = cell.robot(robot)
        rp.tool = self._current_tool()
        o = np.asarray(self.wp_origin.value, dtype=float) * 1e-3
        yaw = float(np.radians(self.wp_yaw.value))
        size = [float(v) for v in self.wp_size.value]
        cell.programs[robot] = {"frame": f"{robot}_base", "origin_xyz_m": o.tolist(), "origin_rpy_rad": [0.0, 0.0, yaw]}
        if cell.stocks:
            cell.stocks[0] = dict(cell.stocks[0], frame=f"{robot}_base", origin_xyz_m=o.tolist(), origin_rpy_rad=[0.0, 0.0, yaw], size_xyz_mm=size)
        else:
            cell.workpiece = dict(cell.workpiece, frame=f"{robot}_base", origin_xyz_m=o.tolist(), origin_rpy_rad=[0.0, 0.0, yaw], stock_size_xyz_mm=size)
        return cell

    def _gcode_text(self) -> tuple[str, str]:
        name = self.gcode_dd.value
        if name in self.uploaded:
            return self.uploaded[name], name
        return self.gcodes[name].read_text(), name

    def _on_upload(self, event) -> None:
        f = event.target.value
        text = f.content.decode("utf-8", errors="replace")
        self.uploaded[f.name] = text
        self.gcode_dd.options = list(self.gcodes) + list(self.uploaded)
        self.gcode_dd.value = f.name
        UI_OUT.mkdir(parents=True, exist_ok=True)
        (UI_OUT / "uploads").mkdir(exist_ok=True)
        (UI_OUT / "uploads" / f.name).write_text(text)
        self._show_gcode_info()

    def _show_gcode_info(self) -> None:
        try:
            from ..gcode import parse_gcode

            text, name = self._gcode_text()
            prog = parse_gcode(text, rapid_feed_mm_min=float(self.rapid_feed.value), source=name)
            lo, hi = prog.bounds()
            self.gcode_info.content = f"`{name}`：{len(prog.segments)} 段，切削 {prog.cutting_length*1e3:.0f} mm，範圍 X {lo[0]*1e3:.0f}…{hi[0]*1e3:.0f}、Y {lo[1]*1e3:.0f}…{hi[1]*1e3:.0f}、Z {lo[2]*1e3:.1f}…{hi[2]*1e3:.1f} mm" + (f"；⚠️ {len(prog.warnings)} 個警告" if prog.warnings else "")
        except Exception as e:  # noqa: BLE001
            self.gcode_info.content = f"⚠️ 解析失敗：{e}"

    def _clear_results(self) -> None:
        self.plan, self.replay_checks, self.removal = None, None, None
        for h in (self.trail_handle, self.surface_handle):
            if h is not None:
                h.remove()
        self.trail_handle = self.surface_handle = None
        self.export_btn.disabled = True
        self.result_md.content = ""

    # ------------------------------------------------------------------ run
    def _run(self) -> None:
        if self._busy:
            return
        self._busy = True
        self.run_btn.disabled = True
        self.play_cb.value = False
        self._clear_results()
        try:
            cell = self._cell_with_overrides()
            robot = self.robot_dd.value
            tool = cell.robot(robot).tool
            text, name = self._gcode_text()
            self._draw_static()

            def prog_cb(p, s):
                self.progress.value = p * 100
                self.status.content = s

            prm = SamplingParams(max_spacing=float(self.spacing.value) * 1e-3)
            plan = plan_trajectory(cell, robot, text, tool, prm, rapid_feed_mm_min=float(self.rapid_feed.value), gcode_name=name, progress=prog_cb)
            self.plan = plan
            self._draw_trail(cell, robot, plan.traj)
            self.time_sl.max = float(plan.traj.t[-1])
            self.time_sl.value = 0.0
            checks = check_replay(cell, {robot: plan.traj}, sim_hz=float(self.sim_hz.value), progress=prog_cb)
            self.replay_checks = checks
            rem = material_removal(cell, {robot: plan.traj}, {robot: tool}, material=self.mat_dd.value, resolution_mm=float(self.res_mm.value), progress=prog_cb)
            self.removal = rem
            self._draw_surface(rem)
            self.result_md.content = self._result_text(plan, checks, rem)
            self.export_btn.disabled = False
            self.progress.value = 100
            self.status.content = "完成" if (plan.checks["pass"] and checks["pass"]) else "完成，但有檢查未通過（看下方）"
            self.play_cb.value = True
        except Exception as e:  # noqa: BLE001
            self.status.content = f"⚠️ 失敗：{e}"
            self.result_md.content = "```\n" + traceback.format_exc()[-1500:] + "\n```"
        finally:
            self._busy = False
            self.run_btn.disabled = False

    def _result_text(self, plan: PlanResult, checks: dict, rem: RemovalResult) -> str:
        c = plan.checks
        pr = checks["per_robot"][self.robot_dd.value]
        rr = rem.per_robot[self.robot_dd.value]
        ok = lambda b: "✅" if b else "❌"  # noqa: E731
        lines = [
            f"**IK** {ok(c['pass'])} 位置誤差 {c['fk_pos_err_max_mm']:.4f} mm、刀軸 {c['fk_dir_err_max_deg']:.4f}°、未收斂 {c['n_not_converged']}、速度 {c['vel_ratio_max']*100:.1f}% 上限、極限餘裕 {c['limit_margin_min']:.2f}",
            f"**軌跡** {plan.traj.n} 點、{plan.traj.duration:.1f} s；關節範圍 [deg] min {c['q_range_deg']['min']} max {c['q_range_deg']['max']}",
            f"**Newton 碰撞** {ok(checks['pass'])} 非預期碰撞 {checks['n_unexpected_collision_frames']} frames、刀尖追蹤 {pr['tcp_err_max_mm']:.4f} mm、刀具最深 {pr['spindle_stock_min_mm']:.2f} mm（程式 {-pr['programmed_max_depth_mm']:.2f}）"
            + (f"、與環境最近 {checks['robot_env_min_mm']:.0f} mm" if checks["robot_env_min_mm"] is not None else "、與圍籬無接觸候選"),
            f"**材料移除** 移除 {rem.total_volume_mm3:.0f} mm³、最深 {rem.max_depth_mm:.2f} mm、切削 {rr['cutting_time_s']:.0f} s、MRR 最大 {rr['mrr_max_mm3_s']:.0f} mm³/s",
            f"**切削力／偏移（估計）** 最大 {rr['force_max_N']:.1f} N、刀尖靜態偏移最大 {rr['deflection_max_mm']:.3f} mm、關節力矩最大 {rr['joint_torque_max_Nm']} N·m（材料 {self.mat_dd.value}；k_c 與關節剛性為佔位值）",
        ]
        if checks["unexpected_first"]:
            u = checks["unexpected_first"][0]
            lines.append(f"⚠️ 第一個非預期碰撞：t = {u['t']:.2f} s，{u['pairs'][0][0]} ↔ {u['pairs'][0][1]}，{u['pairs'][0][2]*1e3:.2f} mm")
        return "\n\n".join(lines)

    def _draw_trail(self, cell: CellConfig, robot: str, traj: JointTrajectory) -> None:
        T_wb = cell.robot(robot).base_xform
        P = (T_wb[:3, :3] @ traj.tcp_pos.T).T + T_wb[:3, 3]
        cut = (traj.motion[1:] != "rapid") & (traj.motion[:-1] != "rapid")
        seg = np.stack([P[:-1], P[1:]], axis=1)
        colors = np.where(cut[:, None, None], np.array([[[255, 120, 20]]]), np.array([[[150, 150, 150]]])).astype(np.uint8)
        colors = np.broadcast_to(colors, seg.shape).copy()
        if self.trail_handle is not None:
            self.trail_handle.remove()
        self.trail_handle = self.server.scene.add_line_segments("/results/toolpath", seg.astype(np.float32), colors, thickness=0.002)

    def _draw_surface(self, rem: RemovalResult) -> None:
        step = max(1, int(round(2.0 / rem.heightmap.dx / 1e3)))
        V, F = surface_mesh(rem.heightmap, step=step)
        if self.surface_handle is not None:
            self.surface_handle.remove()
        self.surface_handle = self.server.scene.add_mesh_simple("/results/machined_surface", V.astype(np.float32), F.astype(np.uint32), color=(225, 210, 160), side="double")

    def _export(self) -> None:
        if self.plan is None:
            return
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        name = Path(self.plan.traj.meta["gcode"]).stem
        folder = UI_OUT / f"{stamp}_{name}_{self.robot_dd.value}" / "trajectory"
        write_trajectory(folder, self.plan.traj)
        self.status.content = f"已匯出：`{folder.relative_to(REPO_ROOT)}`。送真機／假硬體：`ros2 launch milling_bridge play_trajectory.launch.py trajectory_dir:={folder} output_dir:=... dry_run:=false speed_scale:=0.1`"

    # ------------------------------------------------------------------ playback
    def _on_time_slider(self) -> None:
        if self.plan is not None and not self.playing:
            self._show_time(self.time_sl.value)

    def _show_time(self, t: float) -> None:
        if self.plan is None:
            return
        q = self.plan.traj.interp_q(np.array([t]))[0]
        self._pose_robot(self.robot_dd.value, q)

    def _anim_loop(self) -> None:
        last = time.perf_counter()
        while True:
            time.sleep(1 / 30)
            now = time.perf_counter()
            dt = now - last
            last = now
            if self.plan is None or not self.play_cb.value:
                self.playing = False
                continue
            self.playing = True
            t = self.time_sl.value + dt * float(self.speed.value)
            if t > self.plan.traj.t[-1]:
                t = 0.0
            self.time_sl.value = t
            try:
                self._show_time(t)
            except Exception:  # noqa: BLE001
                pass


def main(host: str = "0.0.0.0", port: int = 8080) -> None:
    app = MillingApp(host, port)
    print(f"Milling_Control UI: http://localhost:{port}", flush=True)
    while True:
        time.sleep(1.0)
