"""Cell and robot configuration (configs/robots/*.yaml, configs/cells/*.yaml).

A cell is a list of robot placements plus the table and the workpiece. The G-code *program*
frame is placed in a robot's base frame by ``workpiece.origin_xyz_m`` / ``origin_rpy_rad``
(docs/SPEC.md §2). Everything here is plain NumPy; Newton is not imported.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from . import REPO_ROOT
from .urdf_kinematics import make_transform


def _abs(p: str | Path) -> Path:
    p = Path(p)
    return p if p.is_absolute() else REPO_ROOT / p


@dataclass
class RobotConfig:
    name: str
    urdf: Path
    base_link: str
    tcp_link: str
    flange_link: str
    joint_names: list[str]
    tool_axis_in_tcp: np.ndarray
    named_states: dict[str, np.ndarray]
    velocity_scale: float
    acceleration_limits: np.ndarray
    preferred_state: str = "home"
    path: Path | None = None

    @property
    def q_ref(self) -> np.ndarray:
        return self.named_states[self.preferred_state]


@dataclass
class RobotPlacement:
    name: str
    robot: RobotConfig
    base_xform: np.ndarray  # 4x4, world <- base


@dataclass
class Box:
    """Axis-aligned static box in world (used for stocks, table and environment)."""

    name: str
    center: np.ndarray
    size: np.ndarray
    color: tuple[float, float, float] = (0.6, 0.6, 0.6)
    opacity: float = 1.0
    role: str = "env"  # "stock" | "table" | "env"


@dataclass
class CellConfig:
    name: str
    robots: list[RobotPlacement]
    table: dict
    workpiece: dict
    tool: dict
    programs: dict = field(default_factory=dict)  # robot name -> program frame dict
    stocks: list = field(default_factory=list)  # physical stock boxes
    environment: dict = field(default_factory=dict)
    cameras: dict = field(default_factory=dict)
    path: Path | None = None
    raw: dict = field(default_factory=dict)

    def robot(self, name: str) -> RobotPlacement:
        for r in self.robots:
            if r.name == name:
                return r
        raise KeyError(name)

    # ------------------------------------------------------------------ frames
    def _frame_in_world(self, d: dict) -> np.ndarray:
        frame = d.get("frame", "world")
        T_local = make_transform(d["origin_xyz_m"], d.get("origin_rpy_rad", [0, 0, 0]))
        if frame == "world":
            return T_local
        if frame.endswith("_base"):
            return self.robot(frame[: -len("_base")]).base_xform @ T_local
        raise ValueError(f"unknown frame {frame!r}")

    def program_in_world(self, robot_name: str | None = None) -> np.ndarray:
        """4x4 world <- program. Per-robot ``programs`` entry if present, else ``workpiece``."""
        if robot_name is not None and robot_name in self.programs:
            return self._frame_in_world(self.programs[robot_name])
        if not self.workpiece:
            raise ValueError(f"no program frame for robot {robot_name!r} and no workpiece")
        return self._frame_in_world(self.workpiece)

    def program_in_base(self, robot_name: str) -> np.ndarray:
        """4x4 base <- program for the given robot."""
        T_wb = self.robot(robot_name).base_xform
        return np.linalg.inv(T_wb) @ self.program_in_world(robot_name)

    # ------------------------------------------------------------------ boxes
    def stock_boxes_world(self) -> list[Box]:
        """Stocks in world; the origin is the stock's TOP corner, box extends +X, +Y, -Z."""
        items = self.stocks or ([dict(self.workpiece, size_xyz_mm=self.workpiece["stock_size_xyz_mm"], name="stock")] if self.workpiece else [])
        out = []
        for i, d in enumerate(items):
            sx, sy, sz = np.asarray(d["size_xyz_mm"], dtype=float) * 1e-3
            T = self._frame_in_world(d)
            c = (T @ np.array([sx / 2, sy / 2, -sz / 2, 1.0]))[:3]
            out.append(Box(d.get("name", "stock" if i == 0 else f"stock_{i+1}"), c, np.array([sx, sy, sz]), tuple(d.get("color", (0.80, 0.76, 0.62))), float(d.get("opacity", 0.55)), "stock"))
        return out

    def stock_box_world(self) -> tuple[np.ndarray, np.ndarray]:
        """(center, size) of the first stock (backward compatible)."""
        b = self.stock_boxes_world()[0]
        return b.center, b.size

    def table_box(self) -> Box | None:
        tb = self.table
        if not tb:
            return None
        c = np.array([tb["center_xy_m"][0], tb["center_xy_m"][1], tb["top_z_m"] - tb["size_xyz_m"][2] / 2])
        return Box("table", c, np.asarray(tb["size_xyz_m"], dtype=float), tuple(tb.get("color", (0.35, 0.37, 0.40))), 1.0, "table")

    def environment_boxes(self) -> list[Box]:
        """Floor, safety fence (posts + panels with gate openings) and extra boxes."""
        env = self.environment
        out: list[Box] = []
        if not env:
            return out
        fl = env.get("floor")
        if fl:
            t = float(fl.get("thickness_m", 0.02))
            c = np.array([fl["center_xy_m"][0], fl["center_xy_m"][1], float(fl.get("top_z_m", 0.0)) - t / 2])
            out.append(Box("floor", c, np.array([fl["size_xy_m"][0], fl["size_xy_m"][1], t]), tuple(fl.get("color", (0.22, 0.23, 0.25))), 1.0, "env"))
        fence = env.get("fence")
        if fence:
            out += _fence_boxes(fence)
        for b in env.get("boxes", []):
            out.append(Box(b["name"], np.asarray(b["center_m"], dtype=float), np.asarray(b["size_m"], dtype=float), tuple(b.get("color", (0.6, 0.6, 0.6))), float(b.get("opacity", 1.0)), "env"))
        return out


def _fence_boxes(f: dict) -> list[Box]:
    (x0, y0), (x1, y1) = f["rect_xy_m"]
    h = float(f.get("height_m", 2.0))
    ps = float(f.get("post_size_m", 0.07))
    spacing = float(f.get("post_spacing_m", 1.6))
    pt = float(f.get("panel_thickness_m", 0.02))
    gap = float(f.get("panel_bottom_gap_m", 0.12))
    c_post = tuple(f.get("colors", {}).get("post", (1.0, 0.78, 0.05)))
    c_panel = tuple(f.get("colors", {}).get("panel", (0.12, 0.13, 0.16)))
    op = float(f.get("panel_opacity", 0.35))
    gates = f.get("gates", [])
    # sides: (name, fixed axis value, moving axis range, moving axis index)
    sides = [("-y", y0, (x0, x1), 0), ("+x", x1, (y0, y1), 1), ("+y", y1, (x0, x1), 0), ("-x", x0, (y0, y1), 1)]
    out: list[Box] = []
    k = 0
    for name, fixed, (a0, a1), ax in sides:
        L = a1 - a0
        n = max(1, int(np.ceil(L / spacing)))
        pos = np.linspace(a0, a1, n + 1)
        for i, a in enumerate(pos):
            c = np.zeros(3)
            c[ax] = a
            c[1 - ax] = fixed
            c[2] = h / 2
            if name in ("+y", "-x") and i == len(pos) - 1:
                continue  # corner post already placed by the previous side
            if name in ("+x", "-y") and i == 0 and name == "+x":
                continue
            out.append(Box(f"fence_post_{k}", c, np.array([ps, ps, h]), c_post, 1.0, "env"))
            k += 1
        for i in range(n):
            s0, s1 = pos[i], pos[i + 1]
            skip = False
            for g in gates:
                if g.get("side") == name:
                    gc, gw = float(g["center_m"]), float(g["width_m"])
                    if s1 > gc - gw / 2 and s0 < gc + gw / 2:
                        skip = True
            if skip:
                continue
            c = np.zeros(3)
            c[ax] = (s0 + s1) / 2
            c[1 - ax] = fixed
            c[2] = gap + (h - gap) / 2
            size = np.zeros(3)
            size[ax] = (s1 - s0) - ps
            size[1 - ax] = pt
            size[2] = h - gap
            out.append(Box(f"fence_panel_{name}_{i}", c, size, c_panel, op, "env"))
    return out


def load_robot_config(path: str | Path) -> RobotConfig:
    p = _abs(path)
    d = yaml.safe_load(p.read_text())
    return RobotConfig(
        name=d["name"],
        urdf=_abs(d["urdf"]),
        base_link=d.get("base_link", "base_link"),
        tcp_link=d["tcp_link"],
        flange_link=d.get("flange_link", "flange"),
        joint_names=list(d["joint_names"]),
        tool_axis_in_tcp=np.asarray(d["tool_axis_in_tcp"], dtype=float),
        named_states={k: np.asarray(v, dtype=float) for k, v in d.get("named_states", {}).items()},
        velocity_scale=float(d.get("velocity_scale", 1.0)),
        acceleration_limits=np.asarray(d.get("acceleration_limits_rad_s2", [np.inf] * 6), dtype=float),
        preferred_state=d.get("preferred_state", "home"),
        path=p,
    )


def load_cell(path: str | Path) -> CellConfig:
    p = _abs(path)
    d = yaml.safe_load(p.read_text())
    robots = []
    for r in d["robots"]:
        cfg = load_robot_config(r["robot"])
        robots.append(RobotPlacement(r["name"], cfg, make_transform(r.get("base_xyz_m", [0, 0, 0]), r.get("base_rpy_rad", [0, 0, 0]))))
    return CellConfig(
        name=d["name"],
        robots=robots,
        table=d.get("table", {}),
        workpiece=d.get("workpiece", {}),
        tool=d.get("tool", {}),
        programs=d.get("programs", {}),
        stocks=d.get("stocks", []),
        environment=d.get("environment", {}),
        cameras=d.get("cameras", {}),
        path=p,
        raw=d,
    )
