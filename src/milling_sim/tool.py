"""Cutting tools (configs/tools/*.yaml) and their geometry relative to the spindle nose.

The robot's ``spindle_tcp`` frame is treated as the spindle *nose* reference; the tool tip is
``length_from_nose`` along the tool axis (+X of spindle_tcp, docs/SPEC.md §3.2). IK targets the
tip, Newton gets a cylinder from nose to tip, and the material-removal model uses the diameter
and tip shape (flat / ball).
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from . import REPO_ROOT


@dataclass
class ToolConfig:
    name: str
    type: str  # "flat" | "ball"
    diameter: float  # m
    flute_length: float  # m
    length_from_nose: float  # m, spindle_tcp -> tip along the tool axis
    shank_diameter: float  # m
    flutes: int = 2
    max_rpm: float = 12000.0
    path: Path | None = None

    @property
    def radius(self) -> float:
        return self.diameter / 2

    def to_meta(self) -> dict:
        return {
            "name": self.name,
            "type": self.type,
            "diameter_mm": self.diameter * 1e3,
            "flute_length_mm": self.flute_length * 1e3,
            "length_from_nose_mm": self.length_from_nose * 1e3,
            "shank_diameter_mm": self.shank_diameter * 1e3,
            "flutes": self.flutes,
            "max_rpm": self.max_rpm,
            "config": str(self.path.relative_to(REPO_ROOT)) if self.path and self.path.is_relative_to(REPO_ROOT) else (str(self.path) if self.path else None),
        }

    @staticmethod
    def from_meta(d: dict | None) -> "ToolConfig | None":
        if not d:
            return None
        return ToolConfig(d["name"], d.get("type", "flat"), d["diameter_mm"] * 1e-3, d.get("flute_length_mm", 0.0) * 1e-3, d["length_from_nose_mm"] * 1e-3, d.get("shank_diameter_mm", d["diameter_mm"]) * 1e-3, int(d.get("flutes", 2)), float(d.get("max_rpm", 12000)))


def load_tool(path: str | Path) -> ToolConfig:
    p = Path(path)
    if not p.is_absolute():
        p = REPO_ROOT / p
    d = yaml.safe_load(p.read_text())
    return ToolConfig(
        name=d["name"],
        type=d.get("type", "flat"),
        diameter=float(d["diameter_mm"]) * 1e-3,
        flute_length=float(d.get("flute_length_mm", 0.0)) * 1e-3,
        length_from_nose=float(d["length_from_nose_mm"]) * 1e-3,
        shank_diameter=float(d.get("shank_diameter_mm", d["diameter_mm"])) * 1e-3,
        flutes=int(d.get("flutes", 2)),
        max_rpm=float(d.get("max_rpm", 12000)),
        path=p,
    )


def list_tools(folder: str | Path = REPO_ROOT / "configs/tools") -> dict[str, Path]:
    return {p.stem: p for p in sorted(Path(folder).glob("*.yaml"))}


def rotation_z_to(axis: np.ndarray) -> np.ndarray:
    """Rotation matrix whose local +Z maps onto ``axis`` (unit)."""
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    t = np.array([1.0, 0.0, 0.0]) if abs(a[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    x = np.cross(t, a)
    x /= np.linalg.norm(x)
    y = np.cross(a, x)
    return np.stack([x, y, a], axis=1)
