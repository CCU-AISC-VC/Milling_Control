"""Minimal reader for the Milling_Control trajectory folder (trajectory.csv + meta.json).

Deliberately independent of ``milling_sim`` so the ROS 2 side only needs the standard library.
"""
from __future__ import annotations

import csv
import json
from dataclasses import dataclass
from pathlib import Path


@dataclass
class TrajectoryFile:
    t: list[float]
    q: list[list[float]]  # (N, dof) rad
    joint_names: list[str]
    meta: dict

    @property
    def n(self) -> int:
        return len(self.t)

    @property
    def duration(self) -> float:
        return self.t[-1] - self.t[0] if self.t else 0.0


def read_trajectory(folder: str | Path) -> TrajectoryFile:
    folder = Path(folder)
    meta = json.loads((folder / "meta.json").read_text())
    names = meta["joint_names"]
    t: list[float] = []
    q: list[list[float]] = []
    with (folder / "trajectory.csv").open() as f:
        for row in csv.DictReader(f):
            t.append(float(row["t"]))
            q.append([float(row[n]) for n in names])
    if meta.get("format_version", 1) != 1:
        raise ValueError(f"unsupported trajectory format_version {meta.get('format_version')}")
    return TrajectoryFile(t, q, names, meta)
