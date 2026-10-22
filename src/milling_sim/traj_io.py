"""Trajectory folder format shared by simulation and the real robot (docs/SPEC.md §5).

``<name>/trajectory.csv`` + ``<name>/meta.json``. The ROS 2 bridge reads only ``t`` and the
joint columns; everything else is for simulation and analysis.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import pandas as pd

FORMAT_VERSION = 1


@dataclass
class JointTrajectory:
    t: np.ndarray
    q: np.ndarray  # (N, dof) rad
    joint_names: list[str]
    tcp_pos: np.ndarray  # (N, 3) m in base_link
    tcp_quat_xyzw: np.ndarray  # (N, 4)
    feed_mm_min: np.ndarray
    spindle_rpm: np.ndarray
    segment: np.ndarray
    motion: np.ndarray
    meta: dict = field(default_factory=dict)

    @property
    def n(self) -> int:
        return len(self.t)

    @property
    def duration(self) -> float:
        return float(self.t[-1] - self.t[0]) if self.n else 0.0

    def qd(self) -> np.ndarray:
        """Joint velocities by central differences (rad/s)."""
        return np.gradient(self.q, self.t, axis=0)

    def qdd(self) -> np.ndarray:
        return np.gradient(self.qd(), self.t, axis=0)

    def interp_q(self, t: np.ndarray) -> np.ndarray:
        t = np.clip(np.asarray(t, dtype=float), self.t[0], self.t[-1])
        return np.stack([np.interp(t, self.t, self.q[:, j]) for j in range(self.q.shape[1])], axis=1)


def write_trajectory(folder: str | Path, traj: JointTrajectory) -> Path:
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    cols = {"t": traj.t}
    for j, name in enumerate(traj.joint_names):
        cols[name] = traj.q[:, j]
    cols.update(
        {
            "x": traj.tcp_pos[:, 0],
            "y": traj.tcp_pos[:, 1],
            "z": traj.tcp_pos[:, 2],
            "qx": traj.tcp_quat_xyzw[:, 0],
            "qy": traj.tcp_quat_xyzw[:, 1],
            "qz": traj.tcp_quat_xyzw[:, 2],
            "qw": traj.tcp_quat_xyzw[:, 3],
            "feed_mm_min": traj.feed_mm_min,
            "spindle_rpm": traj.spindle_rpm,
            "segment": traj.segment,
            "motion": traj.motion,
        }
    )
    pd.DataFrame(cols).to_csv(folder / "trajectory.csv", index=False, float_format="%.9g")
    meta = dict(traj.meta)
    meta.setdefault("format_version", FORMAT_VERSION)
    meta.setdefault("created_utc", datetime.now(timezone.utc).isoformat(timespec="seconds"))
    meta["joint_names"] = traj.joint_names
    meta["n_samples"] = int(traj.n)
    meta["duration_s"] = traj.duration
    (folder / "meta.json").write_text(json.dumps(meta, indent=2, default=_json_default))
    return folder


def _json_default(o):
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    if isinstance(o, Path):
        return str(o)
    return str(o)


def read_trajectory(folder: str | Path) -> JointTrajectory:
    folder = Path(folder)
    meta = json.loads((folder / "meta.json").read_text())
    df = pd.read_csv(folder / "trajectory.csv")
    names = meta["joint_names"]
    return JointTrajectory(
        t=df["t"].to_numpy(),
        q=df[names].to_numpy(),
        joint_names=names,
        tcp_pos=df[["x", "y", "z"]].to_numpy(),
        tcp_quat_xyzw=df[["qx", "qy", "qz", "qw"]].to_numpy(),
        feed_mm_min=df["feed_mm_min"].to_numpy(),
        spindle_rpm=df["spindle_rpm"].to_numpy(),
        segment=df["segment"].to_numpy(),
        motion=df["motion"].to_numpy().astype(str),
        meta=meta,
    )
