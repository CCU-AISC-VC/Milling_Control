#!/usr/bin/env python3
"""M1: parse a G-code file, sample it into a timed TCP path, plot and summarise.

Run:  ~/envs/aisc_sim/bin/python scripts/m1_parse_and_sample.py [--gcode assets/gcode/sample_pocket_contour.nc]
Outputs: outputs/m1_gcode/<stem>/
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

from milling_sim import ASSETS_DIR, OUTPUTS_DIR, REPO_ROOT
from milling_sim.gcode import load_gcode
from milling_sim.toolpath import SamplingParams, chord_errors, sample_program
from milling_sim.viz import finish_figure, new_figure


def plot_path(prog, tp, out: Path) -> None:
    import matplotlib.pyplot as plt

    P = tp.p * 1e3
    cut = tp.is_cutting()
    fig = plt.figure(figsize=(11, 5), constrained_layout=True)
    ax3 = fig.add_subplot(1, 2, 1, projection="3d")
    ax2 = fig.add_subplot(1, 2, 2)
    # draw per segment so rapids are dashed
    for k, seg in enumerate(prog.segments):
        m = tp.segment == k
        if m.sum() < 2:
            continue
        if seg.is_cutting:
            ax3.plot(P[m, 0], P[m, 1], P[m, 2], color=plt.cm.viridis((P[m, 2].mean() + 8) / 10), lw=1.2)
            ax2.plot(P[m, 0], P[m, 1], color=plt.cm.viridis((P[m, 2].mean() + 8) / 10), lw=1.2)
        else:
            ax3.plot(P[m, 0], P[m, 1], P[m, 2], color="0.6", ls="--", lw=0.8)
            ax2.plot(P[m, 0], P[m, 1], color="0.6", ls="--", lw=0.8)
    ax3.set_xlabel("X [mm]")
    ax3.set_ylabel("Y [mm]")
    ax3.set_zlabel("Z [mm]")
    ax3.set_title("toolpath (colour = depth, dashed = rapid)")
    ax2.set_xlabel("X [mm]")
    ax2.set_ylabel("Y [mm]")
    ax2.set_aspect("equal")
    ax2.set_title("top view")
    ax2.scatter(P[cut, 0][::40], P[cut, 1][::40], s=3, color="k", zorder=3, label="samples (1/40)")
    ax2.legend(loc="upper right", fontsize=8)
    finish_figure(fig, out / "path.png")


def plot_speed(tp, out: Path) -> None:
    fig, ax = new_figure(2, 1, size=(11, 6))
    ax[0].plot(tp.t, tp.speed * 6e4, lw=0.8, label="actual speed")
    ax[0].plot(tp.t, tp.feed * 6e4, lw=0.8, ls="--", label="commanded feed")
    ax[0].set_ylabel("[mm/min]")
    ax[0].legend(loc="upper right", fontsize=8)
    ax[0].set_title("speed profile (stops at corners and rapid/cut transitions)")
    ax[1].plot(tp.t, tp.p[:, 2] * 1e3, lw=0.8)
    ax[1].set_ylabel("Z [mm]")
    ax[1].set_xlabel("t [s]")
    finish_figure(fig, out / "speed_profile.png")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--gcode", type=Path, default=ASSETS_DIR / "gcode/sample_pocket_contour.nc")
    ap.add_argument("--max-spacing-mm", type=float, default=1.0)
    ap.add_argument("--chord-tol-mm", type=float, default=0.02)
    ap.add_argument("--accel", type=float, default=1.0, help="path acceleration [m/s^2]")
    ap.add_argument("--corner-deg", type=float, default=5.0)
    ap.add_argument("--rapid-feed", type=float, default=3000.0, help="[mm/min]")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()
    out = args.out or (OUTPUTS_DIR / "m1_gcode" / args.gcode.stem)
    out.mkdir(parents=True, exist_ok=True)

    prog = load_gcode(args.gcode, rapid_feed_mm_min=args.rapid_feed)
    prm = SamplingParams(
        max_spacing=args.max_spacing_mm * 1e-3,
        chord_tol=args.chord_tol_mm * 1e-3,
        accel=args.accel,
        corner_angle_deg=args.corner_deg,
    )
    tp = sample_program(prog, prm)
    errs = chord_errors(prog, tp)
    dt = np.diff(tp.t)
    v_int = np.linalg.norm(np.diff(tp.p, axis=0), axis=1) / np.maximum(dt, 1e-12)
    a_int = np.abs(np.diff(v_int)) / np.maximum((dt[1:] + dt[:-1]) / 2, 1e-12)

    df = pd.DataFrame(
        {
            "t": tp.t,
            "x_mm": tp.p[:, 0] * 1e3,
            "y_mm": tp.p[:, 1] * 1e3,
            "z_mm": tp.p[:, 2] * 1e3,
            "speed_mm_min": tp.speed * 6e4,
            "feed_mm_min": tp.feed * 6e4,
            "spindle_rpm": tp.spindle_rpm,
            "segment": tp.segment,
            "line_no": tp.line_no,
            "motion": tp.motion,
        }
    )
    df.to_csv(out / "samples.csv", index=False)

    lo, hi = prog.bounds()
    kinds = {k: int(sum(1 for s in prog.segments if s.kind == k)) for k in ("rapid", "linear", "arc_cw", "arc_ccw")}
    summary = {
        "gcode": str(args.gcode.relative_to(REPO_ROOT)) if args.gcode.is_absolute() else str(args.gcode),
        "md5": prog.md5,
        "n_lines": prog.n_lines,
        "n_segments": len(prog.segments),
        "segments_by_kind": kinds,
        "cutting_length_mm": prog.cutting_length * 1e3,
        "rapid_length_mm": prog.rapid_length * 1e3,
        "bounds_mm": {"min": (lo * 1e3).round(3).tolist(), "max": (hi * 1e3).round(3).tolist()},
        "sampling": {"max_spacing_mm": args.max_spacing_mm, "chord_tol_mm": args.chord_tol_mm, "accel_m_s2": args.accel, "corner_deg": args.corner_deg, "rapid_feed_mm_min": args.rapid_feed},
        "n_samples": int(tp.n),
        "max_chord_error_mm": float(errs.max() * 1e3) if len(errs) else 0.0,
        "max_sample_spacing_mm": float(np.linalg.norm(np.diff(tp.p, axis=0), axis=1).max() * 1e3),
        "duration_s": tp.duration,
        "cutting_time_s": float(tp.seg_time[[s.is_cutting for s in prog.segments]].sum()),
        "min_dt_s": float(dt.min()),
        "max_interval_speed_mm_min": float(v_int.max() * 6e4),
        "max_interval_accel_m_s2": float(a_int.max()) if len(a_int) else 0.0,
        "warnings": prog.warnings,
        "pass": bool(errs.max() * 1e3 <= args.chord_tol_mm * 1.001 and dt.min() > 0 and not prog.warnings),
        "env": {"python": platform.python_version(), "numpy": np.__version__},
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=2))
    plot_path(prog, tp, out)
    plot_speed(tp, out)
    print(json.dumps({k: summary[k] for k in ("n_segments", "segments_by_kind", "cutting_length_mm", "rapid_length_mm", "n_samples", "max_chord_error_mm", "duration_s", "max_interval_accel_m_s2", "warnings", "pass")}, indent=2))
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
