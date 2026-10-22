#!/usr/bin/env python3
"""M1: write the self-generated sample G-code programs to assets/gcode/.

Run:  ~/envs/aisc_sim/bin/python scripts/m1_make_sample_gcode.py
"""
from __future__ import annotations

import sys

import _bootstrap  # noqa: F401

from milling_sim import ASSETS_DIR
from milling_sim.sample_gcode import make_sample_5axis_wave, make_sample_face_boss, make_sample_pocket_contour, make_unit_square


def main() -> int:
    out = ASSETS_DIR / "gcode"
    out.mkdir(parents=True, exist_ok=True)
    files = {
        "sample_pocket_contour.nc": make_sample_pocket_contour(),
        "unit_square_100mm.nc": make_unit_square(),
        "sample_face_boss.nc": make_sample_face_boss(),
        "sample_5axis_wave.nc": make_sample_5axis_wave(),
    }
    for name, text in files.items():
        (out / name).write_text(text)
        print(f"wrote {out / name} ({len(text.splitlines())} lines)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
