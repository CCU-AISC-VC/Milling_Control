#!/usr/bin/env python3
"""M8: start the student web UI (viser).

Run:  ~/envs/aisc_sim/bin/python scripts/m8_ui.py [--port 8080]
Open: http://localhost:8080   (guide: docs/UI_GUIDE.md)
"""
from __future__ import annotations

import argparse

import _bootstrap  # noqa: F401

from milling_sim.ui.app import main

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8080)
    a = ap.parse_args()
    main(a.host, a.port)
