#!/usr/bin/env python3
"""Turn a logo image (PNG/SVG-rendered PNG) into a 3-axis engraving G-code: outline contours plus
hatch fill of the dark regions, for a small ball/flat tool.

    ~/envs/aisc_sim/bin/python scripts/make_logo_gcode.py assets/logo/ccu_emblem_1600.png \
        --out assets/gcode/ccu_emblem.nc --width-mm 160 --depth-mm 0.8 --tool-mm 2.0

Pixels darker than --threshold (on the alpha-composited image over white) are "ink" and get
engraved. Program frame: origin at the stock's top corner, logo centred at (--cx, --cy).
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np


def load_mask(path: Path, threshold: int, min_width_px: int = 1200) -> np.ndarray:
    img = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if img is None:
        raise SystemExit(f"cannot read {path}")
    if img.ndim == 3 and img.shape[2] == 4:
        alpha = img[:, :, 3:4].astype(np.float32) / 255.0
        rgb = img[:, :, :3].astype(np.float32)
        rgb = rgb * alpha + 255.0 * (1 - alpha)  # composite over white
        gray = cv2.cvtColor(rgb.astype(np.uint8), cv2.COLOR_BGR2GRAY)
    elif img.ndim == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img
    if gray.shape[1] < min_width_px:
        f = min_width_px / gray.shape[1]
        gray = cv2.resize(gray, None, fx=f, fy=f, interpolation=cv2.INTER_CUBIC)
    mask = (gray < threshold).astype(np.uint8)
    return mask


def contours_mm(mask: np.ndarray, px_to_mm: float, origin_px: np.ndarray, min_len_mm: float, simplify_mm: float):
    cs, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    out = []
    for c in cs:
        c = c[:, 0, :].astype(np.float64)
        if len(c) < 3:
            continue
        eps = simplify_mm / px_to_mm
        c = cv2.approxPolyDP(c.astype(np.float32), eps, True)[:, 0, :].astype(np.float64)
        if len(c) < 3:
            continue
        pts = (c - origin_px) * px_to_mm
        pts[:, 1] *= -1  # image y down -> program y up
        closed = np.vstack([pts, pts[:1]])
        length = np.linalg.norm(np.diff(closed, axis=0), axis=1).sum()
        if length >= min_len_mm:
            out.append(closed)
    return out


def hatch_mm(mask: np.ndarray, px_to_mm: float, origin_px: np.ndarray, pitch_mm: float, inset_px: int):
    """Horizontal scanlines through the eroded mask; returns a list of (x0, x1, y) in mm, zig-zag ordered."""
    k = max(1, inset_px)
    eroded = cv2.erode(mask, np.ones((2 * k + 1, 2 * k + 1), np.uint8))
    step = max(1, int(round(pitch_mm / px_to_mm)))
    lines = []
    for row_i, r in enumerate(range(0, mask.shape[0], step)):
        row = eroded[r]
        if not row.any():
            continue
        d = np.diff(np.concatenate([[0], row, [0]]).astype(int))
        starts = np.where(d == 1)[0]
        ends = np.where(d == -1)[0] - 1
        segs = [(s, e) for s, e in zip(starts, ends) if e - s >= 2]
        if row_i % 2 == 1:
            segs = segs[::-1]
        for s, e in segs:
            xs = (np.array([s, e], dtype=float) - origin_px[0]) * px_to_mm
            if row_i % 2 == 1:
                xs = xs[::-1]
            y = -(r - origin_px[1]) * px_to_mm
            lines.append((xs[0], xs[1], y))
    return lines


def _f(v: float) -> str:
    return f"{v:.3f}".rstrip("0").rstrip(".") if abs(v) >= 1e-9 else "0"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("image", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--width-mm", type=float, default=160.0)
    ap.add_argument("--cx", type=float, default=100.0)
    ap.add_argument("--cy", type=float, default=100.0)
    ap.add_argument("--depth-mm", type=float, default=0.8)
    ap.add_argument("--tool-mm", type=float, default=2.0)
    ap.add_argument("--threshold", type=int, default=128)
    ap.add_argument("--feed", type=float, default=1200.0)
    ap.add_argument("--plunge", type=float, default=300.0)
    ap.add_argument("--safe-z", type=float, default=5.0)
    ap.add_argument("--rpm", type=float, default=15000.0)
    ap.add_argument("--min-len-mm", type=float, default=3.0)
    ap.add_argument("--no-fill", action="store_true")
    ap.add_argument("--preview", type=Path, default=None, help="write a PNG preview of the toolpaths")
    args = ap.parse_args()

    mask = load_mask(args.image, args.threshold)
    h, w = mask.shape
    px_to_mm = args.width_mm / w
    origin_px = np.array([w / 2.0, h / 2.0])
    contours = contours_mm(mask, px_to_mm, origin_px, args.min_len_mm, simplify_mm=0.05)
    lines = [] if args.no_fill else hatch_mm(mask, px_to_mm, origin_px, pitch_mm=args.tool_mm * 0.6, inset_px=int(round(args.tool_mm * 0.5 / px_to_mm)))

    L = []
    a = L.append
    a(f"( {args.image.name}: engraving, {len(contours)} contours, {len(lines)} hatch lines, tool {args.tool_mm} mm, depth {args.depth_mm} mm )")
    a("G21 G90 G17 G54")
    a(f"S{_f(args.rpm)} M3")
    a(f"G0 Z{_f(args.safe_z)}")
    z = -abs(args.depth_mm)
    first_feed = True

    def feed_word():
        nonlocal first_feed
        if first_feed:
            first_feed = False
            return f" F{_f(args.feed)}"
        return ""

    # hatch fill first (zig-zag rows), lifting between separate segments
    prev_end = None
    for x0, x1, y in lines:
        p0 = np.array([x0 + args.cx, y + args.cy])
        p1 = np.array([x1 + args.cx, y + args.cy])
        if prev_end is None or np.linalg.norm(p0 - prev_end) > args.tool_mm * 1.5:
            if prev_end is not None:
                a(f"G0 Z{_f(args.safe_z)}")
            a(f"G0 X{_f(p0[0])} Y{_f(p0[1])}")
            a(f"G1 Z{_f(z)} F{_f(args.plunge)}")
            first_feed = True
        else:
            a(f"G1 X{_f(p0[0])} Y{_f(p0[1])}{feed_word()}")
        a(f"G1 X{_f(p1[0])} Y{_f(p1[1])}{feed_word()}")
        prev_end = p1
    if prev_end is not None:
        a(f"G0 Z{_f(args.safe_z)}")
    # outlines
    for c in contours:
        c = c + np.array([args.cx, args.cy])
        a(f"G0 X{_f(c[0, 0])} Y{_f(c[0, 1])}")
        a(f"G1 Z{_f(z)} F{_f(args.plunge)}")
        first_feed = True
        for x, y in c[1:]:
            a(f"G1 X{_f(x)} Y{_f(y)}{feed_word()}")
        a(f"G0 Z{_f(args.safe_z)}")
    a("M5")
    a("G0 X0 Y0")
    a("M30")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text("\n".join(L) + "\n")
    cut_len = sum(np.linalg.norm(np.diff(c, axis=0), axis=1).sum() for c in contours) + sum(abs(x1 - x0) for x0, x1, _ in lines)
    print(f"wrote {args.out}: {len(L)} lines, {len(contours)} contours, {len(lines)} hatch segments, cutting length {cut_len:.0f} mm, image {w}x{h} px -> {args.width_mm:.0f} mm")
    if args.preview:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(7, 7))
        for x0, x1, y in lines:
            ax.plot([x0 + args.cx, x1 + args.cx], [y + args.cy, y + args.cy], color="0.6", lw=0.4)
        for c in contours:
            ax.plot(c[:, 0] + args.cx, c[:, 1] + args.cy, color="C1", lw=0.6)
        ax.set_aspect("equal")
        ax.set_title(f"{args.image.name}: toolpaths ({len(contours)} contours, {len(lines)} hatch lines)")
        ax.set_xlabel("X [mm]")
        ax.set_ylabel("Y [mm]")
        fig.savefig(args.preview, dpi=130)
    return 0


if __name__ == "__main__":
    sys.exit(main())
