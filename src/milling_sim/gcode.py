"""G-code parser for the milling dialect in docs/SPEC.md §4.

Produces a list of :class:`Segment` (rapid / linear / arc) in the *program* frame with SI
units (m, m/s). Everything modal (units, G90/G91, plane, feed, spindle) is tracked in
:class:`GState`. Unsupported words are collected in ``Program.warnings`` instead of raising,
except for things that would silently change geometry (cutter compensation, canned cycles),
which raise :class:`GCodeError`.
"""
from __future__ import annotations

import hashlib
import math
import re
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

INCH = 0.0254
MM = 1e-3

PLANES = {17: "XY", 18: "ZX", 19: "YZ"}
# (first axis, second axis, normal axis) index into xyz, and the I/J/K letters for the plane
PLANE_AXES = {"XY": (0, 1, 2), "ZX": (2, 0, 1), "YZ": (1, 2, 0)}
PLANE_IJK = {"XY": ("I", "J"), "ZX": ("K", "I"), "YZ": ("J", "K")}

_WORD_RE = re.compile(r"([A-Za-z])\s*([-+]?(?:\d+\.?\d*|\.\d+))")
_COMMENT_PAREN = re.compile(r"\([^)]*\)")


class GCodeError(ValueError):
    pass


@dataclass
class Segment:
    kind: str  # "rapid" | "linear" | "arc_cw" | "arc_ccw"
    start: np.ndarray  # (3,) m, program frame
    end: np.ndarray  # (3,) m
    feed: float  # m/s (for rapid: the configured rapid feed)
    spindle_rpm: float
    spindle_on: bool
    line_no: int  # 1-based source line
    raw: str
    plane: str = "XY"
    center: np.ndarray | None = None  # (3,) m, arc center (normal-axis coordinate = start's)
    radius: float = 0.0
    sweep: float = 0.0  # signed arc angle [rad]; + = CCW about the plane normal
    turns: int = 0  # extra full turns (P word), usually 0
    axis_start: np.ndarray | None = None  # 5-axis: tool axis unit vector (tip -> spindle) at start, program frame
    axis_end: np.ndarray | None = None  # ... at the end (None = vertical 3-axis)

    @property
    def length(self) -> float:
        if self.kind.startswith("arc"):
            a0, a1, n = PLANE_AXES[self.plane]
            helix = self.end[n] - self.start[n]
            return math.hypot(self.radius * abs(self.sweep), helix)
        return float(np.linalg.norm(self.end - self.start))

    @property
    def is_cutting(self) -> bool:
        return self.kind != "rapid"

    # ---- parametric evaluation, s in [0, 1]
    def point(self, s: float | np.ndarray) -> np.ndarray:
        """Point(s) on the segment; output shape = ``np.shape(s) + (3,)``."""
        s = np.asarray(s, dtype=float)
        if not self.kind.startswith("arc"):
            return self.start + s[..., None] * (self.end - self.start)
        a0, a1, n = PLANE_AXES[self.plane]
        c = self.center
        ang0 = math.atan2(self.start[a1] - c[a1], self.start[a0] - c[a0])
        ang = ang0 + s * self.sweep
        out = np.zeros(s.shape + (3,))
        out[..., a0] = c[a0] + self.radius * np.cos(ang)
        out[..., a1] = c[a1] + self.radius * np.sin(ang)
        out[..., n] = self.start[n] + s * (self.end[n] - self.start[n])
        return out


@dataclass
class GState:
    units: float = MM  # multiplier to m
    absolute: bool = True
    plane: str = "XY"
    feed: float = 0.0  # m/s
    spindle_rpm: float = 0.0
    spindle_on: bool = False
    motion: int | None = None  # modal G0/1/2/3
    pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    work_offset: str = "G54"
    axis: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 1.0]))  # tool axis (tip -> spindle), modal
    five_axis: bool = False  # set once any G0/G1 carried an I J K tool vector


@dataclass
class Program:
    segments: list[Segment]
    warnings: list[str]
    source: str
    md5: str
    n_lines: int
    final_state: GState

    @property
    def five_axis(self) -> bool:
        return bool(self.final_state.five_axis)

    @property
    def cutting_length(self) -> float:
        return sum(s.length for s in self.segments if s.is_cutting)

    @property
    def rapid_length(self) -> float:
        return sum(s.length for s in self.segments if not s.is_cutting)

    def bounds(self) -> tuple[np.ndarray, np.ndarray]:
        pts = np.array([p for s in self.segments for p in (s.start, s.end)])
        return pts.min(0), pts.max(0)


def _strip_comments(line: str) -> tuple[str, str]:
    comment = ""
    m = _COMMENT_PAREN.findall(line)
    if m:
        comment = " ".join(x.strip("()") for x in m)
    line = _COMMENT_PAREN.sub(" ", line)
    if ";" in line:
        line, c2 = line.split(";", 1)
        comment = (comment + " " + c2.strip()).strip()
    return line.strip(), comment


def _words(line: str) -> list[tuple[str, float]]:
    return [(l.upper(), float(v)) for l, v in _WORD_RE.findall(line)]


def parse_gcode(text: str, rapid_feed_mm_min: float = 3000.0, source: str = "<string>") -> Program:
    """Parse G-code text into segments. Positions default to the program origin at start."""
    st = GState()
    segments: list[Segment] = []
    warnings: list[str] = []
    rapid_feed = rapid_feed_mm_min * MM / 60.0
    lines = text.splitlines()
    ended = False

    for ln, raw in enumerate(lines, start=1):
        if ended:
            break
        code, _comment = _strip_comments(raw)
        if not code or code.startswith("%"):
            continue
        words = _words(code)
        if not words:
            warnings.append(f"line {ln}: no words parsed in {raw!r}")
            continue
        # group: M and G words set modes; axis words trigger motion
        gcodes = [int(round(v)) for l, v in words if l == "G"]
        mcodes = [int(round(v)) for l, v in words if l == "M"]
        axis = {l: v for l, v in words if l in "XYZIJKRP"}
        motion_requested: int | None = None

        for g in gcodes:
            if g in (0, 1, 2, 3):
                motion_requested = g
                st.motion = g
            elif g == 20:
                st.units = INCH
            elif g == 21:
                st.units = MM
            elif g == 90:
                st.absolute = True
            elif g == 91:
                st.absolute = False
            elif g in PLANES:
                st.plane = PLANES[g]
            elif 54 <= g <= 59:
                st.work_offset = f"G{g}"
            elif g in (40, 49, 80, 94, 64, 61, 4):
                pass  # comp off / cycle cancel / feed per minute / path mode / dwell: no geometry
            elif g in (41, 42, 43, 44):
                raise GCodeError(f"line {ln}: cutter/tool compensation G{g} is not supported (SPEC §4)")
            elif 81 <= g <= 89:
                raise GCodeError(f"line {ln}: canned cycle G{g} is not supported (SPEC §4)")
            elif g == 92:
                raise GCodeError(f"line {ln}: G92 offset is not supported (SPEC §4)")
            elif g == 28:
                warnings.append(f"line {ln}: G28 (return home) ignored")
            else:
                warnings.append(f"line {ln}: unsupported G{g} ignored")

        for m in mcodes:
            if m in (3, 4):
                st.spindle_on = True
            elif m == 5:
                st.spindle_on = False
            elif m in (2, 30):
                ended = True
            elif m in (0, 1, 6, 7, 8, 9):
                pass  # pause / tool change / coolant: no geometry
            else:
                warnings.append(f"line {ln}: unsupported M{m} ignored")

        for l, v in words:
            if l == "F":
                st.feed = v * st.units / 60.0
            elif l == "S":
                st.spindle_rpm = v
            elif l in ("N", "T", "D", "H", "O"):
                pass
            elif l not in "GMXYZIJKRP":
                warnings.append(f"line {ln}: unknown word {l}{v:g} ignored")

        has_axis = any(k in axis for k in "XYZ")
        if not has_axis and not (motion_requested in (2, 3) and ("R" in axis or any(k in axis for k in "IJK"))):
            continue
        mode = st.motion if motion_requested is None else motion_requested
        if mode is None:
            raise GCodeError(f"line {ln}: axis words before any motion mode (G0/G1/G2/G3)")

        target = st.pos.copy()
        for i, k in enumerate("XYZ"):
            if k in axis:
                val = axis[k] * st.units
                target[i] = (st.pos[i] + val) if not st.absolute else val

        # 5-axis: I J K on a G0/G1 line is the tool axis unit vector (tip -> spindle), modal
        axis_prev = st.axis.copy()
        if mode in (0, 1) and any(k in axis for k in "IJK"):
            v = np.array([axis.get("I", 0.0), axis.get("J", 0.0), axis.get("K", 0.0)])
            n = np.linalg.norm(v)
            if n < 1e-9:
                raise GCodeError(f"line {ln}: zero-length tool axis vector")
            st.axis = v / n
            st.five_axis = True

        if mode == 0:
            seg = Segment("rapid", st.pos.copy(), target, rapid_feed, st.spindle_rpm, st.spindle_on, ln, raw.strip(), st.plane)
        elif mode == 1:
            if st.feed <= 0:
                raise GCodeError(f"line {ln}: G1 without a feed rate")
            seg = Segment("linear", st.pos.copy(), target, st.feed, st.spindle_rpm, st.spindle_on, ln, raw.strip(), st.plane)
        else:
            if st.feed <= 0:
                raise GCodeError(f"line {ln}: G{mode} without a feed rate")
            seg = _make_arc(st, target, axis, mode, ln, raw.strip())
        if st.five_axis:
            seg.axis_start = axis_prev
            seg.axis_end = st.axis.copy()
        if seg.length > 0 or seg.kind.startswith("arc") or (st.five_axis and np.linalg.norm(seg.axis_end - seg.axis_start) > 1e-9):
            segments.append(seg)
        st.pos = target

    return Program(
        segments=segments,
        warnings=warnings,
        source=source,
        md5=hashlib.md5(text.encode()).hexdigest(),
        n_lines=len(lines),
        final_state=st,
    )


def _make_arc(st: GState, target: np.ndarray, axis: dict, mode: int, ln: int, raw: str) -> Segment:
    a0, a1, n = PLANE_AXES[st.plane]
    li, lj = PLANE_IJK[st.plane]
    cw = mode == 2
    start = st.pos.copy()
    p0 = np.array([start[a0], start[a1]])
    p1 = np.array([target[a0], target[a1]])
    if "R" in axis:
        r_signed = axis["R"] * st.units
        r = abs(r_signed)
        d = p1 - p0
        dist = np.linalg.norm(d)
        if dist < 1e-12:
            raise GCodeError(f"line {ln}: R-format arc with coincident endpoints is ambiguous")
        if dist > 2 * r * (1 + 1e-9):
            raise GCodeError(f"line {ln}: arc radius {r*1e3:.4f} mm too small for chord {dist*1e3:.4f} mm")
        h = math.sqrt(max(r * r - (dist / 2) ** 2, 0.0))
        mid = (p0 + p1) / 2
        perp = np.array([-d[1], d[0]]) / dist
        # CW with positive R -> center on the right of the chord (short arc); negative R -> long arc
        sign = -1.0 if cw else 1.0
        if r_signed < 0:
            sign = -sign
        c2 = mid + sign * h * perp
    else:
        if li not in axis and lj not in axis:
            raise GCodeError(f"line {ln}: arc needs {li}/{lj} or R")
        c2 = p0 + np.array([axis.get(li, 0.0), axis.get(lj, 0.0)]) * st.units  # incremental from start
        r = float(np.linalg.norm(p0 - c2))
        r_end = float(np.linalg.norm(p1 - c2))
        if abs(r - r_end) > max(1e-6, 1e-4 * r):
            raise GCodeError(f"line {ln}: arc center inconsistent: r_start {r*1e3:.4f} mm, r_end {r_end*1e3:.4f} mm")
    ang0 = math.atan2(p0[1] - c2[1], p0[0] - c2[0])
    ang1 = math.atan2(p1[1] - c2[1], p1[0] - c2[0])
    sweep = ang1 - ang0
    if cw:
        sweep = sweep - 2 * math.pi if sweep > 1e-9 else sweep
        if abs(sweep) < 1e-9:
            sweep = -2 * math.pi  # full circle
    else:
        sweep = sweep + 2 * math.pi if sweep < -1e-9 else sweep
        if abs(sweep) < 1e-9:
            sweep = 2 * math.pi
    turns = int(axis.get("P", 0))
    if turns > 1:
        sweep += (turns - 1) * (2 * math.pi) * (-1 if cw else 1)
    center = np.zeros(3)
    center[a0], center[a1], center[n] = c2[0], c2[1], start[n]
    return Segment(
        "arc_cw" if cw else "arc_ccw",
        start,
        target,
        st.feed,
        st.spindle_rpm,
        st.spindle_on,
        ln,
        raw,
        st.plane,
        center=center,
        radius=r,
        sweep=sweep,
        turns=turns,
    )


def load_gcode(path: str | Path, **kw) -> Program:
    p = Path(path)
    return parse_gcode(p.read_text(), source=str(p), **kw)
