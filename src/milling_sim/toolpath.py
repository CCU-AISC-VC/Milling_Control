"""Sample parsed G-code segments into a time-stamped TCP path (program frame, SI units).

Geometry: lines are split at ``max_spacing``; arcs at the angle that keeps the chord error
below ``chord_tol`` (and also at ``max_spacing``). Endpoints of every segment are kept.

Time law: trapezoidal speed profile per segment with acceleration ``accel``. The speed at a
junction is the segment feed if the direction change is below ``corner_angle`` and both
segments cut, otherwise 0 (the robot stops at corners and at rapid/cut transitions).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .gcode import PLANE_AXES, Program, Segment


@dataclass
class SamplingParams:
    max_spacing: float = 1e-3  # m
    chord_tol: float = 2e-5  # m
    accel: float = 1.0  # m/s^2 along the path
    corner_angle_deg: float = 5.0
    tool_dir: tuple[float, float, float] = (0.0, 0.0, -1.0)  # cutter axis, body -> tip, program frame
    max_axis_step_deg: float = 2.0  # 5-axis: max tool-axis rotation between samples
    reorient_rate_deg_s: float = 30.0  # 5-axis: speed of a reorientation in place


@dataclass
class Toolpath:
    t: np.ndarray  # (N,)
    p: np.ndarray  # (N, 3) m, program frame
    tool_dir: np.ndarray  # (N, 3) unit, program frame
    speed: np.ndarray  # (N,) m/s actual
    feed: np.ndarray  # (N,) m/s commanded
    spindle_rpm: np.ndarray
    segment: np.ndarray  # (N,) index into program.segments
    line_no: np.ndarray
    motion: np.ndarray  # (N,) str
    params: SamplingParams
    seg_time: np.ndarray = field(default_factory=lambda: np.zeros(0))  # (n_seg,) duration

    @property
    def n(self) -> int:
        return len(self.t)

    @property
    def duration(self) -> float:
        return float(self.t[-1]) if self.n else 0.0

    @property
    def length(self) -> float:
        return float(np.sum(np.linalg.norm(np.diff(self.p, axis=0), axis=1)))

    def is_cutting(self) -> np.ndarray:
        return self.motion != "rapid"


def _slerp(a: np.ndarray, b: np.ndarray, s: np.ndarray) -> np.ndarray:
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    c = float(np.clip(np.dot(a, b), -1.0, 1.0))
    ang = math.acos(c)
    if ang < 1e-9:
        return np.tile(a, (len(s), 1))
    w0 = np.sin((1 - s) * ang) / math.sin(ang)
    w1 = np.sin(s * ang) / math.sin(ang)
    out = w0[:, None] * a[None, :] + w1[:, None] * b[None, :]
    return out / np.linalg.norm(out, axis=1, keepdims=True)


def _segment_s_values(seg: Segment, prm: SamplingParams) -> np.ndarray:
    L = seg.length
    if L <= 0:
        return np.array([0.0, 1.0])
    n_lin = max(1, math.ceil(L / prm.max_spacing))
    if seg.kind.startswith("arc") and seg.radius > 0:
        # chord error e = r(1 - cos(dθ/2)) <= tol
        c = 1 - prm.chord_tol / seg.radius
        dtheta = 2 * math.acos(max(min(c, 1.0), -1.0)) if c < 1.0 else math.pi
        n_arc = max(1, math.ceil(abs(seg.sweep) / dtheta)) if dtheta > 0 else 1
        n = max(n_lin, n_arc)
    else:
        n = n_lin
    if seg.axis_start is not None and seg.axis_end is not None:
        ang = math.acos(float(np.clip(np.dot(seg.axis_start, seg.axis_end), -1.0, 1.0)))
        n = max(n, int(math.ceil(ang / math.radians(prm.max_axis_step_deg))))
    return np.linspace(0.0, 1.0, n + 1)


def _junction_speed(prev: Segment | None, cur: Segment, prm: SamplingParams) -> float:
    if prev is None or not (prev.is_cutting and cur.is_cutting):
        return 0.0
    d0 = _end_tangent(prev)
    d1 = _start_tangent(cur)
    cosang = float(np.clip(np.dot(d0, d1), -1.0, 1.0))
    if math.degrees(math.acos(cosang)) <= prm.corner_angle_deg:
        return min(prev.feed, cur.feed)
    return 0.0


def _tangent(seg: Segment, s: float) -> np.ndarray:
    eps = 1e-6
    a = seg.point(max(0.0, s - eps))
    b = seg.point(min(1.0, s + eps))
    d = b - a
    n = np.linalg.norm(d)
    return d / n if n > 0 else np.zeros(3)


def _start_tangent(seg: Segment) -> np.ndarray:
    return _tangent(seg, 0.0)


def _end_tangent(seg: Segment) -> np.ndarray:
    return _tangent(seg, 1.0)


def _trapezoid_times(ds: np.ndarray, v_max: float, v0: float, v1: float, a: float) -> np.ndarray:
    """Time stamps (starting at 0) at cumulative distances ``ds`` for a trapezoidal profile.

    v0/v1 are the speeds at the ends; the profile accelerates from v0 to v_max, cruises,
    decelerates to v1. If the segment is too short for v_max, a triangular profile is used.
    """
    L = float(ds[-1])
    v_max = max(v_max, 1e-9)
    v0 = min(v0, v_max)
    v1 = min(v1, v_max)
    # peak speed reachable within L
    v_peak = math.sqrt(max((2 * a * L + v0 * v0 + v1 * v1) / 2.0, 0.0))
    v_c = min(v_max, v_peak)
    d_acc = max((v_c * v_c - v0 * v0) / (2 * a), 0.0)
    d_dec = max((v_c * v_c - v1 * v1) / (2 * a), 0.0)
    d_cruise = max(L - d_acc - d_dec, 0.0)
    t_acc = (v_c - v0) / a
    t_dec = (v_c - v1) / a
    t_cruise = d_cruise / v_c
    t = np.zeros_like(ds)
    for i, s in enumerate(ds):
        if s <= d_acc:
            # v0 t + 0.5 a t^2 = s
            t[i] = (-v0 + math.sqrt(v0 * v0 + 2 * a * s)) / a
        elif s <= d_acc + d_cruise:
            t[i] = t_acc + (s - d_acc) / v_c
        else:
            s2 = s - d_acc - d_cruise
            # v_c t - 0.5 a t^2 = s2
            disc = max(v_c * v_c - 2 * a * s2, 0.0)
            t[i] = t_acc + t_cruise + (v_c - math.sqrt(disc)) / a
    return t


def sample_program(prog: Program, prm: SamplingParams | None = None) -> Toolpath:
    prm = prm or SamplingParams()
    T, P, SP, FE, RPM, SEG, LN, MO = [], [], [], [], [], [], [], []
    t_offset = 0.0
    prev: Segment | None = None
    seg_time = []
    tool_dir = np.asarray(prm.tool_dir, dtype=float)
    tool_dir = tool_dir / np.linalg.norm(tool_dir)
    DIRS = []
    for k, seg in enumerate(prog.segments):
        s = _segment_s_values(seg, prm)
        pts = seg.point(s)
        if seg.axis_start is not None and seg.axis_end is not None:
            # 5-axis: slerp the tool axis (tip -> spindle); our tool_dir points spindle -> tip
            dirs = -_slerp(seg.axis_start, seg.axis_end, s)
        else:
            dirs = np.tile(tool_dir, (len(s), 1))
        ds = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
        v0 = _junction_speed(prev, seg, prm)
        nxt = prog.segments[k + 1] if k + 1 < len(prog.segments) else None
        v1 = _junction_speed(seg, nxt, prm) if nxt is not None else 0.0
        if ds[-1] <= 0:
            if seg.axis_start is not None and np.linalg.norm(seg.axis_end - seg.axis_start) > 1e-9:
                # reorientation in place: several samples, duration from the angle at reorient_rate
                ang = math.acos(float(np.clip(np.dot(seg.axis_start, seg.axis_end), -1, 1)))
                tt = s * max(ang / math.radians(prm.reorient_rate_deg_s), 1e-3)
                for i in range(1, len(s)):
                    T.append(t_offset + tt[i])
                    P.append(pts[i])
                    DIRS.append(dirs[i])
                    SP.append(0.0)
                    FE.append(seg.feed)
                    RPM.append(seg.spindle_rpm if seg.spindle_on else 0.0)
                    SEG.append(k)
                    LN.append(seg.line_no)
                    MO.append(seg.kind)
                t_offset += tt[-1]
                seg_time.append(float(tt[-1]))
            else:
                seg_time.append(0.0)
            prev = seg
            continue
        tt = _trapezoid_times(ds, seg.feed, v0, v1, prm.accel)
        # actual speed by finite differences of the profile
        sp = np.gradient(ds, tt) if len(tt) > 1 else np.zeros_like(ds)
        # drop the first sample if it duplicates the previous segment's last point
        start = 1 if (P and np.allclose(P[-1], pts[0], atol=1e-12)) else 0
        for i in range(start, len(s)):
            T.append(t_offset + tt[i])
            P.append(pts[i])
            DIRS.append(dirs[i])
            SP.append(sp[i])
            FE.append(seg.feed)
            RPM.append(seg.spindle_rpm if seg.spindle_on else 0.0)
            SEG.append(k)
            LN.append(seg.line_no)
            MO.append(seg.kind)
        t_offset += tt[-1]
        seg_time.append(float(tt[-1]))
        prev = seg
    P = np.array(P).reshape(-1, 3)
    n = len(T)
    return Toolpath(
        t=np.array(T),
        p=P,
        tool_dir=np.array(DIRS).reshape(-1, 3) if DIRS else np.tile(tool_dir, (n, 1)),
        speed=np.array(SP),
        feed=np.array(FE),
        spindle_rpm=np.array(RPM),
        segment=np.array(SEG, dtype=int),
        line_no=np.array(LN, dtype=int),
        motion=np.array(MO),
        params=prm,
        seg_time=np.array(seg_time),
    )


def chord_errors(prog: Program, tp: Toolpath) -> np.ndarray:
    """Max deviation of each sampled sub-segment's midpoint from the true segment geometry [m]."""
    errs = np.zeros(max(tp.n - 1, 0))
    for i in range(tp.n - 1):
        k = tp.segment[i]
        if tp.segment[i + 1] != k:
            continue
        seg = prog.segments[k]
        mid = (tp.p[i] + tp.p[i + 1]) / 2
        if seg.kind.startswith("arc"):
            a0, a1, _ = PLANE_AXES[seg.plane]
            r_mid = math.hypot(mid[a0] - seg.center[a0], mid[a1] - seg.center[a1])
            errs[i] = abs(seg.radius - r_mid)
        else:
            d = seg.end - seg.start
            L = np.linalg.norm(d)
            errs[i] = np.linalg.norm(np.cross(d / L, mid - seg.start)) if L > 0 else 0.0
    return errs
