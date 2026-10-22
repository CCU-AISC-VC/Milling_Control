"""Unit tests for the G-code parser and toolpath sampler (M1).

Run:  ~/envs/aisc_sim/bin/python -m unittest discover -s tests -v
"""
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from milling_sim.gcode import GCodeError, parse_gcode  # noqa: E402
from milling_sim.sample_gcode import make_sample_pocket_contour, make_unit_square  # noqa: E402
from milling_sim.toolpath import SamplingParams, _trapezoid_times, chord_errors, sample_program  # noqa: E402

MM = 1e-3


class ParserTests(unittest.TestCase):
    def test_square_lengths(self):
        prog = parse_gcode(make_unit_square(100.0))
        self.assertEqual(len(prog.segments), 4)
        for s in prog.segments:
            self.assertEqual(s.kind, "linear")
            self.assertAlmostEqual(s.length, 0.1, places=12)
        self.assertAlmostEqual(prog.cutting_length, 0.4, places=12)
        self.assertAlmostEqual(prog.segments[0].feed, 600 * MM / 60, places=15)

    def test_arc_ij_full_circle_ccw(self):
        prog = parse_gcode("G21 G90 G17\nG0 X130 Y100\nG3 X130 Y100 I-30 J0 F500\nM30\n")
        arc = prog.segments[1]
        self.assertEqual(arc.kind, "arc_ccw")
        np.testing.assert_allclose(arc.center, [0.1, 0.1, 0.0], atol=1e-12)
        self.assertAlmostEqual(arc.radius, 0.03, places=12)
        self.assertAlmostEqual(arc.sweep, 2 * math.pi, places=12)
        self.assertAlmostEqual(arc.length, 2 * math.pi * 0.03, places=12)
        # quarter point should be at (100, 130): CCW from +x goes toward +y
        np.testing.assert_allclose(arc.point(0.25) / MM, [100, 130, 0], atol=1e-9)

    def test_arc_r_short_and_long(self):
        prog = parse_gcode("G21 G90 G17\nG1 X0 Y0 F500\nG2 X10 Y10 R10\n")
        arc = prog.segments[-1]
        np.testing.assert_allclose(arc.center / MM, [10, 0, 0], atol=1e-9)
        self.assertAlmostEqual(arc.sweep, -math.pi / 2, places=12)
        prog = parse_gcode("G21 G90 G17\nG1 X0 Y0 F500\nG2 X10 Y10 R-10\n")
        arc = prog.segments[-1]
        np.testing.assert_allclose(arc.center / MM, [0, 10, 0], atol=1e-9)
        self.assertAlmostEqual(arc.sweep, -3 * math.pi / 2, places=12)

    def test_arc_direction_cw_vs_ccw(self):
        # same endpoints and centre, opposite directions
        cw = parse_gcode("G21 G90\nG1 X10 Y0 F500\nG2 X0 Y10 I-10 J0\n").segments[-1]
        ccw = parse_gcode("G21 G90\nG1 X10 Y0 F500\nG3 X0 Y10 I-10 J0\n").segments[-1]
        self.assertAlmostEqual(cw.sweep, -3 * math.pi / 2, places=12)
        self.assertAlmostEqual(ccw.sweep, math.pi / 2, places=12)

    def test_helix_length(self):
        prog = parse_gcode("G21 G90 G17\nG0 X10 Y0 Z0\nG2 X10 Y0 Z-3 I-10 J0 F300\n")
        arc = prog.segments[-1]
        self.assertAlmostEqual(arc.length, math.hypot(2 * math.pi * 0.01, 0.003), places=12)
        self.assertAlmostEqual(arc.point(0.5)[2], -0.0015, places=12)

    def test_g91_incremental(self):
        prog = parse_gcode("G21 G91\nG1 X10 F500\nG1 X10\nG1 Y5\n")
        np.testing.assert_allclose(prog.segments[-1].end / MM, [20, 5, 0], atol=1e-9)

    def test_g20_inches(self):
        prog = parse_gcode("G20 G90\nG1 X1 F10\n")
        self.assertAlmostEqual(prog.segments[0].length, 0.0254, places=12)
        self.assertAlmostEqual(prog.segments[0].feed, 10 * 0.0254 / 60, places=15)

    def test_modal_motion_and_rapid(self):
        prog = parse_gcode("G21 G90\nG0 X10\nY10\nG1 X0 F100\nY0\n")
        self.assertEqual([s.kind for s in prog.segments], ["rapid", "rapid", "linear", "linear"])

    def test_plane_g18(self):
        # ZX plane arc: I pairs with Z? SPEC: G18 uses (K, I) for (Z, X)
        prog = parse_gcode("G21 G90 G18\nG0 X10 Z0\nG3 X10 Z0 K-10 I0 F300\n")
        arc = prog.segments[-1]
        np.testing.assert_allclose(arc.center / MM, [10, 0, -10], atol=1e-9)
        self.assertAlmostEqual(arc.radius, 0.01, places=12)

    def test_unsupported_raises(self):
        with self.assertRaises(GCodeError):
            parse_gcode("G21 G90\nG41 D1\nG1 X10 F100\n")
        with self.assertRaises(GCodeError):
            parse_gcode("G21 G90\nG81 X0 Y0 Z-5 R2 F100\n")
        with self.assertRaises(GCodeError):
            parse_gcode("G21 G90\nG1 X10\n")  # no feed

    def test_comments_and_m30(self):
        prog = parse_gcode("( header )\nG21 G90 ; inline\nG1 X10 F100 (move)\nM30\nG1 X20\n")
        self.assertEqual(len(prog.segments), 1)
        self.assertEqual(prog.warnings, [])

    def test_sample_program_parses_clean(self):
        prog = parse_gcode(make_sample_pocket_contour())
        self.assertEqual(prog.warnings, [])
        kinds = {s.kind for s in prog.segments}
        self.assertEqual(kinds, {"rapid", "linear", "arc_ccw", "arc_cw"})
        lo, hi = prog.bounds()
        self.assertGreaterEqual(lo[2], -6 * MM - 1e-12)
        self.assertLessEqual(hi[2], 10 * MM + 1e-12)


class SamplerTests(unittest.TestCase):
    def test_trapezoid_times(self):
        L, v, a = 0.5, 0.02, 1.0
        ds = np.linspace(0, L, 501)
        t = _trapezoid_times(ds, v, 0.0, 0.0, a)
        self.assertAlmostEqual(t[-1], L / v + v / a, places=6)  # stop at both ends
        self.assertTrue(np.all(np.diff(t) > 0))
        t2 = _trapezoid_times(ds, v, v, v, a)
        self.assertAlmostEqual(t2[-1], L / v, places=6)  # constant feed through
        # triangular profile when too short to reach v
        Ls = 1e-4
        t3 = _trapezoid_times(np.linspace(0, Ls, 11), v, 0.0, 0.0, a)
        self.assertAlmostEqual(t3[-1], 2 * math.sqrt(Ls / a), places=6)

    def test_chord_error_and_spacing(self):
        prog = parse_gcode("G21 G90 G17\nG0 X130 Y100\nG3 X130 Y100 I-30 J0 F500\n")
        prm = SamplingParams(max_spacing=1e-3, chord_tol=2e-5)
        tp = sample_program(prog, prm)
        errs = chord_errors(prog, tp)
        self.assertLessEqual(errs.max(), prm.chord_tol * 1.0001)
        spacing = np.linalg.norm(np.diff(tp.p, axis=0), axis=1)
        self.assertLessEqual(spacing.max(), prm.max_spacing * 1.0001)
        # every sample lies on the circle
        r = np.hypot(tp.p[tp.segment == 1, 0] - 0.1, tp.p[tp.segment == 1, 1] - 0.1)
        np.testing.assert_allclose(r, 0.03, atol=1e-12)

    def test_time_law(self):
        prog = parse_gcode(make_sample_pocket_contour())
        prm = SamplingParams(accel=1.0)
        tp = sample_program(prog, prm)
        dt = np.diff(tp.t)
        self.assertGreater(dt.min(), 0.0)
        v = np.linalg.norm(np.diff(tp.p, axis=0), axis=1) / dt
        # interval (i, i+1) belongs to the segment that produced sample i+1 (boundary samples
        # are labelled with the previous segment), so compare with the feed of sample i+1
        feed_int = tp.feed[1:]
        self.assertLessEqual((v - feed_int).max(), 1e-9)
        a = np.abs(np.diff(v)) / ((dt[1:] + dt[:-1]) / 2)
        self.assertLessEqual(a.max(), prm.accel * 1.2)
        # a cutting segment followed by a rapid decelerates to 0: its duration exceeds the
        # cruise time L / feed by v / a (full stop costs v/(2a) at each end)
        for k, seg in enumerate(prog.segments[:-1]):
            nxt = prog.segments[k + 1]
            if seg.is_cutting and not nxt.is_cutting:
                self.assertGreater(tp.seg_time[k], seg.length / seg.feed + 0.4 * seg.feed / prm.accel)
        # total time is consistent with length / feed (must be longer because of corner stops)
        self.assertGreater(tp.duration, prog.cutting_length / (800 * MM / 60))

    def test_monotone_and_continuous(self):
        prog = parse_gcode(make_sample_pocket_contour())
        tp = sample_program(prog)
        self.assertTrue(np.all(np.diff(tp.t) > 0))
        # consecutive samples are never farther apart than max_spacing (no gaps between segments)
        gaps = np.linalg.norm(np.diff(tp.p, axis=0), axis=1)
        self.assertLessEqual(gaps.max(), tp.params.max_spacing * 1.0001)


if __name__ == "__main__":
    unittest.main()
