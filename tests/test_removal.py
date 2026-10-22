"""M7: height-map removal volume against analytic values; cutting-force units."""
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from milling_sim.cutting import Material, cutting_force  # noqa: E402
from milling_sim.removal import Cutter, make_heightmap  # noqa: E402


class RemovalTests(unittest.TestCase):
    def test_flat_slot_volume(self):
        # 200 x 100 mm stock, 10 mm flat tool, straight slot 80 mm long at 3 mm depth
        hm = make_heightmap((0.1, 0.05), (0.2, 0.1), z_top=0.0, height=0.05, resolution=2.5e-4)
        cut = Cutter(hm, radius=5e-3, ball=False)
        p0 = np.array([0.05, 0.05, -0.003])
        p1 = np.array([0.13, 0.05, -0.003])
        # sweep in 1 mm steps like the sampler does
        n = 80
        total = 0.0
        pts = np.linspace(p0, p1, n + 1)
        for a, b in zip(pts[:-1], pts[1:]):
            total += cut.sweep(a, b)
        analytic = (0.08 * 0.01 + math.pi * 0.005**2) * 0.003  # slot with round ends
        self.assertLess(abs(total - analytic) / analytic, 0.02)
        self.assertLess(abs(hm.removed_volume() - total) / analytic, 1e-4)  # float32 accumulation
        # a second identical pass removes nothing
        self.assertLess(sum(cut.sweep(a, b) for a, b in zip(pts[:-1], pts[1:])), 1e-12)

    def test_ball_plunge_profile(self):
        hm = make_heightmap((0.0, 0.0), (0.04, 0.04), z_top=0.0, height=0.02, resolution=2e-4)
        cut = Cutter(hm, radius=4e-3, ball=True)
        cut.sweep(np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, -0.002]))
        Z = hm.numpy()
        xs, ys = hm.cell_centers()
        i = np.argmin(np.abs(xs))
        j = np.argmin(np.abs(ys))
        self.assertAlmostEqual(Z[i, j], -0.002, delta=2e-5)  # centre reaches the tip depth
        # ball profile: at d = 2 mm the bottom is z_tip + r - sqrt(r^2 - d^2)
        k = np.argmin(np.abs(xs - 0.002))
        self.assertAlmostEqual(Z[k, j], -0.002 + 0.004 - math.sqrt(0.004**2 - 0.002**2), delta=1e-4)

    def test_force_units(self):
        mat = Material("test", k_c=700.0, radial_ratio=0.4, axial_ratio=0.25, density=2700)
        mrr = np.array([100.0])  # mm^3/s  (e.g. 10 mm wide x 1 mm deep x 10 mm/s)
        rpm = np.array([8000.0])
        F, mag = cutting_force(mrr, rpm, np.array([[1.0, 0.0, 0.0]]), 0.010, mat)
        v_c = math.pi * 10.0 * 8000 / 60  # mm/s
        F_t = 700.0 * 100.0 / v_c  # N
        self.assertAlmostEqual(-F[0, 0], F_t, places=9)
        self.assertAlmostEqual(F[0, 2], 0.25 * F_t, places=9)
        self.assertAlmostEqual(mag[0], F_t * math.sqrt(1 + 0.4**2 + 0.25**2), places=9)


if __name__ == "__main__":
    unittest.main()
