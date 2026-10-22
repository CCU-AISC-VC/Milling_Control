"""2.5D material removal on a height map (Warp), tool-swept volume per trajectory sample.

The stock top surface is a Z-map over a world-XY grid. For every cutting sample pair
(p0 -> p1, tool tip positions in world) the cells inside the 2D capsule of radius r around the
segment are lowered to the tool-bottom height (flat: z_tip; ball: z_tip + r - sqrt(r^2 - d^2)).
Removed volume per segment is accumulated with atomics, which gives the material removal rate
(MRR) time series used by the cutting-force estimate (docs/SPEC.md §8).

Limitations (by design, M7): no side-wall undercut (2.5D), no chip physics, the tool is a
rigid cutter; the map is not aware of the tool's flute length.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import warp as wp


@wp.kernel
def _cut_capsule(
    z: wp.array2d(dtype=float),
    x0: float,
    y0: float,
    dx: float,
    i0: int,
    j0: int,
    ni: int,
    p0: wp.vec3,
    p1: wp.vec3,
    r: float,
    ball: int,
    z_floor: float,
    out_vol: wp.array(dtype=float),
):
    tid = wp.tid()
    i = i0 + tid % ni
    j = j0 + tid // ni
    if i < 0 or j < 0 or i >= z.shape[0] or j >= z.shape[1]:
        return
    cx = x0 + (float(i) + 0.5) * dx
    cy = y0 + (float(j) + 0.5) * dx
    ax = p1[0] - p0[0]
    ay = p1[1] - p0[1]
    L2 = ax * ax + ay * ay
    ex = cx - p0[0]
    ey = cy - p0[1]
    # parameter interval [sa, sb] of the segment where the cell centre is within r of the axis
    sa = 0.0
    sb = 1.0
    if L2 > 1.0e-18:
        b = (ex * ax + ey * ay) / L2
        cc = (ex * ex + ey * ey - r * r) / L2
        disc = b * b - cc
        if disc < 0.0:
            return
        sq = wp.sqrt(disc)
        sa = wp.max(b - sq, 0.0)
        sb = wp.min(b + sq, 1.0)
        if sa > sb:
            return
    else:
        if ex * ex + ey * ey > r * r:
            return
    # lowest tool-bottom height over the interval: check both ends and the closest point
    sc = wp.clamp(sa, 0.0, 1.0)
    if L2 > 1.0e-18:
        sc = wp.clamp((ex * ax + ey * ay) / L2, sa, sb)
    z_tool = 1.0e9
    for k in range(3):
        s = sa
        if k == 1:
            s = sb
        if k == 2:
            s = sc
        zt = p0[2] + s * (p1[2] - p0[2])
        if ball == 1:
            qx = p0[0] + s * ax
            qy = p0[1] + s * ay
            d2 = (cx - qx) * (cx - qx) + (cy - qy) * (cy - qy)
            zt = zt + r - wp.sqrt(wp.max(r * r - wp.min(d2, r * r), 0.0))
        z_tool = wp.min(z_tool, zt)
    z_tool = wp.max(z_tool, z_floor)
    z_old = z[i, j]
    if z_tool < z_old:
        z[i, j] = z_tool
        wp.atomic_add(out_vol, 0, (z_old - z_tool) * dx * dx)


@dataclass
class HeightMap:
    x0: float  # world x of the grid's low edge
    y0: float
    dx: float  # cell size [m]
    z_top: float
    z_floor: float
    z: wp.array2d  # (nx, ny) heights [m]

    @property
    def nx(self) -> int:
        return self.z.shape[0]

    @property
    def ny(self) -> int:
        return self.z.shape[1]

    def numpy(self) -> np.ndarray:
        return self.z.numpy()

    def cell_centers(self) -> tuple[np.ndarray, np.ndarray]:
        xs = self.x0 + (np.arange(self.nx) + 0.5) * self.dx
        ys = self.y0 + (np.arange(self.ny) + 0.5) * self.dx
        return xs, ys

    def removed_volume(self) -> float:
        return float((self.z_top - self.numpy()).sum() * self.dx * self.dx)


def make_heightmap(center_xy, size_xy, z_top: float, height: float, resolution: float = 5e-4, device=None) -> HeightMap:
    nx = int(math.ceil(size_xy[0] / resolution))
    ny = int(math.ceil(size_xy[1] / resolution))
    x0 = center_xy[0] - size_xy[0] / 2
    y0 = center_xy[1] - size_xy[1] / 2
    z = wp.full((nx, ny), float(z_top), dtype=float, device=device)
    return HeightMap(x0, y0, resolution, z_top, z_top - height, z)


class Cutter:
    """Applies tool sweeps to a HeightMap and records removed volume per sweep."""

    def __init__(self, hm: HeightMap, radius: float, ball: bool = False):
        self.hm = hm
        self.r = float(radius)
        self.ball = 1 if ball else 0
        self._vol = wp.zeros(1, dtype=float, device=hm.z.device)

    def sweep(self, p0: np.ndarray, p1: np.ndarray) -> float:
        """Lower the map along the segment p0->p1 (world, m); returns removed volume [m^3]."""
        hm = self.hm
        if min(p0[2], p1[2]) >= hm.z_top:
            return 0.0  # tool above the stock top: nothing to cut
        xmin = min(p0[0], p1[0]) - self.r
        xmax = max(p0[0], p1[0]) + self.r
        ymin = min(p0[1], p1[1]) - self.r
        ymax = max(p0[1], p1[1]) + self.r
        i0 = int(math.floor((xmin - hm.x0) / hm.dx))
        i1 = int(math.ceil((xmax - hm.x0) / hm.dx))
        j0 = int(math.floor((ymin - hm.y0) / hm.dx))
        j1 = int(math.ceil((ymax - hm.y0) / hm.dx))
        i0, i1 = max(i0, 0), min(i1, hm.nx - 1)
        j0, j1 = max(j0, 0), min(j1, hm.ny - 1)
        ni, nj = i1 - i0 + 1, j1 - j0 + 1
        if ni <= 0 or nj <= 0:
            return 0.0
        self._vol.zero_()
        wp.launch(
            _cut_capsule,
            dim=ni * nj,
            inputs=[hm.z, hm.x0, hm.y0, hm.dx, i0, j0, ni, wp.vec3(*[float(v) for v in p0]), wp.vec3(*[float(v) for v in p1]), self.r, self.ball, hm.z_floor, self._vol],
            device=hm.z.device,
        )
        return float(self._vol.numpy()[0])


def surface_mesh(hm: HeightMap, step: int = 1) -> tuple[np.ndarray, np.ndarray]:
    """Triangle mesh (vertices Nx3 world, faces Mx3) of the height map, optionally decimated."""
    Z = hm.numpy()[::step, ::step]
    xs, ys = hm.cell_centers()
    xs, ys = xs[::step], ys[::step]
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    V = np.stack([X.ravel(), Y.ravel(), Z.ravel()], axis=1)
    nx, ny = Z.shape
    idx = np.arange(nx * ny).reshape(nx, ny)
    a = idx[:-1, :-1].ravel()
    b = idx[1:, :-1].ravel()
    c = idx[1:, 1:].ravel()
    d = idx[:-1, 1:].ravel()
    F = np.concatenate([np.stack([a, b, c], axis=1), np.stack([a, c, d], axis=1)])
    return V, F


def solid_from_heightmap(hm: HeightMap, step: int = 2) -> tuple[np.ndarray, np.ndarray]:
    """Closed block mesh: machined top surface + four side walls + bottom (world coords)."""
    Z = hm.numpy()[::step, ::step]
    nx, ny = Z.shape
    # stretch the decimated grid to the exact stock footprint (cell centres stop half a cell short)
    xs = np.linspace(hm.x0, hm.x0 + hm.nx * hm.dx, nx)
    ys = np.linspace(hm.y0, hm.y0 + hm.ny * hm.dx, ny)
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    top = np.stack([X.ravel(), Y.ravel(), Z.ravel()], axis=1)
    idx = np.arange(nx * ny).reshape(nx, ny)
    a, b, c, d = idx[:-1, :-1].ravel(), idx[1:, :-1].ravel(), idx[1:, 1:].ravel(), idx[:-1, 1:].ravel()
    faces = [np.stack([a, b, c], axis=1), np.stack([a, c, d], axis=1)]
    # bottom ring vertices under the boundary of the top grid
    zb = hm.z_floor
    ring_top = np.concatenate([idx[:, 0], idx[-1, 1:], idx[-2::-1, -1], idx[0, -2:0:-1]])  # boundary loop (ccw seen from +z)
    n_top = len(top)
    bottom = top[ring_top].copy()
    bottom[:, 2] = zb
    V = np.concatenate([top, bottom])
    m = len(ring_top)
    for k in range(m):
        t0, t1 = ring_top[k], ring_top[(k + 1) % m]
        b0, b1 = n_top + k, n_top + (k + 1) % m
        faces.append(np.array([[t0, b0, b1], [t0, b1, t1]]))
    # bottom cap: fan from the first bottom vertex (reversed so the normal points down)
    cap = np.array([[n_top, n_top + (k + 1) % m, n_top + k] for k in range(1, m - 1)])
    faces.append(cap)
    return V.astype(np.float32), np.concatenate(faces).astype(np.int32)
