"""Robot-robot clearance from mesh geometry (Warp BVH point queries), independent of Newton's
contact pipeline.

Why: Newton's mesh-mesh contact candidates for shapes that are still far apart can carry a
normal that is inconsistent with the point pair (M6 found "-200 mm penetration" while the two
spindles were 108 mm apart), so they cannot be used as a clearance metric. Box-vs-mesh contacts
(table, stock, fence) were verified and stay with the pipeline.

Method: for every ordered pair of wrist links (J4..spindle) of two robots, transform the
vertices of link A's collision mesh into link B's frame and query the closest point on B's mesh
(unsigned: the spindle STL is not watertight, so face-normal signs are unreliable). The minimum
over both directions is a vertex-to-surface distance; coarse link meshes are subdivided to
<= ``max_edge`` so the overestimate is bounded by about half an edge. Interpenetrating surfaces
intersect, so some sampled vertex lies within ~half an edge of the other surface: a clearance
below ``CONTACT_TOL`` is treated as contact. Pairs whose bounding spheres are farther than
``skip_dist`` apart are skipped and reported with that lower bound.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import trimesh
import warp as wp

from .newton_robot import xform_from_matrix
from .urdf_kinematics import make_transform

WRIST_LINKS = ("J4_link", "J5_link", "J6_link", "spindle_link")
CONTACT_TOL = 5e-3  # m: unsigned clearance below this counts as contact (vertex spacing <= 10 mm)
MAX_QUERY_POINTS = 120_000


@wp.kernel
def _vertex_to_mesh(points: wp.array(dtype=wp.vec3), T_a: wp.transform, T_b_inv: wp.transform, mesh: wp.uint64, max_dist: float, out_min: wp.array(dtype=float)):
    i = wp.tid()
    p_b = wp.transform_point(T_b_inv, wp.transform_point(T_a, points[i]))
    q = wp.mesh_query_point_no_sign(mesh, p_b, max_dist)
    if q.result:
        cp = wp.mesh_eval_position(mesh, q.face, q.u, q.v)
        wp.atomic_min(out_min, 0, wp.length(p_b - cp))


@dataclass
class LinkMesh:
    link: str
    points: wp.array  # vertices in the link frame
    mesh: wp.Mesh
    center: np.ndarray  # bounding sphere in link frame
    radius: float


def load_collision_meshes(urdf_path: str | Path, links=WRIST_LINKS, max_edge: float = 0.01, device=None, seed: int = 0) -> dict[str, LinkMesh]:
    urdf_path = Path(urdf_path)
    root = ET.parse(str(urdf_path)).getroot()
    out = {}
    for lel in root.findall("link"):
        name = lel.get("name")
        if name not in links:
            continue
        col = lel.find("collision")
        geo = col.find("geometry/mesh")
        fn = geo.get("filename")
        scale = np.array([float(v) for v in geo.get("scale", "1 1 1").split()])
        o = col.find("origin")
        T = make_transform([float(v) for v in (o.get("xyz") if o is not None else "0 0 0").split()], [float(v) for v in (o.get("rpy") if o is not None else "0 0 0").split()])
        m = trimesh.load(str(urdf_path.parent / fn), force="mesh")
        m.apply_scale(scale)
        m.apply_transform(T)
        edge_max = float(m.edges_unique_length.max())
        if max_edge and edge_max > 2 * max_edge:
            for it in (8, 12):
                try:
                    m = trimesh.Trimesh(*trimesh.remesh.subdivide_to_size(m.vertices, m.faces, max_edge, max_iter=it)[:2], process=False)
                    break
                except ValueError:
                    continue  # max_iter exceeded: try more iterations, else keep the mesh as is
        V = np.asarray(m.vertices, dtype=np.float32)
        F = np.asarray(m.faces, dtype=np.int32).reshape(-1)
        c = (V.min(0) + V.max(0)) / 2
        r = float(np.linalg.norm(V - c, axis=1).max())
        mesh_pts = wp.array(V, dtype=wp.vec3, device=device)
        mesh = wp.Mesh(points=mesh_pts, indices=wp.array(F, dtype=wp.int32, device=device))
        # query points: all vertices, capped by random subsampling (the BVH target stays complete)
        if len(V) > MAX_QUERY_POINTS:
            sel = np.random.default_rng(seed).choice(len(V), MAX_QUERY_POINTS, replace=False)
            qpts = wp.array(V[sel], dtype=wp.vec3, device=device)
        else:
            qpts = mesh_pts
        out[name] = LinkMesh(name, qpts, mesh, c.astype(float), r)
    missing = set(links) - set(out)
    if missing:
        raise ValueError(f"collision meshes not found for {missing}")
    return out


class MeshClearance:
    """Minimum signed distance between the wrist links of robot pairs, per frame."""

    def __init__(self, scene, robot_names: list[str], links=WRIST_LINKS, max_edge: float = 0.01, skip_dist: float = 0.3, max_query: float = 0.6):
        self.scene = scene
        self.names = list(robot_names)
        self.links = tuple(links)
        self.skip_dist = skip_dist
        self.max_query = max_query
        self.meshes: dict[str, dict[str, LinkMesh]] = {}
        cache: dict[str, dict[str, LinkMesh]] = {}
        for n in self.names:
            urdf = scene.robots[n].urdf
            if urdf not in cache:
                cache[urdf] = load_collision_meshes(urdf, links, max_edge)
            self.meshes[n] = cache[urdf]
        self._out = wp.zeros(1, dtype=float)
        self.n_queries = 0

    def _query(self, pts: wp.array, T_a: np.ndarray, T_b: np.ndarray, mesh: wp.Mesh) -> float:
        self._out.fill_(1e9)
        wp.launch(_vertex_to_mesh, dim=len(pts), inputs=[pts, xform_from_matrix(T_a), xform_from_matrix(np.linalg.inv(T_b)), mesh.id, self.max_query, self._out])
        self.n_queries += 1
        return float(self._out.numpy()[0])

    def min_distance(self, a: str, b: str) -> tuple[float, tuple[str, str] | None]:
        """Unsigned min clearance [m] between robots a and b (wrist links), and the closest link pair."""
        poses_a = {l: self.scene.link_pose(a, l) for l in self.links}
        poses_b = {l: self.scene.link_pose(b, l) for l in self.links}
        best, pair = np.inf, None
        for la in self.links:
            ma = self.meshes[a][la]
            ca = poses_a[la][:3, :3] @ ma.center + poses_a[la][:3, 3]
            for lb in self.links:
                mb = self.meshes[b][lb]
                cb = poses_b[lb][:3, :3] @ mb.center + poses_b[lb][:3, 3]
                bound = float(np.linalg.norm(ca - cb) - ma.radius - mb.radius)
                if bound > self.skip_dist:
                    if bound < best:
                        best, pair = bound, (f"{a}/{la}", f"{b}/{lb}")
                    continue
                d = min(self._query(ma.points, poses_a[la], poses_b[lb], mb.mesh), self._query(mb.points, poses_b[lb], poses_a[la], ma.mesh))
                if d < best:
                    best, pair = d, (f"{a}/{la}", f"{b}/{lb}")
        return best, pair

    def min_distance_all(self) -> tuple[float, tuple[str, str] | None]:
        best, pair = np.inf, None
        for i, a in enumerate(self.names):
            for b in self.names[i + 1 :]:
                d, p = self.min_distance(a, b)
                if d < best:
                    best, pair = d, p
        return best, pair
