#!/usr/bin/env python3
"""M0: flatten the FANUC M-710iD/70 + spindle xacro into a self-contained URDF under assets/.

Why: Newton's ``ModelBuilder.add_urdf`` and cuRobo read plain URDF with resolvable mesh
paths; ``package://`` URIs only resolve inside a built ROS 2 workspace.

Run:  ~/envs/aisc_sim/bin/python scripts/m0_flatten_urdf.py
"""
from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
ROS_PKG = REPO / "ros2_ws/src/m710id_description_spindle"
XACRO_TOP = ROS_PKG / "urdf/m710id_70_with_spindle.urdf.xacro"
ASSET_DIR = REPO / "assets/fanuc_m710id70_spindle"
ROS_SETUP = "/opt/ros/jazzy/setup.bash"


def run_xacro(top: Path, pkg_dir: Path) -> str:
    """Run xacro with ``$(find m710id_description_spindle)`` replaced by the source dir.

    The include inside the top-level xacro uses ``$(find ...)``, which needs the ament index
    of a built workspace. Substituting the path lets us flatten without ``colcon build``.
    """
    src = top.read_text()
    src = src.replace("$(find m710id_description_spindle)", str(pkg_dir))
    tmp = top.with_name("_flatten_tmp.xacro")
    tmp.write_text(src)
    try:
        cmd = f"source {ROS_SETUP} && xacro {tmp}"
        out = subprocess.run(["bash", "-c", cmd], check=True, capture_output=True, text=True)
    finally:
        tmp.unlink(missing_ok=True)
    return out.stdout


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=ASSET_DIR)
    args = ap.parse_args()

    urdf = run_xacro(XACRO_TOP, ROS_PKG)

    # package://m710id_description_spindle/meshes/m710id_70/visual/j1.stl -> meshes/visual/j1.stl
    pat = re.compile(r"package://m710id_description_spindle/meshes/m710id_70/")
    n_refs = len(pat.findall(urdf))
    urdf = pat.sub("meshes/", urdf)
    assert "package://" not in urdf, "unresolved package:// URI left in URDF"

    out_dir: Path = args.out
    (out_dir / "meshes").mkdir(parents=True, exist_ok=True)
    copied = {}
    for sub in ("visual", "collision"):
        dst = out_dir / "meshes" / sub
        dst.mkdir(exist_ok=True)
        for stl in sorted((ROS_PKG / "meshes/m710id_70" / sub).glob("*.stl")):
            shutil.copy2(stl, dst / stl.name)
            copied[f"{sub}/{stl.name}"] = hashlib.md5((dst / stl.name).read_bytes()).hexdigest()

    urdf_path = out_dir / "m710id_70_spindle.urdf"
    urdf_path.write_text(urdf)
    manifest = {
        "source_xacro": str(XACRO_TOP.relative_to(REPO)),
        "urdf": str(urdf_path.relative_to(REPO)),
        "mesh_refs_rewritten": n_refs,
        "meshes_md5": copied,
    }
    (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=2))
    print(f"wrote {urdf_path} ({len(urdf)} chars, {n_refs} mesh refs rewritten, {len(copied)} meshes copied)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
