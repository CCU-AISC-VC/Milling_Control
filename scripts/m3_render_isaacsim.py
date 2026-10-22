#!/usr/bin/env python3
"""M3: render the Newton-recorded USD of a replay with Isaac Sim 6.1 RTX (path tracing).

Two processes by design: Newton records the USD (scripts/m3_sim_replay.py), this script only
opens Kit, adds lights/ground/cameras and renders frames through Replicator, then ffmpeg
encodes the MP4. Pattern taken from AISC_sim_env tests/t4_rtx_replay.py (verified on this
machine).

⚠️ Starting Kit requires the user's consent to the NVIDIA Omniverse EULA (AGENTS.md):
    OMNI_KIT_ACCEPT_EULA=YES ~/envs/aisc_sim/bin/python scripts/m3_render_isaacsim.py \
        --usd outputs/m3_sim/sample_pocket_contour/scene.usd --frames 240

First run compiles shaders (minutes); later runs are fast.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--usd", type=Path, default=REPO / "outputs/m3_sim/sample_pocket_contour/scene.usd")
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--frames", type=int, default=240, help="number of rendered frames (spread over the USD time range)")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--subframes", type=int, default=16, help="RTX path-tracing subframes per frame (quality)")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--orbit-deg", type=float, default=60.0, help="camera orbits this many degrees over the clip")
    ap.add_argument("--cam", type=float, nargs=3, default=(3.4, -3.0, 2.1))
    ap.add_argument("--target", type=float, nargs=3, default=(1.0, 0.0, 0.8))
    ap.add_argument("--closeup", action="store_true", help="use a close-up camera on the stock instead")
    ap.add_argument("--cell", type=Path, default=None, help="cell YAML: author its environment (floor, fence, cabinets) with USD materials and use its cameras")
    ap.add_argument("--camera", default=None, help="camera name from the cell YAML (overview / closeup / side)")
    ap.add_argument("--focal", type=float, default=24.0)
    args = ap.parse_args()
    cell_cfg = None
    if args.cell is not None:
        import yaml

        cell_cfg = yaml.safe_load(Path(args.cell).read_text())
        cams = cell_cfg.get("cameras", {})
        name = args.camera or ("closeup" if args.closeup else "overview")
        if name in cams:
            args.cam, args.target = tuple(cams[name]["pos"]), tuple(cams[name]["target"])
            args.closeup = name != "overview"
    if os.environ.get("OMNI_KIT_ACCEPT_EULA") != "YES":
        print("OMNI_KIT_ACCEPT_EULA is not YES: ask the user for EULA consent first (AGENTS.md).", file=sys.stderr)
        return 2
    args.usd = args.usd.resolve()  # Replicator writes relative output_dir under ~/omni.replicator_out
    suffix = f"rtx_{args.camera}" if args.camera else ("rtx_closeup" if args.closeup else "rtx")
    out = (args.out or (args.usd.parent / suffix)).resolve()
    out.mkdir(parents=True, exist_ok=True)
    if args.closeup and cell_cfg is None:
        args.cam, args.target = (2.35, -1.25, 1.45), (1.42, -0.02, 0.70)

    t0 = time.time()
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "width": args.width, "height": args.height})
    import omni.replicator.core as rep
    import omni.timeline
    import omni.usd
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade

    stage_in = Usd.Stage.Open(str(args.usd))
    t_start, t_end = stage_in.GetStartTimeCode(), stage_in.GetEndTimeCode()
    tps = stage_in.GetTimeCodesPerSecond()
    print("USD time range", t_start, t_end, "tps", tps, "prims", len(list(stage_in.Traverse())), flush=True)

    omni.usd.get_context().open_stage(str(args.usd))
    for _ in range(20):
        app.update()
    stage = omni.usd.get_context().get_stage()
    print("STAGE_OPEN_OK", round(time.time() - t0, 1), "s; upAxis", UsdGeom.GetStageUpAxis(stage), flush=True)

    # --- lights, ground, materials
    UsdLux.DomeLight.Define(stage, "/World/milling_dome").CreateIntensityAttr(700.0)
    key = UsdLux.DistantLight.Define(stage, "/World/milling_key")
    key.CreateIntensityAttr(2500.0)
    key.CreateAngleAttr(1.0)
    UsdGeom.Xformable(key).AddRotateXYZOp().Set(Gf.Vec3f(-50.0, 20.0, 35.0))
    fill = UsdLux.SphereLight.Define(stage, "/World/milling_fill")
    fill.CreateIntensityAttr(20000.0)
    fill.CreateRadiusAttr(0.3)
    UsdGeom.Xformable(fill).AddTranslateOp().Set(Gf.Vec3d(2.5, 1.5, 2.5))
    ground = UsdGeom.Mesh.Define(stage, "/World/milling_ground")
    s = 6.0
    ground.CreatePointsAttr([(-s, -s, 0), (s, -s, 0), (s, s, 0), (-s, s, 0)])
    ground.CreateFaceVertexCountsAttr([4])
    ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    ground.CreateDisplayColorAttr([(0.18, 0.19, 0.21)])
    UsdGeom.Xformable(ground).AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.001))
    # a slightly glossy preview-surface material for everything that has a displayColor
    mat = UsdShade.Material.Define(stage, "/World/milling_mat")
    shader = UsdShade.Shader.Define(stage, "/World/milling_mat/shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.45)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.1)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    # --- cell environment (fence, floor, cabinets) with proper materials; Newton's USD carries
    #     only robots + table + stock (opacity is not exported), so we re-author these here.
    if cell_cfg is not None:
        import sys as _sys

        _sys.path.insert(0, str(REPO / "src"))
        from milling_sim.cell import load_cell

        cell = load_cell(args.cell)
        env_root = UsdGeom.Scope.Define(stage, "/World/cell_env")

        def make_mat(path, color, roughness=0.6, metallic=0.0, opacity=1.0):
            m = UsdShade.Material.Define(stage, path)
            sh = UsdShade.Shader.Define(stage, path + "/shader")
            sh.CreateIdAttr("UsdPreviewSurface")
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
            sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
            if opacity < 1.0:
                sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
                sh.CreateInput("opacityThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
            m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
            return m

        mats = {}
        for bi, b in enumerate(cell.environment_boxes()):
            kind = "post" if b.name.startswith("fence_post") else "panel" if b.name.startswith("fence_panel") else "floor" if b.name == "floor" else "box"
            key = (kind, tuple(round(c, 3) for c in b.color), round(b.opacity, 2))
            if key not in mats:
                rough = {"post": 0.35, "panel": 0.15, "floor": 0.8, "box": 0.5}[kind]
                metal = {"post": 0.2, "panel": 0.0, "floor": 0.0, "box": 0.3}[kind]
                mats[key] = make_mat(f"/World/cell_env/mat_{len(mats)}", b.color, rough, metal, b.opacity)
            # USD prim names: [A-Za-z0-9_]; keep +/- sides distinct and add an index so names are unique
            prim_name = b.name.replace("+", "p").replace("-", "m")
            prim_name = "".join(ch if ch.isalnum() or ch == "_" else "_" for ch in prim_name) + f"_{bi}"
            cube = UsdGeom.Cube.Define(stage, f"/World/cell_env/{prim_name}")
            cube.CreateSizeAttr(1.0)
            xf = UsdGeom.Xformable(cube)
            xf.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in b.center]))
            xf.AddScaleOp().Set(Gf.Vec3f(*[float(v) for v in b.size]))
            UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(mats[key])
        # hide the generic ground we added above when a real floor exists
        if any(b.name == "floor" for b in cell.environment_boxes()):
            UsdGeom.Imageable(ground).MakeInvisible()
        print("ENV_OK", len(cell.environment_boxes()), "boxes", flush=True)

    # --- camera
    cam = UsdGeom.Camera.Define(stage, "/World/milling_cam")
    cam.CreateFocalLengthAttr(args.focal)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100.0))
    cam_xf = UsdGeom.Xformable(cam).AddTransformOp()

    def set_cam(frac: float):
        c = Gf.Vec3d(*args.cam)
        t = Gf.Vec3d(*args.target)
        ang = math.radians(args.orbit_deg * (frac - 0.5))
        d = c - t
        x, y = d[0] * math.cos(ang) - d[1] * math.sin(ang), d[0] * math.sin(ang) + d[1] * math.cos(ang)
        pos = Gf.Vec3d(t[0] + x, t[1] + y, c[2])
        view = Gf.Matrix4d().SetLookAt(pos, t, Gf.Vec3d(0, 0, 1))
        cam_xf.Set(view.GetInverse())

    set_cam(0.0)
    for _ in range(5):
        app.update()
    rp = rep.create.render_product("/World/milling_cam", (args.width, args.height))
    writer = rep.WriterRegistry.get("BasicWriter")
    frames_dir = out / "frames"
    frames_dir.mkdir(exist_ok=True)
    writer.initialize(output_dir=str(frames_dir), rgb=True)
    writer.attach([rp])
    timeline = omni.timeline.get_timeline_interface()
    n = args.frames
    for k in range(n):
        frac = k / max(n - 1, 1)
        tc = t_start + frac * (t_end - t_start)
        set_cam(frac)
        timeline.set_current_time(tc / tps)
        for _ in range(2):
            app.update()
        rep.orchestrator.step(rt_subframes=args.subframes)
        if k % 20 == 0 or k == n - 1:
            print(f"RENDER frame {k+1}/{n} timecode {tc:.1f} ({round(time.time() - t0)} s)", flush=True)
    rep.orchestrator.wait_until_complete()
    writer.detach()
    for _ in range(5):
        app.update()

    # --- encode BEFORE app.close(): SimulationApp.close() ends the interpreter, code after it never runs
    pngs = sorted(frames_dir.glob("rgb_*.png"))
    mp4 = out / (f"render_{args.camera}.mp4" if args.camera else ("render_closeup.mp4" if args.closeup else "render.mp4"))
    if pngs:
        subprocess.run(
            ["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(args.fps), "-pattern_type", "glob", "-i", str(frames_dir / "rgb_*.png"), "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18", str(mp4)],
            check=True,
        )
        import shutil

        shutil.copy2(pngs[len(pngs) // 2], out / "render_mid.png")
        shutil.copy2(pngs[-1], out / "render_last.png")
    info = {"usd": str(args.usd), "frames": len(pngs), "mp4": str(mp4), "width": args.width, "height": args.height, "subframes": args.subframes, "render_time_s": round(time.time() - t0, 1)}
    (out / "render_info.json").write_text(json.dumps(info, indent=2))
    print("RENDER_DONE", json.dumps(info), flush=True)
    app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
