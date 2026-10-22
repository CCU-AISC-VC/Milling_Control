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

import numpy as np

REPO = Path(__file__).resolve().parents[1]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--usd", type=Path, default=REPO / "outputs/m3_sim/sample_pocket_contour/scene.usd")
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--frames", type=int, default=0, help="number of rendered frames spread over the USD time range (0 = one per USD frame, i.e. the replay's own time-lapse speed)")
    ap.add_argument("--speed", type=float, default=0.0, help="alternative to --frames: real-time factor of the clip (needs the replay's video_speed from summary.json)")
    ap.add_argument("--stock-color", type=float, nargs=3, default=(0.42, 0.38, 0.32), help="machined stock diffuse colour (dark matte by default)")
    ap.add_argument("--stock-roughness", type=float, default=0.92)
    ap.add_argument("--light-scale", type=float, default=0.55, help="scale on all light intensities (lower = less washed out)")
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
    # The Fabric scene delegate ignores time-sampled mesh points (the machined stock); use the
    # classic USD imaging delegate so deforming meshes follow the timeline.
    import carb.settings

    carb.settings.get_settings().set_bool("/app/useFabricSceneDelegate", False)
    carb.settings.get_settings().set_bool("/persistent/omnihydra/useFastSceneDelegate", False)
    import omni.replicator.core as rep
    import omni.timeline
    import omni.usd
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade

    stage_in = Usd.Stage.Open(str(args.usd))
    t_start, t_end = stage_in.GetStartTimeCode(), stage_in.GetEndTimeCode()
    tps = stage_in.GetTimeCodesPerSecond()
    print("USD time range", t_start, t_end, "tps", tps, "prims", len(list(stage_in.Traverse())), flush=True)

    # Kit does not animate time-sampled mesh `points` (the progressively machined stock) and
    # ignores prim edits made after the stage is open, so we pre-process the file: every points
    # time sample becomes its own static mesh prim whose `visibility` is time-sampled to cover
    # [t_i, t_i+1); the deforming prim is deactivated. The result is saved next to the source.
    deforming = [UsdGeom.Mesh(pr) for pr in stage_in.Traverse() if pr.IsA(UsdGeom.Mesh) and UsdGeom.Mesh(pr).GetPointsAttr().GetNumTimeSamples() > 1]
    snapshot_sets = []  # (time codes, prim paths) per deforming mesh
    usd_to_open = args.usd
    if deforming:
        stock_mat = UsdShade.Material.Define(stage_in, "/World/stock_snaps_mat")
        stock_sh = UsdShade.Shader.Define(stage_in, "/World/stock_snaps_mat/shader")
        stock_sh.CreateIdAttr("UsdPreviewSurface")
        stock_sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*[float(v) for v in args.stock_color]))
        stock_sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(args.stock_roughness))
        stock_sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        stock_sh.CreateInput("specular", Sdf.ValueTypeNames.Float).Set(0.15)
        stock_mat.CreateSurfaceOutput().ConnectToSource(stock_sh.ConnectableAPI(), "surface")
        for di, mesh in enumerate(deforming):
            pts_attr = mesh.GetPointsAttr()
            times = list(pts_attr.GetTimeSamples())
            # Newton writes topology as time samples too (no default value): read it at the first sample
            t0s = times[0]
            fvi = mesh.GetFaceVertexIndicesAttr().Get(t0s)
            fvc = mesh.GetFaceVertexCountsAttr().Get(t0s)
            nrm_attr = mesh.GetNormalsAttr()
            binding = UsdShade.MaterialBindingAPI(mesh.GetPrim()).GetDirectBinding().GetMaterial()
            color = mesh.GetDisplayColorAttr().Get()
            for i, tc in enumerate(times):
                snap = UsdGeom.Mesh.Define(stage_in, f"/World/stock_snaps/d{di}_s{i}")
                snap.CreatePointsAttr(pts_attr.Get(tc))
                snap.CreateFaceVertexIndicesAttr(fvi)
                snap.CreateFaceVertexCountsAttr(fvc)
                if nrm_attr and nrm_attr.HasValue() and nrm_attr.Get(tc) is not None:
                    snap.CreateNormalsAttr(nrm_attr.Get(tc))
                    snap.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
                snap.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
                snap.CreateExtentAttr(UsdGeom.PointBased(snap).ComputeExtent(pts_attr.Get(tc)))
                if os.environ.get("DEBUG_SNAP_COLOR"):
                    snap.CreateDisplayColorAttr([(1.0, 0.1, 0.1)])
                else:
                    UsdShade.MaterialBindingAPI.Apply(snap.GetPrim()).Bind(stock_mat)
                    snap.CreateDisplayColorAttr([tuple(float(v) for v in args.stock_color)])
                # Kit ignores time-sampled visibility and drops most in-session edits, but it does
                # follow time-sampled transforms (the robots animate that way). So every snapshot
                # is always visible and parked 1 km below the floor except during [t_i, t_i+1):
                # translate keys with a 0.001-timecode ramp so linear interpolation never shows.
                # one key per integer time code so the value is exact at every rendered frame,
                # whatever interpolation Kit applies between keys
                far = Gf.Vec3d(0.0, 0.0, -1000.0)
                home = Gf.Vec3d(0.0, 0.0, 0.0)
                op = UsdGeom.Xformable(snap).AddTranslateOp()
                t_lo = int(round(tc))
                t_hi = int(round(times[i + 1])) if i + 1 < len(times) else int(round(t_end)) + 61
                for k in range(int(round(t_start)), int(round(t_end)) + 61):  # keys well past the end: Kit may evaluate beyond the last frame
                    v = home if t_lo <= k < t_hi else far
                    op.Set(v, float(k))
                    op.Set(v, float(k) + 0.999)  # hold the value: Kit evaluates at fractional times
            mesh.GetPrim().SetActive(False)
            snapshot_sets.append((np.array(times), [str(pr.GetPath()) for pr in [stage_in.GetPrimAtPath(f"/World/stock_snaps/d{di}_s{i}") for i in range(len(times))]]))
            print(f"SNAPSHOTS {mesh.GetPath()}: {len(times)} meshes", flush=True)
        usd_to_open = args.usd.with_name(args.usd.stem + "_render.usd")
        stage_in.GetRootLayer().Export(str(usd_to_open))
        print("RENDER_USD", usd_to_open, flush=True)

    omni.usd.get_context().open_stage(str(usd_to_open))
    for _ in range(20):
        app.update()
    stage = omni.usd.get_context().get_stage()
    print("STAGE_OPEN_OK", round(time.time() - t0, 1), "s; upAxis", UsdGeom.GetStageUpAxis(stage), flush=True)

    # --- lights, ground, materials
    ls = float(args.light_scale)
    UsdLux.DomeLight.Define(stage, "/World/milling_dome").CreateIntensityAttr(700.0 * ls)
    key = UsdLux.DistantLight.Define(stage, "/World/milling_key")
    key.CreateIntensityAttr(2500.0 * ls)
    key.CreateAngleAttr(1.0)
    UsdGeom.Xformable(key).AddRotateXYZOp().Set(Gf.Vec3f(-50.0, 20.0, 35.0))
    fill = UsdLux.SphereLight.Define(stage, "/World/milling_fill")
    fill.CreateIntensityAttr(20000.0 * ls)
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
    timeline.set_looping(False)  # otherwise setting the end time wraps to the start (last frame showed t = 0)
    timeline.set_end_time((t_end + 60.0) / tps)  # keep the clamp away from the last rendered frame
    snap_current = [0] * len(snapshot_sets)

    def show_snapshots(tc):
        return None  # snapshots are driven by their time-sampled transforms in the pre-processed USD
    # Deforming meshes (the progressively machined stock) carry time-sampled `points`; Kit's
    # Fabric scene delegate does not re-read them when the timeline moves, so bake the sample
    # of the current frame into the attribute's default value before each render.
    n = int(args.frames)
    if n <= 0:
        n = int(round(t_end - t_start)) + 1  # one rendered frame per USD frame
        if args.speed > 0:
            info_path = args.usd.parent / "summary.json"
            try:
                vs = float(json.loads(info_path.read_text()).get("video_speed", 1.0))
                n = int(round(n * vs / args.speed))  # re-time the clip to the requested real-time factor
            except Exception:
                pass
    print("RENDER_FRAMES", n, flush=True)
    for k in range(n):
        frac = k / max(n - 1, 1)
        tc = float(round(t_start + frac * (t_end - t_start)))  # integer time codes: exact keys
        set_cam(frac)
        show_snapshots(tc)
        timeline.set_current_time(tc / tps)
        for _ in range(2):
            app.update()
        rep.orchestrator.step(rt_subframes=args.subframes)
        if k >= n - 2:
            print(f"TIMELINE frame {k}: requested tc {tc}, timeline now {timeline.get_current_time() * tps:.2f} tc", flush=True)
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
