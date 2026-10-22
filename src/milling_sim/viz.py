"""Plotting and head-less rendering helpers shared by all stage scripts."""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def look_at_yaw_pitch(cam_pos, target) -> tuple[float, float]:
    """Yaw/pitch [deg] for Newton's GL viewer so the camera at ``cam_pos`` looks at ``target``.

    Newton (Z-up): front = (cos(yaw)cos(pitch), sin(yaw)cos(pitch), sin(pitch)).
    """
    d = np.asarray(target, dtype=float) - np.asarray(cam_pos, dtype=float)
    yaw = math.degrees(math.atan2(d[1], d[0]))
    pitch = math.degrees(math.atan2(d[2], math.hypot(d[0], d[1])))
    return yaw, pitch


class HeadlessGL:
    """Thin wrapper around ``newton.viewer.ViewerGL(headless=True)`` that returns RGB frames."""

    def __init__(self, model, width: int = 1280, height: int = 720):
        import newton.viewer as nv

        self.viewer = nv.ViewerGL(width=width, height=height, headless=True)
        self.viewer.set_model(model)
        self.t = 0.0

    def set_camera(self, pos, target) -> None:
        import warp as wp

        yaw, pitch = look_at_yaw_pitch(pos, target)
        self.viewer.set_camera(wp.vec3(*[float(v) for v in pos]), pitch=pitch, yaw=yaw)

    def render(self, state, points=None, lines=None, dt: float = 1 / 60) -> np.ndarray:
        """Render one frame. ``points``: list of (name, Nx3, radius, rgb); ``lines``: list of (name, Nx3 starts, Nx3 ends, rgb)."""
        import warp as wp

        v = self.viewer
        v.begin_frame(self.t)
        v.log_state(state)
        for name, pts, radius, rgb in points or []:
            pts = np.asarray(pts, dtype=np.float32).reshape(-1, 3)
            cols = np.tile(np.asarray(rgb, dtype=np.float32), (len(pts), 1))
            v.log_points(
                name,
                wp.array(pts, dtype=wp.vec3),
                radii=wp.array(np.full(len(pts), float(radius), dtype=np.float32), dtype=wp.float32),
                colors=wp.array(cols, dtype=wp.vec3),
            )
        for name, starts, ends, rgb in lines or []:
            starts = np.asarray(starts, dtype=np.float32).reshape(-1, 3)
            ends = np.asarray(ends, dtype=np.float32).reshape(-1, 3)
            cols = np.tile(np.asarray(rgb, dtype=np.float32), (len(starts), 1))
            v.log_lines(
                name,
                wp.array(starts, dtype=wp.vec3),
                wp.array(ends, dtype=wp.vec3),
                wp.array(cols, dtype=wp.vec3),
                width=0.01,
            )
        v.end_frame()
        self.t += dt
        return v.get_frame().numpy()

    def close(self) -> None:
        try:
            self.viewer.close()
        except Exception:
            pass


def save_png(path: str | Path, rgb: np.ndarray) -> None:
    import imageio.v3 as iio

    Path(path).parent.mkdir(parents=True, exist_ok=True)
    iio.imwrite(str(path), np.asarray(rgb, dtype=np.uint8))


def save_gif(path: str | Path, frames: list[np.ndarray], fps: int = 20) -> None:
    import imageio.v3 as iio

    Path(path).parent.mkdir(parents=True, exist_ok=True)
    iio.imwrite(str(path), [np.asarray(f, dtype=np.uint8) for f in frames], duration=1000 / fps, loop=0)


def save_mp4(path: str | Path, frames: list[np.ndarray], fps: int = 30, crf: int = 18) -> None:
    """Encode RGB frames with the system ``ffmpeg`` (piped raw video); no Python codec packages needed."""
    import subprocess

    Path(path).parent.mkdir(parents=True, exist_ok=True)
    h, w = np.asarray(frames[0]).shape[:2]
    cmd = [
        "ffmpeg", "-y", "-loglevel", "error",
        "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}", "-r", str(fps), "-i", "-",
        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", str(crf), "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2", str(path),
    ]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    for f in frames:
        proc.stdin.write(np.ascontiguousarray(np.asarray(f, dtype=np.uint8)).tobytes())
    proc.stdin.close()
    if proc.wait() != 0:
        raise RuntimeError(f"ffmpeg failed for {path}")


def new_figure(nrows=1, ncols=1, size=(8, 5)):
    fig, ax = plt.subplots(nrows, ncols, figsize=size, constrained_layout=True)
    return fig, ax


def finish_figure(fig, path: str | Path, dpi: int = 130) -> None:
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=dpi)
    plt.close(fig)
