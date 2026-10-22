"""milling_sim: G-code -> toolpath -> joint trajectory -> Newton simulation for robotic milling.

Reusable code lives here; ``scripts/`` only holds stage workflows and parameters.
Internal units are SI (m, s, rad); G-code is converted at the parser boundary.
"""

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
ASSETS_DIR = REPO_ROOT / "assets"
CONFIGS_DIR = REPO_ROOT / "configs"
OUTPUTS_DIR = REPO_ROOT / "outputs"

__all__ = ["REPO_ROOT", "ASSETS_DIR", "CONFIGS_DIR", "OUTPUTS_DIR"]
