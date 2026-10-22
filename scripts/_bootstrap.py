"""Import this first in every script so ``milling_sim`` resolves without installing it.

Usage (top of a script)::

    import _bootstrap  # noqa: F401
    from milling_sim import ...
"""
import sys
from pathlib import Path

_SRC = Path(__file__).resolve().parents[1] / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))
