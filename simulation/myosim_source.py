"""Pinned access to the upstream MyoSim checkout (third_party/myo_sim).

The upstream repository is NOT vendored into this project. It is cloned
separately (see docs/P1_LICENSE_AND_PROVENANCE.md) and verified against the
pinned commit before any of its Python build code is imported.

At the pinned commit MyoHand is no longer a static XML file: it is composed at
runtime with ``mujoco.MjSpec`` by pruning the full right-arm model
(``myo_sim/build/hand.py``). Two upstream entry points are exposed here:

* ``"fragment"`` -- ``myo_sim.FRAGMENT_SPEC_BUILDERS["myohand_r"]``: the hand
  derived from the pruned right arm, with no torso scaffold and no contact pairs.
* ``"composed"`` -- ``myo_sim.load_spec("myohand_r")``: the same hand attached to
  a passive anatomical torso scaffold, plus the upstream hand contact pairs.
"""

from __future__ import annotations

import contextlib
import os
import subprocess
import sys
import tempfile
import warnings
from collections.abc import Iterator
from dataclasses import dataclass, field
from pathlib import Path
from typing import Literal

import mujoco

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MYOSIM_DIR = PROJECT_ROOT / "third_party" / "myo_sim"

MYOSIM_REPO_URL = "https://github.com/MyoHub/myo_sim"
MYOSIM_PINNED_COMMIT = "93b0ca8f4ec90c9899ee7f05fee561e9911da91b"
MYOSIM_PINNED_VERSION = "0.2.3"

Variant = Literal["fragment", "composed"]


class ProvenanceError(RuntimeError):
    """Raised when the upstream checkout does not match the pinned provenance."""


@dataclass(frozen=True)
class CheckoutInfo:
    """Verified identity of the upstream checkout."""

    path: Path
    commit: str
    dirty_files: tuple[str, ...]


@dataclass
class BuiltModel:
    """A compiled upstream model plus the warnings raised while building it."""

    variant: Variant
    model: mujoco.MjModel
    spec: mujoco.MjSpec
    build_warnings: list[str] = field(default_factory=list)


def _git(path: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", "-C", str(path), *args], capture_output=True, text=True, check=True
    )
    return result.stdout.strip()


def verify_checkout(path: Path = DEFAULT_MYOSIM_DIR, allow_dirty: bool = False) -> CheckoutInfo:
    """Check that ``path`` is a clean git checkout of the pinned MyoSim commit."""
    if not (path / "myo_sim" / "build" / "hand.py").is_file():
        raise ProvenanceError(f"MyoSim checkout not found at {path}")
    commit = _git(path, "rev-parse", "HEAD")
    if commit != MYOSIM_PINNED_COMMIT:
        raise ProvenanceError(f"MyoSim HEAD is {commit}, expected {MYOSIM_PINNED_COMMIT}")
    status = _git(path, "status", "--porcelain")
    dirty = tuple(line for line in status.splitlines() if line.strip())
    if dirty and not allow_dirty:
        raise ProvenanceError(f"MyoSim checkout has local modifications: {dirty}")
    return CheckoutInfo(path=path, commit=commit, dirty_files=dirty)


def _import_myo_sim(path: Path):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
    import myo_sim  # type: ignore[import-not-found]

    module_file = Path(myo_sim.__file__).resolve()
    if path.resolve() not in module_file.parents:
        raise ProvenanceError(f"myo_sim imported from unexpected location {module_file}")
    return myo_sim


def build_myohand(variant: Variant = "fragment", path: Path = DEFAULT_MYOSIM_DIR) -> BuiltModel:
    """Build and compile MyoHand from the verified pinned checkout.

    No upstream parameter is modified. Python warnings raised during building
    and compilation are captured and returned with the model.
    """
    if variant not in ("fragment", "composed"):
        raise ValueError(f"Unknown variant {variant!r}")
    verify_checkout(path)
    myo_sim = _import_myo_sim(path)
    with _capture_native_stderr() as native, warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        if variant == "fragment":
            spec = myo_sim.FRAGMENT_SPEC_BUILDERS["myohand_r"]()
        else:
            spec = myo_sim.load_spec("myohand_r")
        model = spec.compile()
    messages = [f"{w.category.__name__}: {w.message}" for w in caught]
    messages += [f"native stderr: {line}" for line in native.lines]
    return BuiltModel(variant=variant, model=model, spec=spec, build_warnings=messages)


class _NativeCapture:
    lines: list[str]

    def __init__(self) -> None:
        self.lines = []


@contextlib.contextmanager
def _capture_native_stderr() -> Iterator[_NativeCapture]:
    """Capture text written to file descriptor 2 (MuJoCo's C-level warnings)."""
    capture = _NativeCapture()
    sys.stderr.flush()
    saved = os.dup(2)
    with tempfile.TemporaryFile(mode="w+b") as tmp:
        os.dup2(tmp.fileno(), 2)
        try:
            yield capture
        finally:
            sys.stderr.flush()
            os.dup2(saved, 2)
            os.close(saved)
            tmp.seek(0)
            text = tmp.read().decode("utf-8", errors="replace")
            capture.lines.extend(line.strip() for line in text.splitlines() if line.strip())
