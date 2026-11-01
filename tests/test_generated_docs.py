"""Drift test: committed generated P2 documents equal a fresh generation."""

import importlib.util
from pathlib import Path

import pytest

from simulation.myosim_source import DEFAULT_MYOSIM_DIR

ROOT = Path(__file__).resolve().parents[1]
pytestmark = pytest.mark.skipif(not (DEFAULT_MYOSIM_DIR / "myo_sim").is_dir(),
                                reason="third_party/myo_sim not cloned")


def test_generated_files_match_committed(tmp_path):
    spec = importlib.util.spec_from_file_location("p2gen", ROOT / "scripts" / "p2_generate_docs.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    for fresh in mod.generate(tmp_path):
        rel = fresh.relative_to(tmp_path)
        committed = ROOT / rel
        assert committed.is_file(), f"{rel} missing - run scripts/p2_generate_docs.py"
        # git may check text out with either line ending, so compare the content, not the bytes
        assert committed.read_text(encoding="utf-8") == fresh.read_text(encoding="utf-8"), f"{rel} is stale - regenerate"
