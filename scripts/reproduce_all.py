"""Regenerate the BioHand Twin evidence from the current model, in dependency order.

Requires third_party/ checkouts (README). P0 and P1 are run separately because they describe the machine and
the unpatched upstream model. The local visual stages are skipped when Blender or Godot are not found.
Usage: .venv/Scripts/python.exe scripts/reproduce_all.py [--with-visual]
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
STAGES = [
    ["scripts/p1_osim_inventory.py"],
    ["scripts/fix_derive_thumb_intrinsics.py"],
    ["scripts/fix_derive_passive_stiffness.py"],
    ["scripts/p2_generate_docs.py"],
    ["scripts/p3_kinematics_audit.py"],
    ["scripts/p3_wrist_convention_check.py"],
    ["scripts/p3_axis_convention_check.py"],
    ["scripts/p3_patch_check.py"],
    ["scripts/p4_muscle_audit.py"],
    ["scripts/p5_limit_stiffness_check.py"],
    ["scripts/p5_run_tasks.py"],
    ["scripts/p6_reproducibility.py"],
    ["scripts/p8_moment_arm_validation.py"],
    ["scripts/p8_sensitivity_performance.py"],
    ["scripts/fix_pinch_robustness.py"],
    ["scripts/fix_grip_checks.py"],
    ["scripts/fix_calibrate_moment_arms.py"],
    ["scripts/biohand.py", "export-godot-inventory"],
]
RUNS = ["hand_closing", "hand_opening", "thumb_opposition", "wrist_flexion_extension", "wrist_deviation",
        "power_grip_proxy", "pinch", "rest_neutral"]


def run(args: list[str]) -> None:
    print(">>", " ".join(args), flush=True)
    subprocess.run([sys.executable, *args], cwd=ROOT, check=True, stdout=subprocess.DEVNULL)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--with-visual", action="store_true", help="also write playback runs for the Godot viewer")
    args = ap.parse_args()
    for stage in STAGES:
        run(stage)
    if args.with_visual:
        for task in RUNS:
            run(["scripts/biohand.py", "run-task", task])
    print("done; now run: python -m pytest -v")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
