"""Derive the muscle_fixes_v1 patch (EDC5 force, RI2 path, defective lengthranges).

Lengthrange definition (MuJoCo, actuator lengthrange for muscles): the range of actuator length over
the joint ranges. Upstream MyoSim reproduces this definition for its healthy muscles (P4: <= a few mm),
so re-deriving it for muscles whose specified range does not match their geometry reproduces the
upstream method rather than inventing a value.

Procedure (deterministic):
  * geometry = upstream MyoHand + wrist_rom_v1 + EDC5 force + RI2 path fix (only the path matters);
  * joint ranges = the UPSTREAM ranges (the ranges upstream calibration used);
  * extremes of each actuator length over: 3-level grid of its spanned joints (if <= 7 joints),
    4000 seeded uniform samples, then coordinate-wise local refinement of the best min and max;
  * defective rule: < 50 % of the derived span lies inside the specified lengthrange, or the muscle's
    path is changed by this patch.

Writes data/patches/muscle_fixes_v1.yaml and reports/P4/muscle_fixes_derivation.json.
Usage: .venv/Scripts/python.exe scripts/p4_derive_muscle_fixes.py
"""

from __future__ import annotations

import itertools
import json
import sys
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from simulation.kinematics import forward
from simulation.muscles import moment_arm_matrix
from simulation.myosim_source import build_myohand
from simulation.patches import LENGTHRANGE_METHOD, Patch

SEED = 20261031
N_RANDOM = 4000
OVERLAP_RULE = 0.5
EDC5_FIX = {"field": "actuator_force", "actuator": "EDC5_r", "upstream_value_N": 139.4,
            "source_model": "mobl_arms_bimanual", "source_muscle": "EDCL_r", "patched_value_N": 39.4,
            "evidence": "gap T2: MyoHand 139.4 N vs MoBL-ARMS EDCL_r 39.4 N (exactly +100 N; transcription "
                        "error hypothesis); all other 25 extrinsic forces equal MoBL-ARMS"}
RI2_FIX = {"field": "tendon_remove_wrap", "tendon": "RI2_tendon_r", "remove": "2ndmcp_ellipsoid_wrap",
           "upstream_path": ["RI2-P1_r", "RI2-P2_r", "2ndmcp_ellipsoid_wrap", "RI2-P3_r"],
           "evidence": "gap M7': the cylinder wrap causes a 10 mm moment-arm jump near 10 deg MCP flexion and "
                       "11.9 N passive tension at rest. In the 2nd-Hand source RI2..RI5 share one path "
                       "structure; MyoHand converted RI3, RI4 and RI5 without an MCP wrap (smooth). Removing it "
                       "makes RI2 consistent with its converted siblings; mid/high-flexion moment arms unchanged."}


def partial_model() -> tuple[mujoco.MjModel, mujoco.MjModel]:
    up = build_myohand("fragment")
    spec = up.spec
    Patch.load("wrist_rom_v1").apply(spec)
    Patch("tmp", {"changes": [EDC5_FIX, RI2_FIX]}).apply(spec)
    return up.model, spec.compile()


def spanned_joints(m: mujoco.MjModel, ranges: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    d = mujoco.MjData(m)
    span = np.zeros((m.nu, m.njnt), dtype=bool)
    for _ in range(300):
        forward(m, rng.uniform(ranges[:, 0], ranges[:, 1]), d)
        mujoco.mj_transmission(m, d)
        span |= np.abs(moment_arm_matrix(m, d)) > 1e-4
    return span


def length_of(m: mujoco.MjModel, d: mujoco.MjData, q: np.ndarray, a: int) -> float:
    forward(m, q, d)
    mujoco.mj_transmission(m, d)
    return float(d.actuator_length[a])


def extremes(m: mujoco.MjModel, a: int, joints: np.ndarray, ranges: np.ndarray,
             rng: np.random.Generator) -> tuple[float, float]:
    d = mujoco.MjData(m)
    adr = m.jnt_qposadr[joints]
    cands = []
    if len(joints) <= 7:
        levels = [(ranges[j, 0], 0.5 * (ranges[j, 0] + ranges[j, 1]), ranges[j, 1]) for j in joints]
        for combo in itertools.product(*levels):
            q = m.qpos0.copy()
            q[adr] = combo
            cands.append(q)
    for _ in range(N_RANDOM):
        q = m.qpos0.copy()
        q[adr] = rng.uniform(ranges[joints, 0], ranges[joints, 1])
        cands.append(q)
    vals = np.array([length_of(m, d, q, a) for q in cands])
    best = {}
    for sense, idx in (("min", int(np.argmin(vals))), ("max", int(np.argmax(vals)))):
        q, v = cands[idx].copy(), vals[idx]
        sgn = 1.0 if sense == "max" else -1.0
        step = 0.25
        while step > 1e-4:
            improved = False
            for k, j in enumerate(joints):
                for dirn in (1.0, -1.0):
                    trial = q.copy()
                    span_j = ranges[j, 1] - ranges[j, 0]
                    trial[adr[k]] = np.clip(q[adr[k]] + dirn * step * span_j, ranges[j, 0], ranges[j, 1])
                    tv = length_of(m, d, trial, a)
                    if sgn * (tv - v) > 1e-12:
                        q, v, improved = trial, tv, True
            if not improved:
                step *= 0.5
        best[sense] = v
    return best["min"], best["max"]


def main() -> None:
    upstream, m = partial_model()
    ranges = upstream.jnt_range.copy()       # upstream calibration ranges
    rng = np.random.default_rng(SEED)
    span = spanned_joints(m, ranges, rng)
    rows: list[dict[str, Any]] = []
    for a in range(m.nu):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a)
        lo, hi = extremes(m, a, np.where(span[a])[0], ranges, rng)
        spec_lr = upstream.actuator_lengthrange[a]
        overlap = max(0.0, min(spec_lr[1], hi) - max(spec_lr[0], lo)) / (hi - lo)
        lo, hi = float(lo), float(hi)
        rows.append({"actuator": name, "n_joints": int(span[a].sum()),
                     "specified_m": [round(float(x), 6) for x in spec_lr],
                     "derived_m": [round(lo, 6), round(hi, 6)],
                     "delta_lo_mm": round(1000 * (spec_lr[0] - lo), 2),
                     "delta_hi_mm": round(1000 * (spec_lr[1] - hi), 2),
                     "overlap_fraction": round(float(overlap), 3),
                     "path_changed_by_patch": name == "RI2_r"})
    for r in rows:
        r["defective"] = r["overlap_fraction"] < OVERLAP_RULE or r["path_changed_by_patch"]
    healthy = [r for r in rows if not r["defective"]]
    errs = np.array([abs(r["delta_lo_mm"]) for r in healthy] + [abs(r["delta_hi_mm"]) for r in healthy])
    agreement = {"n_healthy": len(healthy), "median_abs_mm": round(float(np.median(errs)), 2),
                 "p90_abs_mm": round(float(np.percentile(errs, 90)), 2), "max_abs_mm": round(float(errs.max()), 2)}
    changes: list[dict[str, Any]] = [EDC5_FIX, RI2_FIX]
    for r in rows:
        if r["defective"]:
            changes.append({"field": "actuator_lengthrange", "actuator": r["actuator"],
                            "upstream_value_m": r["specified_m"], "patched_value_m": r["derived_m"],
                            "derivation": LENGTHRANGE_METHOD,
                            "reason": ("path changed by this patch" if r["path_changed_by_patch"]
                                       else f"only {100 * r['overlap_fraction']:.0f} % of the geometric span lies "
                                            f"inside the upstream lengthrange")})
    patch = {
        "patch_id": "muscle_fixes_v1", "version": 1, "status": "provisional",
        "approved_by": "project owner ('fix the defects', 2026-10-31)",
        "rationale": ("Repairs P1/P4 muscle defects using only source values or MuJoCo's documented lengthrange "
                      "definition. No anatomical attachment, force, or curve shape is invented."),
        "evidence": ["reports/P4/P4_REPORT.md", "reports/P4/muscle_fixes_derivation.json",
                     "reports/P1/osim_vs_myohand.json"],
        "lengthrange_method": {
            "id": LENGTHRANGE_METHOD, "script": "scripts/p4_derive_muscle_fixes.py", "seed": SEED,
            "definition": "min/max actuator length over the upstream joint ranges (MuJoCo lengthrange definition)",
            "defective_rule": f"overlap of derived span with upstream lengthrange < {OVERLAP_RULE}, or path changed",
            "method_agreement_on_healthy_upstream_muscles": agreement},
        "changes": changes,
        "not_changed": "All other actuators, tendons, forces and the MuJoCo muscle curve parameters.",
    }
    (ROOT / "data" / "patches" / "muscle_fixes_v1.yaml").write_text(
        "# GENERATED by scripts/p4_derive_muscle_fixes.py - review before use; do not edit numbers by hand.\n"
        + yaml.safe_dump(patch, sort_keys=False, allow_unicode=True, width=110), encoding="utf-8")
    (ROOT / "reports" / "P4" / "muscle_fixes_derivation.json").write_text(
        json.dumps({"agreement": agreement, "muscles": rows}, indent=1) + "\n")
    print("method agreement on healthy muscles:", agreement)
    for r in rows:
        if r["defective"]:
            print("FIX", r["actuator"], r["specified_m"], "->", r["derived_m"], "overlap", r["overlap_fraction"])


if __name__ == "__main__":
    main()
