"""Derive optional patch moment_arm_calibration_v1: tuned site shifts for the moment arms that failed in P8.

This is a FIT to the reference data, so agreement afterwards is not validation. The patch is deliberately not in
DEFAULT_PATCHES. The unfitted comparison in reports/P8/moment_arm_validation.json stays the validation result.

For each failing muscle, the variables are 3D shifts of the two path sites next to the joint that fails: the last
site proximal and the first site distal to it. The fit minimises the z-scores of every reference value of that
muscle (also those that already pass), plus a penalty on shift size (SHIFT_SCALE_M), within +/-MAX_SHIFT_M per axis.
Curve parameters, F_max and lengthrange are unchanged.

Writes data/patches/moment_arm_calibration_v1.yaml and reports/fixes/moment_arm_calibration.json.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import yaml
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from p8_moment_arm_validation import mean_arm

from simulation.myosim_source import build_myohand
from simulation.patches import build_biohand_model, muscle_validity_envelope

MAX_SHIFT_M = 0.008
SHIFT_SCALE_M = 0.02
#: muscle -> joint whose neighbouring sites are shifted (from the failures in P8 sections 4.1 and 4.2).
TARGETS = {"FCR_r": "flexion_r", "FCU_r": "flexion_r", "ECU_r": "flexion_r",
           "FDP2_r": "mcp2_abduction_r", "EDC2_r": "mcp2_abduction_r"}


def reference_rows() -> list[dict[str, Any]]:
    """All 24 reference values of the P8 wrist and index sets, in model sign."""
    data = yaml.safe_load((ROOT / "data/validation/reference_moment_arms.yaml").read_text(encoding="utf-8"))["sets"]
    g = data["garland2018_wrist_tendon_excursion"]
    rows = []
    for table, joint, sign in ((g["flexion_extension"], "flexion_r", 1.0), (g["radioulnar_deviation"], "deviation_r", -1.0)):
        rows += [{"set": "garland2018", "actuator": a, "joint": joint, "sign": sign, "ref": v[0], "sd": v[1]}
                 for a, v in table.items()]
    t = data["lee2015_index_derived_from_attachments"]
    names = {"FDP": "FDP2_r", "FDS": "FDS2_r", "EDC": "EDC2_r", "LU": "LU_RB2_r", "RI": "RI2_r", "UI": "UI_UB2_r"}
    for table, joint, sd in ((t["mcp_flexion"], "mcp2_flexion_r", t["measured_sd_mm"]["flexion_extension"]),
                             (t["mcp_abduction"], "mcp2_abduction_r", t["measured_sd_mm"]["abduction_adduction"])):
        rows += [{"set": "lee2015", "actuator": names[k], "joint": joint, "sign": 1.0, "ref": v, "sd": sd}
                 for k, v in table.items()]
    return rows


def evaluate(m: mujoco.MjModel, env: dict[str, tuple[float, float]], row: dict[str, Any]) -> float:
    j = m.joint(row["joint"]).id
    lo, hi = env.get(row["joint"], tuple(m.jnt_range[j]))
    return row["sign"] * mean_arm(m, row["actuator"], row["joint"], float(lo), float(hi))


def passed(value: float, row: dict[str, Any]) -> bool:
    return bool(np.sign(value) == np.sign(row["ref"]) and abs(value - row["ref"]) <= 2 * row["sd"])


def neighbour_sites(spec: mujoco.MjSpec, m: mujoco.MjModel, actuator: str, joint: str) -> list[str]:
    """Last path site proximal to the joint and first path site distal to it."""
    distal_root = m.jnt_bodyid[m.joint(joint).id]

    def is_distal(body: int) -> bool:
        while body != 0:
            if body == distal_root:
                return True
            body = m.body_parentid[body]
        return False

    target = spec.actuator(actuator).target
    sites = [w.target.name for w in spec.tendon(target).path if w.type == mujoco.mjtWrap.mjWRAP_SITE]
    flags = [is_distal(int(m.site_bodyid[m.site(s).id])) for s in sites]
    first = flags.index(True)
    return [sites[first - 1], sites[first]]


def main() -> None:
    built = build_biohand_model()
    m = built.model
    env = muscle_validity_envelope(build_myohand("fragment").model, m)
    rows = reference_rows()
    before = [evaluate(m, env, r) for r in rows]
    shifts: dict[str, list[float]] = {}
    fits = {}
    for act, joint in TARGETS.items():
        sites = neighbour_sites(built.spec, m, act, joint)
        ids = [m.site(s).id for s in sites]
        base = m.site_pos[ids].copy()
        mine = [r for r in rows if r["actuator"] == act]

        def residual(x: np.ndarray, ids: list[int] = ids, base: np.ndarray = base, mine: list[Any] = mine) -> np.ndarray:
            m.site_pos[ids] = base + x.reshape(-1, 3)
            z = [(evaluate(m, env, r) - r["ref"]) / r["sd"] for r in mine]
            return np.concatenate([z, x / SHIFT_SCALE_M])

        res = least_squares(residual, np.zeros(6), bounds=(-MAX_SHIFT_M, MAX_SHIFT_M), diff_step=None, x_scale=1e-3,
                            max_nfev=25)
        m.site_pos[ids] = base + res.x.reshape(-1, 3)
        fits[act] = {"sites": sites, "cost": round(float(res.cost), 4), "nfev": int(res.nfev)}
        for s, v in zip(sites, res.x.reshape(-1, 3), strict=True):
            shifts[s] = [round(float(a), 6) for a in v]
    after = [evaluate(m, env, r) for r in rows]

    table = []
    for r, b, a in zip(rows, before, after, strict=True):
        table.append({"set": r["set"], "actuator": r["actuator"], "joint": r["joint"], "reference_mm": r["ref"],
                      "sd_mm": r["sd"], "unfitted_mm": round(b, 2), "fitted_mm": round(a, 2),
                      "unfitted_passed": passed(b, r), "fitted_passed": passed(a, r)})
    summary = {k: {"unfitted": sum(t["unfitted_passed"] for t in table if t["set"] == k),
                   "fitted": sum(t["fitted_passed"] for t in table if t["set"] == k),
                   "n": sum(t["set"] == k for t in table)} for k in ("garland2018", "lee2015")}
    patch = {
        "patch_id": "moment_arm_calibration_v1", "version": 1, "status": "optional",
        "classification": "TUNED to the data it is compared with; agreement is a fit, not validation",
        "approved_by": "project owner ('fix this', 2026-11-01); not part of DEFAULT_PATCHES",
        "evidence": ["reports/fixes/moment_arm_calibration.json"],
        "changes": [{"field": "site_shift", "site": s, "shift_m": v, "max_abs_axis_m": MAX_SHIFT_M}
                    for s, v in shifts.items()],
    }
    text = "# GENERATED by scripts/fix_calibrate_moment_arms.py - do not edit numbers by hand.\n"
    text += yaml.safe_dump(patch, sort_keys=False, width=110)
    (ROOT / "data" / "patches" / "moment_arm_calibration_v1.yaml").write_text(text, encoding="utf-8")
    largest = max(float(np.linalg.norm(v)) for v in shifts.values())
    out = {"warning": patch["classification"], "summary": summary, "largest_site_shift_mm": round(largest * 1e3, 2),
           "fits": fits, "rows": table}
    (ROOT / "reports" / "fixes" / "moment_arm_calibration.json").write_text(json.dumps(out, indent=1) + "\n", encoding="utf-8")
    print(json.dumps({k: out[k] for k in ("summary", "largest_site_shift_mm", "fits")}, indent=1))
    for t in table:
        if not (t["unfitted_passed"] and t["fitted_passed"]):
            print(t["actuator"], t["joint"], t["reference_mm"], t["unfitted_mm"], "->", t["fitted_mm"],
                  t["unfitted_passed"], t["fitted_passed"])


if __name__ == "__main__":
    main()
