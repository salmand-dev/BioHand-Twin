"""P8: compare model moment arms with published data (data/validation/reference_moment_arms.yaml).

Model value = mean of r = -dL/dq (mm) over the stated joint range, other joints at qpos0, for the upstream
MyoHand and the BioHand reference model. Every reference value is compared; none is left out.
Output: reports/P8/moment_arm_validation.json
"""

from __future__ import annotations

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
from simulation.patches import build_biohand_model, muscle_validity_envelope

ACT = {"FDP": "FDP{d}_r", "FDS": "FDS{d}_r", "EDC": "EDC{d}_r", "LU": "LU_RB{d}_r", "RI": "RI{d}_r", "UI": "UI_UB{d}_r"}
N = 91


def mean_arm(m: mujoco.MjModel, actuator: str, joint: str, lo: float, hi: float) -> float:
    d = mujoco.MjData(m)
    j = m.joint(joint).id
    vals = []
    for qv in np.linspace(lo, hi, N):
        q = m.qpos0.copy()
        q[m.jnt_qposadr[j]] = qv
        forward(m, q, d)
        mujoco.mj_transmission(m, d)
        vals.append(moment_arm_matrix(m, d)[m.actuator(actuator).id, m.jnt_dofadr[j]])
    return float(1000.0 * np.mean(vals))


def stats(rows: list[dict[str, Any]], key: str = "model_mm") -> dict[str, Any]:
    ref = np.array([r["reference_mm"] for r in rows])
    mod = np.array([r[key] for r in rows])
    err = mod - ref
    return {"n": len(rows), "rmse_mm": round(float(np.sqrt(np.mean(err ** 2))), 2),
            "mean_abs_err_mm": round(float(np.mean(np.abs(err))), 2),
            "max_abs_err_mm": round(float(np.max(np.abs(err))), 2),
            "pearson_r": round(float(np.corrcoef(ref, mod)[0, 1]), 3),
            "sign_agreement": round(float(np.mean(np.sign(ref) == np.sign(mod))), 3)}


def main() -> None:
    ref = yaml.safe_load((ROOT / "data" / "validation" / "reference_moment_arms.yaml").read_text(encoding="utf-8"))["sets"]
    upstream, biohand = build_myohand("fragment").model, build_biohand_model().model
    env = muscle_validity_envelope(upstream, biohand)
    out: dict[str, Any] = {}

    # --- 1. wrist: measured (Garland 2018). Model deviation + = radial; reference + = ulnar, so negate.
    g = ref["garland2018_wrist_tendon_excursion"]
    rows = []
    for label, joint, table, sign in (("flexion_extension", "flexion_r", g["flexion_extension"], 1.0),
                                      ("radioulnar_deviation", "deviation_r", g["radioulnar_deviation"], -1.0)):
        lo, hi = env[joint]
        wlo, whi = biohand.jnt_range[biohand.joint(joint).id]
        for act, (mean, sd) in table.items():
            v = sign * mean_arm(biohand, act, joint, lo, hi)
            vfull = sign * mean_arm(biohand, act, joint, wlo, whi)
            ok = bool(np.sign(v) == np.sign(mean) and abs(v - mean) <= 2 * sd)
            rows.append({"axis": label, "actuator": act, "reference_mm": mean, "sd_mm": sd, "model_mm": round(v, 2),
                         "model_full_range_mm": round(vfull, 2), "z": round((v - mean) / sd, 2), "passed": ok})
    out["wrist_measured_garland2018"] = {
        "rows": rows, "n_passed": sum(r["passed"] for r in rows), "n": len(rows), "stats": stats(rows),
        "assumption": "model pro_sup_r = 0 taken as mid-prone; model mean over the muscle validity envelope"}

    # --- 2. index MCP: derived from measured attachments (Lee 2015 Table 2)
    t2 = ref["lee2015_index_derived_from_attachments"]
    rows = []
    for label, joint, table, sd in (("mcp_flexion", "mcp2_flexion_r", t2["mcp_flexion"], t2["measured_sd_mm"]["flexion_extension"]),
                                    ("mcp_abduction", "mcp2_abduction_r", t2["mcp_abduction"], t2["measured_sd_mm"]["abduction_adduction"])):
        lo, hi = biohand.jnt_range[biohand.joint(joint).id]
        for name, val in table.items():
            act = ACT[name].format(d=2)
            v = mean_arm(biohand, act, joint, lo, hi)
            u = mean_arm(upstream, act, joint, lo, hi)
            rows.append({"joint": label, "muscle": name, "actuator": act, "reference_mm": val, "model_mm": round(v, 2),
                         "upstream_mm": round(u, 2), "passed": bool(np.sign(v) == np.sign(val) and abs(v - val) <= 2 * sd)})
    out["index_mcp_derived_lee2015"] = {"rows": rows, "n_passed": sum(r["passed"] for r in rows), "n": len(rows),
                                        "stats": stats(rows), "stats_upstream": stats(rows, "upstream_mm")}

    # --- 3. conversion fidelity against the 2nd-Hand OpenSim model (Lee 2015 Tables 3-6)
    t3 = ref["lee2015_source_model"]
    rows = []
    for d, joints in t3["digits"].items():
        for prefix, table in joints.items():
            deg = t3["ranges_deg"][prefix]
            kind = prefix.split("_")[0]
            joint = f"{kind}{d}_{prefix.split('_')[1]}_r"
            for name, val in table.items():
                act = ACT[name].format(d=d)
                v = mean_arm(biohand, act, joint, np.radians(deg[0]), np.radians(deg[1]))
                u = mean_arm(upstream, act, joint, np.radians(deg[0]), np.radians(deg[1]))
                rows.append({"digit": d, "joint": joint, "muscle": name, "reference_mm": val, "model_mm": round(v, 2),
                             "upstream_mm": round(u, 2), "within_2p1mm": bool(abs(v - val) <= 2.1)})
    groups = {g_: [r for r in rows if r["joint"].startswith(g_)] for g_ in ("mcp", "pm", "md")}
    ab = [r for r in rows if "abduction" in r["joint"]]
    fe = [r for r in rows if "flexion" in r["joint"]]
    out["fidelity_vs_2nd_hand"] = {
        "rows": rows, "n": len(rows), "share_within_2p1mm": round(float(np.mean([r["within_2p1mm"] for r in rows])), 3),
        "stats_all": stats(rows), "stats_upstream_all": stats(rows, "upstream_mm"),
        "stats_flexion_joints": stats(fe), "stats_abduction": stats(ab),
        "stats_by_joint_type": {k: stats(v) for k, v in groups.items()}}

    path = ROOT / "reports" / "P8"
    path.mkdir(parents=True, exist_ok=True)
    (path / "moment_arm_validation.json").write_text(json.dumps(out, indent=1) + "\n")
    for k, v in out.items():
        print(k, {x: y for x, y in v.items() if x != "rows"})
    for r in out["wrist_measured_garland2018"]["rows"]:
        print(f"  {r['axis'][:6]} {r['actuator']:7s} ref {r['reference_mm']:6.1f} +- {r['sd_mm']:.1f}  model {r['model_mm']:6.2f}  full {r['model_full_range_mm']:6.2f}  z {r['z']:+5.1f}  {'PASS' if r['passed'] else 'FAIL'}")
    for r in out["index_mcp_derived_lee2015"]["rows"]:
        print(f"  {r['joint']:13s} {r['muscle']:4s} ref {r['reference_mm']:5.1f}  model {r['model_mm']:6.2f}  upstream {r['upstream_mm']:6.2f}  {'PASS' if r['passed'] else 'FAIL'}")
    worst = sorted(out["fidelity_vs_2nd_hand"]["rows"], key=lambda r: -abs(r["model_mm"] - r["reference_mm"]))[:8]
    print("  worst fidelity:", [(r["joint"], r["muscle"], r["reference_mm"], r["model_mm"]) for r in worst])


if __name__ == "__main__":
    main()
