"""P3 kinematics and joint-constraint audit of the reference model.

Outputs:
  reports/P3/p3_kinematics_output.json   sweeps, continuity, workspace, penetration, ROM, limits
  reports/P3/fingertip_trajectories.csv  composite flexion trajectories (units + metadata header)

Usage: .venv/Scripts/python.exe scripts/p3_kinematics_audit.py
"""

from __future__ import annotations

import csv
import hashlib
import io
import json
import sys
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from simulation.kinematics import (
    FINGERTIP_SITES,
    forward,
    qpos_from_angles,
    site_positions,
    sweep_joint,
    to_display_degrees,
)
from simulation.myosim_source import MYOSIM_PINNED_COMMIT, build_myohand
from simulation.patches import build_biohand_model

N_SWEEP = 181
DIGITS = (2, 3, 4, 5)

# Published reference ranges (degrees). Retrieved at abstract level via search summaries on
# 2026-10-31; see docs/REFERENCES.md. Functional = range needed for activities of daily living.
REFERENCE_ROM = {
    "Ryu1991_wrist_ADL_max": {
        "source": "Ryu J, Cooney WP, Askew LJ, An KN, Chao EY. Functional ranges of motion of the wrist "
                  "joint. J Hand Surg Am 1991;16(3):409-419 (abstract): 40 subjects, biaxial goniometer",
        "wrist_flexion": 54.0, "wrist_extension": 60.0, "ulnar_deviation": 40.0, "radial_deviation": 17.0},
    "Bain2015_finger_functional": {
        "source": "Bain GI, Polites N, Higgs BG, Heptinstall RJ, McGrath AM. The functional range of motion "
                  "of the finger joints. J Hand Surg Eur 2015;40(4):406-411 (abstract): 10 volunteers, "
                  "Sollerman test",
        "MCP": [19.0, 71.0], "PIP": [23.0, 87.0], "DIP": [10.0, 64.0]},
}


def bound_lever(model: mujoco.MjModel, data: mujoco.MjData, joint: int, point: np.ndarray) -> float:
    """Distance from ``point`` to the joint axis (world), for the continuity bound."""
    anchor, axis = data.xanchor[joint], data.xaxis[joint]
    v = point - anchor
    return float(np.linalg.norm(v - v.dot(axis) * axis))


def sweeps(model: mujoco.MjModel) -> list[dict[str, Any]]:
    out = []
    data = mujoco.MjData(model)
    for j in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        res = sweep_joint(model, name, N_SWEEP)
        dtheta = float(res.angles_rad[1] - res.angles_rad[0])
        row: dict[str, Any] = {"joint": name, "n": N_SWEEP, "dtheta_rad": dtheta, "tips": {}}
        for key in FINGERTIP_SITES:
            disp = float(np.linalg.norm(res.tips_m[key][-1] - res.tips_m[key][0]))
            if disp < 1e-9:
                continue  # tip not distal to this joint
            levers = []
            for a in (res.angles_rad[0], res.angles_rad[-1], 0.5 * (res.angles_rad[0] + res.angles_rad[-1])):
                forward(model, qpos_from_angles(model, {name: float(a)}), data)
                levers.append(bound_lever(model, data, j, site_positions(model, data, {key: FINGERTIP_SITES[key]})[key]))
            step = res.max_step_m(key)
            # chord of a circle of radius r for angle dtheta: 2 r sin(dtheta/2) <= r dtheta
            bound = max(levers) * dtheta
            row["tips"][key] = {"endpoint_displacement_m": round(disp, 6), "max_step_m": step,
                                "continuity_bound_m": bound, "continuous": bool(step <= bound * (1 + 1e-9))}
        out.append(row)
    return out


def composite_flexion_csv(model: mujoco.MjModel) -> tuple[str, dict[str, Any]]:
    """Linear 0 -> 1 interpolation of all finger flexion joints from 0 rad to their upper limit."""
    flex = [f"{p}{d}_flexion_r" for d in DIGITS for p in ("mcp", "pm", "md")]
    n = 101
    data = mujoco.MjData(model)
    buf = io.StringIO()
    buf.write(f"# BioHand Twin P3 composite finger flexion; model MyoSim {MYOSIM_PINNED_COMMIT}; fragment variant\n"
              "# s: fraction of range (0 = qpos0, 1 = upper limit of every MCP/PIP/DIP flexion joint); "
              "thumb and wrist at qpos0\n# positions in MuJoCo world frame, metres\n")
    w = csv.writer(buf, lineterminator="\n")
    w.writerow(["s"] + [f"{k}_{ax}_m" for k in FINGERTIP_SITES for ax in "xyz"])
    tips_start, tips_end = {}, {}
    for i, s in enumerate(np.linspace(0, 1, n)):
        angles = {jn: float(s * model.jnt_range[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)][1])
                  for jn in flex}
        forward(model, qpos_from_angles(model, angles), data)
        tips = site_positions(model, data, FINGERTIP_SITES)
        if i == 0:
            tips_start = tips
        tips_end = tips
        w.writerow([f"{s:.2f}"] + [f"{v:.6f}" for k in FINGERTIP_SITES for v in tips[k]])
    summary = {k: round(float(np.linalg.norm(tips_end[k] - tips_start[k])), 4) for k in FINGERTIP_SITES}
    return buf.getvalue(), {"fingertip_travel_m": summary}


def collision_geoms(model: mujoco.MjModel) -> list[int]:
    return [g for g in range(model.ngeom) if model.geom_group[g] == 4]


def min_distances(model: mujoco.MjModel, data: mujoco.MjData) -> list[dict[str, Any]]:
    """Signed distance between collision geoms of bodies that are not parent/child or same body."""
    geoms = collision_geoms(model)
    res = []
    fromto = np.zeros(6)
    for a_i, a in enumerate(geoms):
        for b in geoms[a_i + 1:]:
            ba, bb = model.geom_bodyid[a], model.geom_bodyid[b]
            if ba == bb or model.body_parentid[ba] == bb or model.body_parentid[bb] == ba:
                continue
            dist = mujoco.mj_geomDistance(model, data, a, b, 0.01, fromto)
            if dist < 0:
                res.append({"geom1": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, a),
                            "geom2": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, b),
                            "penetration_m": round(-float(dist), 5)})
    return sorted(res, key=lambda r: -r["penetration_m"])


def penetration_checks(model: mujoco.MjModel) -> dict[str, Any]:
    up = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j): float(model.jnt_range[j][1]) for j in range(model.njnt)}
    lo = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j): float(model.jnt_range[j][0]) for j in range(model.njnt)}
    flex = [f"{p}{d}_flexion_r" for d in DIGITS for p in ("mcp", "pm", "md")]
    poses = {
        "neutral_qpos0": {},
        "full_finger_flexion": {jn: up[jn] for jn in flex},
        "half_finger_flexion": {jn: 0.5 * up[jn] for jn in flex},
        "index_ulnar_middle_radial (converge II-III)": {"mcp2_abduction_r": lo["mcp2_abduction_r"],
                                                         "mcp3_abduction_r": up["mcp3_abduction_r"]},
        "ring_ulnar_little_radial (converge IV-V)":
            {"mcp4_abduction_r": lo["mcp4_abduction_r"], "mcp5_abduction_r": up["mcp5_abduction_r"]},
        "thumb_full_negative (flexion in MoBL convention)": {
            "cmc_flexion_r": lo["cmc_flexion_r"], "mp_flexion_r": lo["mp_flexion_r"], "ip_flexion_r": lo["ip_flexion_r"]},
    }
    data = mujoco.MjData(model)
    out = {}
    baseline: dict[tuple[str, str], float] = {}
    for label, angles in poses.items():
        forward(model, qpos_from_angles(model, angles), data)
        pen = min_distances(model, data)
        keyed = {(r["geom1"], r["geom2"]): r["penetration_m"] for r in pen}
        if label == "neutral_qpos0":
            baseline = keyed
        added = sorted(({"geom1": k[0], "geom2": k[1], "penetration_m": v,
                         "baseline_m": baseline.get(k, 0.0)} for k, v in keyed.items()
                        if v > baseline.get(k, 0.0) + 1e-4), key=lambda r: -r["penetration_m"])
        out[label] = {"n_penetrating_pairs": len(pen),
                      "n_new_or_deeper_vs_neutral": len(added),
                      "max_new_or_deeper_m": added[0]["penetration_m"] if added else 0.0,
                      "new_or_deeper": added[:8]}
    return {"geometry": "model collision proxies (geom group 4: capsules/ellipsoids); bone meshes not used",
            "excluded": "same-body and parent-child pairs",
            "note": "penetrations already present at qpos0 (e.g. welded humerus vs radius capsules, overlapping "
                    "metacarpal skin capsules) are a property of the proxy geometry; poses report only pairs "
                    "that are new or deeper than at qpos0",
            "neutral_pairs": [{"geom1": k[0], "geom2": k[1], "penetration_m": v} for k, v in baseline.items()],
            "poses": out}


def rom_comparison(model: mujoco.MjModel) -> list[dict[str, Any]]:
    def rng(name: str) -> tuple[float, float]:
        lo_, hi_ = model.jnt_range[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)]
        return float(to_display_degrees(lo_)), float(to_display_degrees(hi_))
    ryu, bain = REFERENCE_ROM["Ryu1991_wrist_ADL_max"], REFERENCE_ROM["Bain2015_finger_functional"]
    fl_lo, fl_hi = rng("flexion_r")      # + = flexion
    dv_lo, dv_hi = rng("deviation_r")    # + = radial in MyoHand (P2/P3 evidence)
    rows = [
        {"quantity": "wrist flexion", "model_deg": fl_hi, "reference_deg": ryu["wrist_flexion"], "reference": "Ryu1991"},
        {"quantity": "wrist extension", "model_deg": -fl_lo, "reference_deg": ryu["wrist_extension"], "reference": "Ryu1991"},
        {"quantity": "radial deviation", "model_deg": dv_hi, "reference_deg": ryu["radial_deviation"], "reference": "Ryu1991"},
        {"quantity": "ulnar deviation", "model_deg": -dv_lo, "reference_deg": ryu["ulnar_deviation"], "reference": "Ryu1991"},
    ]
    for r in rows:
        r["model_covers_reference"] = r["model_deg"] >= r["reference_deg"]
    for kind, prefix in (("MCP", "mcp"), ("PIP", "pm"), ("DIP", "md")):
        for d in DIGITS:
            lo_, hi_ = rng(f"{prefix}{d}_flexion_r")
            ref = bain[kind]
            rows.append({"quantity": f"{kind} flexion digit {d}", "model_deg": [lo_, hi_], "reference_deg": ref,
                         "reference": "Bain2015", "model_covers_reference": lo_ <= ref[0] and hi_ >= ref[1]})
    return rows


def limit_compliance(model: mujoco.MjModel) -> list[dict[str, Any]]:
    """Static penetration of each joint's limits under its maximal isometric muscle torque.

    tau_max(direction) = sum over muscles of F_max * |moment arm|, for muscles whose moment arm drives
    the joint toward that limit (moment arms evaluated at the limit). Actuation and gravity are disabled;
    the torque is applied as qfrc_applied; 3 s of simulation (default solver settings).
    """
    m = build_myohand("fragment").model
    m.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_ACTUATION) | int(mujoco.mjtDisableBit.mjDSBL_GRAVITY)
    data = mujoco.MjData(m)
    out = []
    fmax = m.actuator_gainprm[:, 2]
    for j in range(m.njnt):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)
        dof = m.jnt_dofadr[j]
        row: dict[str, Any] = {"joint": name, "solref": m.jnt_solref[j].tolist(), "solimp": m.jnt_solimp[j].tolist()}
        for side, limit in (("upper", m.jnt_range[j][1]), ("lower", m.jnt_range[j][0])):
            mujoco.mj_resetData(m, data)
            data.qpos[m.jnt_qposadr[j]] = limit
            mujoco.mj_forward(m, data)
            moment = data.actuator_moment  # sparse in MuJoCo 3.x
            dense = np.zeros((m.nu, m.nv))
            mujoco.mju_sparse2dense(dense, moment, data.moment_rownnz, data.moment_rowadr, data.moment_colind)
            arms = dense[:, dof]   # generalized force per unit actuator force (muscle force is negative)
            # muscle tension T>0 gives actuator force -T; generalized force = -T * arm
            toward = (-arms > 0) if side == "upper" else (-arms < 0)
            tau = float(np.sum(fmax[toward] * np.abs(arms[toward])))
            sign = 1.0 if side == "upper" else -1.0
            for _ in range(int(3.0 / m.opt.timestep)):
                data.qfrc_applied[dof] = sign * tau
                mujoco.mj_step(m, data)
            q = float(data.qpos[m.jnt_qposadr[j]])
            row[side] = {"tau_max_Nm": round(tau, 4),
                         "penetration_rad": round(max(0.0, sign * (q - limit)), 5),
                         "penetration_deg": round(float(to_display_degrees(max(0.0, sign * (q - limit)))), 2),
                         "residual_speed_rad_s": round(abs(float(data.qvel[dof])), 6)}
        out.append(row)
    return out


def neutral_reproducibility() -> dict[str, Any]:
    digests = []
    for _ in range(2):
        m = build_myohand("fragment").model
        d = forward(m, m.qpos0.copy())
        digests.append(hashlib.sha256(np.concatenate([d.xpos.ravel(), d.xquat.ravel(),
                                                      d.site_xpos.ravel()]).tobytes()).hexdigest())
    return {"builds": 2, "fk_digest_sha256": digests[0], "identical": digests[0] == digests[1]}


def main() -> None:
    model = build_myohand("fragment").model
    sw = sweeps(model)
    csv_text, comp = composite_flexion_csv(model)
    data = forward(model, model.qpos0.copy())
    result = {
        "model_commit": MYOSIM_PINNED_COMMIT,
        "neutral_reproducibility": neutral_reproducibility(),
        "fingertips_at_qpos0_m": {k: v.round(5).tolist() for k, v in site_positions(model, data, FINGERTIP_SITES).items()},
        "sweeps": sw,
        "all_sweeps_continuous": all(t["continuous"] for r in sw for t in r["tips"].values()),
        "composite_flexion": comp,
        "penetration": penetration_checks(model),
        "rom_comparison": rom_comparison(model),
        "rom_comparison_biohand_patched": rom_comparison(build_biohand_model().model),
        "reference_rom": REFERENCE_ROM,
        "limit_compliance": limit_compliance(model),
    }
    out = ROOT / "reports" / "P3"
    out.mkdir(parents=True, exist_ok=True)
    (out / "p3_kinematics_output.json").write_text(json.dumps(result, indent=1) + "\n")
    (out / "fingertip_trajectories.csv").write_text(csv_text, encoding="utf-8")
    print("neutral reproducible:", result["neutral_reproducibility"]["identical"])
    print("all sweeps continuous:", result["all_sweeps_continuous"])
    print("composite fingertip travel (m):", comp["fingertip_travel_m"])
    print("neutral penetrating pairs:", [(r["geom1"], r["geom2"], r["penetration_m"])
                                         for r in result["penetration"]["neutral_pairs"]])
    for k, v in result["penetration"]["poses"].items():
        print(f"penetration {k}: new/deeper={v['n_new_or_deeper_vs_neutral']} max={v['max_new_or_deeper_m']} m",
              [(t['geom1'], t['geom2'], t['penetration_m']) for t in v["new_or_deeper"][:4]])
    for label in ("rom_comparison", "rom_comparison_biohand_patched"):
        for r in result[label]:
            if not r["quantity"].startswith(("MCP", "PIP", "DIP")):
                print(label, r["quantity"], round(r["model_deg"], 1), "ref", r["reference_deg"],
                      "covers", r["model_covers_reference"])
    for r in result["limit_compliance"]:
        print(f"limit {r['joint']:17s} up tau={r['upper']['tau_max_Nm']:7.3f} pen={r['upper']['penetration_deg']:6.2f}deg "
              f"| low tau={r['lower']['tau_max_Nm']:7.3f} pen={r['lower']['penetration_deg']:6.2f}deg")


if __name__ == "__main__":
    main()
