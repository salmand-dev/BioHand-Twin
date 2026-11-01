"""Derive patch thumb_intrinsics_v1: add APB, FPB (2 heads) and ADP (2 heads) from measured cadaver data.

Source: data/sources/mirakhorlo2016_thumb.yaml (one specimen, CC BY 4.0). No coordinate is chosen by hand.

Mapping specimen -> model (each bone separately, so it does not depend on the specimen's or the model's pose):
  * palm segment ("MC3" in the source, thirdmc_r here): least-squares similarity (Umeyama, no reflection) over
    6 joint centres: wrist, thumb CMC, MCP 2..5. The fit residual is reported as mapping uncertainty.
  * thumb proximal phalanx (PP1, proximal_thumb_r): MCP centre -> IP centre defines the bone axis and the scale,
    the MCP flexion axis fixes the roll.
Path: origin -> insertion, straight, as the source lists 0 via points. Several origin points -> their centroid.
Force: PCSA x specific tension (validated by simulation.patches on every build).
Lengthrange: same geometric rule as muscle_fixes_v1. Curve parameters: copied from OP_r (MuJoCo muscle model).

Writes data/patches/thumb_intrinsics_v1.yaml and reports/fixes/thumb_intrinsics_derivation.json.
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
sys.path.insert(0, str(ROOT / "scripts"))

from p4_derive_muscle_fixes import SEED, extremes, spanned_joints

from simulation.muscles import moment_arm_matrix
from simulation.myosim_source import build_myohand
from simulation.patches import DEFAULT_PATCHES, LENGTHRANGE_METHOD, Patch

SOURCE = "data/sources/mirakhorlo2016_thumb.yaml"
BASE_PATCHES = ("wrist_rom_v1", "muscle_fixes_v1", "limit_stiffness_v1")
#: Expected actions fixed before running, from Smutz et al. 1998 as summarised in the literature retrieved
#: (APB mainly abductor; ADP flexor and adductor; FPB flexor). Model sign: CMC/MCP flexion is negative.
EXPECTED_SIGNS = {
    "APB_r": {"cmc_abduction_r": 1},
    "FPBs_r": {"mp_flexion_r": -1},
    "FPBd_r": {"mp_flexion_r": -1},
    "ADPo_r": {"cmc_abduction_r": -1, "cmc_flexion_r": -1, "mp_flexion_r": -1},
    "ADPt_r": {"cmc_abduction_r": -1, "cmc_flexion_r": -1, "mp_flexion_r": -1},
}


def similarity(src: np.ndarray, dst: np.ndarray) -> tuple[float, np.ndarray, np.ndarray]:
    """Umeyama least-squares similarity: dst ~ s R src + t, with det(R) = +1."""
    mu_s, mu_d = src.mean(axis=0), dst.mean(axis=0)
    a, b = src - mu_s, dst - mu_d
    u, sv, vt = np.linalg.svd(b.T @ a / len(src))
    sign = np.diag([1.0, 1.0, np.sign(np.linalg.det(u @ vt))])
    rot = u @ sign @ vt
    scale = float(np.trace(np.diag(sv) @ sign) / (a**2).sum(axis=1).mean())
    return scale, rot, mu_d - scale * rot @ mu_s


def bone_frame(axis_along: np.ndarray, roll_axis: np.ndarray) -> np.ndarray:
    """Right-handed frame: e1 along the bone, e3 = roll axis made orthogonal to e1."""
    e1 = axis_along / np.linalg.norm(axis_along)
    e3 = roll_axis - roll_axis.dot(e1) * e1
    e3 /= np.linalg.norm(e3)
    return np.column_stack([e1, np.cross(e3, e1), e3])


def mapped_sites(src: dict[str, Any], m: mujoco.MjModel, d: mujoco.MjData) -> tuple[dict[str, Any], dict[str, Any]]:
    def anchor(joint: str) -> np.ndarray:
        return d.xanchor[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, joint)].copy()

    def body_pos(body: str) -> np.ndarray:
        return d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body)].copy()

    piv = src["joint_pivots_mm"]
    palm_src = np.array([piv[k] for k in ("wrist", "cmc1", "mcp2", "mcp3", "mcp4", "mcp5")], dtype=float) * 1e-3
    palm_dst = np.array([anchor("flexion_r"), anchor("cmc_flexion_r"), body_pos("2proxph_r"), body_pos("3proxph_r"),
                         body_pos("4proxph_r"), body_pos("5proxph_r")])
    s_palm, r_palm, t_palm = similarity(palm_src, palm_dst)
    res = np.linalg.norm(s_palm * palm_src @ r_palm.T + t_palm - palm_dst, axis=1)

    lm = {k: np.array(v, dtype=float) * 1e-3 for k, v in src["landmarks_mm"].items()}
    mcp_s, ip_s = 0.5 * (lm["MCP1u"] + lm["MCP1r"]), 0.5 * (lm["IP1u"] + lm["IP1r"])
    mcp_m, ip_m = anchor("mp_flexion_r"), anchor("ip_flexion_r")
    s_pp = float(np.linalg.norm(ip_m - mcp_m) / np.linalg.norm(ip_s - mcp_s))
    axis_m = d.xaxis[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "mp_flexion_r")]
    f_s = bone_frame(ip_s - mcp_s, np.array(src["mcp1_flexion_axis"], dtype=float))
    f_m = bone_frame(ip_m - mcp_m, axis_m)
    r_pp = f_m @ f_s.T

    def local(body: str, world: np.ndarray) -> list[float]:
        b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body)
        return [round(float(x), 6) for x in d.xmat[b].reshape(3, 3).T @ (world - d.xpos[b])]

    paths = {}
    for name, mus in src["muscles"].items():
        origin_src = np.mean(np.array(mus["origin_points_mm"], dtype=float), axis=0) * 1e-3
        origin = s_palm * r_palm @ origin_src + t_palm
        ins = mcp_m + s_pp * r_pp @ (np.array(mus["insertion_mm"], dtype=float) * 1e-3 - mcp_s)
        paths[name] = [{"body": "thirdmc_r", "pos_m": local("thirdmc_r", origin)},
                       {"body": "proximal_thumb_r", "pos_m": local("proximal_thumb_r", ins)}]
    info = {"palm_scale": round(s_palm, 4), "palm_residuals_mm": [round(float(x) * 1e3, 2) for x in res],
            "palm_rms_mm": round(float(np.sqrt((res**2).mean())) * 1e3, 2), "pp1_scale": round(s_pp, 4),
            "pp1_axis_misalignment_deg": round(float(np.degrees(np.arccos(np.clip(
                (r_pp @ (np.array(src["mcp1_flexion_axis"]) / np.linalg.norm(src["mcp1_flexion_axis"]))).dot(axis_m),
                -1, 1)))), 2)}
    return paths, info


def main() -> None:
    src = yaml.safe_load((ROOT / SOURCE).read_text(encoding="utf-8"))
    up = build_myohand("fragment")
    upstream_ranges = up.model.jnt_range.copy()
    for pid in BASE_PATCHES:
        Patch.load(pid).apply(up.spec)
    base = up.spec.compile()
    d = mujoco.MjData(base)
    mujoco.mj_forward(base, d)
    paths, info = mapped_sites(src, base, d)

    changes: list[dict[str, Any]] = []
    for name, mus in src["muscles"].items():
        changes.append({"field": "add_muscle", "actuator": name, "name": mus["name"], "source_file": SOURCE,
                        "source_muscle": name, "template_actuator": "OP_r",
                        "force_N": round(mus["pcsa_mm2"] * src["specific_tension"]["value_N_per_mm2"], 6),
                        "path": paths[name], "lengthrange_m": [0.01, 0.2], "derivation": LENGTHRANGE_METHOD})
    trial = {"patch_id": "t", "changes": changes}
    spec = build_myohand("fragment").spec
    for pid in BASE_PATCHES:
        Patch.load(pid).apply(spec)
    Patch("t", trial).apply(spec)
    m = spec.compile()
    rng = np.random.default_rng(SEED)
    span = spanned_joints(m, upstream_ranges, rng)
    ids = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in src["muscles"]}
    for ch in changes:
        a = ids[ch["actuator"]]
        lo, hi = extremes(m, a, np.flatnonzero(span[a]), upstream_ranges, rng)
        ch["lengthrange_m"] = [round(float(lo), 6), round(float(hi), 6)]

    patch = {
        "patch_id": "thumb_intrinsics_v1", "version": 1, "status": "provisional",
        "classification": "literature-derived, mapped by rule; one specimen; not validated as a physiological model",
        "approved_by": "project owner ('fix this', 2026-11-01)",
        "rationale": "Gap M1: APB, FPB and ADP are absent from MyoHand, MoBL-ARM"
                     "S and 2nd-Hand, so their geometry and PCSA come from Mirakho"
                     "rlo et al. 2016. They raise pinch force capacity; joint spri"
                     "ngs, not these muscles, decided whether contact was sustained (reports/fixes/FIXES_REPORT.md).",
        "evidence": ["reports/fixes/thumb_intrinsics_derivation.json", SOURCE],
        "not_changed": "All upstream actuators, including OP_r (180 N, a source-model value).",
        "changes": changes,
    }
    header = "# GENERATED by scripts/fix_derive_thumb_intrinsics.py - do not edit numbers by hand.\n"
    text = header + yaml.safe_dump(patch, sort_keys=False, allow_unicode=True, width=110)
    (ROOT / "data" / "patches" / "thumb_intrinsics_v1.yaml").write_text(text, encoding="utf-8")

    # Moment arms r = -dL/dq, sampled over each joint's range with the other joints at qpos0.
    final = build_myohand("fragment").spec
    for pid in BASE_PATCHES:
        Patch.load(pid).apply(final)
    Patch("thumb_intrinsics_v1", patch).apply(final)
    fm = final.compile()
    fd = mujoco.MjData(fm)
    table: dict[str, Any] = {}
    all_ok = True
    for name in src["muscles"]:
        a = mujoco.mj_name2id(fm, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        table[name] = {}
        for joint in ("cmc_flexion_r", "cmc_abduction_r", "mp_flexion_r", "ip_flexion_r"):
            j = mujoco.mj_name2id(fm, mujoco.mjtObj.mjOBJ_JOINT, joint)
            vals = []
            for q in np.linspace(fm.jnt_range[j, 0], fm.jnt_range[j, 1], 21):
                fd.qpos[:] = fm.qpos0
                fd.qpos[fm.jnt_qposadr[j]] = q
                mujoco.mj_forward(fm, fd)
                vals.append(float(moment_arm_matrix(fm, fd)[a, j]))
            fd.qpos[:] = fm.qpos0
            mujoco.mj_forward(fm, fd)
            at0 = float(moment_arm_matrix(fm, fd)[a, j])
            exp = EXPECTED_SIGNS[name].get(joint)
            ok = None if exp is None else bool(np.sign(at0) == exp)
            all_ok = all_ok and ok is not False
            table[name][joint] = {"at_qpos0_mm": round(at0 * 1e3, 2), "min_mm": round(min(vals) * 1e3, 2),
                                  "max_mm": round(max(vals) * 1e3, 2), "expected_sign": exp, "sign_ok": ok}
    out = {"mapping": info, "force_N": {c["actuator"]: c["force_N"] for c in changes},
           "lengthrange_m": {c["actuator"]: c["lengthrange_m"] for c in changes},
           "moment_arms": table, "all_expected_signs_ok": all_ok, "default_patches_at_derivation": list(DEFAULT_PATCHES)}
    report = ROOT / "reports" / "fixes"
    report.mkdir(parents=True, exist_ok=True)
    (report / "thumb_intrinsics_derivation.json").write_text(json.dumps(out, indent=1) + "\n", encoding="utf-8")
    print(json.dumps({k: out[k] for k in ("mapping", "force_N", "lengthrange_m", "all_expected_signs_ok")}, indent=1))
    for name, row in table.items():
        print(name, {j: (v["at_qpos0_mm"], v["sign_ok"]) for j, v in row.items()})


if __name__ == "__main__":
    main()
