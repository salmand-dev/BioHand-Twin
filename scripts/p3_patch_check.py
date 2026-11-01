"""Check the wrist_rom_v1 patch: muscle operating ranges and numerical stability.

MuJoCo muscle normalisation (documented in the MuJoCo modelling guide, muscles):
    L0 = (lengthrange[1] - lengthrange[0]) / (range[1] - range[0])
    LT = lengthrange[0] - range[0] * L0
    normalised fibre length  l = (actuator_length - LT) / L0
Active force is zero outside [lmin, lmax]. Upstream lengthrange was calibrated for the narrower
upstream ranges, so the patch can push muscles outside their calibrated operating region.

Grid: wrist flexion x deviation (13 x 9 over the given ranges) x pro_sup {-90, 0, 90 deg}
x finger posture {extended (all 0), flexed (MCP/PIP/DIP at upper limit)}; thumb at qpos0.
Output: reports/P3/patch_check.json
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from simulation.kinematics import forward
from simulation.myosim_source import build_myohand
from simulation.patches import build_biohand_model

FINGER_FLEX = [f"{p}{d}_flexion_r" for d in (2, 3, 4, 5) for p in ("mcp", "pm", "md")]


def grid_lengths(model: mujoco.MjModel, wrist_ranges: dict[str, np.ndarray]) -> np.ndarray:
    jid = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j): j for j in range(model.njnt)}
    data = mujoco.MjData(model)
    rows = []
    for ps in np.radians([-90.0, 0.0, 90.0]):
        for fingers_flexed in (False, True):
            for fl in np.linspace(*wrist_ranges["flexion_r"], 13):
                for dv in np.linspace(*wrist_ranges["deviation_r"], 9):
                    q = model.qpos0.copy()
                    q[model.jnt_qposadr[jid["pro_sup_r"]]] = ps
                    q[model.jnt_qposadr[jid["flexion_r"]]] = fl
                    q[model.jnt_qposadr[jid["deviation_r"]]] = dv
                    if fingers_flexed:
                        for jn in FINGER_FLEX:
                            q[model.jnt_qposadr[jid[jn]]] = model.jnt_range[jid[jn]][1]
                    forward(model, q, data)
                    mujoco.mj_transmission(model, data)
                    rows.append(data.actuator_length.copy())
    return np.array(rows)


def normalised(model: mujoco.MjModel, lengths: np.ndarray) -> np.ndarray:
    gp, lr = model.actuator_gainprm, model.actuator_lengthrange
    l0 = (lr[:, 1] - lr[:, 0]) / (gp[:, 1] - gp[:, 0])
    lt = lr[:, 0] - gp[:, 0] * l0
    return (lengths - lt) / l0


def operating_summary(model: mujoco.MjModel, wrist_ranges: dict[str, np.ndarray]) -> dict[str, Any]:
    lengths = grid_lengths(model, wrist_ranges)
    lnorm = normalised(model, lengths)
    lmin, lmax = model.actuator_gainprm[:, 4], model.actuator_gainprm[:, 5]
    lr = model.actuator_lengthrange
    per = {}
    for a in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, a)
        outside_active = np.mean((lnorm[:, a] < lmin[a]) | (lnorm[:, a] > lmax[a]))
        outside_lr = np.mean((lengths[:, a] < lr[a, 0] - 1e-9) | (lengths[:, a] > lr[a, 1] + 1e-9))
        per[name] = {"lnorm_min": round(float(lnorm[:, a].min()), 3), "lnorm_max": round(float(lnorm[:, a].max()), 3),
                     "fraction_outside_active_window": round(float(outside_active), 3),
                     "fraction_outside_lengthrange": round(float(outside_lr), 3)}
    return {"n_samples": int(lengths.shape[0]), "muscles": per}


def smoke(model: mujoco.MjModel) -> dict[str, Any]:
    out = {}
    for policy in ("zero", "full", "random_seeded"):
        d = mujoco.MjData(model)
        rng = np.random.default_rng(20261031)
        viol, finite = 0.0, True
        for _ in range(int(2.0 / model.opt.timestep)):
            d.ctrl[:] = {"zero": 0.0, "full": 1.0}.get(policy, None) if policy != "random_seeded" \
                else rng.uniform(0, 1, model.nu)
            mujoco.mj_step(model, d)
            q = d.qpos[model.jnt_qposadr]
            viol = max(viol, float(np.max(np.maximum(model.jnt_range[:, 0] - q, q - model.jnt_range[:, 1]))))
            if not (np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all()):
                finite = False
                break
        warn = {mujoco.mjtWarning(i).name: int(d.warning[i].number) for i in range(len(d.warning)) if d.warning[i].number}
        out[policy] = {"finite": finite, "max_limit_violation_rad": round(viol, 5), "warnings": warn}
    return out


def main() -> None:
    upstream = build_myohand("fragment").model
    patched = build_biohand_model(patches=("wrist_rom_v1",)).model  # wrist patch only

    def ranges(m: mujoco.MjModel) -> dict[str, np.ndarray]:
        return {jn: m.jnt_range[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)].copy()
                for jn in ("flexion_r", "deviation_r")}

    before = operating_summary(patched, ranges(upstream))   # same muscles, old wrist ROM
    after = operating_summary(patched, ranges(patched))     # new wrist ROM
    flagged = sorted(n for n, v in after["muscles"].items()
                     if v["fraction_outside_active_window"] > before["muscles"][n]["fraction_outside_active_window"] + 1e-9
                     or v["fraction_outside_lengthrange"] > before["muscles"][n]["fraction_outside_lengthrange"] + 1e-9)
    result = {"patch": "wrist_rom_v1",
              "wrist_ranges_deg": {"upstream": {k: np.degrees(v).round(2).tolist() for k, v in ranges(upstream).items()},
                                   "patched": {k: np.degrees(v).round(2).tolist() for k, v in ranges(patched).items()}},
              "operating_range_upstream_rom": before, "operating_range_patched_rom": after,
              "muscles_worse_with_patch": flagged,
              "smoke_patched": smoke(patched), "smoke_upstream": smoke(upstream)}
    out = ROOT / "reports" / "P3"
    (out / "patch_check.json").write_text(json.dumps(result, indent=1) + "\n")
    print("muscles whose operating range worsens with the patch:", len(flagged))
    for n in flagged:
        b, a = before["muscles"][n], after["muscles"][n]
        print(f"  {n:9s} outside-active {b['fraction_outside_active_window']:.3f} -> {a['fraction_outside_active_window']:.3f} "
              f"| outside-lengthrange {b['fraction_outside_lengthrange']:.3f} -> {a['fraction_outside_lengthrange']:.3f} "
              f"| lnorm {a['lnorm_min']}..{a['lnorm_max']}")
    print("smoke patched:", result["smoke_patched"])
    print("smoke upstream:", result["smoke_upstream"])


if __name__ == "__main__":
    main()
