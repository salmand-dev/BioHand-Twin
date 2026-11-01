"""Evidence for patch limit_stiffness_v1: limit penetration and stability, before vs after.

For every joint and both limits: hold the joint at the limit and apply its maximal agonist muscle torque
(sum of F_max * |moment arm| over muscles pushing toward that limit, moment arms at the limit) as
qfrc_applied for 3 s, with actuation and gravity disabled. Also: 4 s of seeded random muscle control
(energy, NaN, warnings). Output: reports/P5/limit_stiffness_check.json
"""

from __future__ import annotations

import copy
import json
import sys
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from simulation.muscles import moment_arm_matrix
from simulation.patches import build_biohand_model

BEFORE = ("wrist_rom_v1", "muscle_fixes_v1")
AFTER = ("wrist_rom_v1", "muscle_fixes_v1", "limit_stiffness_v1")


def static_penetration(model: mujoco.MjModel) -> list[dict[str, Any]]:
    m = copy.copy(model)
    m.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_ACTUATION) | int(mujoco.mjtDisableBit.mjDSBL_GRAVITY)
    rows = []
    for j in range(m.njnt):
        dof = m.jnt_dofadr[j]
        row: dict[str, Any] = {"joint": mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)}
        for side, idx, sgn in (("upper", 1, 1.0), ("lower", 0, -1.0)):
            d = mujoco.MjData(m)
            lim = m.jnt_range[j][idx]
            d.qpos[m.jnt_qposadr[j]] = lim
            mujoco.mj_forward(model, d)
            r = moment_arm_matrix(model, d)[:, dof]
            push = r * sgn > 0
            tau = float(np.sum(model.actuator_gainprm[push, 2] * np.abs(r[push])))
            for _ in range(int(3.0 / m.opt.timestep)):
                d.qfrc_applied[dof] = sgn * tau
                mujoco.mj_step(m, d)
            pen = max(0.0, sgn * float(d.qpos[m.jnt_qposadr[j]] - lim))
            row[side] = {"tau_Nm": round(tau, 3), "penetration_deg": round(float(np.degrees(pen)), 2)}
        rows.append(row)
    return rows


def random_run(model: mujoco.MjModel) -> dict[str, Any]:
    m = copy.copy(model)
    m.opt.enableflags |= int(mujoco.mjtEnableBit.mjENBL_ENERGY)
    d = mujoco.MjData(m)
    rng = np.random.default_rng(20261031)
    viol, ke, finite = 0.0, 0.0, True
    for _ in range(int(4.0 / m.opt.timestep)):
        d.ctrl[:] = rng.uniform(0, 1, m.nu)
        mujoco.mj_step(m, d)
        q = d.qpos[m.jnt_qposadr]
        viol = max(viol, float(np.max(np.maximum(m.jnt_range[:, 0] - q, q - m.jnt_range[:, 1]))))
        ke = max(ke, float(d.energy[1]))
        finite &= bool(np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all())
    return {"finite": finite, "max_limit_violation_deg": round(float(np.degrees(viol)), 2),
            "max_kinetic_energy_J": round(ke, 5),
            "warnings": {mujoco.mjtWarning(i).name: int(d.warning[i].number)
                         for i in range(len(d.warning)) if d.warning[i].number}}


def main() -> None:
    out: dict[str, Any] = {}
    for label, patches in (("before", BEFORE), ("after", AFTER)):
        m = build_biohand_model(patches=patches).model
        pen = static_penetration(m)
        worst = max(max(r["upper"]["penetration_deg"], r["lower"]["penetration_deg"]) for r in pen)
        out[label] = {"patches": list(patches), "solref": np.unique(m.jnt_solref, axis=0).tolist(),
                      "worst_static_penetration_deg": worst, "static": pen, "random_control_4s": random_run(m)}
        print(label, "worst static penetration (deg):", worst, "| random:", out[label]["random_control_4s"])
    path = ROOT / "reports" / "P5"
    path.mkdir(parents=True, exist_ok=True)
    (path / "limit_stiffness_check.json").write_text(json.dumps(out, indent=1) + "\n")


if __name__ == "__main__":
    main()
