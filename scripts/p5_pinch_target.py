"""Derive the pinch target pose (thumb + index joints) by kinematic optimization.

Objective: signed distance between the thumb and index distal collision capsules equals -2 mm (slight
overlap, so contact forms), plus a small penalty toward each joint's mid-range; all joints within their
model ranges; wrist and forearm at 0 (braced in the task). Deterministic multi-start (seeded).
Writes reports/P5/pinch_target.json; the pinch task config copies target_angles_rad from it.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import minimize

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from simulation.patches import build_biohand_model

JOINTS = ["cmc_flexion_r", "cmc_abduction_r", "mp_flexion_r", "ip_flexion_r",
          "mcp2_flexion_r", "mcp2_abduction_r", "pm2_flexion_r", "md2_flexion_r"]
TARGET_OVERLAP_M = -0.002
REG = 1e-4
SEED = 7


def main() -> None:
    m = build_biohand_model().model
    d = mujoco.MjData(m)
    adr = [m.jnt_qposadr[m.joint(j).id] for j in JOINTS]
    bounds = [tuple(m.jnt_range[m.joint(j).id]) for j in JOINTS]
    mid = np.array([0.5 * (lo + hi) for lo, hi in bounds])
    th = [m.geom(n).id for n in ("distal_thumb_coll_r", "distal_thumb_coll_2_r")]
    ix = [m.geom(n).id for n in ("distph2_coll_r", "distph2_coll_2_r")]
    ft = np.zeros(6)

    def dist(x: np.ndarray) -> float:
        d.qpos[:] = m.qpos0
        d.qpos[adr] = x
        mujoco.mj_kinematics(m, d)
        return min(mujoco.mj_geomDistance(m, d, a, b, 0.2, ft) for a in th for b in ix)

    def obj(x: np.ndarray) -> float:
        return 1e4 * (dist(x) - TARGET_OVERLAP_M) ** 2 + REG * float(np.sum((x - mid) ** 2))

    rng = np.random.default_rng(SEED)
    best = None
    for _ in range(30):
        x0 = np.array([rng.uniform(lo, hi) for lo, hi in bounds])
        r = minimize(obj, x0, bounds=bounds, method="Powell", options={"maxiter": 6000, "xtol": 1e-5, "ftol": 1e-10})
        if best is None or r.fun < best.fun:
            best = r
    target = {j: round(float(v), 5) for j, v in zip(JOINTS, best.x, strict=True)}
    out = {"joints": target, "capsule_distance_m": round(dist(best.x), 5), "objective": float(best.fun),
           "target_overlap_m": TARGET_OVERLAP_M, "regularisation": REG, "seed": SEED,
           "note": "kinematic target for the pinch control policy; not an anatomical posture claim"}
    (ROOT / "reports" / "P5").mkdir(parents=True, exist_ok=True)
    (ROOT / "reports" / "P5" / "pinch_target.json").write_text(json.dumps(out, indent=1) + "\n")
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
