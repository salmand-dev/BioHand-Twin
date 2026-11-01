"""Forward-dynamics optimization of a constant pinch activation pattern (cf. McFarland et al. 2023,
who optimized activations by forward simulation to find pinch/grip forces).

Variables: constant activations (0..1) of thumb and index muscles. For each candidate, simulate 2.0 s
(braced wrist/forearm, gravity on). Over the last 0.2 s evaluate, for the task's CONTACT-ENABLED pairs
only, the mean positive gap (distance if not touching) and the mean contact normal force.
Objective: gap_m - 1e-3 * force_N + 1e-3 * sum(a^2): first close the gap, then press, at low effort.
(v1 of this script used plain geometric distance including a segment without a contact pair; the
optimizer exploited interpenetration - kept as a documented lesson.) Deterministic Powell from a fixed
start (all 0.3) and from WARM_START; the lower objective is kept. Writes reports/fixes/pinch_policy_opt.json (the v5 result is kept as pinch_v5_policy_opt.json). The muscle list includes patch
thumb_intrinsics_v1; the 12-muscle result without that patch is reports/P5/pinch_policy_opt.json.
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

from simulation.tasks import build_task_model, contact_normal_forces, load_config

MUSCLES = ["FPL_r", "OP_r", "EPL_r", "EPB_r", "APL_r", "APB_r", "FPBs_r", "FPBd_r", "ADPo_r", "ADPt_r",
           "FDS2_r", "FDP2_r", "EDC2_r", "EIP_r", "RI2_r", "LU_RB2_r", "UI_UB2_r"]

SIM_S, EVAL_S = 2.0, 0.5   # the task window; a 1 s window found contact that was later lost
#: Solution of the same optimization over a 1 s window (contact at 1 s, lost by 1.5 s).
WARM_START = [0.0024, 0.153, 0.3552, 0.071, 0.1257, 0.597, 0.0325, 0.0147, 0.2921, 0.2203, 0.0344, 0.118, 0.113, 0.3628, 0.681, 0.3259, 0.143]


def main() -> None:
    cfg = load_config("pinch")
    model, _ = build_task_model(cfg)
    ids = [model.actuator(n).id for n in MUSCLES]
    pairs = [(model.geom(p["geom1"]).id, model.geom(p["geom2"]).id) for p in cfg["contact_pairs"]]
    thumb_geoms = {a for a, _ in pairs}
    ft = np.zeros(6)
    n_steps = int(SIM_S / model.opt.timestep)
    n_eval = int(EVAL_S / model.opt.timestep)
    evals = {"n": 0}

    def simulate(x: np.ndarray) -> tuple[float, float, float]:
        d = mujoco.MjData(model)
        u = np.zeros(model.nu)
        u[ids] = np.clip(x, 0.0, 1.0)
        gap_sum, force_sum = 0.0, 0.0
        for k in range(n_steps):
            d.ctrl[:] = u
            mujoco.mj_step(model, d)
            if k >= n_steps - n_eval:
                gap_sum += max(0.0, min(mujoco.mj_geomDistance(model, d, a, b, 0.2, ft) for a, b in pairs))
                force_sum += contact_normal_forces(model, d, thumb_geoms)[0]
        return gap_sum / n_eval, force_sum / n_eval, float(np.sum(u[ids] ** 2))

    def objective(x: np.ndarray) -> float:
        evals["n"] += 1
        gap, force, effort = simulate(x)
        return gap - 1e-3 * force + 1e-3 * effort

    results = {}
    for label, x0 in (("fixed_0p3", np.full(len(MUSCLES), 0.3)), ("warm_no_joint_spring", np.array(WARM_START))):
        results[label] = minimize(objective, x0, method="Powell", bounds=[(0.0, 1.0)] * len(MUSCLES),
                                  options={"maxfev": 600, "xtol": 1e-3, "ftol": 1e-6})
    best = min(results, key=lambda k: results[k].fun)
    res = results[best]
    gap, force, _effort = simulate(res.x)
    out = {"muscles": {n: round(float(v), 4) for n, v in zip(MUSCLES, res.x, strict=True)},
           "mean_gap_last_0p5s_m": round(gap, 5), "mean_contact_force_last_0p5s_N": round(force, 4),
           "objective": float(res.fun), "evaluations": evals["n"], "contact_reached": bool(force > 0.0),
           "objective_by_start": {k: float(v.fun) for k, v in results.items()}, "best_start": best,
           "method": "Powell from 2 starts, bounds [0,1], 2.0 s forward simulation per evaluation"}
    (ROOT / "reports" / "fixes" / "pinch_policy_opt.json").write_text(json.dumps(out, indent=1) + "\n")
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
