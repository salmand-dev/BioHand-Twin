"""Evidence for the pinch task after patch thumb_intrinsics_v1 (policy: pose tracking + static optimization).

1. 2x2 factorial: the same task and policy with and without the new muscles and the joint springs.
2. Exploit guards over the 2 s task: penetration, limit overshoot, final posture, which pairs carry the force.
3. Sensitivity: new-muscle force +/- the specific-tension SD; new origins moved by the palm mapping residual;
   all F_max x0.8, x1.2; timestep 1 ms.
4. Force capacity by linear programming over sampled postures with pad contact: maximum normal force such that thumb
   and index joint torques balance the contact force with activations in [0, 1], using length-dependent muscle force,
   with and without the new muscles. Wrist and forearm are braced, so only thumb and index joints are balanced.
   Compared with measured tip pinch strength (Park et al. 1998): a plausibility check, not validation.

Writes reports/fixes/pinch_evidence.json.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import yaml
from scipy.optimize import linprog

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from simulation.muscles import moment_arm_matrix
from simulation.patches import DEFAULT_PATCHES
from simulation.tasks import (
    PoseTracker,
    build_task_model,
    contact_normal_forces,
    load_config,
    run_task,
)

NEW = ("APB_r", "FPBs_r", "FPBd_r", "ADPo_r", "ADPt_r")
BALANCED = ("cmc_flexion_r", "cmc_abduction_r", "mp_flexion_r", "ip_flexion_r",
            "mcp2_flexion_r", "mcp2_abduction_r", "pm2_flexion_r", "md2_flexion_r")
MEASURED = {"men_mean_N": round(6.2 * 9.80665, 1), "women_mean_N": round(3.9 * 9.80665, 1),
            "source": "Park et al. 1998, doi:10.35371/kjoem.1998.10.3.362, n = 1018, tip pinch, pinch gauge"}
CAPACITY_SEED, CAPACITY_SAMPLES, CAPACITY_POSTURES, CONTACT_TOL_M = 20261101, 400000, 500, 0.0015


def set_new_force(m: mujoco.MjModel, factor: float) -> None:
    for name in NEW:
        a = m.actuator(name).id
        m.actuator_gainprm[a, 2] *= factor
        m.actuator_biasprm[a, 2] *= factor


def scale_all(m: mujoco.MjModel, factor: float) -> None:
    m.actuator_gainprm[:, 2] *= factor
    m.actuator_biasprm[:, 2] *= factor


def shift_origins(m: mujoco.MjModel, offset: np.ndarray) -> None:
    for name in NEW:
        m.site_pos[m.site(f"{name.removesuffix('_r')}-P1_r").id] += offset


def summary(cfg: dict[str, Any], **kw: Any) -> dict[str, Any]:
    r = run_task(cfg, **kw)
    keys = ("pinch_force_mean_last_0p5s", "pinch_force_min_last_0p5s", "thumb_index_final_distance",
            "max_limit_violation_rad", "final_max_abs_qvel_rad_s")
    return {"passed": bool(r.success["all_passed"]), **{k: r.metrics.get(k) for k in keys}}


def simulate(cfg: dict[str, Any]) -> tuple[mujoco.MjModel, mujoco.MjData, dict[str, Any]]:
    m, _ = build_task_model(cfg)
    d = mujoco.MjData(m)
    tracker = PoseTracker(m, cfg["policy"], cfg)
    pairs = {p["name"]: (m.geom(p["geom1"]).id, m.geom(p["geom2"]).id) for p in cfg["contact_pairs"]}
    thumb = {a for a, _ in pairs.values()}
    deepest, overshoot = 0.0, 0.0
    for _ in range(int(cfg["duration_s"] / m.opt.timestep)):
        d.ctrl[:] = tracker.control(d)
        mujoco.mj_step(m, d)
        for c in d.contact[:d.ncon]:
            if c.geom1 in thumb or c.geom2 in thumb:
                deepest = min(deepest, float(c.dist))
        for j in range(m.njnt):
            q = d.qpos[m.jnt_qposadr[j]]
            overshoot = max(overshoot, float(m.jnt_range[j, 0] - q), float(q - m.jnt_range[j, 1]))
    posture = {j: round(float(np.degrees(d.qpos[m.jnt_qposadr[m.joint(j).id]])), 1) for j in BALANCED}
    limits = {j: np.degrees(m.jnt_range[m.joint(j).id]).round(1).tolist() for j in BALANCED}
    near = [j for j in BALANCED if min(posture[j] - limits[j][0], limits[j][1] - posture[j]) < 3.0]
    active = sorted({mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, int(g)) for c in d.contact[:d.ncon]
                     for g in (c.geom1, c.geom2) if c.geom1 in thumb or c.geom2 in thumb})
    return m, d, {"deepest_penetration_mm": round(-deepest * 1e3, 3),
                  "max_limit_overshoot_deg": round(float(np.degrees(overshoot)), 2),
                  "final_posture_deg": posture, "joints_within_3deg_of_a_limit": near,
                  "geoms_in_contact_at_end": active,
                  "contact_force_final_N": round(float(contact_normal_forces(m, d, thumb)[0]), 3)}


def long_run(cfg: dict[str, Any], seconds: float = 10.0) -> dict[str, Any]:
    """Does the contact persist beyond the 2 s task, and does the hand reach the target pose?"""
    m, _ = build_task_model(cfg)
    d = mujoco.MjData(m)
    tracker = PoseTracker(m, cfg["policy"], cfg)
    thumb = {m.geom(p["geom1"]).id for p in cfg["contact_pairs"]}
    forces = []
    start = int(0.3 / m.opt.timestep)   # after the first impact
    for k in range(int(seconds / m.opt.timestep)):
        d.ctrl[:] = tracker.control(d)
        mujoco.mj_step(m, d)
        if k >= start:
            forces.append(float(contact_normal_forces(m, d, thumb)[0]))
    f = np.array(forces)
    err = {j: round(float(np.degrees(d.qpos[m.jnt_qposadr[m.joint(j).id]] - v)), 2)
           for j, v in cfg["policy"]["target_angles_rad"].items()}
    return {"seconds": seconds, "force_min_N": round(float(f.min()), 3), "force_mean_N": round(float(f.mean()), 3),
            "force_max_N": round(float(f.max()), 3), "share_of_time_in_contact": round(float((f > 0).mean()), 4),
            "final_pose_error_deg": err, "final_max_abs_qvel_rad_s": round(float(np.abs(d.qvel).max()), 3)}


def lp_capacity(m: mujoco.MjModel, dd: mujoco.MjData, jac_axes: tuple[np.ndarray, np.ndarray, np.ndarray], mu: float,
                dofs: list[int], af: np.ndarray, b: np.ndarray, rm: np.ndarray, excluded: list[int]) -> float:
    """Largest normal force that muscles can balance with a friction pyramid; 0 if none is feasible."""
    a_m = np.column_stack([rm[:, dof] * af for dof in dofs]).T
    a_eq = np.column_stack([a_m, *jac_axes])
    a_ub = np.zeros((4, m.nu + 3))
    for k, (axis, sign) in enumerate(((1, 1), (1, -1), (2, 1), (2, -1))):
        a_ub[k, m.nu] = -mu
        a_ub[k, m.nu + axis] = sign
    cost = np.zeros(m.nu + 3)
    cost[m.nu] = -1.0
    bounds = [(0.0, 0.0 if i in excluded else 1.0) for i in range(m.nu)] + [(0, None), (None, None), (None, None)]
    lp = linprog(cost, A_eq=a_eq, b_eq=b, A_ub=a_ub, b_ub=np.zeros(4), bounds=bounds, method="highs")
    return float(lp.x[m.nu]) if lp.success else 0.0


def capacity(cfg: dict[str, Any]) -> dict[str, Any]:
    """Quasi-static maximum thumb-index pad force over sampled postures, with and without the new muscles."""
    m, _ = build_task_model(cfg)
    dd = mujoco.MjData(m)
    geoms = [(m.geom(p["geom1"]).id, m.geom(p["geom2"]).id) for p in cfg["contact_pairs"]]
    adr = [m.jnt_qposadr[m.joint(j).id] for j in BALANCED]
    lo, hi = np.array([m.jnt_range[m.joint(j).id] for j in BALANCED]).T
    dofs = [int(m.jnt_dofadr[m.joint(j).id]) for j in BALANCED]
    excluded = [m.actuator(n).id for n in NEW]
    rng = np.random.default_rng(CAPACITY_SEED)
    fromto = np.zeros(6)
    results: list[dict[str, Any]] = []
    tried = 0
    while tried < CAPACITY_SAMPLES and len(results) < CAPACITY_POSTURES:
        tried += 1
        q = rng.uniform(lo, hi)
        dd.qpos[:] = m.qpos0
        dd.qpos[adr] = q
        mujoco.mj_kinematics(m, dd)
        best = min(((mujoco.mj_geomDistance(m, dd, a, b, 0.05, fromto), a, b, fromto.copy()) for a, b in geoms),
                   key=lambda t: t[0])
        dist, g1, g2, ft = best
        if not 0.0 < dist < CONTACT_TOL_M:
            continue
        dd.qvel[:] = 0.0
        dd.act[:] = 0.0
        mujoco.mj_forward(m, dd)
        passive = dd.actuator_force.copy()
        rm = moment_arm_matrix(m, dd)
        gravity = -dd.qfrc_bias[dofs].copy()
        dd.act[:] = 1.0
        mujoco.mj_forward(m, dd)
        af = np.abs(dd.actuator_force - passive)
        normal = (ft[3:] - ft[:3]) / dist
        t1 = np.cross(normal, [1.0, 0.0, 0.0]) if abs(normal[0]) < 0.9 else np.cross(normal, [0.0, 1.0, 0.0])
        t1 /= np.linalg.norm(t1)
        t2 = np.cross(normal, t1)
        pos = 0.5 * (ft[:3] + ft[3:])
        jac1, jac2 = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        mujoco.mj_jac(m, dd, jac1, None, pos, m.geom_bodyid[g1])
        mujoco.mj_jac(m, dd, jac2, None, pos, m.geom_bodyid[g2])
        rel = (jac2 - jac1)[:, dofs].T
        axes = (rel @ normal, rel @ t1, rel @ t2)
        spring = np.array([-m.jnt_stiffness[m.joint(j).id] * (dd.qpos[m.jnt_qposadr[m.joint(j).id]]
                           - m.qpos_spring[m.jnt_qposadr[m.joint(j).id]]) for j in BALANCED])
        b = -(gravity + rm[:, dofs].T @ -passive + spring)
        row = {"posture_deg": [round(float(v), 1) for v in np.degrees(q)]}
        for mu in (0.0, 1.0):
            row[f"with_new_mu{mu:.0f}"] = lp_capacity(m, dd, axes, mu, dofs, af, b, rm, [])
            row[f"without_new_mu{mu:.0f}"] = lp_capacity(m, dd, axes, mu, dofs, af, b, rm, excluded)
        results.append(row)
    stats = {}
    for key in ("with_new_mu0", "without_new_mu0", "with_new_mu1", "without_new_mu1"):
        vals = np.array([r[key] for r in results])
        stats[key] = {"median_N": round(float(np.median(vals)), 2), "p95_N": round(float(np.percentile(vals, 95)), 2),
                      "max_N": round(float(vals.max()), 2), "feasible_share": round(float((vals > 0).mean()), 3)}
    top = max(results, key=lambda r: r["with_new_mu1"])
    return {"samples_tried": tried, "postures_with_pad_contact": len(results), "seed": CAPACITY_SEED,
            "tolerance_mm": CONTACT_TOL_M * 1e3, "stats": stats, "best_posture_with_new_mu1": top,
            "best_share_of_measured_women": round(stats["with_new_mu1"]["max_N"] / MEASURED["women_mean_N"], 3),
            "best_share_of_measured_men": round(stats["with_new_mu1"]["max_N"] / MEASURED["men_mean_N"], 3),
            "assumptions": "one contact point; mu = 0 and mu = 1 friction pyramid; quasi-static; balances thumb and "
                           "index joints only; sampled postures, so the maximum is a lower bound on the model capacity"}


def main() -> None:
    cfg = load_config("pinch")
    src = yaml.safe_load((ROOT / "data/sources/mirakhorlo2016_thumb.yaml").read_text(encoding="utf-8"))
    tension = src["specific_tension"]
    derivation = json.loads((ROOT / "reports/fixes/thumb_intrinsics_derivation.json").read_text())
    rms_m = derivation["mapping"]["palm_rms_mm"] * 1e-3
    rel = tension["sd_N_per_mm2"] / tension["value_N_per_mm2"]

    base = [p for p in DEFAULT_PATCHES if p not in ("thumb_intrinsics_v1", "passive_stiffness_v1")]
    factorial = {}
    for label, extra in (("neither", []), ("new muscles only", ["thumb_intrinsics_v1"]),
                         ("stiffness only", ["passive_stiffness_v1"]),
                         ("both", ["thumb_intrinsics_v1", "passive_stiffness_v1"])):
        factorial[label] = summary({**cfg, "patches": base + extra})

    cases: dict[str, Any] = {"baseline": {}, "timestep 1 ms": {"timestep": 1e-3}}
    cases[f"new muscle force x{1 - rel:.2f}"] = {"model_hook": lambda m: set_new_force(m, 1 - rel)}
    cases[f"new muscle force x{1 + rel:.2f}"] = {"model_hook": lambda m: set_new_force(m, 1 + rel)}
    cases["all muscle F_max x0.8"] = {"model_hook": lambda m: scale_all(m, 0.8)}
    cases["all muscle F_max x1.2"] = {"model_hook": lambda m: scale_all(m, 1.2)}
    for axis, label in enumerate("xyz"):
        for sign in (1.0, -1.0):
            off = np.zeros(3)
            off[axis] = sign * rms_m
            key = f"origins {'+' if sign > 0 else '-'}{label} by palm RMS"
            cases[key] = {"model_hook": lambda m, o=off: shift_origins(m, o)}
    rows = {k: summary(cfg, **v) for k, v in cases.items()}
    _, _, guard = simulate(cfg)
    out = {"factorial_2x2": factorial, "guards": guard, "long_run": long_run(cfg), "sensitivity": rows,
           "passed_count": sum(r["passed"] for r in rows.values()), "n_cases": len(rows),
           "capacity": capacity(cfg), "measured_tip_pinch": MEASURED}
    (ROOT / "reports" / "fixes" / "pinch_evidence.json").write_text(
        json.dumps(out, indent=1, default=float) + "\n", encoding="utf-8")
    print(json.dumps({k: v for k, v in out.items() if k != "sensitivity"}, indent=1, default=float))
    for k, r in rows.items():
        print(k, r["passed"], round(r["pinch_force_mean_last_0p5s"], 3), round(r["pinch_force_min_last_0p5s"], 3))


if __name__ == "__main__":
    main()
