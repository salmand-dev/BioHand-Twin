"""P4 muscle-tendon audit on the BioHand reference model (upstream MyoHand + wrist_rom_v1).

Outputs reports/P4/:
  moment_arm_curves.json     r(q) = -dL/dq over each spanned joint's full BioHand range
  p4_muscle_audit.json       activation dynamics, isolated activation, ablation, length sampling

Usage: .venv/Scripts/python.exe scripts/p4_muscle_audit.py
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

from simulation.kinematics import FINGERTIP_SITES, forward, site_positions
from simulation.muscles import moment_arm_matrix, muscle_state, normalisation, without_actuator
from simulation.myosim_source import build_myohand
from simulation.patches import build_biohand_model

N_CURVE = 61
SPAN_TOL_M = 1e-3          # |r| >= 1 mm somewhere in the sweep -> joint is spanned
JUMP_TOL_M = 1e-3          # |dr| > 1 mm between adjacent samples -> possible discontinuity
SIGN_TOL_M = 5e-4          # sign change counted only between |r| > 0.5 mm values
SEED = 20261031
FINGER_FLEXORS = [f"{m}{d}_r" for d in (2, 3, 4, 5) for m in ("FDS", "FDP")]


def names(model: mujoco.MjModel, obj: mujoco.mjtObj, n: int) -> list[str]:
    return [mujoco.mj_id2name(model, obj, i) or str(i) for i in range(n)]


def moment_arm_curves(model: mujoco.MjModel, upstream: mujoco.MjModel) -> dict[str, Any]:
    act_names = names(model, mujoco.mjtObj.mjOBJ_ACTUATOR, model.nu)
    jnt_names = names(model, mujoco.mjtObj.mjOBJ_JOINT, model.njnt)
    data = mujoco.MjData(model)
    backgrounds = {"neutral": model.qpos0.copy(),
                   "mid_range": 0.5 * (model.jnt_range[:, 0] + model.jnt_range[:, 1])}
    curves: dict[str, Any] = {}
    flags: list[dict[str, Any]] = []
    for j, jn in enumerate(jnt_names):
        lo, hi = model.jnt_range[j]
        q_sweep = np.linspace(lo, hi, N_CURVE)
        dof = model.jnt_dofadr[j]
        up_lo, up_hi = upstream.jnt_range[j]
        for bg_name, bg in backgrounds.items():
            r = np.zeros((N_CURVE, model.nu))
            for i, qv in enumerate(q_sweep):
                q = bg.copy()
                q[model.jnt_qposadr[j]] = qv
                forward(model, q, data)
                mujoco.mj_transmission(model, data)
                r[i] = moment_arm_matrix(model, data)[:, dof]
            for a, an in enumerate(act_names):
                ra = r[:, a]
                if np.max(np.abs(ra)) < SPAN_TOL_M:
                    continue
                jumps = np.abs(np.diff(ra))
                big = ra[np.abs(ra) > SIGN_TOL_M]
                sign_changes = int(np.sum(np.diff(np.sign(big)) != 0)) if big.size > 1 else 0
                extended = (q_sweep < up_lo - 1e-9) | (q_sweep > up_hi + 1e-9)
                key = f"{an}|{jn}|{bg_name}"
                curves[key] = {"q_rad": q_sweep.round(5).tolist(), "r_m": ra.round(6).tolist()}
                rec = {"actuator": an, "joint": jn, "background": bg_name,
                       "r_min_mm": round(1000 * float(ra.min()), 2), "r_max_mm": round(1000 * float(ra.max()), 2),
                       "max_jump_mm": round(1000 * float(jumps.max()), 3), "sign_changes": sign_changes,
                       "in_patch_extension": bool(extended.any())}
                if extended.any():
                    rec["sign_change_only_in_extension"] = bool(
                        sign_changes > 0 and np.all(np.sign(ra[~extended][np.abs(ra[~extended]) > SIGN_TOL_M]) ==
                                                    np.sign(ra[~extended][np.abs(ra[~extended]) > SIGN_TOL_M][0])))
                if rec["max_jump_mm"] > 1000 * JUMP_TOL_M or sign_changes:
                    flags.append(rec)
                curves[key]["summary"] = rec
    return {"curves": curves, "flags": flags,
            "settings": {"n": N_CURVE, "span_tol_m": SPAN_TOL_M, "jump_tol_m": JUMP_TOL_M,
                         "sign_tol_m": SIGN_TOL_M, "backgrounds": list(backgrounds)}}


def activation_dynamics(model: mujoco.MjModel) -> dict[str, Any]:
    """Step u: 0 -> 1 (0.2 s) -> 0 (0.3 s) on one muscle; time to 63 % rise and 63 % fall."""
    a = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "FDP3_r")
    d = mujoco.MjData(model)
    trace = []
    for k in range(int(0.5 / model.opt.timestep)):
        t = k * model.opt.timestep
        d.ctrl[:] = 0.0
        d.ctrl[a] = 1.0 if t < 0.2 else 0.0
        mujoco.mj_step(model, d)
        trace.append((t + model.opt.timestep, float(d.act[model.actuator_actadr[a]])))
    tr = np.array(trace)
    rise = tr[tr[:, 1] >= 0.632][0, 0]
    peak = tr[tr[:, 0] <= 0.2 + 1e-9][-1, 1]
    fall_idx = np.where((tr[:, 0] > 0.2) & (tr[:, 1] <= peak * 0.368))[0]
    fall = float(tr[fall_idx[0], 0] - 0.2) if fall_idx.size else None
    return {"actuator": "FDP3_r", "dynprm_tau_act_s": float(model.actuator_dynprm[a, 0]),
            "dynprm_tau_deact_s": float(model.actuator_dynprm[a, 1]),
            "time_to_63pct_rise_s": round(float(rise), 4),
            "time_to_63pct_fall_s": None if fall is None else round(fall, 4),
            "activation_at_0p2s": round(float(peak), 4),
            "note": "MuJoCo muscle activation dynamics scale the time constants with activation level "
                    "(see MuJoCo docs, muscle actuators); values here are measured, not modelled by BioHand."}


def isolated_activation(model: mujoco.MjModel) -> dict[str, Any]:
    """Each muscle alone at u=1 for 0.2 s from qpos0, gravity off, vs. an all-zero baseline."""
    m = model
    m.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_GRAVITY)
    steps = int(0.2 / m.opt.timestep)

    def run(ctrl: np.ndarray) -> np.ndarray:
        d = mujoco.MjData(m)
        for _ in range(steps):
            d.ctrl[:] = ctrl
            mujoco.mj_step(m, d)
        return d.qpos.copy()

    base = run(np.zeros(m.nu))
    d0 = forward(m, m.qpos0.copy())
    mujoco.mj_transmission(m, d0)
    r0 = moment_arm_matrix(m, d0)
    jn = names(m, mujoco.mjtObj.mjOBJ_JOINT, m.njnt)
    rows = []
    for a, an in enumerate(names(m, mujoco.mjtObj.mjOBJ_ACTUATOR, m.nu)):
        u = np.zeros(m.nu)
        u[a] = 1.0
        dq = run(u) - base
        checks = []
        for j in range(m.njnt):
            r = r0[a, m.jnt_dofadr[j]]
            if abs(r) < 1e-3:
                continue
            lo, hi = m.jnt_range[j]
            q0 = m.qpos0[m.jnt_qposadr[j]]
            blocked = (r < 0 and q0 <= lo + 1e-9) or (r > 0 and q0 >= hi - 1e-9)
            dqj = float(dq[m.jnt_qposadr[j]])
            checks.append({"joint": jn[j], "r_mm": round(1000 * r, 2), "dq_rad": round(dqj, 5),
                           "blocked_by_limit": bool(blocked),
                           "agrees": bool(np.sign(dqj) == np.sign(r) and abs(dqj) > 1e-4)})
        free = [c for c in checks if not c["blocked_by_limit"]]
        primary = max(free, key=lambda c: abs(c["r_mm"])) if free else None
        rows.append({"actuator": an, "primary_joint": primary["joint"] if primary else None,
                     "primary_agrees": primary["agrees"] if primary else None,
                     "free_joint_agreement": round(float(np.mean([c["agrees"] for c in free])), 3) if free else None,
                     "joints": checks})
    m.opt.disableflags &= ~int(mujoco.mjtDisableBit.mjDSBL_GRAVITY)
    prim = [r["primary_agrees"] for r in rows if r["primary_agrees"] is not None]
    return {"protocol": "u_a=1 for 0.2 s from qpos0, gravity off; dq = q - q(all u=0); expected sign = sign(r) at "
                        "qpos0 for joints with |r| >= 1 mm; joints whose expected motion pushes into a limit at qpos0 "
                        "are marked blocked and excluded from agreement",
            "primary_joint_agreement": f"{sum(prim)}/{len(prim)}", "actuators": rows}


def ablation(model: mujoco.MjModel) -> dict[str, Any]:
    """Remove each actuator (gear 0) and quantify fingertip changes on two deterministic tasks."""
    steps = int(1.0 / model.opt.timestep)
    rng = np.random.default_rng(SEED)
    random_ctrl = rng.uniform(0, 1, (steps, model.nu))
    flex_ctrl = np.zeros(model.nu)
    for n in FINGER_FLEXORS:
        flex_ctrl[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n)] = 0.5

    def run(m: mujoco.MjModel, task: str) -> dict[str, np.ndarray]:
        d = mujoco.MjData(m)
        for k in range(steps):
            d.ctrl[:] = random_ctrl[k] if task == "random_seeded" else flex_ctrl
            mujoco.mj_step(m, d)
        mujoco.mj_forward(m, d)
        return {**site_positions(m, d, FINGERTIP_SITES), "_q": d.qpos.copy()}

    tasks = ("finger_flexors_0p5", "random_seeded")
    ref = {t: run(model, t) for t in tasks}
    rows = []
    for an in names(model, mujoco.mjtObj.mjOBJ_ACTUATOR, model.nu):
        abl = without_actuator(model, an)
        row: dict[str, Any] = {"actuator": an}
        for t in tasks:
            out = run(abl, t)
            tip = {k: round(1000 * float(np.linalg.norm(out[k] - ref[t][k])), 2) for k in FINGERTIP_SITES}
            dq = out["_q"] - ref[t]["_q"]
            row[t] = {"fingertip_shift_mm": tip, "max_tip_shift_mm": max(tip.values()),
                      "max_joint_change_deg": round(float(np.degrees(np.abs(dq).max())), 2),
                      "joint_most_changed": names(model, mujoco.mjtObj.mjOBJ_JOINT, model.njnt)[int(np.argmax(np.abs(dq)))]}
        rows.append(row)
    return {"method": "actuator removed by zeroing its gear on a model copy (no active or passive force); "
                      "1 s simulation from qpos0; differences vs intact model",
            "tasks": {"finger_flexors_0p5": "FDS2-5 and FDP2-5 at u=0.5, others 0",
                      "random_seeded": f"u ~ U(0,1) per step, seed {SEED}"},
            "actuators": rows}


def sampled_length_ranges(model: mujoco.MjModel, n: int = 20000) -> dict[str, Any]:
    """Geometric min/max actuator length over uniformly sampled joint configurations."""
    rng = np.random.default_rng(SEED)
    d = mujoco.MjData(model)
    lengths = np.zeros((n, model.nu))
    for i in range(n):
        q = rng.uniform(model.jnt_range[:, 0], model.jnt_range[:, 1])
        forward(model, q, d)
        mujoco.mj_transmission(model, d)
        lengths[i] = d.actuator_length
    l0, lt = normalisation(model)
    out = {}
    for a, an in enumerate(names(model, mujoco.mjtObj.mjOBJ_ACTUATOR, model.nu)):
        lr = model.actuator_lengthrange[a]
        lmin_s, lmax_s = float(lengths[:, a].min()), float(lengths[:, a].max())
        overlap = max(0.0, min(lr[1], lmax_s) - max(lr[0], lmin_s)) / (lmax_s - lmin_s)
        ln = (lengths[:, a] - lt[a]) / l0[a]
        out[an] = {"specified_lengthrange_m": lr.round(5).tolist(),
                   "sampled_min_max_m": [round(lmin_s, 5), round(lmax_s, 5)],
                   "fraction_of_sampled_span_inside_specified": round(overlap, 3),
                   "normalised_length_min_max": [round(float(ln.min()), 3), round(float(ln.max()), 3)],
                   "fraction_samples_outside_active_window": round(float(np.mean(
                       (ln < model.actuator_gainprm[a, 4]) | (ln > model.actuator_gainprm[a, 5]))), 3)}
    return {"n_samples": n, "seed": SEED, "joint_ranges": "BioHand (patched)", "muscles": out}


def main() -> None:
    upstream = build_myohand("fragment").model
    model = build_biohand_model().model
    out = ROOT / "reports" / "P4"
    out.mkdir(parents=True, exist_ok=True)
    curves = moment_arm_curves(model, upstream)
    (out / "moment_arm_curves.json").write_text(json.dumps(curves, indent=0) + "\n")
    d = mujoco.MjData(model)
    mujoco.mj_forward(model, d)
    state0 = muscle_state(model, d)
    result = {
        "model": "BioHand reference = MyoHand fragment + wrist_rom_v1",
        "moment_arm_flags": curves["flags"],
        "n_curves": len(curves["curves"]),
        "passive_tension_at_qpos0_N": {n: round(float(v), 3) for n, v in zip(
            names(model, mujoco.mjtObj.mjOBJ_ACTUATOR, model.nu), state0.passive_tension_N, strict=True)},
        "activation_dynamics": activation_dynamics(model),
        "isolated_activation": isolated_activation(model),
        "ablation": ablation(model),
        "sampled_length_ranges": sampled_length_ranges(model),
    }
    (out / "p4_muscle_audit.json").write_text(json.dumps(result, indent=1) + "\n")
    print("curves:", result["n_curves"], "| flags:", len(curves["flags"]))
    for f in curves["flags"]:
        print("  FLAG", f)
    print("activation dynamics:", result["activation_dynamics"])
    print("isolated activation primary-joint agreement:", result["isolated_activation"]["primary_joint_agreement"])
    for r in result["isolated_activation"]["actuators"]:
        if r["primary_agrees"] is False:
            print("  DISAGREE", r["actuator"], r["primary_joint"], [c for c in r["joints"] if c["joint"] == r["primary_joint"]])
    abl = sorted(result["ablation"]["actuators"], key=lambda r: -r["finger_flexors_0p5"]["max_tip_shift_mm"])
    print("ablation top (flexor task):", [(r["actuator"], r["finger_flexors_0p5"]["max_tip_shift_mm"]) for r in abl[:6]])
    sl = result["sampled_length_ranges"]["muscles"]
    bad = {k: v for k, v in sl.items() if v["fraction_of_sampled_span_inside_specified"] < 0.5
           or v["fraction_samples_outside_active_window"] > 0}
    for k, v in bad.items():
        print("  LENGTH", k, v)


if __name__ == "__main__":
    main()
