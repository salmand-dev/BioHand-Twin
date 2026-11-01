"""Deterministic functional-task runner (P5).

A task = YAML config (data/experiments/tasks/<id>.yaml): initial joint angles (rad), open-loop muscle
activation phases, optional scenario fixtures (contact pairs, a static object), duration (the stopping
condition), recorded telemetry and numerical success criteria. MuJoCo is the only dynamics; this module
sets controls, steps, and measures. Scenario fixtures are task apparatus, not anatomy, and are recorded
in the output metadata.
"""

from __future__ import annotations

import csv
import io
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import yaml

from simulation.kinematic_semantics import hand_frame
from simulation.kinematics import FINGERTIP_SITES, qpos_from_angles
from simulation.muscles import muscle_state
from simulation.myosim_source import MYOSIM_PINNED_COMMIT, build_myohand
from simulation.patches import (
    DEFAULT_PATCHES,
    build_biohand_model,
    muscle_validity_envelope,
    outside_muscle_validity,
)

ROOT = Path(__file__).resolve().parents[1]
TASK_DIR = ROOT / "data" / "experiments" / "tasks"
GROUPS = ROOT / "data" / "experiments" / "muscle_groups.yaml"
OPS = {"<": np.less, "<=": np.less_equal, ">": np.greater, ">=": np.greater_equal}
FINGER_FLEX = [(f"mcp{d}_flexion_r", f"pm{d}_flexion_r") for d in (2, 3, 4, 5)]


def load_config(task_id: str) -> dict[str, Any]:
    cfg = yaml.safe_load((TASK_DIR / f"{task_id}.yaml").read_text(encoding="utf-8"))
    if cfg["id"] != task_id:
        raise ValueError(f"config id {cfg['id']!r} != file name {task_id!r}")
    return cfg


def _hand_frame_at_neutral(model: mujoco.MjModel) -> tuple[Any, mujoco.MjData]:
    d = mujoco.MjData(model)
    mujoco.mj_forward(model, d)
    return hand_frame(model, d), d


def build_task_model(cfg: dict[str, Any], timestep: float | None = None) -> tuple[mujoco.MjModel, dict[str, Any]]:
    """BioHand reference model + scenario fixtures declared in the task config."""
    patches = tuple(cfg.get("patches", DEFAULT_PATCHES))
    built = build_biohand_model(patches=patches)
    spec, fixtures = built.spec, []
    for pair in cfg.get("contact_pairs", []):
        p = spec.add_pair()
        p.name, p.geomname1, p.geomname2, p.condim = pair["name"], pair["geom1"], pair["geom2"], 3
        fixtures.append({"type": "contact_pair", **pair, "contact_params": "MuJoCo defaults (not validated)"})
    initial = cfg.get("initial_angles_rad", {})
    for jn in cfg.get("brace_joints", []):
        eq = spec.add_equality()
        eq.name, eq.type, eq.objtype = f"brace_{jn}", mujoco.mjtEq.mjEQ_JOINT, mujoco.mjtObj.mjOBJ_JOINT
        eq.name1 = jn
        eq.data[:5] = [float(initial.get(jn, 0.0)), 0.0, 0.0, 0.0, 0.0]
        # rigid apparatus: same stiffness rule as the joint limits (time constant = 2 x timestep)
        eq.solref = [2.0 * float(built.model.opt.timestep), 1.0]
        fixtures.append({"type": "joint_brace", "joint": jn, "held_at_rad": float(initial.get(jn, 0.0)),
                         "implementation": "MuJoCo joint equality constraint, solref = (2 x timestep, 1)",
                         "status": "experimental apparatus (forearm/wrist brace), not anatomy"})
    obj = cfg.get("object")
    if obj:
        frame, d0 = _hand_frame_at_neutral(built.model)
        bases = [d0.xpos[built.model.body(f"{k}proxph_r").id] for k in (2, 3, 4, 5)]
        centre = np.mean(bases, axis=0) - frame.dorsal * (obj["radius_m"] + obj["palmar_clearance_m"])
        # cylinder axis (local z) along the hand's radial axis
        z = frame.radial
        x = frame.distal - frame.distal.dot(z) * z
        x /= np.linalg.norm(x)
        rot = np.column_stack([x, np.cross(z, x), z])
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, rot.ravel())
        g = spec.worldbody.add_geom()
        g.name, g.type = obj["name"], mujoco.mjtGeom.mjGEOM_CYLINDER
        g.size = [obj["radius_m"], obj["half_length_m"], 0]
        g.pos, g.quat = centre.tolist(), quat.tolist()
        g.contype, g.conaffinity, g.condim = 1, 1, 3
        g.rgba = [0.3, 0.5, 0.9, 0.5]
        fixtures.append({"type": "static_object", **obj, "centre_world_m": centre.round(5).tolist(),
                         "quat_wxyz_world": quat.round(6).tolist(), "geom_axis": "local z (MuJoCo cylinder)",
                         "placement": "palmar to the mean of proximal phalanx II-V origins at qpos0; "
                                      "axis along the hand radial axis",
                         "status": "illustrative task fixture, not anatomy"})
    model = spec.compile()
    if timestep is not None:
        model.opt.timestep = timestep
    return model, {"patches": list(patches), "fixtures": fixtures, "timestep_s": float(model.opt.timestep),
                   "solver": int(model.opt.solver), "integrator": int(model.opt.integrator),
                   "iterations": int(model.opt.iterations), "myosim_commit": MYOSIM_PINNED_COMMIT}


def control_at(t: float, cfg: dict[str, Any], groups: dict[str, list[str]], model: mujoco.MjModel) -> np.ndarray:
    """Open-loop activation command at time t: per muscle, max over active phases (linear ramps)."""
    u = np.zeros(model.nu)
    for ph in cfg.get("phases", []):
        if not ph["start_s"] <= t < ph["end_s"]:
            continue
        ramp = ph.get("ramp_s", 0.0)
        frac = 1.0 if ramp <= 0 else min(1.0, (t - ph["start_s"]) / ramp)
        for grp, level in ph["activations"].items():
            for name in groups.get(grp, [grp]):
                a = model.actuator(name).id
                u[a] = max(u[a], float(level) * frac)
    return np.clip(u, 0.0, 1.0)


def contact_normal_forces(model: mujoco.MjModel, data: mujoco.MjData, geoms: set[int] | None) -> tuple[float, set[int]]:
    """Total contact normal force (N) involving any geom in ``geoms`` (all contacts if None) and hand bodies touched."""
    total, bodies = 0.0, set()
    f6 = np.zeros(6)
    for i in range(data.ncon):
        c = data.contact[i]
        if geoms is not None and c.geom1 not in geoms and c.geom2 not in geoms:
            continue
        mujoco.mj_contactForce(model, data, i, f6)
        total += abs(float(f6[0]))
        for g in (c.geom1, c.geom2):
            if geoms is None or g not in geoms:
                bodies.add(int(model.geom_bodyid[g]))
    return total, bodies


@dataclass
class TaskResult:
    task_id: str
    meta: dict[str, Any]
    columns: list[str]
    rows: np.ndarray
    metrics: dict[str, Any] = field(default_factory=dict)
    success: dict[str, Any] = field(default_factory=dict)

    def column(self, name: str) -> np.ndarray:
        return self.rows[:, self.columns.index(name)]

    def to_csv(self) -> str:
        buf = io.StringIO()
        buf.write(f"# BioHand Twin task {self.task_id}; MyoSim {self.meta['myosim_commit']}; "
                  f"patches {self.meta['patches']}; timestep {self.meta['timestep_s']} s\n"
                  "# units: time s, q rad, act dimensionless, tension N, len m, tip m (MuJoCo world), contact N\n")
        w = csv.writer(buf, lineterminator="\n")
        w.writerow(self.columns)
        for r in self.rows:
            w.writerow([f"{v:.6g}" for v in r])
        return buf.getvalue()


def run_task(cfg: dict[str, Any], timestep: float | None = None, model_hook: Any = None) -> TaskResult:
    """Run a task. ``model_hook(model)`` may perturb the compiled model (P8 sensitivity analysis only)."""
    groups = yaml.safe_load(GROUPS.read_text(encoding="utf-8"))
    model, meta = build_task_model(cfg, timestep)
    if model_hook is not None:
        model_hook(model)
    model.opt.enableflags |= int(mujoco.mjtEnableBit.mjENBL_ENERGY)
    upstream = build_myohand("fragment").model
    envelope = muscle_validity_envelope(upstream, model)
    data = mujoco.MjData(model)
    data.qpos[:] = qpos_from_angles(model, cfg.get("initial_angles_rad", {}))
    mujoco.mj_forward(model, data)
    obj_geoms = {model.geom(cfg["object"]["name"]).id} if cfg.get("object") else None
    if obj_geoms is not None and contact_normal_forces(model, data, obj_geoms)[1]:
        raise ValueError(f"{cfg['id']}: object is in contact with the hand at t=0; increase clearance")
    pair_geoms = None
    if cfg.get("contact_pairs"):
        pair_geoms = {model.geom(p["geom1"]).id for p in cfg["contact_pairs"]}
    joints = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j) for j in range(model.njnt)]
    acts = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, a) for a in range(model.nu)]
    tip_ids = {k: model.site(v).id for k, v in FINGERTIP_SITES.items()}
    columns = (["time_s"] + [f"q_{j}_rad" for j in joints] + [f"act_{a}" for a in acts]
               + [f"tension_{a}_N" for a in acts] + [f"len_{a}_m" for a in acts]
               + [f"tip_{k}_{ax}_m" for k in tip_ids for ax in "xyz"] + ["contact_normal_N", "outside_envelope"])
    n_steps = round(cfg["duration_s"] / model.opt.timestep)
    every = max(1, round(cfg.get("record_dt_s", 0.02) / model.opt.timestep))
    rows, finite, viol, ke, outside = [], True, 0.0, 0.0, 0
    contact_bodies: set[int] = set()

    def record() -> None:
        nonlocal outside
        ms = muscle_state(model, data)
        force, bodies = contact_normal_forces(model, data, obj_geoms if obj_geoms is not None else pair_geoms) \
            if (obj_geoms or pair_geoms) else (0.0, set())
        flag = bool(outside_muscle_validity(model, data.qpos, envelope))
        outside += int(flag)
        rows.append([data.time, *data.qpos[model.jnt_qposadr], *ms.activation, *ms.tension_N, *ms.length_m,
                     *[v for k in tip_ids for v in data.site_xpos[tip_ids[k]]], force, float(flag)])
        contact_bodies.update(bodies)

    record()
    policy = cfg.get("policy", {"type": "open_loop"})
    tracker = PoseTracker(model, policy, cfg) if policy["type"] == "pose_tracking_static_opt" else None
    for k in range(n_steps):
        if tracker is not None:
            data.ctrl[:] = tracker.control(data)
        else:
            data.ctrl[:] = control_at(data.time, cfg, groups, model)
        mujoco.mj_step(model, data)
        q = data.qpos[model.jnt_qposadr]
        viol = max(viol, float(np.max(np.maximum(model.jnt_range[:, 0] - q, q - model.jnt_range[:, 1]))))
        ke = max(ke, float(data.energy[1]))
        if not (np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()):
            finite = False
            break
        if (k + 1) % every == 0:
            record()
    res = TaskResult(cfg["id"], meta, columns, np.array(rows))
    res.metrics = {
        "finite": finite, "max_limit_violation_rad": round(viol, 5), "max_kinetic_energy_J": round(ke, 6),
        "warnings": {mujoco.mjtWarning(i).name: int(data.warning[i].number)
                     for i in range(len(data.warning)) if data.warning[i].number},
        "fraction_samples_outside_muscle_validity_envelope": round(outside / len(rows), 4),
        "final_max_abs_qvel_rad_s": round(float(np.max(np.abs(data.qvel))), 5),
        "hand_bodies_in_contact": sorted(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or str(b)
                                         for b in contact_bodies),
    }
    if tracker is not None:
        res.metrics["static_opt_solves"] = tracker.n_solves
        res.metrics["static_opt_max_residual_Nm"] = round(tracker.max_residual, 5)
        res.metrics["static_opt_median_residual_Nm"] = round(float(np.median(tracker.residuals)), 5)
    res.metrics.update(task_metrics(res, model, cfg))
    res.success = evaluate(res.metrics, cfg.get("success", []))
    return res


class PoseTracker:
    """Closed-loop muscle control: PD joint torques + gravity/bias compensation -> static optimization.

    Joints in ``target_angles_rad`` track their target (weight 1); other unbraced joints hold their initial
    angle with weight ``hold_weight``; braced joints get weight 0 (held by the brace). A new activation
    vector is solved every ``control_dt_s`` and held in between.
    """

    def __init__(self, model: mujoco.MjModel, policy: dict[str, Any], cfg: dict[str, Any]) -> None:
        from simulation.static_opt import solve_activations
        self._solve = solve_activations
        self.model, self.kp, self.kd = model, float(policy["kp_Nm_per_rad"]), float(policy["kd_Nms_per_rad"])
        self.lam = float(policy.get("lambda", 1e-3))
        self.period = max(1, round(policy.get("control_dt_s", 0.01) / model.opt.timestep))
        q_init = qpos_from_angles(model, cfg.get("initial_angles_rad", {}))
        self.q_target = qpos_from_angles(model, policy["target_angles_rad"], base=q_init)
        self.weights = np.full(model.nv, float(policy.get("hold_weight", 0.2)))
        for jn in policy["target_angles_rad"]:
            self.weights[model.jnt_dofadr[model.joint(jn).id]] = 1.0
        for jn in cfg.get("brace_joints", []):
            self.weights[model.jnt_dofadr[model.joint(jn).id]] = 0.0
        self.step, self.u = 0, np.zeros(model.nu)
        self.n_solves, self.max_residual = 0, 0.0
        self.residuals: list[float] = []

    def control(self, data: mujoco.MjData) -> np.ndarray:
        if self.step % self.period == 0:
            m = self.model
            err = self.q_target[m.jnt_qposadr] - data.qpos[m.jnt_qposadr]
            tau = self.kp * err - self.kd * data.qvel + data.qfrc_bias - data.qfrc_passive
            res = self._solve(m, data, tau, dof_weights=self.weights, lam=self.lam)
            self.u = res.activation
            self.n_solves += 1
            self.max_residual = max(self.max_residual, res.residual_norm)
            self.residuals.append(res.residual_norm)
        self.step += 1
        return self.u


def task_metrics(res: TaskResult, model: mujoco.MjModel, cfg: dict[str, Any]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    t = res.column("time_s")
    for spec in cfg.get("metrics", []):
        kind, name = spec["kind"], spec["name"]
        win = (t >= spec.get("from_s", -np.inf)) & (t <= spec.get("to_s", np.inf))
        if kind == "finger_flexion_fraction":
            fr = []
            for mcp, pip in FINGER_FLEX:
                for jn in (mcp, pip):
                    hi = model.jnt_range[model.joint(jn).id][1]
                    fr.append(res.column(f"q_{jn}_rad")[win] / hi)
            out[name] = round(float(np.mean(np.array(fr), axis=0)[-1]), 4)
        elif kind in ("joint_max", "joint_min", "joint_final"):
            q = res.column(f"q_{spec['joint']}_rad")[win]
            out[name] = round(float({"joint_max": q.max(), "joint_min": q.min(), "joint_final": q[-1]}[kind]), 5)
        elif kind == "tip_distance":
            a, b = spec["tips"]
            pa = np.column_stack([res.column(f"tip_{a}_{x}_m") for x in "xyz"])
            pb = np.column_stack([res.column(f"tip_{b}_{x}_m") for x in "xyz"])
            dist = np.linalg.norm(pa - pb, axis=1)
            out[name] = round(float(dist[win].min() if spec.get("reduce") == "min" else dist[win][-1]), 5)
            out[name + "_initial"] = round(float(dist[0]), 5)
            out[name + "_ratio"] = round(out[name] / out[name + "_initial"], 4)
        elif kind == "contact_force_mean":
            out[name] = round(float(res.column("contact_normal_N")[win].mean()), 4)
        elif kind == "contact_force_min":
            out[name] = round(float(res.column("contact_normal_N")[win].min()), 4)
        elif kind == "n_contact_bodies":
            out[name] = len(res.metrics.get("hand_bodies_in_contact", []))
        else:
            raise ValueError(f"unknown metric kind {kind!r}")
    return out


def evaluate(metrics: dict[str, Any], criteria: list[dict[str, Any]]) -> dict[str, Any]:
    checks = []
    for c in criteria:
        val = metrics[c["metric"]]
        ok = bool(OPS[c["op"]](val, c["value"]))
        checks.append({**c, "observed": val, "passed": ok})
    health = metrics["finite"] and not metrics["warnings"]
    return {"all_passed": health and all(c["passed"] for c in checks), "numerically_healthy": health,
            "checks": checks}
