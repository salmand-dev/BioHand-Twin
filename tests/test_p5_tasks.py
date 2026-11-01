"""P5 tests: limit-stiffness patch, static optimization, inverse dynamics, task runner, recorded results."""

import copy
import json
from pathlib import Path

import mujoco
import numpy as np
import pytest
import yaml

from simulation.inverse import inverse_dynamics
from simulation.myosim_source import DEFAULT_MYOSIM_DIR, build_myohand
from simulation.patches import Patch, PatchError, build_biohand_model
from simulation.static_opt import muscle_affine_terms, solve_activations
from simulation.tasks import TASK_DIR, build_task_model, control_at, evaluate, load_config, run_task

ROOT = Path(__file__).resolve().parents[1]
pytestmark = pytest.mark.skipif(not (DEFAULT_MYOSIM_DIR / "myo_sim").is_dir(),
                                reason="third_party/myo_sim not cloned")


@pytest.fixture(scope="module")
def model():
    return build_biohand_model().model


def warmed(m, n=100, seed=3):
    d = mujoco.MjData(m)
    rng = np.random.default_rng(seed)
    for _ in range(n):
        d.ctrl[:] = rng.uniform(0, 1, m.nu)
        mujoco.mj_step(m, d)
    mujoco.mj_forward(m, d)
    return d


# ---- limit stiffness patch ----

def test_limit_patch_sets_twice_timestep(model):
    assert np.allclose(model.jnt_solref[:, 0], 2 * model.opt.timestep)
    assert np.allclose(model.jnt_solref[:, 1], 1.0)


def test_limit_patch_rejects_off_rule_value():
    up = build_myohand("fragment")
    p = Patch.load("limit_stiffness_v1")
    bad = Patch(p.patch_id, copy.deepcopy(p.data))
    bad.data["changes"][0]["patched_value"] = [0.001, 1.0]
    with pytest.raises(PatchError):
        bad.validate(up.model, up.spec)


def test_recorded_limit_stiffness_evidence():
    out = json.loads((ROOT / "reports" / "P5" / "limit_stiffness_check.json").read_text())
    assert out["before"]["worst_static_penetration_deg"] > 30
    assert out["after"]["worst_static_penetration_deg"] < 2
    assert out["after"]["random_control_4s"]["finite"] and out["after"]["random_control_4s"]["warnings"] == {}


# ---- static optimization ----

def test_affine_muscle_terms_reproduce_mujoco(model):
    d = warmed(model)
    mom, g, b = muscle_affine_terms(model, d)
    act = d.act[model.actuator_actadr]
    assert np.allclose(g * act + b, d.actuator_force, atol=1e-10)
    assert np.allclose(mom.T @ (g * act + b), d.qfrc_actuator, atol=1e-10)


def test_static_opt_recovers_feasible_torque(model):
    d = warmed(model)
    mom, g, b = muscle_affine_terms(model, d)
    a_true = np.random.default_rng(4).uniform(0.1, 0.9, model.nu)
    tau = mom.T @ (g * a_true + b)
    res = solve_activations(model, d, tau, lam=1e-9)
    assert res.success and res.residual_norm < 1e-6
    assert np.all((res.activation >= 0) & (res.activation <= 1))


def test_static_opt_respects_bounds_for_infeasible_torque(model):
    d = warmed(model)
    res = solve_activations(model, d, np.full(model.nv, 1e3))
    assert np.all((res.activation >= 0) & (res.activation <= 1)) and res.residual_norm > 1.0


# ---- inverse dynamics ----

def test_inverse_dynamics_matches_forward_forces():
    cfg = load_config("hand_closing")
    m, _ = build_task_model(cfg)
    groups = yaml.safe_load((ROOT / "data" / "experiments" / "muscle_groups.yaml").read_text())
    d = mujoco.MjData(m)
    for _ in range(200):
        d.ctrl[:] = control_at(d.time, cfg, groups, m)
        mujoco.mj_forward(m, d)
        q_inv = inverse_dynamics(m, d.qpos, d.qvel, d.qacc, d.act)
        assert np.allclose(q_inv, d.qfrc_actuator + d.qfrc_applied, atol=1e-9)
        mujoco.mj_step(m, d)


# ---- task runner ----

def test_all_configs_load_and_have_success_criteria():
    ids = sorted(p.stem for p in TASK_DIR.glob("*.yaml"))
    assert ids == ["hand_closing", "hand_opening", "pinch", "power_grip_proxy", "rest_neutral",
                   "thumb_opposition", "wrist_deviation", "wrist_flexion_extension"]
    for i in ids:
        cfg = load_config(i)
        assert cfg["duration_s"] > 0 and "success" in cfg


def test_runner_is_deterministic_and_brace_holds():
    cfg = load_config("hand_closing")
    a, b = run_task(cfg), run_task(cfg)
    assert np.array_equal(a.rows, b.rows)
    assert np.max(np.abs(a.column("q_flexion_r_rad"))) < 0.01   # wrist brace holds (soft equality)


def test_object_overlap_at_start_is_refused():
    cfg = copy.deepcopy(load_config("power_grip_proxy"))
    cfg["initial_angles_rad"] = {}          # thumb at qpos0 overlaps the cylinder
    with pytest.raises(ValueError, match="in contact"):
        run_task(cfg)


def test_evaluate_requires_numerical_health():
    m = {"finite": True, "warnings": {}, "x": 1.0}
    assert evaluate(m, [{"metric": "x", "op": ">=", "value": 1.0}])["all_passed"]
    assert not evaluate({**m, "finite": False}, [{"metric": "x", "op": ">=", "value": 1.0}])["all_passed"]


def test_recorded_task_outcomes():
    s = json.loads((ROOT / "reports" / "P5" / "p5_tasks_summary.json").read_text())
    passed = {k for k, v in s.items() if v["passed"]}
    assert passed == {"hand_closing", "hand_opening", "pinch", "power_grip_proxy", "rest_neutral", "thumb_opposition",
                      "wrist_deviation", "wrist_flexion_extension"}
    assert all(v["healthy"] and v["deterministic"] for v in s.values())
    assert all(v["passed"] == v["fine_dt_passed"] for v in s.values())   # same verdict at dt = 1 ms
