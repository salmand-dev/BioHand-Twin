"""P4 tests: muscle semantics, moment arms, activation, ablation, validity envelope, recorded findings."""

import json
from pathlib import Path

import mujoco
import numpy as np
import pytest

from simulation.kinematics import forward
from simulation.muscles import moment_arm_matrix, muscle_state, without_actuator
from simulation.myosim_source import DEFAULT_MYOSIM_DIR, build_myohand
from simulation.patches import (
    build_biohand_model,
    muscle_validity_envelope,
    outside_muscle_validity,
)

ROOT = Path(__file__).resolve().parents[1]
pytestmark = pytest.mark.skipif(not (DEFAULT_MYOSIM_DIR / "myo_sim").is_dir(),
                                reason="third_party/myo_sim not cloned")


@pytest.fixture(scope="module")
def model():
    return build_biohand_model().model


def stepped(m, n=150, seed=0):
    d = mujoco.MjData(m)
    rng = np.random.default_rng(seed)
    for _ in range(n):
        d.ctrl[:] = rng.uniform(0, 1, m.nu)
        mujoco.mj_step(m, d)
    mujoco.mj_forward(m, d)
    return d


def test_moment_arms_equal_minus_dl_dq(model):
    d = stepped(model)
    r = moment_arm_matrix(model, d)
    q, h = d.qpos.copy(), 1e-6
    dd = mujoco.MjData(model)
    for k in range(model.nv):
        lens = []
        for s in (1, -1):
            dd.qpos[:] = q
            dd.qpos[k] += s * h
            mujoco.mj_fwdPosition(model, dd)
            mujoco.mj_transmission(model, dd)
            lens.append(dd.actuator_length.copy())
        assert np.allclose(r[:, k], -(lens[0] - lens[1]) / (2 * h), atol=1e-8)


def test_tension_decomposes_into_mujoco_active_and_passive(model):
    s = muscle_state(model, stepped(model))
    assert np.allclose(s.tension_N, s.active_tension_N + s.passive_tension_N, atol=1e-9)
    assert np.all(s.tension_N >= -1e-12) and np.all(s.passive_tension_N >= -1e-12)
    assert np.all((s.activation >= 0) & (s.activation <= 1))


def test_positive_moment_arm_means_positive_torque(model):
    d = stepped(model)
    s = muscle_state(model, d)
    r = moment_arm_matrix(model, d)
    assert np.allclose(r.T @ s.tension_N, d.qfrc_actuator, atol=1e-8)


def test_activation_rises_faster_than_it_falls():
    out = json.loads((ROOT / "reports" / "P4" / "p4_muscle_audit.json").read_text())["activation_dynamics"]
    assert out["time_to_63pct_rise_s"] < out["time_to_63pct_fall_s"]
    assert out["dynprm_tau_act_s"] == pytest.approx(0.01) and out["dynprm_tau_deact_s"] == pytest.approx(0.04)


def test_ablation_removes_force_and_leaves_original_intact(model):
    a = model.actuator("FDP3_r").id
    ablated = without_actuator(model, "FDP3_r")
    assert model.actuator_gear[a, 0] == 1.0 and ablated.actuator_gear[a, 0] == 0.0
    d = mujoco.MjData(ablated)
    d.ctrl[:] = 1.0
    for _ in range(20):
        mujoco.mj_step(ablated, d)
    assert d.actuator_force[a] == 0.0
    assert np.all(d.actuator_force[np.arange(model.nu) != a] <= 0)
    with pytest.raises(KeyError):
        without_actuator(model, "NOPE")


def test_recorded_isolated_activation_and_ablation():
    out = json.loads((ROOT / "reports" / "P4" / "p4_muscle_audit.json").read_text())
    assert out["isolated_activation"]["primary_joint_agreement"] == "44/44"
    abl = {r["actuator"]: r for r in out["ablation"]["actuators"]}
    assert len(abl) == 44
    # removing a finger flexor in the flexor task changes fingertips; removing an extensor-only wrist
    # muscle (inactive in that task, passive only) changes them less than removing FDP2
    assert abl["FDP2_r"]["finger_flexors_0p5"]["max_tip_shift_mm"] > abl["ECRB_r"]["finger_flexors_0p5"]["max_tip_shift_mm"]


def test_validity_envelope_is_intersection_of_source_ranges(model):
    env = muscle_validity_envelope(build_myohand("fragment").model, model)
    assert np.degrees(env["flexion_r"]) == pytest.approx([-45.0, 45.0], abs=1e-4)
    assert np.degrees(env["deviation_r"]) == pytest.approx([-10.0, 10.0], abs=1e-4)
    assert set(env) == {"flexion_r", "deviation_r"}   # only joints widened by a patch are restricted
    q = model.qpos0.copy()
    q[model.jnt_qposadr[model.joint("flexion_r").id]] = np.radians(60.0)
    assert outside_muscle_validity(model, q, env) == ["flexion_r"]
    assert outside_muscle_validity(model, model.qpos0, env) == []


def test_recorded_upstream_defects_before_fix():
    """Upstream defects as found before muscle_fixes_v1 (evidence kept in *_pre_fix.json)."""
    curves = json.loads((ROOT / "reports" / "P4" / "moment_arm_curves_pre_fix.json").read_text())["curves"]

    def worst_jump(key):
        v = curves[key]
        r, q = np.array(v["r_m"]), np.array(v["q_rad"])
        i = int(np.argmax(np.abs(np.diff(r))))
        return abs(r[i + 1] - r[i]), np.degrees(0.5 * (q[i] + q[i + 1]))

    for m in ("FDS2_r", "FDS3_r"):
        jump, at = worst_jump(f"{m}|flexion_r|neutral")
        assert jump > 0.02 and at < -45.0        # >20 mm break, beyond the upstream range
    jump, at = worst_jump("RI2_r|mcp2_flexion_r|neutral")
    assert jump > 0.009 and 0.0 < at < 15.0      # break inside the upstream range (M7)
    pre = json.loads((ROOT / "reports" / "P4" / "p4_muscle_audit_pre_fix.json").read_text())
    sl = pre["sampled_length_ranges"]["muscles"]
    bad = {k for k, v in sl.items() if v["fraction_of_sampled_span_inside_specified"] < 0.5}
    assert bad == {"FDS4_r", "FDP5_r", "RI5_r", "LU_RB4_r", "UI_UB4_r", "LU_RB5_r"}


def test_recorded_state_after_fix():
    """After muscle_fixes_v1 (current BioHand reference model)."""
    out = json.loads((ROOT / "reports" / "P4" / "p4_muscle_audit.json").read_text())
    sl = out["sampled_length_ranges"]["muscles"]
    assert min(v["fraction_of_sampled_span_inside_specified"] for v in sl.values()) > 0.5
    assert max(v["fraction_samples_outside_active_window"] for v in sl.values()) == 0.0
    assert out["passive_tension_at_qpos0_N"]["RI2_r"] < 0.5
    assert out["isolated_activation"]["primary_joint_agreement"] == "44/44"
    flex = {r["actuator"]: r["finger_flexors_0p5"]["max_tip_shift_mm"] for r in out["ablation"]["actuators"]}
    fdp = [flex[f"FDP{d}_r"] for d in (2, 3, 4, 5)]
    assert max(fdp) / min(fdp) < 1.5   # every FDP now matters comparably (before: 106 mm vs 1.4 mm)
    curves = json.loads((ROOT / "reports" / "P4" / "moment_arm_curves.json").read_text())["curves"]
    r = np.array(curves["RI2_r|mcp2_flexion_r|neutral"]["r_m"])
    assert np.abs(np.diff(r)).max() < 1e-3 and r.min() > 0   # smooth MCP flexor, like RI3


def test_moment_arms_continuous_inside_envelope(model):
    """Inside the validity envelope, wrist moment arms change by < 1 mm per 0.25 deg for every muscle."""
    d = mujoco.MjData(model)
    j = model.joint("flexion_r").id
    prev = None
    for qv in np.radians(np.arange(-45.0, 45.0001, 0.25)):
        q = model.qpos0.copy()
        q[model.jnt_qposadr[j]] = qv
        forward(model, q, d)
        mujoco.mj_transmission(model, d)
        r = moment_arm_matrix(model, d)[:, model.jnt_dofadr[j]]
        if prev is not None:
            assert np.max(np.abs(r - prev)) < 1e-3
        prev = r
