"""P3 tests: forward kinematics, transforms, angle units, sweeps, limits and conventions."""

import json
from pathlib import Path

import mujoco
import numpy as np
import pytest

from biomechanics.rigid import axis_angle_matrix, hinge_transform, pose_from_pos_quat
from simulation.kinematics import (
    FINGERTIP_SITES,
    JointLimitError,
    forward,
    qpos_from_angles,
    site_jacobian,
    site_positions,
    sweep_joint,
    to_display_degrees,
)
from simulation.myosim_source import DEFAULT_MYOSIM_DIR, build_myohand

ROOT = Path(__file__).resolve().parents[1]
needs_model = pytest.mark.skipif(not (DEFAULT_MYOSIM_DIR / "myo_sim").is_dir(),
                                 reason="third_party/myo_sim not cloned")


# ---------- unit: rigid-body math ----------

def test_axis_angle_matches_mujoco():
    rng = np.random.default_rng(0)
    for _ in range(50):
        axis = rng.normal(size=3)
        angle = rng.uniform(-np.pi, np.pi)
        q = np.zeros(4)
        mujoco.mju_axisAngle2Quat(q, axis / np.linalg.norm(axis), angle)
        ref = np.zeros(9)
        mujoco.mju_quat2Mat(ref, q)
        assert np.allclose(axis_angle_matrix(axis, angle), ref.reshape(3, 3), atol=1e-12)


def test_axis_angle_right_handed_and_units():
    # +90 deg (pi/2 rad) about +z maps +x to +y; angles are radians
    assert np.allclose(axis_angle_matrix(np.array([0, 0, 1.0]), np.pi / 2) @ [1, 0, 0], [0, 1, 0], atol=1e-12)
    assert not np.allclose(axis_angle_matrix(np.array([0, 0, 1.0]), 90.0) @ [1, 0, 0], [0, 1, 0], atol=1e-3)


def test_hinge_transform_fixes_anchor():
    anchor = np.array([0.1, -0.2, 0.3])
    t = hinge_transform(anchor, np.array([1.0, 2.0, 3.0]), 0.7)
    assert np.allclose(t[:3, :3] @ anchor + t[:3, 3], anchor)


def test_invalid_axis_rejected():
    with pytest.raises(ValueError):
        axis_angle_matrix(np.zeros(3), 1.0)


def test_display_degrees():
    assert to_display_degrees(np.pi) == pytest.approx(180.0)


# ---------- integration: model kinematics ----------

@pytest.fixture(scope="module")
def model():
    return build_myohand("fragment").model


def independent_fk(m, q):
    """Body world transforms composed with numpy only (no MuJoCo kinematics)."""
    t = [np.eye(4)] * m.nbody
    for b in range(1, m.nbody):
        tb = t[m.body_parentid[b]] @ pose_from_pos_quat(m.body_pos[b], m.body_quat[b])
        for j in range(m.body_jntadr[b], m.body_jntadr[b] + m.body_jntnum[b]):
            tb = tb @ hinge_transform(m.jnt_pos[j], m.jnt_axis[j], q[m.jnt_qposadr[j]])
        t[b] = tb
    return t


@needs_model
def test_independent_fk_matches_mujoco(model):
    rng = np.random.default_rng(1)
    d = mujoco.MjData(model)
    for _ in range(30):
        q = rng.uniform(model.jnt_range[:, 0], model.jnt_range[:, 1])
        forward(model, q, d)
        t = independent_fk(model, q)
        for b in range(1, model.nbody):
            assert np.allclose(t[b][:3, 3], d.xpos[b], atol=1e-12)
            assert np.allclose(t[b][:3, :3], d.xmat[b].reshape(3, 3), atol=1e-12)


@needs_model
def test_qpos_from_angles_enforces_limits_and_radians(model):
    qpos_from_angles(model, {"mcp2_flexion_r": np.pi / 2 - 1e-9})
    with pytest.raises(JointLimitError):
        qpos_from_angles(model, {"mcp2_flexion_r": 90.0})  # degrees passed by mistake
    with pytest.raises(JointLimitError):
        qpos_from_angles(model, {"flexion_r": 1.0})        # beyond +45 deg model limit
    with pytest.raises(KeyError):
        qpos_from_angles(model, {"no_such_joint": 0.0})


@needs_model
def test_site_jacobian_matches_finite_difference(model):
    rng = np.random.default_rng(2)
    q = rng.uniform(model.jnt_range[:, 0] * 0.5, model.jnt_range[:, 1] * 0.5)
    d = forward(model, q)
    h = 1e-6
    for site in FINGERTIP_SITES.values():
        jac = site_jacobian(model, d, site)
        fd = np.zeros_like(jac)
        for k in range(model.nv):
            qp, qm = q.copy(), q.copy()
            qp[k] += h
            qm[k] -= h
            pp = site_positions(model, forward(model, qp), {"s": site})["s"]
            pm = site_positions(model, forward(model, qm), {"s": site})["s"]
            fd[:, k] = (pp - pm) / (2 * h)
        assert np.allclose(jac, fd, atol=1e-7), site


@needs_model
def test_neutral_pose_reproducible():
    a, b = build_myohand("fragment").model, build_myohand("fragment").model
    da, db = forward(a, a.qpos0.copy()), forward(b, b.qpos0.copy())
    assert np.array_equal(da.xpos, db.xpos) and np.array_equal(da.site_xpos, db.site_xpos)


@needs_model
@pytest.mark.parametrize("joint", ["flexion_r", "deviation_r", "mcp3_flexion_r", "pm2_flexion_r",
                                   "md5_flexion_r", "cmc_abduction_r", "ip_flexion_r"])
def test_sweep_stays_in_range_and_is_continuous(model, joint):
    res = sweep_joint(model, joint, n=91)
    j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
    assert res.angles_rad.min() >= model.jnt_range[j][0] and res.angles_rad.max() <= model.jnt_range[j][1]
    dtheta = res.angles_rad[1] - res.angles_rad[0]
    for key in FINGERTIP_SITES:
        steps = np.linalg.norm(np.diff(res.tips_m[key], axis=0), axis=1)
        if steps.max() == 0:
            continue
        # a fingertip within 0.3 m of any hand joint moves at most 0.3 m * dtheta per step
        assert steps.max() <= 0.3 * dtheta
        assert np.all(np.isfinite(res.tips_m[key]))


# ---------- recorded P3 evidence ----------

@needs_model
def test_recorded_p3_findings():
    out = json.loads((ROOT / "reports" / "P3" / "p3_kinematics_output.json").read_text())
    assert out["neutral_reproducibility"]["identical"]
    assert out["all_sweeps_continuous"]
    rom = {r["quantity"]: r for r in out["rom_comparison"]}
    assert not rom["ulnar deviation"]["model_covers_reference"]      # 10 deg vs 40 deg (Ryu 1991)
    assert all(r["model_covers_reference"] for q, r in rom.items() if q.startswith(("MCP", "PIP", "DIP")))
    wrist = json.loads((ROOT / "reports" / "P3" / "wrist_convention_check.json").read_text())
    assert wrist["coordinates"]["deviation_r"]["cosine_between_directions"] < -0.9   # inverted vs MoBL
    assert wrist["coordinates"]["flexion_r"]["cosine_between_directions"] > 0.9
    axes = {r["joint"]: r for r in json.loads(
        (ROOT / "reports" / "P3" / "axis_convention_check.json").read_text())["joints"]}
    same = [j for j, r in axes.items() if r.get("verdict") == "same convention"]
    assert len(axes) == 22 and len(same) == 21 and "deviation_r" not in same


@needs_model
def test_recorded_rom_with_wrist_patch():
    out = json.loads((ROOT / "reports" / "P3" / "p3_kinematics_output.json").read_text())
    rom = {r["quantity"]: r for r in out["rom_comparison_biohand_patched"]}
    assert rom["wrist flexion"]["model_covers_reference"] and rom["wrist extension"]["model_covers_reference"]
    # deviation stays at the MoBL-ARMS source range, which is below Ryu 1991 ADL maxima (documented)
    assert rom["ulnar deviation"]["model_deg"] == pytest.approx(25.0, abs=1e-3)
    assert rom["radial deviation"]["model_deg"] == pytest.approx(10.0, abs=1e-3)
    assert not rom["ulnar deviation"]["model_covers_reference"]
