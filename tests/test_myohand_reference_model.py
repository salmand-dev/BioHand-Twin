"""Integration tests: pinned MyoSim MyoHand build, inventory and numerical health.

Skipped (not failed) when third_party/myo_sim is absent, so unit tests can run in
a clean checkout; P1 evidence requires these tests to run, not skip.
"""

import json
from pathlib import Path

import mujoco
import numpy as np
import pytest

from simulation.myosim_source import (
    DEFAULT_MYOSIM_DIR,
    MYOSIM_PINNED_COMMIT,
    build_myohand,
    verify_checkout,
)

pytestmark = [
    pytest.mark.integration,
    pytest.mark.skipif(
        not (DEFAULT_MYOSIM_DIR / "myo_sim").is_dir(), reason="third_party/myo_sim not cloned"
    ),
]

ROOT = Path(__file__).resolve().parents[1]

EXPECTED_JOINTS = [
    "pro_sup_r", "deviation_r", "flexion_r",
    "cmc_flexion_r", "cmc_abduction_r", "mp_flexion_r", "ip_flexion_r",
] + [
    name
    for d in (2, 3, 4, 5)
    for name in (f"mcp{d}_flexion_r", f"mcp{d}_abduction_r", f"pm{d}_flexion_r", f"md{d}_flexion_r")
]


@pytest.fixture(scope="module")
def fragment():
    return build_myohand("fragment")


def test_checkout_is_pinned_and_clean():
    info = verify_checkout()
    assert info.commit == MYOSIM_PINNED_COMMIT
    assert info.dirty_files == ()


def test_fragment_builds_without_warnings(fragment):
    assert fragment.build_warnings == []


def test_fragment_counts_match_upstream_readme_claims(fragment):
    m = fragment.model
    # README_hand.md at the pinned commit states 23 DoF and 39 muscle actuators.
    assert m.nv == 23 and m.nq == 23
    assert m.nu == 39 and m.ntendon == 39
    assert m.neq == 0 and m.nsensor == 0


def test_joint_names_order_and_types(fragment):
    m = fragment.model
    names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(m.njnt)]
    assert names == EXPECTED_JOINTS
    assert set(m.jnt_type.tolist()) == {int(mujoco.mjtJoint.mjJNT_HINGE)}
    assert m.jnt_limited.all()
    assert np.all((m.jnt_range[:, 0] <= m.qpos0) & (m.qpos0 <= m.jnt_range[:, 1]))


def test_every_actuator_is_a_muscle_on_its_own_tendon(fragment):
    m = fragment.model
    for a in range(m.nu):
        assert m.actuator_trntype[a] == mujoco.mjtTrn.mjTRN_TENDON
        assert m.actuator_gaintype[a] == mujoco.mjtGain.mjGAIN_MUSCLE
        assert m.actuator_dyntype[a] == mujoco.mjtDyn.mjDYN_MUSCLE
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a)
        tendon = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_TENDON, int(m.actuator_trnid[a, 0]))
        assert tendon == name.removesuffix("_r") + "_tendon_r"


def test_carpals_are_lumped_into_one_wrist_segment(fragment):
    """All carpal bodies except lunate are welded; wrist DoFs live on lunate_r."""
    m = fragment.model
    carpals = ["scaphoid_r", "pisiform_r", "triquetrum_r", "capitate_r",
               "trapezium_r", "trapezoid_r", "hamate_r"]
    for name in carpals:
        b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)
        assert m.body_jntnum[b] == 0, name
    lunate = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "lunate_r")
    jnames = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)
              for j in range(m.njnt) if m.jnt_bodyid[j] == lunate}
    assert jnames == {"deviation_r", "flexion_r"}


def test_zero_control_simulation_is_finite_and_within_limits(fragment):
    m = fragment.model
    d = mujoco.MjData(m)
    for _ in range(int(2.0 / m.opt.timestep)):
        mujoco.mj_step(m, d)
    assert np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all()
    q = d.qpos[m.jnt_qposadr]
    # MuJoCo joint limits are soft; 0.01 rad tolerance (observed max 0.0042 rad).
    assert np.all(q >= m.jnt_range[:, 0] - 0.01) and np.all(q <= m.jnt_range[:, 1] + 0.01)


def test_seeded_random_control_is_bitwise_deterministic(fragment):
    m = fragment.model
    finals = []
    for _ in range(2):
        d = mujoco.MjData(m)
        rng = np.random.default_rng(7)
        for _ in range(300):
            d.ctrl[:] = rng.uniform(0, 1, m.nu)
            mujoco.mj_step(m, d)
        finals.append(d.qpos.copy())
    assert np.array_equal(finals[0], finals[1])


def test_committed_inventory_is_consistent_with_fresh_build(fragment):
    path = ROOT / "data" / "model_inventory.json"
    inv = json.loads(path.read_text())
    m = fragment.model
    assert inv["_meta"]["upstream"]["commit"] == MYOSIM_PINNED_COMMIT
    assert inv["counts"]["nv"] == m.nv and inv["counts"]["nu"] == m.nu
    assert [j["name"] for j in inv["joints"]] == EXPECTED_JOINTS
    gp = [a["mujoco_muscle_params"]["peak_active_force_N"] for a in inv["actuators"]]
    assert np.allclose(gp, m.actuator_gainprm[:, 2])


def test_hashed_upstream_files_unchanged():
    import hashlib

    hashes = json.loads((ROOT / "data" / "provenance" / "myohand_file_hashes.json").read_text())
    for rel, digest in hashes["files"].items():
        data = (DEFAULT_MYOSIM_DIR / rel).read_bytes()
        assert hashlib.sha256(data).hexdigest() == digest, rel
