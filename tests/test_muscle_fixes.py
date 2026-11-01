"""Tests for patch muscle_fixes_v1 (EDC5 force, RI2 path, derived lengthranges)."""

import copy

import mujoco
import numpy as np
import pytest

from simulation.kinematics import forward
from simulation.myosim_source import DEFAULT_MYOSIM_DIR, build_myohand
from simulation.patches import Patch, PatchError, build_biohand_model

pytestmark = pytest.mark.skipif(not (DEFAULT_MYOSIM_DIR / "myo_sim").is_dir(),
                                reason="third_party/myo_sim not cloned")


@pytest.fixture(scope="module")
def models():
    return (build_myohand("fragment"), build_biohand_model(patches=("wrist_rom_v1",)).model,
            build_biohand_model(patches=("wrist_rom_v1", "muscle_fixes_v1")).model)


def test_only_declared_actuators_change(models):
    _, wrist_only, fixed = models
    p = Patch.load("muscle_fixes_v1")
    declared_lr = {c["actuator"] for c in p.data["changes"] if c["field"] == "actuator_lengthrange"}
    for a in range(fixed.nu):
        name = mujoco.mj_id2name(fixed, mujoco.mjtObj.mjOBJ_ACTUATOR, a)
        same_lr = np.allclose(fixed.actuator_lengthrange[a], wrist_only.actuator_lengthrange[a])
        assert same_lr == (name not in declared_lr), name
        same_f = fixed.actuator_gainprm[a, 2] == wrist_only.actuator_gainprm[a, 2]
        assert same_f == (name != "EDC5_r"), name
    assert fixed.actuator_gainprm[fixed.actuator("EDC5_r").id, 2] == pytest.approx(39.4)
    assert np.array_equal(fixed.jnt_range, wrist_only.jnt_range)
    assert np.array_equal(fixed.body_mass, wrist_only.body_mass)


def test_ri2_path_is_sites_only_and_smooth(models):
    _, _, fixed = models
    t = fixed.tendon("RI2_tendon_r").id
    types = fixed.wrap_type[fixed.tendon_adr[t]:fixed.tendon_adr[t] + fixed.tendon_num[t]]
    assert set(types.tolist()) == {int(mujoco.mjtWrap.mjWRAP_SITE)} and len(types) == 3


def test_patch_rejects_wrong_force_source(models):
    up, _, _ = models
    p = Patch.load("muscle_fixes_v1")
    bad = Patch(p.patch_id, copy.deepcopy(p.data))
    next(c for c in bad.data["changes"] if c["field"] == "actuator_force")["patched_value_N"] = 50.0
    with pytest.raises(PatchError):
        bad.validate(up.model, up.spec)


def test_patch_rejects_unknown_derivation(models):
    up, _, _ = models
    p = Patch.load("muscle_fixes_v1")
    bad = Patch(p.patch_id, copy.deepcopy(p.data))
    next(c for c in bad.data["changes"] if c["field"] == "actuator_lengthrange")["derivation"] = "by_hand"
    with pytest.raises(PatchError):
        bad.validate(up.model, up.spec)


def test_derived_lengthranges_independently_reproduced(models):
    """Independent seeded sampling over the upstream ranges must lie inside, and nearly fill, the derived range."""
    up, _, fixed = models
    p = Patch.load("muscle_fixes_v1")
    lr = {c["actuator"]: c["patched_value_m"] for c in p.data["changes"] if c["field"] == "actuator_lengthrange"}
    rng = np.random.default_rng(12345)
    d = mujoco.MjData(fixed)
    ranges = up.model.jnt_range
    lens = []
    for _ in range(6000):
        forward(fixed, rng.uniform(ranges[:, 0], ranges[:, 1]), d)
        mujoco.mj_transmission(fixed, d)
        lens.append(d.actuator_length.copy())
    lens = np.array(lens)
    for name, (lo, hi) in lr.items():
        a = fixed.actuator(name).id
        assert lens[:, a].min() >= lo - 1e-5 and lens[:, a].max() <= hi + 1e-5, name
        assert (lens[:, a].max() - lens[:, a].min()) >= 0.8 * (hi - lo), name


def test_repaired_model_is_stable(models):
    _, _, fixed = models
    for policy in ("zero", "full"):
        d = mujoco.MjData(fixed)
        for _ in range(1000):
            d.ctrl[:] = 0.0 if policy == "zero" else 1.0
            mujoco.mj_step(fixed, d)
        assert np.isfinite(d.qpos).all() and not any(w.number for w in d.warning)
