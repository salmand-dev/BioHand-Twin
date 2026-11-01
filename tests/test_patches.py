"""Tests for the explicit BioHand patch mechanism and wrist_rom_v1."""

import copy
import json
from pathlib import Path

import mujoco
import numpy as np
import pytest

from simulation.kinematic_semantics import hand_frame
from simulation.kinematics import forward, qpos_from_angles, site_positions
from simulation.myosim_source import DEFAULT_MYOSIM_DIR, build_myohand
from simulation.patches import Patch, PatchError, build_biohand_model

ROOT = Path(__file__).resolve().parents[1]
pytestmark = pytest.mark.skipif(not (DEFAULT_MYOSIM_DIR / "myo_sim").is_dir(),
                                reason="third_party/myo_sim not cloned")


@pytest.fixture(scope="module")
def models():
    return build_myohand("fragment").model, build_biohand_model(patches=("wrist_rom_v1",)).model


def test_patch_changes_only_the_two_wrist_ranges(models):
    up, pa = models
    changed = [mujoco.mj_id2name(pa, mujoco.mjtObj.mjOBJ_JOINT, j)
               for j in range(pa.njnt) if not np.array_equal(up.jnt_range[j], pa.jnt_range[j])]
    assert changed == ["deviation_r", "flexion_r"]
    for attr in ("actuator_gainprm", "actuator_biasprm", "actuator_lengthrange", "body_mass", "body_pos",
                 "body_quat", "jnt_axis", "jnt_pos", "dof_damping", "site_pos", "geom_size"):
        assert np.array_equal(getattr(up, attr), getattr(pa, attr)), attr
    assert np.degrees(pa.jnt_range[pa.joint("deviation_r").id]) == pytest.approx([-25.0, 10.0])
    assert np.degrees(pa.jnt_range[pa.joint("flexion_r").id]) == pytest.approx([-70.0, 70.0])


def test_patch_values_must_be_derivable_from_source(models):
    up, _ = models
    p = Patch.load("wrist_rom_v1")
    tampered = Patch(p.patch_id, copy.deepcopy(p.data))
    tampered.data["changes"][0]["patched_value_rad"] = [-0.5, 0.2]
    with pytest.raises(PatchError):
        tampered.validate(up)
    wrong_sign = Patch(p.patch_id, copy.deepcopy(p.data))
    wrong_sign.data["changes"][0]["sign_mapping"] = 1
    with pytest.raises(PatchError):
        wrong_sign.validate(up)


def test_patch_refuses_if_upstream_changed(models):
    up, _ = models
    p = Patch.load("wrist_rom_v1")
    stale = Patch(p.patch_id, copy.deepcopy(p.data))
    stale.data["changes"][1]["upstream_value_rad"] = [-1.0, 1.0]
    with pytest.raises(PatchError):
        stale.validate(up)


def test_patched_negative_deviation_is_ulnar(models):
    _, pa = models
    d0 = forward(pa, pa.qpos0.copy())
    radial = hand_frame(pa, d0).radial
    tip0 = site_positions(pa, d0, {"m": "MFtip_r"})["m"]
    lower = float(pa.jnt_range[pa.joint("deviation_r").id][0])   # -25 deg (patched)
    d1 = forward(pa, qpos_from_angles(pa, {"deviation_r": lower}))
    tip1 = site_positions(pa, d1, {"m": "MFtip_r"})["m"]
    assert (tip1 - tip0) @ radial < -0.01   # >1 cm toward the ulnar side


def test_recorded_patch_check():
    out = json.loads((ROOT / "reports" / "P3" / "patch_check.json").read_text())
    for policy in ("zero", "full", "random_seeded"):
        assert out["smoke_patched"][policy]["finite"]
        assert out["smoke_patched"][policy]["warnings"] == {}
    after = out["operating_range_patched_rom"]["muscles"]
    before = out["operating_range_upstream_rom"]["muscles"]
    outside_after = {n for n, v in after.items() if v["fraction_outside_active_window"] > 0}
    outside_before = {n for n, v in before.items() if v["fraction_outside_active_window"] > 0}
    # the patch introduces no muscle outside MuJoCo's active force window
    assert outside_after <= outside_before
    assert outside_before == {"FDP5_r", "FDS4_r", "RI5_r"}
