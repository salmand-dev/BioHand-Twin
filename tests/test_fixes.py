"""Tests for the thumb intrinsic, passive stiffness and optional calibration patches, and their recorded evidence."""

import copy
import importlib.util
import json
import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest
import yaml

from simulation.myosim_source import DEFAULT_MYOSIM_DIR, build_myohand
from simulation.patches import DEFAULT_PATCHES, Patch, PatchError, build_biohand_model
from simulation.tasks import load_config, run_task

ROOT = Path(__file__).resolve().parents[1]
FIXES = ROOT / "reports" / "fixes"
SOURCE = yaml.safe_load((ROOT / "data/sources/mirakhorlo2016_thumb.yaml").read_text(encoding="utf-8"))
needs_model = pytest.mark.skipif(not (DEFAULT_MYOSIM_DIR / "myo_sim").is_dir(), reason="third_party/myo_sim not cloned")


def load_script(name):
    sys.path.insert(0, str(ROOT / "scripts"))
    spec = importlib.util.spec_from_file_location(name, ROOT / "scripts" / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def recorded(name):
    return json.loads((FIXES / name).read_text(encoding="utf-8"))


# ---------- source data and mapping maths (no model needed) ----------

@pytest.mark.parametrize("name", list(SOURCE["muscles"]))
def test_transcribed_pcsa_matches_the_papers_formula(name):
    """PCSA = mass / (density x optimal fibre length), density 0.0010567 g/mm3: catches transcription errors."""
    m = SOURCE["muscles"][name]
    assert m["mass_g"] / (0.0010567 * m["fibre_mm"]) == pytest.approx(m["pcsa_mm2"], rel=0.02)


def test_source_records_provenance():
    for key in ("doi", "data_doi", "data_md5", "licence", "date_retrieved", "specimen", "frame"):
        assert SOURCE["source"][key]
    assert SOURCE["source"]["licence"] == "CC BY 4.0"


# ---------- patches ----------

@needs_model
def test_similarity_recovers_a_known_transform():
    mod = load_script("fix_derive_thumb_intrinsics")
    rng = np.random.default_rng(1)
    src = rng.normal(size=(6, 3))
    angle = 0.7
    rot = np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
    scale, r, t = mod.similarity(src, 1.3 * src @ rot.T + np.array([1.0, 2.0, 3.0]))
    assert scale == pytest.approx(1.3)
    assert np.allclose(r, rot)
    assert np.allclose(t, [1.0, 2.0, 3.0])
    frame = mod.bone_frame(np.array([1.0, 1.0, 0.0]), np.array([0.2, 0.0, 1.0]))
    assert np.allclose(frame.T @ frame, np.eye(3))
    assert np.linalg.det(frame) == pytest.approx(1.0)


@needs_model
def test_upstream_is_untouched_and_default_adds_five_muscles():
    assert build_myohand("fragment").model.nu == 39
    m = build_biohand_model().model
    assert m.nu == 44
    assert {"APB_r", "FPBs_r", "FPBd_r", "ADPo_r", "ADPt_r"} <= {m.actuator(i).name for i in range(m.nu)}
    assert m.njnt == 23


@needs_model
@pytest.mark.parametrize("name", list(SOURCE["muscles"]))
def test_new_muscle_force_is_pcsa_times_specific_tension(name):
    m = build_biohand_model().model
    a = m.actuator(name).id
    expected = SOURCE["muscles"][name]["pcsa_mm2"] * SOURCE["specific_tension"]["value_N_per_mm2"]
    assert m.actuator_gainprm[a, 2] == pytest.approx(expected)
    assert m.actuator_biasprm[a, 2] == pytest.approx(expected)


@needs_model
def test_tampered_new_muscle_force_is_rejected():
    p = Patch.load("thumb_intrinsics_v1")
    data = copy.deepcopy(p.data)
    data["changes"][0]["force_N"] *= 2
    with pytest.raises(PatchError):
        Patch(p.patch_id, data).validate(build_myohand("fragment").model)


@needs_model
def test_expected_actions_of_new_muscles_recorded():
    assert recorded("thumb_intrinsics_derivation.json")["all_expected_signs_ok"] is True


@needs_model
def test_new_muscle_paths_cross_only_thumb_joints():
    m = build_biohand_model().model
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    from simulation.muscles import moment_arm_matrix
    r = moment_arm_matrix(m, d)
    thumb = {"cmc_flexion_r", "cmc_abduction_r", "mp_flexion_r"}
    for name in SOURCE["muscles"]:
        crossed = {m.joint(j).name for j in range(m.njnt) if abs(r[m.actuator(name).id, m.jnt_dofadr[j]]) > 1e-6}
        assert crossed <= thumb, name


@needs_model
def test_stiffness_values_carry_their_evidence_labels():
    m = build_biohand_model().model
    p = Patch.load("passive_stiffness_v1")
    labels = {c["joint"]: c["evidence"] for c in p.data["changes"]}
    assert labels["flexion_r"] == "measured"
    assert labels["cmc_abduction_r"] == "assumed_in_published_model"
    assert labels["mcp3_flexion_r"] == "estimated_by_analogy_with_index_mcp"
    assert m.jnt_stiffness[m.joint("flexion_r").id] == pytest.approx(0.7875, abs=1e-3)
    assert m.jnt_stiffness[m.joint("pm3_flexion_r").id] == 0.0


@needs_model
def test_tampered_stiffness_is_rejected():
    p = Patch.load("passive_stiffness_v1")
    data = copy.deepcopy(p.data)
    data["changes"][0]["stiffness_Nm_per_rad"] = 50.0
    with pytest.raises(PatchError):
        Patch(p.patch_id, data).validate(build_myohand("fragment").model)
    data = copy.deepcopy(p.data)
    data["changes"][4]["evidence"] = "measured"
    with pytest.raises(PatchError):
        Patch(p.patch_id, data).validate(build_myohand("fragment").model)


# ---------- optional calibration stays separate from validation ----------

def test_calibration_is_not_part_of_the_reference_model():
    assert "moment_arm_calibration_v1" not in DEFAULT_PATCHES
    p = Patch.load("moment_arm_calibration_v1")
    assert p.data["classification"].startswith("TUNED")


@needs_model
def test_site_shift_must_be_labelled_tuned_and_bounded():
    p = Patch.load("moment_arm_calibration_v1")
    upstream = build_myohand("fragment").model
    data = copy.deepcopy(p.data)
    data["classification"] = "validated"
    with pytest.raises(PatchError):
        Patch(p.patch_id, data).validate(upstream)
    data = copy.deepcopy(p.data)
    data["changes"][0]["shift_m"] = [1.0, 0.0, 0.0]
    with pytest.raises(PatchError):
        Patch(p.patch_id, data).validate(upstream)


def test_validation_result_stays_the_unfitted_one():
    fit = recorded("moment_arm_calibration.json")
    val = json.loads((ROOT / "reports/P8/moment_arm_validation.json").read_text())
    assert fit["summary"]["garland2018"]["unfitted"] == val["wrist_measured_garland2018"]["n_passed"] == 8
    assert fit["summary"]["lee2015"]["unfitted"] == val["index_mcp_derived_lee2015"]["n_passed"] == 10
    assert "not validation" in fit["warning"]


@needs_model
def test_calibration_patch_builds_and_changes_only_declared_sites():
    base = build_biohand_model().model
    fitted = build_biohand_model((*DEFAULT_PATCHES, "moment_arm_calibration_v1")).model
    declared = {c["site"] for c in Patch.load("moment_arm_calibration_v1").data["changes"]}
    for s in range(base.nsite):
        name = base.site(s).name
        assert np.allclose(base.site_pos[s], fitted.site_pos[s]) != (name in declared), name


# ---------- recorded functional evidence ----------

def test_pinch_factorial_shows_stiffness_decides_contact_force():
    f = recorded("pinch_evidence.json")["factorial_2x2"]
    assert f["both"]["passed"] and f["stiffness only"]["passed"]
    assert not f["neither"]["passed"] and not f["new muscles only"]["passed"]


def test_new_muscles_raise_pinch_force_capacity():
    s = recorded("pinch_evidence.json")["capacity"]["stats"]
    for mu in ("mu0", "mu1"):
        assert s[f"with_new_{mu}"]["median_N"] > s[f"without_new_{mu}"]["median_N"]
        assert s[f"with_new_{mu}"]["feasible_share"] > s[f"without_new_{mu}"]["feasible_share"]


def test_pinch_guards_recorded():
    g = recorded("pinch_evidence.json")["guards"]
    assert g["joints_within_3deg_of_a_limit"] == []
    assert g["max_limit_overshoot_deg"] < 1.0
    assert all("distal" in n or "distph" in n for n in g["geoms_in_contact_at_end"])


def test_contact_softness_explains_the_grip_inertia_effect():
    g = recorded("grip_checks.json")
    assert g["inertia_spread_N"]["8 s"] > 50.0
    assert all(v["inertia_spread_N"] < 15.0 for v in g["contact_hypothesis_8s"].values())


@needs_model
def test_pinch_task_uses_distal_contact_pairs_only_and_passes():
    cfg = load_config("pinch")
    assert not any("midph" in p["geom2"] for p in cfg["contact_pairs"])
    assert run_task(cfg).success["all_passed"]
