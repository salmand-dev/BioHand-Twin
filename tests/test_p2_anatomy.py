"""P2 tests: rigid segments, glossary consistency, joint semantics, UI contract."""

import json
from pathlib import Path

import mujoco
import pytest
import yaml

from simulation.kinematic_semantics import joint_kinematic_evidence
from simulation.myosim_source import DEFAULT_MYOSIM_DIR, build_myohand
from simulation.naming import Glossary, GlossaryError
from simulation.segments import rigid_segments, segment_of_body
from simulation.ui_contract import motion_units, unit_for_body

ROOT = Path(__file__).resolve().parents[1]
needs_model = pytest.mark.skipif(not (DEFAULT_MYOSIM_DIR / "myo_sim").is_dir(),
                                 reason="third_party/myo_sim not cloned")


@pytest.fixture(scope="module")
def model():
    return build_myohand("fragment").model


@pytest.fixture(scope="module")
def glossary():
    return Glossary.load()


def names(m, obj, n):
    return [mujoco.mj_id2name(m, obj, i) for i in range(n)]


# ---------- glossary (unit) ----------

def test_glossary_loads_and_validates(glossary):
    assert len(glossary.entries("bodies")) == 38
    assert len(glossary.entries("joints")) == 23
    assert len(glossary.entries("actuators")) == 44  # 39 upstream + 5 from patch thumb_intrinsics_v1


def test_glossary_rejects_duplicate_alias():
    g = Glossary({"status_values": {"n/a": ""}, "bodies": [
        {"model_name": "a", "status": "n/a", "aliases": ["x"]},
        {"model_name": "b", "status": "n/a", "aliases": ["x"]}]})
    with pytest.raises(GlossaryError):
        g.validate()


def test_alias_resolution(glossary):
    assert glossary.resolve("ECRL") == "ECRL_r"
    assert glossary.resolve("proxph2") == "2proxph_r"
    assert glossary.canonical("secondmc_r") == "metacarpal II"


# ---------- model consistency (integration) ----------

@needs_model
def test_every_model_element_has_exactly_one_glossary_entry(model, glossary):
    for section, obj, n in [("bodies", mujoco.mjtObj.mjOBJ_BODY, model.nbody),
                            ("joints", mujoco.mjtObj.mjOBJ_JOINT, model.njnt),
                            ("actuators", mujoco.mjtObj.mjOBJ_ACTUATOR, model.nu)]:
        expected = names(model, obj, n)
        if section == "actuators":
            source = Path(__file__).resolve().parents[1] / "data" / "sources" / "mirakhorlo2016_thumb.yaml"
            expected = expected + list(yaml.safe_load(source.read_text(encoding="utf-8"))["muscles"])
        assert sorted(e["model_name"] for e in glossary.entries(section)) == sorted(expected)


@needs_model
def test_rigid_segments_partition_bodies(model):
    segs = rigid_segments(model)
    assert len(segs) == 18
    all_bodies = [b for s in segs for b in s.bodies]
    assert sorted(all_bodies) == sorted(names(model, mujoco.mjtObj.mjOBJ_BODY, model.nbody))
    assert sum(s.n_dof for s in segs) == model.nv
    assert sum(len(s.mesh_geoms) for s in segs) == 32
    carpus = next(s for s in segs if s.root_body == "lunate_r")
    assert set(carpus.bodies) == {
        "lunate_r", "scaphoid_r", "pisiform_r", "triquetrum_r", "capitate_r", "trapezium_r",
        "trapezoid_r", "hamate_r", "secondmc_r", "thirdmc_r", "fourthmc_r", "fifthmc_r"}
    assert carpus.joints == ["deviation_r", "flexion_r"]


@needs_model
def test_glossary_representation_agrees_with_segments(model, glossary):
    seg_of = segment_of_body(model)
    for e in glossary.entries("bodies"):
        rep, b = e["representation"], e["model_name"]
        if rep.startswith("rigid_part_of:"):
            assert seg_of[b] == rep.split(":", 1)[1] != b, b
        elif rep == "simulated_segment":
            assert seg_of[b] == b and model.body_jntnum[model.body(b).id] > 0, b
        elif rep in ("fixed_reference", "scaffold_non_hand"):
            assert seg_of[b] == "world", b


@needs_model
def test_joint_muscle_evidence_matches_model(model, glossary):
    """Every 'drives +/-' claim in the glossary is checked against dL/dq at qpos0."""
    inv = json.loads((ROOT / "data" / "model_inventory.json").read_text())
    dl = {t["name"].removesuffix("_tendon_r") + "_r": t["dL_dq_at_qpos0_m_per_rad"] for t in inv["tendons"]}
    for j in glossary.entries("joints"):
        jn = j["model_name"]
        for act in j["muscles_driving_positive"]:
            assert dl[act].get(jn, 0.0) < 0, (jn, act)  # shortening drives the joint positive
        for act in j["muscles_driving_negative"]:
            assert dl[act].get(jn, 0.0) > 0, (jn, act)


@needs_model
def test_joint_kinematic_evidence_matches_glossary(model, glossary):
    ev = {e.joint: e for e in joint_kinematic_evidence(model, model.qpos0.copy())}
    for j in glossary.entries("joints"):
        kind, e = j["kinematic_expected"], ev[j["model_name"]]
        _, radial, dorsal = e.displacement_hand_frame_m
        if kind == "palmar":
            assert dorsal < -1e-4, j["model_name"]
        elif kind == "radial":
            assert radial > 1e-3, j["model_name"]
        elif kind == "pronation":
            assert e.rotation_about_distal_rad < -0.05, j["model_name"]


@needs_model
def test_ui_contract_never_exposes_lumped_bone_as_independent(model, glossary):
    units = motion_units(model, glossary)
    assert len(units) == 18
    trapezoid = unit_for_body(units, "trapezoid_r")
    assert trapezoid.unit_id == "lunate_r" and trapezoid.lumped and trapezoid.movable
    assert "12 bones moving as one" in trapezoid.display_name
    assert not unit_for_body(units, "ulna_r").movable
    assert sum(1 for u in units if u.lumped and u.movable) == 1
    for u in units:  # each body belongs to exactly one unit
        for b in u.member_bodies:
            assert unit_for_body(units, b) is u
