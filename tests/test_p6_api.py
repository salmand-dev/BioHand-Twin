"""P6 tests: session API, versioned schemas, exports, reproducibility, fingerprint."""

import csv
import io
import json

import jsonschema
import mujoco
import numpy as np
import pytest

from biomechanics.frames import mujoco_to_godot_pos
from simulation.api import (
    SCHEMA_VERSION,
    BioHandSession,
    ControlError,
    load_schema,
    model_fingerprint,
    simulate_task,
)
from simulation.kinematics import JointLimitError
from simulation.myosim_source import DEFAULT_MYOSIM_DIR
from simulation.patches import build_biohand_model
from simulation.tasks import load_config, run_task

pytestmark = pytest.mark.skipif(not (DEFAULT_MYOSIM_DIR / "myo_sim").is_dir(),
                                reason="third_party/myo_sim not cloned")


@pytest.fixture(scope="module")
def session():
    return BioHandSession()


@pytest.fixture(scope="module")
def schemas():
    return load_schema("telemetry_frame"), load_schema("session_metadata")


def test_schemas_are_valid_draft_2020_12(schemas):
    for s in schemas:
        jsonschema.Draft202012Validator.check_schema(s)
    assert SCHEMA_VERSION in schemas[0]["properties"]["schema_version"]["enum"]


def test_frames_and_metadata_validate(session, schemas):
    session.reset()
    session.set_controls({"FDP3_r": 0.5})
    for f in session.record(0.2, every_n_steps=20, include_bodies=True, include_paths=True):
        jsonschema.validate(f, schemas[0])
    jsonschema.validate(session.metadata(), schemas[1])


def test_frame_rejects_unknown_fields(schemas):
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate({"schema_version": SCHEMA_VERSION, "time_s": 0, "joints": {}, "muscles": {},
                             "fingertips_m": {}, "contact_normal_N": 0,
                             "flags": {"outside_muscle_validity_envelope": [], "max_limit_violation_rad": 0,
                                       "finite": True}, "extra": 1}, schemas[0])


def test_control_validation(session):
    with pytest.raises(ControlError):
        session.set_controls({"NOPE": 0.1})
    with pytest.raises(ControlError):
        session.set_controls({"FDP2_r": 1.5})
    with pytest.raises(ControlError):
        session.set_controls(np.zeros(3))
    with pytest.raises(JointLimitError):
        session.reset({"mcp2_flexion_r": 3.0})


def test_reset_and_step_are_deterministic():
    a, b = BioHandSession(), BioHandSession()
    out = []
    for s in (a, b):
        s.reset({"mcp3_flexion_r": 0.4})
        s.set_controls({"EDC3_r": 0.6, "FDS3_r": 0.2})
        out.append(json.dumps(s.record(0.3, every_n_steps=15)))
    assert out[0] == out[1]


def test_api_task_frames_equal_runner_bitwise():
    _, frames = simulate_task("thumb_opposition", every_n_steps=10, include_bodies=False)
    r = run_task(load_config("thumb_opposition"))
    assert len(frames) == len(r.rows)
    for i, f in enumerate(frames):
        for j, v in f["joints"].items():
            assert v["q_rad"] == r.column(f"q_{j}_rad")[i]


def test_godot_body_poses_use_the_single_conversion(session):
    session.reset()
    f = session.frame(include_bodies=True)
    m, d = session.model, session.data
    for name, pose in f["bodies_godot"].items():
        assert np.allclose(pose["pos"], mujoco_to_godot_pos(d.xpos[m.body(name).id]))


def test_csv_export_has_metadata_units_and_consistent_columns(session):
    session.reset()
    frames = session.record(0.1, every_n_steps=25)
    text = session.export_csv(frames)
    meta = [ln for ln in text.splitlines() if ln.startswith("#")]
    assert any(f"schema_version: {SCHEMA_VERSION}" in ln for ln in meta)
    assert any("model_fingerprint_sha256" in ln for ln in meta) and any("units:" in ln for ln in meta)
    rows = list(csv.reader(io.StringIO("\n".join(ln for ln in text.splitlines() if not ln.startswith("#")))))
    header, body = rows[0], rows[1:]
    assert len(body) == len(frames) and all(len(r) == len(header) for r in body)
    assert all(c == "time_s" or c.endswith(("_rad", "_rad_s", "_N", "_m")) or c.startswith(("ctrl_", "act_"))
               or c == "n_outside_envelope" for c in header)


def test_json_export_round_trip(session, schemas):
    session.reset()
    frames = session.record(0.05, every_n_steps=25)
    doc = json.loads(session.export_json(frames))
    assert doc["schema_version"] == SCHEMA_VERSION and doc["frames"] == json.loads(json.dumps(frames))
    jsonschema.validate(doc["metadata"], schemas[1])


def test_fingerprint_stable_and_sensitive():
    a = model_fingerprint(build_biohand_model().model)
    b = model_fingerprint(build_biohand_model().model)
    c = model_fingerprint(build_biohand_model(patches=("wrist_rom_v1",)).model)
    assert a == b and a != c
    m = build_biohand_model().model
    m.opt.timestep = 0.001
    assert model_fingerprint(m) != a


def test_inventory_exposes_lumped_units_and_names(session):
    inv = session.inventory()
    lumped = [u for u in inv["motion_units"] if u["lumped"] and u["movable"]]
    assert len(lumped) == 1 and lumped[0]["root_body"] == "lunate_r"
    names = {a["name"]: a for a in inv["actuators"]}
    assert names["RI2_r"]["name_status"] == "unresolved"
    j = {x["name"]: x for x in inv["joints"]}
    assert np.allclose(np.degrees(j["flexion_r"]["muscle_validity_envelope_rad"]), [-45, 45], atol=1e-4)
    assert "abductor digiti minimi" in inv["not_represented"]
    assert "adductor pollicis" not in inv["not_represented"]  # added by patch thumb_intrinsics_v1


def test_task_session_records_contact_for_grip():
    s, frames = simulate_task("power_grip_proxy", every_n_steps=50, include_bodies=False)
    assert frames[0]["contact_normal_N"] == 0.0 and frames[-1]["contact_normal_N"] > 1.0
    assert s.metadata()["simulation"]["task"]["id"] == "power_grip_proxy"
    assert mujoco.mj_name2id(s.model, mujoco.mjtObj.mjOBJ_GEOM, "grip_cylinder") >= 0
