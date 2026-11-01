"""P8 tests: reference data integrity, completeness of the comparison, recorded outcomes, extension point."""

import json
from pathlib import Path

import numpy as np
import pytest
import yaml

from simulation.extensions import ControlPolicy, as_controller
from simulation.muscles import moment_arm_matrix
from simulation.myosim_source import DEFAULT_MYOSIM_DIR

ROOT = Path(__file__).resolve().parents[1]
REF = yaml.safe_load((ROOT / "data" / "validation" / "reference_moment_arms.yaml").read_text(encoding="utf-8"))["sets"]
needs_model = pytest.mark.skipif(not (DEFAULT_MYOSIM_DIR / "myo_sim").is_dir(), reason="third_party/myo_sim not cloned")


def results(name):
    return json.loads((ROOT / "reports" / "P8" / name).read_text())


def test_reference_values_as_transcribed():
    g = REF["garland2018_wrist_tendon_excursion"]
    assert g["flexion_extension"]["ECRB_r"] == [-10.8, 2.8] and g["radioulnar_deviation"]["ECU_r"] == [23.7, 3.3]
    # Garland sign convention: flexors +, extensors - in FE; ulnar muscles + in RUD
    assert all(g["flexion_extension"][m][0] > 0 for m in ("FCR_r", "FCU_r"))
    assert all(g["radioulnar_deviation"][m][0] > 0 for m in ("FCU_r", "ECU_r"))


def test_every_reference_value_is_compared():
    out = results("moment_arm_validation.json")
    g = REF["garland2018_wrist_tendon_excursion"]
    assert out["wrist_measured_garland2018"]["n"] == len(g["flexion_extension"]) + len(g["radioulnar_deviation"])
    t2 = REF["lee2015_index_derived_from_attachments"]
    assert out["index_mcp_derived_lee2015"]["n"] == len(t2["mcp_flexion"]) + len(t2["mcp_abduction"])
    n = sum(len(tab) for d in REF["lee2015_source_model"]["digits"].values() for tab in d.values())
    assert out["fidelity_vs_2nd_hand"]["n"] == n == 76


def test_recorded_validation_outcomes_including_failures():
    out = results("moment_arm_validation.json")
    w = out["wrist_measured_garland2018"]
    assert w["n_passed"] == 8 and w["stats"]["sign_agreement"] == 1.0
    assert {(r["axis"], r["actuator"]) for r in w["rows"] if not r["passed"]} == {
        ("flexion_extension", "FCR_r"), ("flexion_extension", "FCU_r"), ("flexion_extension", "ECU_r"),
        ("radioulnar_deviation", "FCR_r")}
    assert out["index_mcp_derived_lee2015"]["n_passed"] == 10


def test_sensitivity_recorded_and_healthy():
    s = results("sensitivity.json")
    assert len(s) == 8
    for task in s.values():
        assert len(task["rows"]) == 10 and all(r["finite"] and not r["warnings"] for r in task["rows"].values())
    grip = s["power_grip_proxy"]["rows"]
    assert grip["F_max x1.2"]["metric_value"] > grip["baseline"]["metric_value"] > grip["F_max x0.8"]["metric_value"]


class _ConstantPolicy:
    def act(self, frame):
        assert frame["schema_version"]
        return {"FDP2_r": 0.5}


@needs_model
def test_extension_point_goes_through_session_validation():
    from simulation.api import BioHandSession, ControlError

    assert isinstance(_ConstantPolicy(), ControlPolicy)
    s = BioHandSession()
    s.record(0.02, every_n_steps=5, controller=as_controller(_ConstantPolicy()))
    assert s.data.ctrl[s.model.actuator("FDP2_r").id] == 0.5
    assert np.isfinite(moment_arm_matrix(s.model, s.data)).all()

    class Bad:
        def act(self, frame):
            return {"FDP2_r": 2.0}

    with pytest.raises(ControlError):
        s.record(0.01, every_n_steps=5, controller=as_controller(Bad()))
