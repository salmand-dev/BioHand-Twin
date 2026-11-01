"""Integration tests: licence texts and traceability of MyoHand to its OpenSim sources.

Skipped when the SimTK archives have not been extracted into third_party/.
"""

import hashlib
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
LICENSES = ROOT / "data" / "provenance" / "licenses"
OSIM_2ND = ROOT / "third_party/simtk_2nd_hand/hand_model_simtk/2nd_Hand_Model_Mass.osim"
OSIM_MOBL = ROOT / "third_party/simtk_mobl_arms_bimanual/Bimanual Upper Arm Model/MoBL_ARMS_bimanual_6_2_21.osim"

needs_sources = pytest.mark.skipif(
    not (OSIM_2ND.is_file() and OSIM_MOBL.is_file()), reason="SimTK sources not extracted"
)


def test_licence_texts_archived_verbatim():
    expected = {
        "MoBL-ARMS_use_agreement.txt": "cb0b2c4b62c34d60337d869a7986aea4853425c7e6c120ceacc844dd6a1d0192",
        "2nd-Hand_MIT_license.txt": "e202a0098192dc1fc32ff867fb85baf9b30bdf3766cbd9387dfe5e4514c249f0",
    }
    for name, digest in expected.items():
        assert hashlib.sha256((LICENSES / name).read_bytes()).hexdigest() == digest
    mobl_text = (LICENSES / "MoBL-ARMS_use_agreement.txt").read_text(encoding="utf-8")
    assert "solely for non-commercial purposes" in mobl_text


@pytest.mark.integration
@needs_sources
def test_source_osim_hashes_match_manifest():
    assert hashlib.sha256(OSIM_MOBL.read_bytes()).hexdigest().startswith("a776ad04")
    assert hashlib.sha256(OSIM_2ND.read_bytes()).hexdigest().startswith("66a7b315")


@pytest.mark.integration
@needs_sources
def test_recorded_traceability_findings():
    comp = json.loads((ROOT / "reports" / "P1" / "osim_vs_myohand.json").read_text())
    mobl = {a["myohand_actuator"]: a for a in comp["mobl_arms_bimanual"]["actuators"]}
    matched = [a for a in mobl.values() if a["osim_muscle"]]
    assert len(matched) == 26
    equal = [a for a in matched if abs(a["myohand_peak_force_N"] - a["max_isometric_force"]) <= 0.5]
    assert len(equal) == 25
    # T2: EDC5 differs from MoBL EDCL_r by exactly 100 N.
    assert mobl["EDC5_r"]["myohand_peak_force_N"] - mobl["EDC5_r"]["max_isometric_force"] == pytest.approx(100.0)
    # T5: wrist flexion range narrowed from +/-70 deg (source) to +/-45 deg (MyoHand).
    hand = {j["myohand_joint"]: j for j in comp["2nd_hand_mass"]["joints"]}
    assert hand["flexion_r"]["osim_range_rad"] == pytest.approx([-1.22173048, 1.22173048])
    assert hand["flexion_r"]["myohand_range_rad"] == pytest.approx([-0.785398, 0.785398])
    assert sum(1 for j in hand.values() if j.get("range_identical_1e-4")) == 20
