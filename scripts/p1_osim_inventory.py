"""Parse the upstream OpenSim source models and compare them with MyoHand.

Reads (never modifies) the SimTK archives extracted into third_party/:
  third_party/simtk_2nd_hand/hand_model_simtk/2nd_Hand_Model_Mass.osim
  third_party/simtk_2nd_hand/hand_model_simtk/2nd_Hand_Model_An_attachments.osim
  third_party/simtk_mobl_arms_bimanual/Bimanual Upper Arm Model/MoBL_ARMS_bimanual_6_2_21.osim

Outputs:
  data/reference_osim_inventory.json      bodies/coordinates/muscles/couplers per .osim
  reports/P1/osim_vs_myohand.json         joint-range, muscle and mass comparison

Usage: .venv/Scripts/python.exe scripts/p1_osim_inventory.py
"""

from __future__ import annotations

import datetime as dt
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
TP = ROOT / "third_party"
OSIM_FILES = {
    "2nd_hand_mass": TP / "simtk_2nd_hand/hand_model_simtk/2nd_Hand_Model_Mass.osim",
    "2nd_hand_an_attachments": TP / "simtk_2nd_hand/hand_model_simtk/2nd_Hand_Model_An_attachments.osim",
    "mobl_arms_bimanual": TP / "simtk_mobl_arms_bimanual/Bimanual Upper Arm Model/MoBL_ARMS_bimanual_6_2_21.osim",
}
MUSCLE_FIELDS = [
    "max_isometric_force", "optimal_fiber_length", "tendon_slack_length",
    "pennation_angle_at_optimal", "max_contraction_velocity",
    "activation_time_constant", "deactivation_time_constant",
]


def _text(el: ET.Element | None) -> str | None:
    return el.text.strip() if el is not None and el.text is not None else None


def _floats(el: ET.Element | None) -> list[float] | None:
    t = _text(el)
    return [float(x) for x in t.split()] if t else None


def _float(el: ET.Element | None) -> float | None:
    v = _floats(el)
    return v[0] if v else None


def parse_osim(path: Path) -> dict[str, Any]:
    root = ET.parse(path).getroot()
    model = root.find("Model")
    assert model is not None, path
    out: dict[str, Any] = {
        "file": path.relative_to(ROOT).as_posix(),
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "opensim_document_version": root.get("Version"),
        "model_name": model.get("name"),
        "credits": _text(model.find("credits")),
        "publications": _text(model.find("publications")),
        "length_units": _text(model.find("length_units")),
        "force_units": _text(model.find("force_units")),
        "gravity": _floats(model.find("gravity")),
        "bodies": [], "coordinates": [], "muscles": [], "coupler_constraints": [],
    }
    for body in model.iter("Body"):
        joint_el = body.find("Joint")
        jtype = jname = parent = None
        if joint_el is not None and len(joint_el):
            j = joint_el[0]
            jtype, jname, parent = j.tag, j.get("name"), _text(j.find("parent_body"))
            for c in j.iter("Coordinate"):
                out["coordinates"].append({
                    "name": c.get("name"), "joint": jname, "body": body.get("name"),
                    "range_rad": _floats(c.find("range")),
                    "default_value": _float(c.find("default_value")),
                    "locked": _text(c.find("locked")), "clamped": _text(c.find("clamped")),
                })
        inertia = [_float(body.find(f"inertia_{k}")) for k in ("xx", "yy", "zz", "xy", "xz", "yz")]
        out["bodies"].append({
            "name": body.get("name"), "parent": parent, "joint": jname, "joint_type": jtype,
            "mass_kg": _float(body.find("mass")),
            "mass_center_m": _floats(body.find("mass_center")),
            "inertia_kgm2_xx_yy_zz_xy_xz_yz": inertia,
        })
    defaults = model.find("defaults")
    default_ids = {id(e) for e in defaults.iter()} if defaults is not None else set()
    for el in model.iter():
        if id(el) in default_ids:
            continue
        if el.tag.endswith("Muscle") or el.tag.endswith("Muscle_Deprecated"):
            m: dict[str, Any] = {"name": el.get("name"), "type": el.tag}
            for f in MUSCLE_FIELDS:
                m[f] = _float(el.find(f))
            m["n_path_points"] = sum(
                1 for p in el.iter() if p.tag in ("PathPoint", "ConditionalPathPoint", "MovingPathPoint")
            )
            out["muscles"].append(m)
        elif el.tag == "CoordinateCouplerConstraint":
            fn = el.find("coupled_coordinates_function")
            ftype = fn[0].tag if fn is not None and len(fn) else None
            out["coupler_constraints"].append({
                "name": el.get("name"),
                "disabled": _text(el.find("isDisabled")),
                "dependent": _text(el.find("dependent_coordinate_name")),
                "independent": _text(el.find("independent_coordinate_names")),
                "function": ftype,
            })
    out["counts"] = {k: len(out[k]) for k in ("bodies", "coordinates", "muscles", "coupler_constraints")}
    return out


# MoBL names finger compartments by digit letter (I=index, M=middle, R=ring, L=little);
# MyoHand uses digit numbers 2-5. This is a naming-convention mapping, not a data change.
DIGIT_LETTER = {"I": "2", "M": "3", "R": "4", "L": "5"}


def _normalise(name: str) -> str:
    base = name.removesuffix("_r")
    for prefix in ("FDS", "FDP", "EDC"):
        if base.startswith(prefix) and base[len(prefix):] in DIGIT_LETTER:
            return prefix + DIGIT_LETTER[base[len(prefix):]]
    return base


def compare(osim: dict[str, Any], inv: dict[str, Any]) -> dict[str, Any]:
    coords = {c["name"].removesuffix("_r"): c for c in osim["coordinates"]
              if not c["name"].endswith("_l")}
    muscles = {_normalise(m["name"]): m for m in osim["muscles"] if not m["name"].endswith("_l")}
    joints = []
    for j in inv["joints"]:
        base = j["name"].removesuffix("_r")
        c = coords.get(base)
        row = {"myohand_joint": j["name"], "myohand_range_rad": j["range_rad"],
               "osim_coordinate": c["name"] if c else None,
               "osim_range_rad": c["range_rad"] if c else None}
        if c and c["range_rad"]:
            row["range_identical_1e-4"] = all(
                abs(a - b) < 1e-4 for a, b in zip(j["range_rad"], c["range_rad"], strict=True))
        joints.append(row)
    acts = []
    for a in inv["actuators"]:
        base = a["name"].removesuffix("_r")
        m = muscles.get(base)
        f_mj = a["mujoco_muscle_params"]["peak_active_force_N"]
        row = {"myohand_actuator": a["name"], "myohand_peak_force_N": f_mj,
               "osim_muscle": m["name"] if m else None}
        if m:
            row.update({k: m[k] for k in MUSCLE_FIELDS})
            if m["max_isometric_force"]:
                row["force_ratio_myohand_over_osim"] = round(f_mj / m["max_isometric_force"], 4)
        acts.append(row)
    return {
        "joints": joints, "actuators": acts,
        "osim_muscles_not_in_myohand": sorted(
            set(muscles) - {a["name"].removesuffix("_r") for a in inv["actuators"]}),
        "osim_coupler_constraints": osim["counts"]["coupler_constraints"],
        "myohand_equality_constraints": inv["counts"]["neq"],
    }


def main() -> None:
    parsed = {k: parse_osim(p) for k, p in OSIM_FILES.items() if p.is_file()}
    missing = [k for k, p in OSIM_FILES.items() if not p.is_file()]
    now = dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds")
    (ROOT / "data" / "reference_osim_inventory.json").write_text(
        json.dumps({"_meta": {"generated_utc": now, "missing_files": missing}, **parsed}, indent=1))
    inv = json.loads((ROOT / "data" / "model_inventory.json").read_text())
    comp = {"_meta": {"generated_utc": now, "myohand_commit": inv["_meta"]["upstream"]["commit"]},
            **{k: compare(v, inv) for k, v in parsed.items()}}
    (ROOT / "reports" / "P1" / "osim_vs_myohand.json").write_text(json.dumps(comp, indent=1))
    for k, v in parsed.items():
        c = comp[k]
        same = sum(1 for j in c["joints"] if j.get("range_identical_1e-4"))
        found = sum(1 for j in c["joints"] if j["osim_coordinate"])
        mfound = sum(1 for a in c["actuators"] if a["osim_muscle"])
        print(f"{k}: {v['counts']} | joints matched by name {found}/23, identical range {same} "
              f"| actuators matched {mfound}/39 | muscle types {sorted({m['type'] for m in v['muscles']})}")
    if missing:
        print("missing:", missing)


if __name__ == "__main__":
    main()
