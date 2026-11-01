"""P3: per-joint axis and sign comparison of MyoHand against 2nd-Hand (Lee 2015) source .osim.

At the neutral pose (all coordinates 0) every 2nd-Hand body from the radius distally has zero
orientation_in_parent and zero rotation, so all its TransformAxis axes are expressed in a frame
parallel to the radius frame (checked below). For each MyoHand joint we compute:
    effective source rotation vector  = sum_k  f_k'(0) * axis_k   (all transform axes driven by
                                                                     the coordinate, incl. couplers)
    MyoHand rotation vector           = joint axis expressed in the radius frame
and report the cosine between them. cos ~ +1: same convention; cos ~ -1: sign inverted;
|cos| small: different axis. Ranges are compared as well.

Output: reports/P3/axis_convention_check.json
"""

from __future__ import annotations

import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from simulation.myosim_source import build_myohand

OSIM = ROOT / "third_party/simtk_2nd_hand/hand_model_simtk/2nd_Hand_Model_Mass.osim"
DISTAL_OF_RADIUS = {
    "radius", "lunate", "scaphoid", "pisiform", "triquetrum", "capitate", "trapezium", "trapezoid",
    "hamate", "firstmc1", "firstmc", "secondmc", "thirdmc", "fourthmc", "fifthmc", "proximal_thumb",
    "distal_thumb", *[f"{p}{d}" for p in ("proxph", "midph", "distph") for d in (2, 3, 4, 5)]}


def _vec(t: str | None) -> np.ndarray:
    return np.array((t or "").split(), dtype=float)


def _slope0(fn: ET.Element) -> float:
    if fn.tag == "LinearFunction":
        return float(fn.findtext("coefficients").split()[0])
    if fn.tag == "Constant":
        return 0.0
    x, y = _vec(fn.findtext("x")), _vec(fn.findtext("y"))
    i = int(np.argmin(np.abs(x)))
    lo, hi = (i - 1, i + 1) if 0 < i < len(x) - 1 and x[i] == 0 else (max(i - 1, 0), min(i + 1, len(x) - 1))
    if lo == hi:
        lo, hi = 0, len(x) - 1
    return float((y[hi] - y[lo]) / (x[hi] - x[lo]))


def parse() -> tuple[dict[str, np.ndarray], dict[str, np.ndarray], list[str]]:
    root = ET.parse(OSIM).getroot()
    rot: dict[str, np.ndarray] = {}
    ranges: dict[str, np.ndarray] = {}
    notes = []
    couplers = {c.findtext("dependent_coordinate_name").strip():
                (c.findtext("independent_coordinate_names").strip(), c.find("coupled_coordinates_function")[0])
                for c in root.iter("CoordinateCouplerConstraint")}
    for body in root.iter("Body"):
        name = body.get("name")
        if name not in DISTAL_OF_RADIUS:
            continue
        cj = body.find("Joint")[0]
        if np.any(_vec(cj.findtext("orientation_in_parent")) != 0):
            notes.append(f"{name}: non-zero orientation_in_parent - frame assumption violated")
        for c in cj.iter("Coordinate"):
            ranges[c.get("name")] = _vec(c.findtext("range"))
        for ta in cj.iter("TransformAxis"):
            if not ta.get("name", "").startswith("rotation"):
                continue
            coords = (ta.findtext("coordinates") or "").split()
            fn_el = ta.find("function")
            if not coords or fn_el is None or not len(fn_el):
                continue
            coord, slope = coords[0], _slope0(fn_el[0])
            if coord in couplers:  # dependent coordinate: chain rule through the coupler
                indep, cfn = couplers[coord]
                coord, slope = indep, slope * _slope0(cfn)
            rot[coord] = rot.get(coord, np.zeros(3)) + slope * _vec(ta.findtext("axis"))
    return rot, ranges, notes


def main() -> None:
    rot, ranges, notes = parse()
    m = build_myohand("fragment").model
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    r_rad = d.xmat[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "radius_r")].reshape(3, 3)
    rows = []
    for j in range(m.njnt):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)
        base = name.removesuffix("_r")
        if base == "pro_sup":  # radius is the reference frame itself; compare in ulna frame instead
            continue
        mj_axis = r_rad.T @ (d.xmat[m.jnt_bodyid[j]].reshape(3, 3) @ m.jnt_axis[j])
        src = rot.get(base)
        row = {"joint": name, "myohand_axis_radius_frame": np.round(mj_axis, 4).tolist(),
               "source_rotation_vector_radius_frame": None if src is None else np.round(src, 4).tolist(),
               "myohand_range_deg": np.round(np.degrees(m.jnt_range[j]), 1).tolist(),
               "source_range_deg": None if base not in ranges else np.round(np.degrees(ranges[base]), 1).tolist()}
        if src is not None and np.linalg.norm(src) > 0:
            cos = float(mj_axis @ src / np.linalg.norm(src))
            row["cosine"] = round(cos, 4)
            row["verdict"] = ("same convention" if cos > 0.9 else "SIGN INVERTED" if cos < -0.9
                              else "different axis (|cos| <= 0.9)")
            row["ranges_equal"] = bool(np.allclose(m.jnt_range[j], ranges[base], atol=1e-4))
        rows.append(row)
    out = ROOT / "reports" / "P3"
    out.mkdir(parents=True, exist_ok=True)
    (out / "axis_convention_check.json").write_text(json.dumps(
        {"source": OSIM.relative_to(ROOT).as_posix(), "frame_notes": notes, "joints": rows}, indent=1) + "\n")
    for r in rows:
        print(f"{r['joint']:17s} cos={r.get('cosine')!s:8s} {r.get('verdict', '-'):30s} "
              f"range MH {r['myohand_range_deg']} src {r['source_range_deg']}")
    print("frame notes:", notes or "none")


if __name__ == "__main__":
    main()
