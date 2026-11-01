"""P3: compare wrist sign conventions of MyoHand and its source (MoBL-ARMS bimanual .osim).

Method (first-order kinematics at the neutral pose, all coordinates 0):
* MoBL wrist = two rotations in series:
    proximal row about axis a (radius frame) by f(q),
    hand about axis b (proximal-row frame = radius frame at neutral) by g(q),
  where f and g are the spline / coupler functions read from the .osim file.
* For a point p on the hand (middle-fingertip site of MyoHand, expressed in the radius
  frame; the radius frames of both models coincide, verified below), the displacement for a
  small +dq is  dp = f'(0) (a x (p - c1)) + g'(0) (b x (p - c2)) * dq,
  with c1 = radiocarpal centre and c2 = midcarpal centre.
* The same displacement is computed for MyoHand by finite differences.
Both are projected on the hand frame (distal, radial, dorsal) of simulation.kinematic_semantics.

This reads the source model only; it does not modify anything.
Output: reports/P3/wrist_convention_check.json
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

from simulation.kinematic_semantics import hand_frame
from simulation.myosim_source import build_myohand

OSIM = ROOT / "third_party/simtk_mobl_arms_bimanual/Bimanual Upper Arm Model/MoBL_ARMS_bimanual_6_2_21.osim"
TIP_SITE = "MFtip_r"


def _vec(text: str | None) -> np.ndarray:
    return np.array((text or "").split(), dtype=float)


def _slope_at_zero(x: np.ndarray, y: np.ndarray) -> float:
    """Slope of the piecewise function through (x, y) at 0 (secant of the bracketing knots).

    The knots used by MoBL here are either 2-point lines or 3-point splines with a knot at 0;
    the sign of the slope (all that this check uses) is the same for spline and secant.
    """
    i = int(np.searchsorted(x, 0.0))
    lo, hi = max(i - 1, 0), min(i + 1, len(x) - 1) if x[i] == 0.0 else i
    return float((y[hi] - y[lo]) / (x[hi] - x[lo]))


def read_mobl() -> dict:
    root = ET.parse(OSIM).getroot()
    out: dict = {}
    for body in root.iter("Body"):
        if body.get("name") not in ("proximal_row_r", "hand_r"):
            continue
        cj = body.find("Joint")[0]
        out[body.get("name") + "_loc_in_parent"] = _vec(cj.findtext("location_in_parent"))
        out[body.get("name") + "_orient_in_parent"] = _vec(cj.findtext("orientation_in_parent"))
        for c in cj.iter("Coordinate"):
            out["range:" + c.get("name")] = _vec(c.findtext("range"))
        for ta in cj.iter("TransformAxis"):
            coords = (ta.findtext("coordinates") or "").split()
            if coords:
                fn = ta.find("function")[0]
                out[(body.get("name"), coords[0])] = {
                    "axis": _vec(ta.findtext("axis")), "transform_axis": ta.get("name"),
                    "function": fn.tag,
                    "x": _vec(fn.findtext("x")) if fn.find("x") is not None else None,
                    "y": _vec(fn.findtext("y")) if fn.find("y") is not None else None,
                    "slope": float(fn.findtext("coefficients").split()[0])
                    if fn.tag == "LinearFunction" else None,
                }
    for c in root.iter("CoordinateCouplerConstraint"):
        if c.get("name", "").endswith("_r") and c.get("name", "").startswith("wrist_hand"):
            fn = c.find("coupled_coordinates_function")[0]
            out[c.findtext("dependent_coordinate_name").strip()] = {
                "independent": c.findtext("independent_coordinate_names").strip(),
                "x": _vec(fn.findtext("x")), "y": _vec(fn.findtext("y"))}
    return out


def main() -> None:
    mobl = read_mobl()
    built = build_myohand("fragment")
    m = built.model
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    rid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "radius_r")
    lid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "lunate_r")
    cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "capitate_r")
    sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, TIP_SITE)
    r_rad = d.xmat[rid].reshape(3, 3)

    def to_radius(p_world: np.ndarray) -> np.ndarray:
        return r_rad.T @ (p_world - d.xpos[rid])

    checks = {
        "lunate_offset_equal": bool(np.allclose(m.body_pos[lid], mobl["proximal_row_r_loc_in_parent"], atol=1e-6)),
        "lunate_rotation_identity": bool(np.allclose(m.body_quat[lid], [1, 0, 0, 0])),
        "osim_orientations_zero": bool(np.allclose(mobl["proximal_row_r_orient_in_parent"], 0)
                                       and np.allclose(mobl["hand_r_orient_in_parent"], 0)),
    }
    frame = hand_frame(m, d)
    hand_axes_radius = np.vstack([r_rad.T @ frame.distal, r_rad.T @ frame.radial, r_rad.T @ frame.dorsal])
    p = to_radius(d.site_xpos[sid])
    c1 = mobl["proximal_row_r_loc_in_parent"]
    c2 = c1 + mobl["hand_r_loc_in_parent"]
    myohand_capitate_vs_osim_c2 = float(np.linalg.norm(to_radius(d.xpos[cid]) - c2))

    result: dict = {"tip_site": TIP_SITE, "frame_checks": checks,
                    "myohand_capitate_minus_osim_midcarpal_centre_m": round(myohand_capitate_vs_osim_c2, 6),
                    "coordinates": {}}
    for coord, dep in [("deviation_r", "wrist_hand_r1_r"), ("flexion_r", "wrist_hand_r3_r")]:
        row = mobl[("proximal_row_r", coord)]
        f1 = _slope_at_zero(row["x"], row["y"])
        cpl = mobl[dep]
        g1_coupler = _slope_at_zero(cpl["x"], cpl["y"])
        hand_ax = mobl[("hand_r", dep)]
        g1 = g1_coupler * hand_ax["slope"]
        dp_mobl = f1 * np.cross(row["axis"], p - c1) + g1 * np.cross(hand_ax["axis"], p - c2)
        # MyoHand: finite difference of the same site
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, coord)
        dq = 1e-5
        dd = mujoco.MjData(m)
        dd.qpos[m.jnt_qposadr[j]] = dq
        mujoco.mj_forward(m, dd)
        dp_myo = (r_rad.T @ (dd.site_xpos[sid] - d.site_xpos[sid])) / dq
        mob_hf, myo_hf = hand_axes_radius @ dp_mobl, hand_axes_radius @ dp_myo
        cos = float(dp_mobl @ dp_myo / (np.linalg.norm(dp_mobl) * np.linalg.norm(dp_myo)))
        result["coordinates"][coord] = {
            "mobl_proximal_row_axis_radius_frame": row["axis"].round(6).tolist(),
            "mobl_hand_axis_radius_frame": hand_ax["axis"].round(6).tolist(),
            "mobl_row_spline_slope": round(f1, 6), "mobl_hand_coupled_slope": round(g1, 6),
            "myohand_axis_radius_frame": np.round(
                d.xmat[lid].reshape(3, 3).T @ (d.xmat[lid].reshape(3, 3) @ m.jnt_axis[j]), 6).tolist(),
            "tip_velocity_per_rad_hand_frame_mobl_m": mob_hf.round(5).tolist(),
            "tip_velocity_per_rad_hand_frame_myohand_m": myo_hf.round(5).tolist(),
            "cosine_between_directions": round(cos, 4),
            "coordinate_range_rad_mobl": mobl["range:" + coord].round(6).tolist(),
            "range_rad_myohand": m.jnt_range[j].round(6).tolist(),
        }
    out = ROOT / "reports" / "P3"
    out.mkdir(parents=True, exist_ok=True)
    (out / "wrist_convention_check.json").write_text(json.dumps(result, indent=1) + "\n")
    print(json.dumps(result, indent=1))


if __name__ == "__main__":
    main()
