"""Machine-readable inventory of a compiled MuJoCo model.

Everything reported here is read from the compiled ``MjModel`` (``source:
"upstream_mjcf"``) or computed from it by a stated numerical procedure
(``source: "derived"``). Nothing is looked up from literature or estimated.
Units are SI (m, kg, s, N) and radians.
"""

from __future__ import annotations

from typing import Any

import mujoco
import numpy as np

JOINT_TYPES = {0: "free", 1: "ball", 2: "slide", 3: "hinge"}
GEOM_TYPES = {
    0: "plane", 1: "hfield", 2: "sphere", 3: "capsule", 4: "ellipsoid",
    5: "cylinder", 6: "box", 7: "mesh", 8: "sdf",
}
WRAP_TYPES = {0: "none", 1: "joint", 2: "pulley", 3: "site", 4: "sphere", 5: "cylinder"}
TRN_TYPES = {0: "joint", 1: "jointinparent", 2: "slidercrank", 3: "tendon", 4: "site", 5: "body"}
DYN_TYPES = {0: "none", 1: "integrator", 2: "filter", 3: "filterexact", 4: "muscle", 5: "user"}
GAIN_TYPES = {0: "fixed", 1: "affine", 2: "muscle", 3: "user"}
BIAS_TYPES = {0: "none", 1: "affine", 2: "muscle", 3: "user"}
EQ_TYPES = {0: "connect", 1: "weld", 2: "joint", 3: "tendon", 4: "flex", 5: "flexvert", 6: "distance"}

# Tolerance below which a finite-difference dL/dq is treated as "does not span".
SPAN_TOLERANCE_M = 1e-6
FD_STEP_RAD = 1e-6


def _name(model: mujoco.MjModel, obj: mujoco.mjtObj, idx: int) -> str | None:
    return mujoco.mj_id2name(model, obj, idx) if idx >= 0 else None


def _vec(a: np.ndarray, digits: int = 9) -> list[float]:
    return [round(float(x), digits) for x in np.asarray(a).ravel()]


def tendon_length_jacobian(model: mujoco.MjModel, qpos: np.ndarray) -> np.ndarray:
    """Return dL/dq (shape ntendon x nq) by central finite differences.

    Only valid for models whose joints are all hinge/slide (nq == nv), which is
    checked. MuJoCo convention: actuator moment = gear * dL/dq, so a negative
    value means tendon shortening for positive joint rotation.
    """
    if model.nq != model.nv or any(t not in (2, 3) for t in model.jnt_type):
        raise ValueError("Finite-difference Jacobian requires hinge/slide joints only")
    data = mujoco.MjData(model)
    jac = np.zeros((model.ntendon, model.nq))
    for j in range(model.nq):
        lengths = []
        for sign in (1.0, -1.0):
            data.qpos[:] = qpos
            data.qpos[j] += sign * FD_STEP_RAD
            mujoco.mj_fwdPosition(model, data)
            lengths.append(data.ten_length.copy())
        jac[:, j] = (lengths[0] - lengths[1]) / (2 * FD_STEP_RAD)
    return jac


def _bodies(model: mujoco.MjModel, spec: mujoco.MjSpec | None) -> list[dict[str, Any]]:
    explicit: dict[str, bool] = {}
    if spec is not None:
        for b in spec.bodies:
            if b.name:
                explicit[b.name] = bool(getattr(b, "explicitinertial", False))
    floor_mass = 0.001
    out = []
    for i in range(model.nbody):
        name = _name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        jnts = [
            _name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
            for j in range(model.njnt) if model.jnt_bodyid[j] == i
        ]
        geoms = [g for g in range(model.ngeom) if model.geom_bodyid[g] == i]
        meshes = sorted({
            _name(model, mujoco.mjtObj.mjOBJ_MESH, int(model.geom_dataid[g])) or ""
            for g in geoms if model.geom_type[g] == 7
        })
        out.append({
            "id": i,
            "name": name,
            "parent": _name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.body_parentid[i])),
            "joints": jnts,
            "is_welded_to_parent": i > 0 and not jnts,
            "pos_in_parent_m": _vec(model.body_pos[i]),
            "quat_in_parent_wxyz": _vec(model.body_quat[i]),
            "mass_kg": float(model.body_mass[i]),
            "principal_inertia_kgm2": _vec(model.body_inertia[i], 12),
            "inertial_specified_explicitly": explicit.get(name or "", None),
            "mass_equals_compiler_floor": bool(np.isclose(model.body_mass[i], floor_mass)),
            "n_geoms": len(geoms),
            "meshes": meshes,
            "source": "upstream_mjcf",
        })
    return out


def _joints(model: mujoco.MjModel) -> list[dict[str, Any]]:
    out = []
    for j in range(model.njnt):
        out.append({
            "id": j,
            "name": _name(model, mujoco.mjtObj.mjOBJ_JOINT, j),
            "type": JOINT_TYPES[int(model.jnt_type[j])],
            "body": _name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.jnt_bodyid[j])),
            "qpos_adr": int(model.jnt_qposadr[j]),
            "dof_adr": int(model.jnt_dofadr[j]),
            "axis_body_frame": _vec(model.jnt_axis[j]),
            "anchor_body_frame_m": _vec(model.jnt_pos[j]),
            "limited": bool(model.jnt_limited[j]),
            "range_rad": _vec(model.jnt_range[j]),
            "range_deg_display_only": _vec(np.degrees(model.jnt_range[j]), 3),
            "qpos0_rad": float(model.qpos0[model.jnt_qposadr[j]]),
            "stiffness_Nm_per_rad": float(model.jnt_stiffness[j]),
            "springref_rad": float(model.qpos_spring[model.jnt_qposadr[j]]),
            "damping_Nms_per_rad": float(model.dof_damping[model.jnt_dofadr[j]]),
            "armature_kgm2": float(model.dof_armature[model.jnt_dofadr[j]]),
            "frictionloss_Nm": float(model.dof_frictionloss[model.jnt_dofadr[j]]),
            "source": "upstream_mjcf",
        })
    return out


def _geoms(model: mujoco.MjModel) -> list[dict[str, Any]]:
    out = []
    for g in range(model.ngeom):
        gtype = GEOM_TYPES.get(int(model.geom_type[g]), str(model.geom_type[g]))
        out.append({
            "id": g,
            "name": _name(model, mujoco.mjtObj.mjOBJ_GEOM, g),
            "body": _name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[g])),
            "type": gtype,
            "mesh": _name(model, mujoco.mjtObj.mjOBJ_MESH, int(model.geom_dataid[g]))
            if gtype == "mesh" else None,
            "group": int(model.geom_group[g]),
            "contype": int(model.geom_contype[g]),
            "conaffinity": int(model.geom_conaffinity[g]),
            "size_m": _vec(model.geom_size[g]),
        })
    return out


def _sites(model: mujoco.MjModel) -> list[dict[str, Any]]:
    return [
        {
            "id": s,
            "name": _name(model, mujoco.mjtObj.mjOBJ_SITE, s),
            "body": _name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.site_bodyid[s])),
            "pos_body_frame_m": _vec(model.site_pos[s]),
        }
        for s in range(model.nsite)
    ]


def _tendons(model: mujoco.MjModel, jac: np.ndarray | None) -> list[dict[str, Any]]:
    out = []
    data = mujoco.MjData(model)
    mujoco.mj_fwdPosition(model, data)
    for t in range(model.ntendon):
        adr, num = int(model.tendon_adr[t]), int(model.tendon_num[t])
        path = []
        for w in range(adr, adr + num):
            wtype = WRAP_TYPES[int(model.wrap_type[w])]
            obj = int(model.wrap_objid[w])
            if wtype == "site":
                target = _name(model, mujoco.mjtObj.mjOBJ_SITE, obj)
                body = _name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.site_bodyid[obj]))
            elif wtype in ("sphere", "cylinder"):
                target = _name(model, mujoco.mjtObj.mjOBJ_GEOM, obj)
                body = _name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[obj]))
            elif wtype == "joint":
                target = _name(model, mujoco.mjtObj.mjOBJ_JOINT, obj)
                body = None
            else:
                target, body = None, None
            path.append({"type": wtype, "target": target, "body": body})
        site_bodies = [p["body"] for p in path if p["type"] == "site"]
        entry: dict[str, Any] = {
            "id": t,
            "name": _name(model, mujoco.mjtObj.mjOBJ_TENDON, t),
            "kind": "spatial" if any(p["type"] == "site" for p in path) else "fixed",
            "n_path_elements": num,
            "path": path,
            "first_site_body": site_bodies[0] if site_bodies else None,
            "last_site_body": site_bodies[-1] if site_bodies else None,
            "wrap_geoms": sorted({p["target"] for p in path if p["type"] in ("sphere", "cylinder")}),
            "length_at_qpos0_m": float(data.ten_length[t]),
            "stiffness_N_per_m": _vec(np.atleast_1d(model.tendon_stiffness[t])),
            "damping_Ns_per_m": _vec(np.atleast_1d(model.tendon_damping[t])),
            "limited": bool(model.tendon_limited[t]),
            "source": "upstream_mjcf",
        }
        if jac is not None:
            spans = {
                _name(model, mujoco.mjtObj.mjOBJ_JOINT, j): round(float(jac[t, model.jnt_qposadr[j]]), 9)
                for j in range(model.njnt)
                if abs(jac[t, model.jnt_qposadr[j]]) > SPAN_TOLERANCE_M
            }
            entry["dL_dq_at_qpos0_m_per_rad"] = spans
            entry["dL_dq_source"] = (
                f"derived: central finite difference, step {FD_STEP_RAD} rad, "
                f"|dL/dq| > {SPAN_TOLERANCE_M} m/rad, evaluated at model.qpos0 only"
            )
        out.append(entry)
    return out


def _actuators(model: mujoco.MjModel) -> list[dict[str, Any]]:
    out = []
    for a in range(model.nu):
        trn = TRN_TYPES[int(model.actuator_trntype[a])]
        target_id = int(model.actuator_trnid[a, 0])
        target_obj = {
            "tendon": mujoco.mjtObj.mjOBJ_TENDON,
            "joint": mujoco.mjtObj.mjOBJ_JOINT,
            "site": mujoco.mjtObj.mjOBJ_SITE,
        }.get(trn)
        gain = GAIN_TYPES[int(model.actuator_gaintype[a])]
        entry: dict[str, Any] = {
            "id": a,
            "name": _name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, a),
            "transmission": trn,
            "target": _name(model, target_obj, target_id) if target_obj is not None else None,
            "gear": float(model.actuator_gear[a, 0]),
            "dyntype": DYN_TYPES[int(model.actuator_dyntype[a])],
            "gaintype": gain,
            "biastype": BIAS_TYPES[int(model.actuator_biastype[a])],
            "ctrlrange": _vec(model.actuator_ctrlrange[a]),
            "ctrllimited": bool(model.actuator_ctrllimited[a]),
            "lengthrange_m": _vec(model.actuator_lengthrange[a]),
            "raw_dynprm": _vec(model.actuator_dynprm[a, :3]),
            "raw_gainprm": _vec(model.actuator_gainprm[a, :9]),
            "raw_biasprm": _vec(model.actuator_biasprm[a, :9]),
            "source": "upstream_mjcf",
        }
        if gain == "muscle":
            gp = model.actuator_gainprm[a]
            lr = model.actuator_lengthrange[a]
            l0 = (lr[1] - lr[0]) / (gp[1] - gp[0])
            entry["mujoco_muscle_params"] = {
                "tau_activation_s": float(model.actuator_dynprm[a, 0]),
                "tau_deactivation_s": float(model.actuator_dynprm[a, 1]),
                "operating_range_normalized_length": _vec(gp[0:2]),
                "peak_active_force_N": float(gp[2]),
                "scale": float(gp[3]),
                "lmin_normalized": float(gp[4]),
                "lmax_normalized": float(gp[5]),
                "vmax_L0_per_s": float(gp[6]),
                "fpmax_normalized": float(gp[7]),
                "fvmax_normalized": float(gp[8]),
                "note": "MuJoCo built-in muscle model parameters as compiled; "
                "these are the converted/optimized MyoSim values, not original OpenSim values.",
            }
            entry["derived_L0_equivalent_m"] = {
                "value": float(l0),
                "formula": "L0 = (lengthrange[1]-lengthrange[0]) / (range[1]-range[0]) "
                "(MuJoCo muscle model normalization)",
                "source": "derived",
                "caveat": "Not an anatomical optimal fibre length measurement.",
            }
        out.append(entry)
    return out


def _sensors(model: mujoco.MjModel) -> list[dict[str, Any]]:
    return [
        {
            "id": s,
            "name": _name(model, mujoco.mjtObj.mjOBJ_SENSOR, s),
            "type": int(model.sensor_type[s]),
            "dim": int(model.sensor_dim[s]),
        }
        for s in range(model.nsensor)
    ]


def _equalities(model: mujoco.MjModel) -> list[dict[str, Any]]:
    return [
        {
            "id": e,
            "name": _name(model, mujoco.mjtObj.mjOBJ_EQUALITY, e),
            "type": EQ_TYPES.get(int(model.eq_type[e]), str(model.eq_type[e])),
            "active_at_init": bool(model.eq_active0[e]),
        }
        for e in range(model.neq)
    ]


def _contacts(model: mujoco.MjModel) -> dict[str, Any]:
    pairs = [
        {
            "name": _name(model, mujoco.mjtObj.mjOBJ_PAIR, p),
            "geom1": _name(model, mujoco.mjtObj.mjOBJ_GEOM, int(model.pair_geom1[p])),
            "geom2": _name(model, mujoco.mjtObj.mjOBJ_GEOM, int(model.pair_geom2[p])),
            "condim": int(model.pair_dim[p]),
        }
        for p in range(model.npair)
    ]
    excludes = []
    for e in range(model.nexclude):
        signature = int(model.exclude_signature[e])
        b1, b2 = signature >> 16, signature & 0xFFFF
        excludes.append({
            "body1": _name(model, mujoco.mjtObj.mjOBJ_BODY, b1),
            "body2": _name(model, mujoco.mjtObj.mjOBJ_BODY, b2),
        })
    collision_geoms = [
        _name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or f"<unnamed geom {g} on "
        f"{_name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[g]))}>"
        for g in range(model.ngeom)
        if model.geom_contype[g] or model.geom_conaffinity[g]
    ]
    return {"pairs": pairs, "excludes": excludes, "collision_enabled_geoms": collision_geoms}


def counts(model: mujoco.MjModel) -> dict[str, int]:
    """Return the element counts reported by the compiled model."""
    keys = [
        "nq", "nv", "nu", "na", "nbody", "njnt", "ngeom", "nsite", "nmesh", "ntendon",
        "nwrap", "nsensor", "neq", "npair", "nexclude", "nkey",
    ]
    return {k: int(getattr(model, k)) for k in keys}


def build_inventory(model: mujoco.MjModel, spec: mujoco.MjSpec | None = None) -> dict[str, Any]:
    """Build the full inventory dictionary for ``model``."""
    try:
        jac = tendon_length_jacobian(model, model.qpos0.copy())
    except ValueError:
        jac = None
    opt = model.opt
    return {
        "counts": counts(model),
        "options": {
            "timestep_s": float(opt.timestep),
            "integrator": int(opt.integrator),
            "gravity_m_s2": _vec(opt.gravity),
            "iterations": int(opt.iterations),
            "solver": int(opt.solver),
            "cone": int(opt.cone),
        },
        "bodies": _bodies(model, spec),
        "joints": _joints(model),
        "dofs": [
            {"dof": d, "joint": _name(model, mujoco.mjtObj.mjOBJ_JOINT, int(model.dof_jntid[d]))}
            for d in range(model.nv)
        ],
        "actuators": _actuators(model),
        "tendons": _tendons(model, jac),
        "sites": _sites(model),
        "geoms": _geoms(model),
        "meshes": [
            {
                "name": _name(model, mujoco.mjtObj.mjOBJ_MESH, m),
                "n_vertices": int(model.mesh_vertnum[m]),
                "n_faces": int(model.mesh_facenum[m]),
            }
            for m in range(model.nmesh)
        ],
        "sensors": _sensors(model),
        "equality_constraints": _equalities(model),
        "contacts": _contacts(model),
    }
