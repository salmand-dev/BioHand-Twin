"""BioHand Twin Python API (P6): session control, telemetry frames, exports.

MuJoCo remains the only dynamics; this API sets controls, steps, reads state and serialises it in the
versioned formats defined in ``schemas/``. Godot (P7) consumes the playback export; it never simulates.

Typical use::

    s = BioHandSession()
    s.reset({"mcp2_flexion_r": 0.5})
    s.set_controls({"FDP2_r": 0.4})
    s.step(250)
    frame = s.frame()            # dict valid against schemas/telemetry_frame.schema.json
"""

from __future__ import annotations

import csv
import hashlib
import io
import json
import platform
from collections.abc import Mapping
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from biomechanics.frames import mujoco_to_godot_pos, mujoco_to_godot_quat
from simulation.kinematics import FINGERTIP_SITES, qpos_from_angles
from simulation.muscles import muscle_state
from simulation.myosim_source import MYOSIM_PINNED_COMMIT, build_myohand
from simulation.naming import Glossary
from simulation.patches import (
    DEFAULT_PATCHES,
    build_biohand_model,
    muscle_validity_envelope,
    outside_muscle_validity,
)
from simulation.segments import rigid_segments
from simulation.tasks import contact_normal_forces, load_config, run_task

#: 1.1.0 (P7): adds optional muscle_paths_godot (additive, backward compatible).
SCHEMA_VERSION = "1.1.0"
SCHEMA_DIR = Path(__file__).resolve().parents[1] / "schemas"
LICENCE_NOTICE = ("Model derived from MoBL-ARMS (non-commercial use only; cite Saul et al. 2015 and McFarland "
                  "et al. 2019) and 2nd-Hand (MIT, Stanford 2015) via MyoSim (Apache-2.0).")
DISCLAIMER = ("Research/education software. Not a medical device and not clinically validated. Outputs are model "
              "estimates; see reports/ for limitations.")
UNITS = {"time": "s", "angle": "rad", "angular_velocity": "rad/s", "length": "m", "force": "N",
         "activation": "dimensionless 0..1", "position": "m", "quaternion_godot": "x,y,z,w"}
_INTEGRATORS = {0: "Euler", 1: "RK4", 2: "implicit", 3: "implicitfast"}
_SOLVERS = {0: "PGS", 1: "CG", 2: "Newton"}
#: Compiled-model arrays that define geometry, kinematics, actuation and solver behaviour.
_FINGERPRINT_FIELDS = (
    "body_pos", "body_quat", "body_mass", "body_inertia", "body_ipos", "body_iquat", "jnt_type", "jnt_axis",
    "jnt_pos", "jnt_range", "jnt_solref", "jnt_solimp", "dof_damping", "dof_armature", "dof_frictionloss",
    "geom_type", "geom_size", "geom_pos", "geom_quat", "site_pos", "tendon_adr", "tendon_num", "wrap_type",
    "wrap_objid", "actuator_gainprm", "actuator_biasprm", "actuator_dynprm", "actuator_lengthrange",
    "actuator_gear", "actuator_ctrlrange", "mesh_vert", "eq_data", "pair_geom1", "pair_geom2")


class ControlError(ValueError):
    """Raised for invalid control commands."""


def model_fingerprint(model: mujoco.MjModel) -> str:
    """SHA-256 over the compiled arrays that determine the simulation (plus the timestep)."""
    h = hashlib.sha256()
    for name in _FINGERPRINT_FIELDS:
        arr = np.ascontiguousarray(getattr(model, name))
        h.update(name.encode())
        h.update(str(arr.dtype).encode() + str(arr.shape).encode())
        h.update(arr.tobytes())
    h.update(np.float64(model.opt.timestep).tobytes())
    return h.hexdigest()


def _package_version() -> str:
    try:
        return version("biohand-twin")
    except PackageNotFoundError:
        return "0.0.0+unknown"


class BioHandSession:
    """A single simulation session on the BioHand reference model (optionally with a task scenario)."""

    def __init__(self, patches: tuple[str, ...] = DEFAULT_PATCHES, task_id: str | None = None) -> None:
        self.task_cfg = load_config(task_id) if task_id else None
        self._task_meta: dict[str, Any] | None
        if self.task_cfg is not None:
            from simulation.tasks import build_task_model
            self.model, self._task_meta = build_task_model(self.task_cfg)
            self.patches = tuple(self._task_meta["patches"])
        else:
            self.model = build_biohand_model(patches=patches).model
            self.patches, self._task_meta = tuple(patches), None
        self.data = mujoco.MjData(self.model)
        self.glossary = Glossary.load()
        self.envelope = muscle_validity_envelope(build_myohand("fragment").model, self.model)
        self._joint_names = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, j) or str(j)
                             for j in range(self.model.njnt)]
        self._act_names = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, a) or str(a)
                           for a in range(self.model.nu)]
        self._body_names = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, b) or str(b)
                            for b in range(self.model.nbody)]
        self.fingerprint = model_fingerprint(self.model)
        self.reset()

    # ---------------- control ----------------
    def reset(self, initial_angles_rad: Mapping[str, float] | None = None) -> None:
        """Reset time, velocities, activations and controls; set joint angles (rad, range-checked)."""
        mujoco.mj_resetData(self.model, self.data)
        angles = dict(self.task_cfg.get("initial_angles_rad", {})) if self.task_cfg else {}
        angles.update(initial_angles_rad or {})
        self.data.qpos[:] = qpos_from_angles(self.model, angles)
        mujoco.mj_forward(self.model, self.data)

    def set_controls(self, controls: Mapping[str, float] | np.ndarray) -> None:
        """Set muscle excitations in [0, 1], by actuator name (others unchanged) or as a full vector."""
        if isinstance(controls, Mapping):
            for name, value in controls.items():
                if name not in self._act_names:
                    raise ControlError(f"unknown actuator {name!r}")
                if not 0.0 <= float(value) <= 1.0:
                    raise ControlError(f"{name}: control {value} outside [0, 1]")
                self.data.ctrl[self._act_names.index(name)] = float(value)
        else:
            u = np.asarray(controls, dtype=float)
            if u.shape != (self.model.nu,) or np.any((u < 0) | (u > 1)) or not np.isfinite(u).all():
                raise ControlError(f"control vector must have shape ({self.model.nu},) with values in [0, 1]")
            self.data.ctrl[:] = u

    def step(self, n: int = 1) -> None:
        if n < 1:
            raise ValueError("n must be >= 1")
        for _ in range(n):
            mujoco.mj_step(self.model, self.data)

    @property
    def time_s(self) -> float:
        return float(self.data.time)

    # ---------------- inspection ----------------
    def inventory(self) -> dict[str, Any]:
        """Joints, actuators, motion units and validity envelope with canonical names (for UIs)."""
        m, g = self.model, self.glossary
        joints = []
        for j, name in enumerate(self._joint_names):
            e = g.entry(name)
            joints.append({"name": name, "canonical_name": e.get("canonical_name"),
                           "positive_direction": e.get("positive_direction"),
                           "negative_direction": e.get("negative_direction"),
                           "range_rad": m.jnt_range[j].tolist(),
                           "muscle_validity_envelope_rad": list(self.envelope.get(name, m.jnt_range[j]))})
        actuators = []
        for a, name in enumerate(self._act_names):
            e = g.entry(name)
            actuators.append({"name": name, "canonical_name": e.get("canonical_name"),
                              "name_status": e.get("status"), "representation": e.get("representation"),
                              "peak_force_N": float(m.actuator_gainprm[a, 2])})
        units = [{"root_body": s.root_body, "bodies": s.bodies, "joints": s.joints,
                  "movable": not s.is_world_fixed, "lumped": sum(
                      1 for b in s.bodies if g.entry(b).get("structure_type") == "bone") > 1}
                 for s in rigid_segments(m)]
        return {"schema_version": SCHEMA_VERSION, "metadata": self.metadata(), "joints": joints,
                "actuators": actuators, "motion_units": units,
                "not_represented": [e["canonical_name"] for e in g.entries("not_represented")]}

    def metadata(self) -> dict[str, Any]:
        m = self.model
        return {
            "schema_version": SCHEMA_VERSION,
            "software": {"biohand_twin": _package_version(), "python": platform.python_version(),
                         "mujoco": mujoco.__version__},
            "model": {"name": "BioHand reference model (MyoHand fragment + patches)",
                      "myosim_commit": MYOSIM_PINNED_COMMIT, "patches": list(self.patches),
                      "fingerprint_sha256": self.fingerprint, "licence_notice": LICENCE_NOTICE},
            "simulation": {"timestep_s": float(m.opt.timestep),
                           "integrator": _INTEGRATORS.get(int(m.opt.integrator), str(m.opt.integrator)),
                           "solver": _SOLVERS.get(int(m.opt.solver), str(m.opt.solver)),
                           "iterations": int(m.opt.iterations),
                           "task": None if self.task_cfg is None or self._task_meta is None else {
                               "id": self.task_cfg["id"], "fixtures": self._task_meta["fixtures"]}},
            "units": UNITS, "disclaimer": DISCLAIMER,
        }

    def frame(self, include_bodies: bool = False, include_paths: bool = False) -> dict[str, Any]:
        """Current state as a telemetry frame (schemas/telemetry_frame.schema.json)."""
        m, d = self.model, self.data
        mujoco.mj_forward(m, d)
        ms = muscle_state(m, d)
        q = d.qpos[m.jnt_qposadr]
        viol = float(np.max(np.maximum(0.0, np.maximum(m.jnt_range[:, 0] - q, q - m.jnt_range[:, 1]))))
        geoms = None
        if self.task_cfg and self.task_cfg.get("object"):
            geoms = {m.geom(self.task_cfg["object"]["name"]).id}
        elif self.task_cfg and self.task_cfg.get("contact_pairs"):
            geoms = {m.geom(p["geom1"]).id for p in self.task_cfg["contact_pairs"]}
        contact = contact_normal_forces(m, d, geoms)[0] if geoms else 0.0
        f: dict[str, Any] = {
            "schema_version": SCHEMA_VERSION, "time_s": float(d.time),
            "joints": {n: {"q_rad": float(d.qpos[m.jnt_qposadr[j]]), "qvel_rad_s": float(d.qvel[m.jnt_dofadr[j]])}
                       for j, n in enumerate(self._joint_names)},
            "muscles": {n: {"control": float(d.ctrl[a]), "activation": float(ms.activation[a]),
                            "tension_N": max(0.0, float(ms.tension_N[a])),
                            "active_tension_N": max(0.0, float(ms.active_tension_N[a])),
                            "passive_tension_N": max(0.0, float(ms.passive_tension_N[a])),
                            "length_m": float(ms.length_m[a]), "normalised_length": float(ms.normalised_length[a]),
                            "in_active_window": bool(ms.in_active_window[a])}
                        for a, n in enumerate(self._act_names)},
            "fingertips_m": {k: d.site_xpos[m.site(v).id].tolist() for k, v in FINGERTIP_SITES.items()},
            "contact_normal_N": float(contact),
            "flags": {"outside_muscle_validity_envelope": outside_muscle_validity(m, d.qpos, self.envelope),
                      "max_limit_violation_rad": viol,
                      "finite": bool(np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all())},
        }
        if include_paths:
            pts = d.wrap_xpos.reshape(-1, 3)
            f["muscle_paths_godot"] = {}
            for a, name in enumerate(self._act_names):
                t = int(m.actuator_trnid[a, 0])
                seg = pts[d.ten_wrapadr[t]:d.ten_wrapadr[t] + d.ten_wrapnum[t]]
                f["muscle_paths_godot"][name] = [mujoco_to_godot_pos(p).tolist() for p in seg]
        if include_bodies:
            f["bodies_godot"] = {
                n: {"pos": mujoco_to_godot_pos(d.xpos[b]).tolist(),
                    "quat_xyzw": mujoco_to_godot_quat(d.xquat[b]).tolist()}
                for b, n in enumerate(self._body_names) if b > 0}
        return f

    # ---------------- recording / export ----------------
    def record(self, duration_s: float, every_n_steps: int = 10, controller: Any = None,
               include_bodies: bool = False, include_paths: bool = False) -> list[dict[str, Any]]:
        """Step for ``duration_s`` and return frames every ``every_n_steps`` (frame 0 = current state).

        ``controller(session) -> None`` may set controls before each step.
        """
        frames = [self.frame(include_bodies, include_paths)]
        n = round(duration_s / self.model.opt.timestep)
        for k in range(n):
            if controller is not None:
                controller(self)
            self.step()
            if (k + 1) % every_n_steps == 0:
                frames.append(self.frame(include_bodies, include_paths))
        return frames

    def export_json(self, frames: list[dict[str, Any]]) -> str:
        return json.dumps({"schema_version": SCHEMA_VERSION, "metadata": self.metadata(), "frames": frames},
                          indent=None, separators=(",", ":"))

    def export_csv(self, frames: list[dict[str, Any]]) -> str:
        """Flat CSV: metadata header lines ('# key: value'), unit-suffixed columns."""
        buf = io.StringIO()
        meta = self.metadata()
        for line in (f"schema_version: {SCHEMA_VERSION}", f"myosim_commit: {MYOSIM_PINNED_COMMIT}",
                     f"patches: {','.join(self.patches)}", f"model_fingerprint_sha256: {self.fingerprint}",
                     f"timestep_s: {meta['simulation']['timestep_s']}", f"mujoco: {mujoco.__version__}",
                     f"task: {self.task_cfg['id'] if self.task_cfg else 'none'}",
                     "units: column suffix (_s, _rad, _rad_s, _N, _m); activation/control dimensionless 0..1",
                     f"licence: {LICENCE_NOTICE}", f"disclaimer: {DISCLAIMER}"):
            buf.write(f"# {line}\n")
        cols = ["time_s"]
        cols += [f"q_{j}_rad" for j in self._joint_names] + [f"qvel_{j}_rad_s" for j in self._joint_names]
        for a in self._act_names:
            cols += [f"ctrl_{a}", f"act_{a}", f"tension_{a}_N", f"len_{a}_m"]
        cols += [f"tip_{k}_{ax}_m" for k in FINGERTIP_SITES for ax in "xyz"]
        cols += ["contact_normal_N", "max_limit_violation_rad", "n_outside_envelope"]
        w = csv.writer(buf, lineterminator="\n")
        w.writerow(cols)
        for f in frames:
            row: list[Any] = [f["time_s"]]
            row += [f["joints"][j]["q_rad"] for j in self._joint_names]
            row += [f["joints"][j]["qvel_rad_s"] for j in self._joint_names]
            for a in self._act_names:
                mm = f["muscles"][a]
                row += [mm["control"], mm["activation"], mm["tension_N"], mm["length_m"]]
            row += [v for k in FINGERTIP_SITES for v in f["fingertips_m"][k]]
            row += [f["contact_normal_N"], f["flags"]["max_limit_violation_rad"],
                    len(f["flags"]["outside_muscle_validity_envelope"])]
            w.writerow([f"{v:.9g}" if isinstance(v, float) else v for v in row])
        return buf.getvalue()

    def run_task(self) -> Any:
        """Run this session's task with the P5 runner (returns simulation.tasks.TaskResult)."""
        if self.task_cfg is None:
            raise ValueError("session has no task")
        return run_task(self.task_cfg)


def load_schema(name: str) -> dict[str, Any]:
    return json.loads((SCHEMA_DIR / f"{name}.schema.json").read_text(encoding="utf-8"))


def simulate_task(task_id: str, every_n_steps: int = 10, include_bodies: bool = True,
                  include_paths: bool = False) -> tuple[BioHandSession, list[dict[str, Any]]]:
    """Run a configured task through the API (same control policy as the P5 runner) and return frames."""
    import yaml

    from simulation.tasks import GROUPS, PoseTracker, control_at

    session = BioHandSession(task_id=task_id)
    cfg = session.task_cfg
    assert cfg is not None
    groups = yaml.safe_load(GROUPS.read_text(encoding="utf-8"))
    policy = cfg.get("policy", {"type": "open_loop"})
    tracker = PoseTracker(session.model, policy, cfg) if policy["type"] == "pose_tracking_static_opt" else None

    def controller(s: BioHandSession) -> None:
        u = tracker.control(s.data) if tracker is not None else control_at(s.data.time, cfg, groups, s.model)
        s.set_controls(u)

    frames = session.record(cfg["duration_s"], every_n_steps, controller, include_bodies, include_paths)
    return session, frames
