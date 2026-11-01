"""Explicit, versioned BioHand patches applied to the upstream MjSpec at build time.

A patch never edits upstream files. Every patched value must be derivable from a cited source
model value (data/reference_osim_inventory.json) through the declared sign mapping; ``validate``
recomputes it and refuses mismatches, so no number can be introduced by hand.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import yaml

from simulation.myosim_source import BuiltModel, Variant, build_myohand

ROOT = Path(__file__).resolve().parents[1]
PATCH_DIR = ROOT / "data" / "patches"
REFERENCE_OSIM = ROOT / "data" / "reference_osim_inventory.json"
#: Patches that define the BioHand reference model (applied in this order).
DEFAULT_PATCHES: tuple[str, ...] = ("wrist_rom_v1", "muscle_fixes_v1", "limit_stiffness_v1", "thumb_intrinsics_v1",
                                    "passive_stiffness_v1")
TOL_RAD = 1e-6
#: Identifier of the only accepted lengthrange derivation (scripts/p4_derive_muscle_fixes.py).
LENGTHRANGE_METHOD = "geometric_minmax_over_upstream_joint_ranges_v1"
_TENDON_PROPS = ("width", "rgba", "group", "material", "damping", "stiffness", "armature",
                 "frictionloss", "limited", "range", "margin", "springlength")


class PatchError(ValueError):
    """Raised when a patch is inconsistent with its sources or the model."""


def _actuator_id(model: mujoco.MjModel, name: str) -> int:
    a = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    if a < 0:
        raise PatchError(f"unknown actuator {name!r}")
    return a


def _rebuild_tendon_without(spec: mujoco.MjSpec, tendon: str, remove: str) -> None:
    """Re-create a site-only spatial tendon without one wrap object (MjSpec has no per-element delete).

    The tendon keeps its name, so the actuator that targets it by name is unchanged.
    """
    t = spec.tendon(tendon)
    if t is None:
        raise PatchError(f"tendon {tendon!r} not in spec")
    path = [(w.type, w.target.name) for w in t.path]
    if any(kind not in (mujoco.mjtWrap.mjWRAP_SITE, mujoco.mjtWrap.mjWRAP_CYLINDER, mujoco.mjtWrap.mjWRAP_SPHERE)
           for kind, _ in path):
        raise PatchError(f"{tendon}: unsupported path element type")
    if any(kind != mujoco.mjtWrap.mjWRAP_SITE for kind, name in path if name != remove):
        raise PatchError(f"{tendon}: rebuilding only supports tendons whose remaining elements are sites")
    props = {k: getattr(t, k) for k in _TENDON_PROPS}
    spec.delete(t)
    new = spec.add_tendon(name=tendon)
    for k, v in props.items():
        setattr(new, k, v)
    for _, name in path:
        if name != remove:
            new.wrap_site(name)


@dataclass(frozen=True)
class Patch:
    patch_id: str
    data: dict[str, Any]

    @classmethod
    def load(cls, patch_id: str) -> Patch:
        data = yaml.safe_load((PATCH_DIR / f"{patch_id}.yaml").read_text(encoding="utf-8"))
        if data.get("patch_id") != patch_id:
            raise PatchError(f"patch file id {data.get('patch_id')!r} != {patch_id!r}")
        return cls(patch_id, data)

    def validate(self, upstream: mujoco.MjModel, upstream_spec: mujoco.MjSpec | None = None) -> None:
        """Refuse the patch unless every change is consistent with upstream and its declared source."""
        sources = json.loads(REFERENCE_OSIM.read_text())
        for ch in self.data["changes"]:
            field = ch["field"]
            if field == "range":
                self._validate_range(ch, upstream, sources)
            elif field == "actuator_force":
                a = _actuator_id(upstream, ch["actuator"])
                if not np.isclose(upstream.actuator_gainprm[a, 2], ch["upstream_value_N"]):
                    raise PatchError(f"{ch['actuator']}: upstream force changed; re-audit before patching")
                muscles = {m["name"]: m for m in sources[ch["source_model"]]["muscles"]}
                src = muscles[ch["source_muscle"]]["max_isometric_force"]
                if not np.isclose(src, ch["patched_value_N"]):
                    raise PatchError(f"{ch['actuator']}: patched force {ch['patched_value_N']} != source {src}")
            elif field == "tendon_remove_wrap":
                if upstream_spec is None:
                    raise PatchError("tendon_remove_wrap needs the upstream spec to validate")
                t = upstream_spec.tendon(ch["tendon"])
                if t is None:
                    raise PatchError(f"unknown tendon {ch['tendon']!r}")
                path = [w.target.name for w in t.path]
                if path != ch["upstream_path"]:
                    raise PatchError(f"{ch['tendon']}: upstream path {path} != declared {ch['upstream_path']}")
                if ch["remove"] not in path:
                    raise PatchError(f"{ch['tendon']}: {ch['remove']} not in path")
            elif field == "actuator_lengthrange":
                a = _actuator_id(upstream, ch["actuator"])
                if not np.allclose(upstream.actuator_lengthrange[a], ch["upstream_value_m"], atol=1e-6):
                    raise PatchError(f"{ch['actuator']}: upstream lengthrange changed; re-derive")
                lo, hi = ch["patched_value_m"]
                if not (0 < lo < hi) or ch.get("derivation") != LENGTHRANGE_METHOD:
                    raise PatchError(f"{ch['actuator']}: invalid derived lengthrange or unknown derivation")
            elif field == "joint_solref":
                if ch["joints"] != "all":
                    raise PatchError("joint_solref currently supports joints: all")
                if not np.allclose(upstream.jnt_solref, ch["upstream_value"]):
                    raise PatchError("upstream joint solref changed; re-audit before patching")
                expected = [2.0 * float(upstream.opt.timestep), float(ch["upstream_value"][1])]
                if not np.allclose(ch["patched_value"], expected):
                    raise PatchError(f"joint_solref {ch['patched_value']} != rule value {expected}")
            elif field == "site_shift":
                if mujoco.mj_name2id(upstream, mujoco.mjtObj.mjOBJ_SITE, ch["site"]) < 0:
                    raise PatchError(f"unknown site {ch['site']!r}")
                if max(abs(v) for v in ch["shift_m"]) > ch["max_abs_axis_m"] + 1e-9:
                    raise PatchError(f"{ch['site']}: shift exceeds the declared bound")
                if not self.data.get("classification", "").startswith("TUNED"):
                    raise PatchError("site_shift patches must be classified as TUNED")
            elif field == "add_muscle":
                self._validate_add_muscle(ch, upstream)
            elif field == "joint_stiffness":
                self._validate_joint_stiffness(ch, upstream)
            else:
                raise PatchError(f"unsupported field {field!r}")

    @staticmethod
    def _validate_range(ch: dict[str, Any], upstream: mujoco.MjModel, sources: dict[str, Any]) -> None:
        j = mujoco.mj_name2id(upstream, mujoco.mjtObj.mjOBJ_JOINT, ch["joint"])
        if j < 0:
            raise PatchError(f"unknown joint {ch['joint']!r}")
        if not np.allclose(upstream.jnt_range[j], ch["upstream_value_rad"], atol=TOL_RAD):
            raise PatchError(f"{ch['joint']}: upstream range changed; re-audit before patching")
        coords = {c["name"]: c for c in sources[ch["source_model"]]["coordinates"]}
        src = np.array(coords[ch["source_coordinate"]]["range_rad"], dtype=float)
        s = ch["sign_mapping"]
        if s not in (1, -1):
            raise PatchError("sign_mapping must be +1 or -1")
        derived = np.sort(s * src)
        if not np.allclose(derived, ch["patched_value_rad"], atol=TOL_RAD):
            raise PatchError(f"{ch['joint']}: patched value {ch['patched_value_rad']} != derived {derived}")

    @staticmethod
    def _validate_add_muscle(ch: dict[str, Any], upstream: mujoco.MjModel) -> None:
        """A new muscle must be absent upstream and its force must equal PCSA x specific tension from its source."""
        name = ch["actuator"]
        if mujoco.mj_name2id(upstream, mujoco.mjtObj.mjOBJ_ACTUATOR, name) >= 0:
            raise PatchError(f"{name}: already exists upstream")
        src = yaml.safe_load((ROOT / ch["source_file"]).read_text(encoding="utf-8"))
        rule = src["muscles"][ch["source_muscle"]]["pcsa_mm2"] * src["specific_tension"]["value_N_per_mm2"]
        if not np.isclose(rule, ch["force_N"], atol=1e-6):
            raise PatchError(f"{name}: force {ch['force_N']} != PCSA x specific tension {rule}")
        for point in ch["path"]:
            if mujoco.mj_name2id(upstream, mujoco.mjtObj.mjOBJ_BODY, point["body"]) < 0:
                raise PatchError(f"{name}: unknown body {point['body']!r}")
        lo, hi = ch["lengthrange_m"]
        if not (0 < lo < hi) or ch.get("derivation") != LENGTHRANGE_METHOD:
            raise PatchError(f"{name}: invalid derived lengthrange or unknown derivation")

    @staticmethod
    def _validate_joint_stiffness(ch: dict[str, Any], upstream: mujoco.MjModel) -> None:
        """Joint stiffness must be zero upstream, and bounded by the mean of its labelled source values."""
        j = mujoco.mj_name2id(upstream, mujoco.mjtObj.mjOBJ_JOINT, ch["joint"])
        if j < 0:
            raise PatchError(f"unknown joint {ch['joint']!r}")
        if np.any(upstream.jnt_stiffness[j] != 0.0):
            raise PatchError(f"{ch['joint']}: upstream stiffness changed; re-audit before patching")
        src = yaml.safe_load((ROOT / ch["source_file"]).read_text(encoding="utf-8"))
        row = src["joints"][ch["joint"]]
        target = float(np.mean(row["values_Nm_per_rad"]))
        if not np.isclose(target, ch["target_total_Nm_per_rad"], atol=1e-6):
            raise PatchError(f"{ch['joint']}: target {ch['target_total_Nm_per_rad']} != source {target}")
        if ch["evidence"] != row["evidence"]:
            raise PatchError(f"{ch['joint']}: evidence label differs from the source file")
        if not 0.0 <= ch["stiffness_Nm_per_rad"] <= target:
            raise PatchError(f"{ch['joint']}: joint spring must lie in [0, target total stiffness]")

    @staticmethod
    def _apply_add_muscle(ch: dict[str, Any], spec: mujoco.MjSpec) -> None:
        template = spec.actuator(ch["template_actuator"])
        tendon = spec.add_tendon(name=f"{ch['actuator'].removesuffix('_r')}_tendon_r")
        tendon.width = spec.tendon(template.target).width
        tendon.rgba = spec.tendon(template.target).rgba
        for k, point in enumerate(ch["path"], start=1):
            site = f"{ch['actuator'].removesuffix('_r')}-P{k}_r"
            s = spec.body(point["body"]).add_site(name=site, pos=point["pos_m"])
            s.size = [0.0025, 0.005, 0.005]
            s.group = 3
            tendon.wrap_site(site)
        act = spec.add_actuator(name=ch["actuator"], target=tendon.name)
        for prop in ("trntype", "dyntype", "gaintype", "biastype", "ctrllimited", "ctrlrange", "dynprm"):
            setattr(act, prop, getattr(template, prop))
        act.gainprm = template.gainprm
        act.biasprm = template.biasprm
        act.gainprm[2] = ch["force_N"]
        act.biasprm[2] = ch["force_N"]
        act.lengthrange = list(ch["lengthrange_m"])

    def apply(self, spec: mujoco.MjSpec) -> None:
        for ch in self.data["changes"]:
            field = ch["field"]
            if field == "add_muscle":
                self._apply_add_muscle(ch, spec)
                continue
            if field == "site_shift":
                site = spec.site(ch["site"])
                site.pos = [float(p + s) for p, s in zip(site.pos, ch["shift_m"], strict=True)]
                continue
            if field == "joint_stiffness":
                joint = spec.joint(ch["joint"])
                joint.stiffness[0] = ch["stiffness_Nm_per_rad"]
                continue
            if field == "range":
                joint = spec.joint(ch["joint"])
                if joint is None:
                    raise PatchError(f"joint {ch['joint']!r} not in spec")
                joint.range = list(ch["patched_value_rad"])
            elif field == "actuator_force":
                act = spec.actuator(ch["actuator"])
                act.gainprm[2] = ch["patched_value_N"]
                act.biasprm[2] = ch["patched_value_N"]
            elif field == "tendon_remove_wrap":
                _rebuild_tendon_without(spec, ch["tendon"], ch["remove"])
            elif field == "actuator_lengthrange":
                spec.actuator(ch["actuator"]).lengthrange = list(ch["patched_value_m"])
            elif field == "joint_solref":
                for joint in spec.joints:
                    joint.solref_limit = list(ch["patched_value"])


@dataclass
class PatchedModel(BuiltModel):
    patches: tuple[str, ...] = ()


def build_biohand_model(patches: tuple[str, ...] = DEFAULT_PATCHES,
                        variant: Variant = "fragment") -> PatchedModel:
    """Build the BioHand reference model: pinned upstream MyoHand + validated patches."""
    upstream = build_myohand(variant)
    loaded = [Patch.load(p) for p in patches]
    for p in loaded:
        p.validate(upstream.model, upstream.spec)
    for p in loaded:
        p.apply(upstream.spec)
    model = upstream.spec.compile()
    return PatchedModel(variant=variant, model=model, spec=upstream.spec,
                        build_warnings=upstream.build_warnings, patches=tuple(patches))


def muscle_validity_envelope(upstream: mujoco.MjModel, patched: mujoco.MjModel) -> dict[str, tuple[float, float]]:
    """Joint intervals (rad) over which upstream muscle paths were calibrated AND the joint may move.

    Upstream MyoSim muscle paths/lengthrange were calibrated for the upstream joint ranges. Where a
    patch widens a range, muscle geometry beyond the upstream range is unvalidated (P4 found path
    breaks and moment-arm reversals there). The envelope is the intersection of the two ranges,
    so both bounds are source values; nothing is tuned.
    """
    env = {}
    for j in range(patched.njnt):
        name = mujoco.mj_id2name(patched, mujoco.mjtObj.mjOBJ_JOINT, j) or str(j)
        lo = max(float(upstream.jnt_range[j][0]), float(patched.jnt_range[j][0]))
        hi = min(float(upstream.jnt_range[j][1]), float(patched.jnt_range[j][1]))
        # Only joints whose range was widened by a patch are restricted; elsewhere the muscle paths were
        # calibrated over the whole joint range (soft-limit overshoot is reported separately).
        if not np.allclose([lo, hi], patched.jnt_range[j]):
            env[name] = (lo, hi)
    return env


def outside_muscle_validity(model: mujoco.MjModel, qpos: np.ndarray,
                            envelope: dict[str, tuple[float, float]], tol_rad: float = 1e-9) -> list[str]:
    """Names of joints whose current angle lies outside the muscle-path validity envelope."""
    out = []
    for name, (lo, hi) in envelope.items():
        j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        q = float(qpos[model.jnt_qposadr[j]])
        if q < lo - tol_rad or q > hi + tol_rad:
            out.append(name)
    return out
