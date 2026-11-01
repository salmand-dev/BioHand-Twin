"""P1 reference-model audit: build MyoHand from the pinned MyoSim commit and record evidence.

Outputs (overwritten on each run, deterministic apart from timestamps):
  data/model_inventory.json                 full inventory of the primary variant
  data/provenance/myohand_file_hashes.json  SHA-256 of every upstream file used
  reports/P1/p1_model_audit_output.json     variant comparison + smoke-simulation results

Usage:
  .venv/Scripts/python.exe scripts/p1_model_audit.py [--legacy-xml PATH]
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import platform
import sys
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from simulation.inventory import build_inventory, counts
from simulation.myosim_source import (
    DEFAULT_MYOSIM_DIR,
    MYOSIM_PINNED_VERSION,
    MYOSIM_REPO_URL,
    BuiltModel,
    build_myohand,
    verify_checkout,
)

SMOKE_DURATION_S = 2.0
SMOKE_SEED = 20261031

# Upstream source files the MyoHand builders read (see myo_sim/build/compose.py).
UPSTREAM_SOURCE_FILES = [
    "LICENSE",
    "pyproject.toml",
    "myo_sim/__init__.py",
    "myo_sim/fragments.py",
    "myo_sim/build/compose.py",
    "myo_sim/build/hand.py",
    "myo_sim/build/utils.py",
    "myo_sim/models/arm/README_hand.md",
    "myo_sim/models/arm/assets/myoarm_r_assets.xml",
    "myo_sim/models/arm/assets/myoarm_r_chain.xml",
    "myo_sim/models/arm/assets/myoarm_r_muscle.xml",
    "myo_sim/models/arm/assets/myoarm_r_tendon.xml",
    "myo_sim/models/contacts/myohand_contacts.xml",
    "myo_sim/models/torso/assets/myotorso_assets.xml",
    "myo_sim/models/torso/assets/myotorso_chain.xml",
    "myo_sim/models/head/assets/myohead_simple_assets.xml",
    "myo_sim/models/scene/myosuite_scene.xml",
]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def mesh_files(built: BuiltModel) -> list[Path]:
    meshdir = Path(built.spec.meshdir) if built.spec.meshdir else DEFAULT_MYOSIM_DIR
    files = set()
    for mesh in built.spec.meshes:
        if mesh.file:
            p = Path(mesh.file)
            files.add((p if p.is_absolute() else meshdir / p).resolve())
    return sorted(files)


def smoke_simulation(model: mujoco.MjModel, policy: str) -> dict[str, Any]:
    """Run a short open-loop simulation and report numerical health (not validation)."""
    data = mujoco.MjData(model)
    rng = np.random.default_rng(SMOKE_SEED)
    n_steps = round(SMOKE_DURATION_S / model.opt.timestep)
    lo, hi = model.jnt_range[:, 0], model.jnt_range[:, 1]
    max_violation = 0.0
    finite = True
    for _ in range(n_steps):
        if policy == "zero":
            data.ctrl[:] = 0.0
        elif policy == "full":
            data.ctrl[:] = 1.0
        elif policy == "random_seeded":
            data.ctrl[:] = rng.uniform(0.0, 1.0, model.nu)
        mujoco.mj_step(model, data)
        q = data.qpos[model.jnt_qposadr]
        max_violation = max(max_violation, float(np.max(np.maximum(lo - q, q - hi))))
        if not (np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()):
            finite = False
            break
    warnings_ = {
        mujoco.mjtWarning(i).name: int(data.warning[i].number)
        for i in range(len(data.warning))
        if data.warning[i].number
    }
    return {
        "policy": policy,
        "duration_s": SMOKE_DURATION_S,
        "n_steps": n_steps,
        "seed": SMOKE_SEED if policy == "random_seeded" else None,
        "all_states_finite": finite,
        "max_joint_limit_violation_rad": max_violation,
        "final_qpos_rad": [round(float(x), 6) for x in data.qpos],
        "max_abs_actuator_force_N": float(np.max(np.abs(data.actuator_force))),
        "mujoco_warnings": warnings_,
    }


def determinism_check(model: mujoco.MjModel) -> dict[str, Any]:
    finals = []
    for _ in range(2):
        data = mujoco.MjData(model)
        rng = np.random.default_rng(SMOKE_SEED)
        for _ in range(500):
            data.ctrl[:] = rng.uniform(0.0, 1.0, model.nu)
            mujoco.mj_step(model, data)
        finals.append(data.qpos.copy())
    return {"steps": 500, "seed": SMOKE_SEED, "bitwise_identical": bool(np.array_equal(*finals))}


def set_diff(a: list[str | None], b: list[str | None]) -> dict[str, list[str]]:
    sa, sb = {x for x in a if x}, {x for x in b if x}
    return {"only_in_composed": sorted(sb - sa), "only_in_fragment": sorted(sa - sb)}


def names(model: mujoco.MjModel, obj: mujoco.mjtObj, n: int) -> list[str | None]:
    return [mujoco.mj_id2name(model, obj, i) for i in range(n)]


def legacy_summary(xml: Path) -> dict[str, Any]:
    model = mujoco.MjModel.from_xml_path(str(xml))
    return {
        "xml": str(xml),
        "upstream_commit": "33f3ded946f55adbdcf963c99999587aadaf975f",
        "counts": counts(model),
        "joints": names(model, mujoco.mjtObj.mjOBJ_JOINT, model.njnt),
        "actuators": names(model, mujoco.mjtObj.mjOBJ_ACTUATOR, model.nu),
        "bodies": names(model, mujoco.mjtObj.mjOBJ_BODY, model.nbody),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--legacy-xml", type=Path, default=None)
    args = parser.parse_args()

    checkout = verify_checkout()
    now = dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds")
    fragment = build_myohand("fragment")
    composed = build_myohand("composed")
    fm, cm = fragment.model, composed.model

    meta = {
        "generated_utc": now,
        "generator": "scripts/p1_model_audit.py",
        "python": platform.python_version(),
        "mujoco_python": mujoco.__version__,
        "upstream": {
            "repo": MYOSIM_REPO_URL,
            "commit": checkout.commit,
            "package_version": MYOSIM_PINNED_VERSION,
            "working_tree_clean": not checkout.dirty_files,
        },
        "units": "SI (m, kg, s, N, N*m); angles in rad",
    }

    inventory = {
        "_meta": {
            **meta,
            "variant": "fragment",
            "entry_point": 'myo_sim.FRAGMENT_SPEC_BUILDERS["myohand_r"]()  '
            "(= myo_sim.build.compose.build_right_hand_from_arm_spec)",
            "build_warnings": fragment.build_warnings,
            "value_sources": {
                "upstream_mjcf": "value read from the compiled upstream model, unmodified",
                "derived": "computed from the compiled model by the stated formula/procedure",
            },
            "caveat": "Inventory of what the model contains. It is not evidence of anatomical "
            "accuracy; see docs/P1_SCIENTIFIC_GAP_ANALYSIS.md.",
        },
        **build_inventory(fm, fragment.spec),
    }
    (ROOT / "data").mkdir(exist_ok=True)
    (ROOT / "data" / "model_inventory.json").write_text(json.dumps(inventory, indent=1))

    hashed: dict[str, str] = {}
    for rel in UPSTREAM_SOURCE_FILES:
        hashed[rel] = sha256(DEFAULT_MYOSIM_DIR / rel)
    for p in mesh_files(composed):
        hashed[p.relative_to(DEFAULT_MYOSIM_DIR.resolve()).as_posix()] = sha256(p)
    hashes = {"_meta": {**meta, "algorithm": "sha256"}, "files": dict(sorted(hashed.items()))}
    (ROOT / "data" / "provenance").mkdir(parents=True, exist_ok=True)
    (ROOT / "data" / "provenance" / "myohand_file_hashes.json").write_text(
        json.dumps(hashes, indent=1)
    )

    audit = {
        "_meta": meta,
        "variants": {
            "fragment": {"counts": counts(fm), "build_warnings": fragment.build_warnings},
            "composed": {
                "entry_point": 'myo_sim.load_spec("myohand_r")',
                "counts": counts(cm),
                "build_warnings": composed.build_warnings,
            },
        },
        "composed_vs_fragment": {
            "bodies": set_diff(names(fm, mujoco.mjtObj.mjOBJ_BODY, fm.nbody),
                               names(cm, mujoco.mjtObj.mjOBJ_BODY, cm.nbody)),
            "joints": set_diff(names(fm, mujoco.mjtObj.mjOBJ_JOINT, fm.njnt),
                               names(cm, mujoco.mjtObj.mjOBJ_JOINT, cm.njnt)),
            "actuators": set_diff(names(fm, mujoco.mjtObj.mjOBJ_ACTUATOR, fm.nu),
                                  names(cm, mujoco.mjtObj.mjOBJ_ACTUATOR, cm.nu)),
            "tendons": set_diff(names(fm, mujoco.mjtObj.mjOBJ_TENDON, fm.ntendon),
                                names(cm, mujoco.mjtObj.mjOBJ_TENDON, cm.ntendon)),
            "joint_ranges_identical": bool(np.array_equal(fm.jnt_range, cm.jnt_range)),
            "actuator_gainprm_identical": bool(
                np.array_equal(fm.actuator_gainprm, cm.actuator_gainprm)
            ),
            "contact_pairs_composed": build_inventory(cm)["contacts"]["pairs"],
        },
        "smoke_simulation_fragment": [
            smoke_simulation(fm, p) for p in ("zero", "full", "random_seeded")
        ],
        "determinism_fragment": determinism_check(fm),
        "n_hashed_files": len(hashed),
    }
    if args.legacy_xml:
        legacy = legacy_summary(args.legacy_xml)
        strip = {n.removesuffix("_r") for n in names(fm, mujoco.mjtObj.mjOBJ_JOINT, fm.njnt) if n}
        legacy["joint_names_match_fragment_after_suffix_strip"] = strip == set(legacy["joints"])
        legacy["actuator_names_match_fragment_after_suffix_strip"] = {
            n.removesuffix("_r") for n in names(fm, mujoco.mjtObj.mjOBJ_ACTUATOR, fm.nu) if n
        } == set(legacy["actuators"])
        audit["legacy_static_v0_1_7"] = legacy
    out = ROOT / "reports" / "P1"
    out.mkdir(parents=True, exist_ok=True)
    (out / "p1_model_audit_output.json").write_text(json.dumps(audit, indent=1))

    print(json.dumps({k: audit[k] for k in ("variants", "determinism_fragment")}, indent=1))
    for s in audit["smoke_simulation_fragment"]:
        print(s["policy"], "finite:", s["all_states_finite"],
              "max_limit_violation_rad:", round(s["max_joint_limit_violation_rad"], 5),
              "warnings:", s["mujoco_warnings"])
    if "legacy_static_v0_1_7" in audit:
        lg = audit["legacy_static_v0_1_7"]
        print("legacy counts:", lg["counts"])
        print("legacy names match:", lg["joint_names_match_fragment_after_suffix_strip"],
              lg["actuator_names_match_fragment_after_suffix_strip"])
    print("hashed files:", len(hashed))


if __name__ == "__main__":
    main()
