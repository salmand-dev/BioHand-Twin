# P2 — Anatomical Hierarchy and Coordinate Convention: Report

- Date: 2026-10-31
- Model: MyoHand fragment, MyoSim `93b0ca8f4ec90c9899ee7f05fee561e9911da91b`
- Toolchain: Python 3.11.4, MuJoCo 3.17.0, Blender 3.6.23, Godot 4.7.2
- Git commit: none (the project is not a git repository; owner decision deferred)
- Evidence:
  - `reports/P2/test_run_output.txt` — **53 passed, 0 skipped, 0 failed**; ruff and mypy clean
  - `reports/P2/joint_semantics_evidence.json`
  - `reports/P2/godot_dump.txt`

## Decisions applied (from `docs/P2_IMPLEMENTATION_PLAN.md` §0)

| # | Decision | Applied |
|---|---|---|
| 0.1 | Licence | Resolved in P1: non-commercial research/education. The visual pipeline is **local-only**: `build/` and `godot_app/local_assets/` are gitignored |
| 0.2 | Variant | **fragment** (`FRAGMENT_SPEC_BUILDERS["myohand_r"]`) |
| 0.3 | git init | Deferred by the owner |
| 0.4 | Anatomy reference for names | **Not yet chosen.** Names are `provisional` (standard English terms, citation pending), except those verified against Lee et al. 2015 |
| 0.5 | ISB frames | Not implemented: Wu et al. 2005 has not been retrieved. The MuJoCo joint coordinates are the only joint convention |

## What was built

| WP | Deliverable | Verified by |
|---|---|---|
| 1 | `simulation/segments.py` — rigid segments derived from the model (jointless bodies merged into their parent) | `test_rigid_segments_partition_bodies` |
| 2 | `data/anatomy/glossary.yaml` (38 bodies, 23 joints, 39 actuators, 15 not-represented structures) and `simulation/naming.py` (schema validation, alias resolution) | `test_every_model_element_has_exactly_one_glossary_entry`, `test_glossary_representation_agrees_with_segments`, alias/duplicate tests |
| 3 | `biomechanics/frames.py` — the single MuJoCo↔Godot conversion | 14 unit tests (basis, det = +1, round trips < 1e-12, agreement with `mju_quat2Mat`/`mju_mulQuat`, rotated-point consistency, invalid input) |
| 3 | `simulation/kinematic_semantics.py` — hand frame and joint direction evidence | `test_joint_kinematic_evidence_matches_glossary`, `test_joint_muscle_evidence_matches_model` |
| 4–5 | Generated `docs/JOINTS_AND_DOF.md`, `docs/MUSCLE_MODEL.md`, `docs/ANATOMICAL_MAPPING.md`, `docs/COORDINATE_SYSTEM.md` (`scripts/p2_generate_docs.py`) | `test_generated_docs.py` (byte-identical regeneration) |
| 6 | `blender/visual_to_physics_mapping.csv` (91 rows: 32 bone meshes plus 59 wrap/collision geoms marked `debug_only`); local pipeline `scripts/p2_export_visual_scene.py` → `blender/scripts/build_glb.py` → `build/visual/myohand_fragment.glb` (sha256 `76b4b729…24bf38`, not committed) | `test_visual_export.py` (6 tests incl. Godot import) |
| 7 | `simulation/ui_contract.py` — motion units for the UI | `test_ui_contract_never_exposes_lumped_bone_as_independent` |

## Key results

1. **18 motion units.** One of them is the lumped **carpus + palm** segment: 12 bones (8 carpals and MC II–V) that move as one through `deviation_r` and `flexion_r`. Selecting any of those bones in the UI selects the whole segment, labelled "lunate segment (12 bones moving as one)". Ulna, humerus, scapula and clavicle form the fixed world segment.
2. **Joint positive directions.** These are derived from two independent kinds of evidence that agree, verified by tests at qpos0 only:
   - `flexion_r`, all MCP/PIP/DIP flexion joints: positive = **flexion** (palmar).
   - `deviation_r`: positive = **radial** deviation, so the range −10…+25° means 25° radial and 10° ulnar. **[CHECK]** against Holzbaur 2005 in P3.
   - `mcp*_abduction_r`: positive = **radial** deviation for every digit. That is anatomical *adduction* for digits IV–V, so the UI must not label it "abduction" without the digit context.
   - `pro_sup_r`: positive = **pronation**.
   - Thumb `cmc_flexion`, `mp_flexion` and `ip_flexion`: positive = **extension**, from muscle evidence only. Their names say "flexion" but positive values are extension. The hand-frame kinematic test cannot separate thumb directions.
3. **Coordinate chain verified end to end.** MuJoCo → Blender → GLB → Godot 4.7:
   - All 38 body origins and 32 mesh frames match `frames.mujoco_to_godot_pos`/`C·R·Cᵀ`. The worst deviation is 1.4 µm in the GLB, from Blender's float32 storage; the tolerance is 10 µm and the justification is in the test.
   - Godot's own imported node transforms match to ≤ 0.2 µm.
   - Mesh vertex bounds are preserved to within 1e-6 m.
4. **Hand orientation at qpos0** (derived): forearm along about world +X, radial side about +Y, dorsum about +Z (palm down). This comes from the upstream "preview pose", not an anatomical position.
5. **Parameter provenance in the docs.** Every F_max and joint range is labelled inherited, modified or added against the OpenSim sources. Modified: EDC5, six intrinsics, wrist flexion and two thumb CMC ranges. Added: OP.

## New findings in P2

- **Inertial centres.** 27 of 38 bodies, including all phalanges, have their inertial centre exactly at the body origin, i.e. on the joint axis. This is non-anatomical and adds to gap I1.
- **Inconsistent upstream geom names.** `capitate`, `thumbprox` and `5distph` lack the `_r` suffix, and several geom names equal body names. The GLB therefore names visual nodes `<geom>__visual`.

## Acceptance criteria (`docs/P2_IMPLEMENTATION_PLAN.md` §4)

| # | Criterion | Status |
|---|---|---|
| 1 | 100 % of mesh geoms in the visual mapping, each mapped or marked visual-only | **PASS** (32/32 mapped; primitives marked `debug_only`) |
| 2 | 100 % of bodies, joints and actuators in the glossary; verified names sourced; unresolved listed | **PASS** with a caveat: most names are `provisional` (citation pending, decision 0.4). Unresolved: 12 intrinsic actuators (RI/LU_RB/UI_UB) |
| 3 | Coordinate tests pass (round trip < 1e-12, det = +1) | **PASS** |
| 4 | Rigid segments derived and tested; UI contract exposes only rigid segments | **PASS** |
| 5 | Generated docs match the inventory (drift test) | **PASS** |
| 6 | Phase log updated; report with limitations | **PASS** (this file) |

**P2 status: COMPLETE (2026-10-31)**, with the limitations below carried forward.

## Limitations carried forward

- **Provisional names.** Canonical anatomical names are mostly `provisional` until an anatomy reference is chosen and cited (decision 0.4). The UI must display the status.
- **Single-pose evidence.** Joint direction semantics are validated at qpos0 only. Full-range confirmation is P3.
- **No ISB frames.** Anatomical (ISB) reporting frames are not implemented.
- **Bone meshes are MoBL-derived.** They are non-commercial and must never be committed or redistributed.
- **Local GLB tests.** The visual tests run only where the local GLB has been built. A clean checkout skips them (they are not failed). The evidence above comes from a run where they executed.
- **Godot-side checks.** The Godot check uses a headless script (`godot_app/tools/dump_glb_nodes.gd`). No UI exists yet (P7).

## Reproduction

```bash
.venv/Scripts/python.exe scripts/p2_generate_docs.py
```

```bash
.venv/Scripts/python.exe scripts/p2_export_visual_scene.py
```

```bash
"C:/Program Files/Blender Foundation/Blender 3.6/blender.exe" --background --factory-startup --python blender/scripts/build_glb.py -- build/visual/scene build/visual/myohand_fragment.glb
```

```bash
cp build/visual/myohand_fragment.glb godot_app/local_assets/
```

```bash
.venv/Scripts/python.exe -m pytest -v
```
