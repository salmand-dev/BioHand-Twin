# P2 Implementation Plan — Anatomical Hierarchy and Coordinate Convention

Status: **IMPLEMENTED 2026-10-31** — see `reports/P2/P2_REPORT.md` for results and deviations. Decisions 0.3 (git), 0.4 (anatomy reference) and 0.5 (ISB) remain open.

Inputs: `data/model_inventory.json`, `reports/P1/P1_REFERENCE_MODEL_AUDIT.md`, `docs/P1_SCIENTIFIC_GAP_ANALYSIS.md`.

## 0. Preconditions and owner decisions

| # | Decision / precondition | Why it matters | Recommendation |
|---|---|---|---|
| 0.1 | Resolve the MoBL-ARMS licence terms (`docs/P1_LICENSE_AND_PROVENANCE.md` §6) | Decides whether STL-derived GLBs may ever be distributed. That affects the whole visual pipeline | Until resolved, run the visual pipeline **locally only**: generated GLBs are gitignored, built from the user's own MyoSim checkout |
| 0.2 | Choose the reference variant | Changes body/geom sets and contact pairs | **fragment** (`FRAGMENT_SPEC_BUILDERS["myohand_r"]`): warning-free, no torso. The composed variant adds only torso bodies and 3 radius–thorax pairs |
| 0.3 | `git init` and first commit (scaffold + P0/P1) | Needed for "Git commit" in every validation report | Do before P2 code |
| 0.4 | Pick the anatomy reference(s) for canonical names | Project rules require a source for every name | Terminologia Anatomica (English equivalents) for names, plus one standard anatomy text for structure/attachment statements. The owner must confirm access |
| 0.5 | Anatomical frame convention | Joint angle sign and zero definitions | Adopt ISB recommendations for wrist/hand (Wu et al., J Biomech 2005, part II) **for reporting only**. MuJoCo joint coordinates stay the simulation truth. Retrieve the paper first |

## 1. Objectives (from ROADMAP P2)

1. Map every MuJoCo body, joint, actuator, tendon and mesh to a canonical anatomical name, with aliases and an explicit **representation status**.
2. Derive the **functional segment hierarchy** (rigid groups) from the model itself, so lumped structures can never be presented as independently moving.
3. Define and test one coordinate convention chain: MuJoCo world → anatomical reporting frames → Godot/glTF.
4. Create the visual-to-physics mapping, with every visual node either mapped or marked visual-only.

## 2. Work packages (small, testable increments, in order)

### WP1 — Rigid-segment derivation (`simulation/segments.py`)

- `rigid_groups(model) -> list[RigidSegment]`: merge every jointless body into its parent's group (union-find over `body_parentid` where `body_jntnum == 0`). Each group records:
  - the root body;
  - member bodies and meshes;
  - the joints that move it relative to its parent group.
- Expected outcome for the fragment (to be **verified by test, not assumed**):
  1. a world-fixed group: root, scaffold, humerus, ulna;
  2. radius;
  3. one **wrist/palm segment** — lunate plus 7 welded carpals plus MCs II–V (since MCs II–V are welded to capitate);
  4. thumb MC;
  5. thumb proximal and distal phalanges;
  6. 12 finger phalanges.
- Tests:
  - group count;
  - membership of the carpal/MC group;
  - each group has 0 or ≥1 joints with the documented DoF;
  - the sum of member counts equals nbody.

### WP2 — Glossary and canonical naming (`data/anatomy/glossary.yaml`, `simulation/naming.py`)

- One entry per model element (38 bodies, 23 joints, 39 actuators, 33 meshes). Fields:
  - `model_name`
  - `aliases` (legacy unsuffixed name, OpenSim name where known)
  - `canonical_name`
  - `structure_type`
  - `representation`: `simulated_segment` | `rigid_part_of:<segment>` | `scaffold_non_hand` | `combined_actuator` | `visual_only` | `not_represented`
  - `name_source` (citation)
  - `status`: `verified` | `unresolved`
- Also list **absent** structures named in the gap analysis (APB, FPB, ADP, ADM, FDMB, ODM, supinator, sesamoids, ligaments) as `not_represented`, so the UI can state their absence explicitly.
- Known unresolved items, which must stay `unresolved` until a source is read:
  - RI/UI ↔ dorsal/palmar interosseous per digit (Lee 2015 correction);
  - the meaning of the LU_RB / UI_UB combinations;
  - mesh-to-bone identity of `thumbprox`/`thumbdist` (thumb proximal/distal phalanx) — confirm from MoBL.
- `naming.py`:
  - load and validate the schema;
  - `canonical(model_name)` and `aliases_of(canonical)`;
  - refuse duplicate canonical names.
- Tests:
  - every inventory element has exactly one entry;
  - no orphan entries;
  - every `verified` entry has a non-empty `name_source`;
  - alias uniqueness.

### WP3 — Coordinate conventions (`biomechanics/frames.py`, `docs/COORDINATE_SYSTEM.md`)

1. **MuJoCo world** (source of truth): right-handed, +Z up (gravity = (0, 0, −9.81) [MODEL]), metres. Quaternions are **w, x, y, z**.
2. **Model hand orientation**: derive, do not assume, the world-frame directions of:
   - forearm long axis (`radius_r` origin → `lunate_r` origin);
   - palm normal (cross product of vectors from `lunate_r` to the `secondmc_r` and `fifthmc_r` origins);
   - radial direction.

   Record them in `docs/COORDINATE_SYSTEM.md` with the script that computed them.
3. **Anatomical reporting frames** (optional per decision 0.5): a fixed rotation per segment from the MuJoCo body frame. Defined only from model landmarks, with the landmark choice cited. Not used by the simulator.
4. **Godot / glTF**: right-handed, +Y up, metres; Godot quaternions are **x, y, z, w**.
   - Position mapping: `(x, y, z)_mj → (x, z, −y)_godot`, a rotation of −90° about X with det = +1.
   - Rotation mapping: `R_g = C R_mj Cᵀ`.

   A single function pair implements this: `mujoco_to_godot_pos/quat` and the inverse.
5. **Blender** (asset preparation only): right-handed, +Z up. The glTF exporter's +Y-up conversion is checked with a known-asymmetric test asset. The scale is checked as metres (MyoSim meshes are in metres per compiled extents — verify).
6. Tests:
   - basis-vector mapping;
   - det(C) = +1;
   - pos/quat round-trip to < 1e-12;
   - quat ↔ matrix consistency against MuJoCo's `mju_quat2Mat`;
   - wxyz/xyzw ordering;
   - property tests on random rotations;
   - FK landmark test: body world positions after `mj_forward`, converted, match a direct matrix transform.

### WP4 — Joint and DoF specification (`docs/JOINTS_AND_DOF.md`, generated)

- Generated from the inventory by `scripts/p2_generate_docs.py`:
  - per joint: body, axis (body frame and world frame at qpos0), range in rad (deg for display), damping, armature;
  - positive-direction semantics, using the sign evidence of P1 §4 at qpos0 only and marked "to be confirmed in P3".
- Define the BioHand **neutral pose** = `model.qpos0` (all zeros, all within range [MODEL]). State explicitly that this is the model's zero, not an anatomically defined neutral, until it is compared with a cited definition.
- Drift test: regenerating the docs reproduces the committed file byte-for-byte.

### WP5 — Muscle model specification (`docs/MUSCLE_MODEL.md`)

- Document MuJoCo muscle semantics as implemented: activation dynamics, FL/FV/FP curves via `gainprm`/`biasprm`, inextensible tendon, and the L0 normalisation. Cite the MuJoCo docs and version 3.17.0.
- Per-actuator table (generated): tendon, F_max, lengthrange, spanned DoFs at qpos0. Each value carries a provenance class from `model_manifest.yaml` (`tuned` / `generic default`).
- Explicit "not represented" list (M1–M11 from the gap analysis).
- No new parameter is introduced.

### WP6 — Visual-to-physics mapping (`blender/visual_to_physics_mapping.csv`, `docs/ANATOMICAL_MAPPING.md`)

- Generate one row per visual node. Initially these are the 32 bone mesh geoms of the fragment, plus collision/wrap geoms marked `debug_only`. Columns: `visual_node`, `canonical_name`, `mujoco_body`, `rigid_segment` (WP1), `source` (mesh file + sha256), `representation`, `notes`.
- The local Blender import/export script (`blender/scripts/export_myohand_glb.py`, run headless with Blender 3.6.23):
  - reads STLs from the user's pinned checkout;
  - names nodes by MuJoCo geom name;
  - writes the GLB to a **gitignored** `build/visual/`.

  It is **not** run in CI with committed outputs while decision 0.1 is open.
- Tests:
  - every CSV row resolves to an existing body and geom;
  - every mesh geom has a row;
  - every `rigid_part_of` row agrees with WP1;
  - the CSV `representation` agrees with the glossary.

### WP7 — Lumped-segment guard (UI contract, Python side)

- Add `simulation/ui_contract.py::selectable_motion_units()`. It returns the rigid segments (WP1), not the bones, as the units that can be posed or animated independently. P7's Godot UI must consume this list. Selecting the trapezoid, for example, highlights the whole wrist/palm segment and shows "moves with carpus + MC II–V (lumped in reference model)".
- Test: no motion unit contains welded bodies from different groups, and every lumped bone maps to exactly one unit.

## 3. Deliverables

| File | Type |
|---|---|
| `simulation/segments.py`, `simulation/naming.py`, `simulation/ui_contract.py`, `biomechanics/frames.py` | typed code |
| `data/anatomy/glossary.yaml` | data, with sources |
| `docs/ANATOMICAL_MAPPING.md`, `docs/COORDINATE_SYSTEM.md`, `docs/JOINTS_AND_DOF.md`, `docs/MUSCLE_MODEL.md` | docs (tables generated) |
| `blender/visual_to_physics_mapping.csv` | data (names/hashes only, no geometry) |
| `scripts/p2_generate_docs.py`, `blender/scripts/export_myohand_glb.py` | tooling |
| `tests/test_segments.py`, `test_naming.py`, `test_frames.py`, `test_visual_mapping.py`, `test_generated_docs.py` | tests |
| `reports/P2/P2_REPORT.md` + test output | evidence |

## 4. Acceptance criteria (ROADMAP P2, made measurable)

1. 100 % of mesh geoms appear in the visual mapping, each mapped to a body or marked visual-only (test).
2. 100 % of bodies, joints, actuators and meshes have a glossary entry. Every `verified` name has a source, and all unresolved items are listed in the report.
3. Coordinate tests pass: round-trip error < 1e-12 and det = +1 (test).
4. The rigid-segment list is derived from the model and covered by tests. The UI contract exposes only rigid segments as motion units.
5. Generated docs match the inventory (drift test).
6. `PHASE_LOG.md` is updated and `reports/P2/P2_REPORT.md` is written with limitations.

## 5. Explicitly out of scope for P2

Joint sweeps and ROM comparison (P3); moment-arm curves and actuator sign tests (P4); any parameter change; Godot scenes beyond the transform function; contact setup for grasping.

## 6. Risks

| Risk | Mitigation |
|---|---|
| Licence forbids redistribution of meshes or derived GLBs | Local-only build path (WP6). Alternatively source independently licensed bone meshes later, mapped by the same CSV |
| Upstream changes MyoHand again (it already changed in 0.2.0) | Pinned commit plus hash test. Upgrades only via an explicit re-audit |
| Anatomy names cannot be sourced for combined actuators | Keep `unresolved`; display the upstream abbreviation with an "unverified mapping" badge |
| Canonical frame ambiguity (model zero vs ISB neutral) | Report both, label clearly, and never silently convert |
