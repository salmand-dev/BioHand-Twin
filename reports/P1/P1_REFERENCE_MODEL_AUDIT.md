# P1 — Reference Model Audit: MyoHand (MyoSim)

- Date: 2026-10-31
- Upstream: https://github.com/MyoHub/myo_sim @ `93b0ca8f4ec90c9899ee7f05fee561e9911da91b` (2026-09-15, "v0.2.3 release (#134)", package version 0.2.3, **no git tag**)
- Toolchain: Python 3.11.4, MuJoCo 3.17.0, Windows 10
- Evidence:
  - `data/model_inventory.json` — full inventory of the primary variant (252 KB)
  - `reports/P1/p1_model_audit_output.json` — variant comparison, smoke simulations, determinism
  - `reports/P1/p1_model_audit_console.txt` — console output
  - `reports/P1/test_run_output.txt` — pytest, ruff and mypy output
  - `data/provenance/myohand_file_hashes.json` — SHA-256 hashes of 73 upstream files
- Reproduce: `.venv/Scripts/python.exe scripts/p1_model_audit.py --legacy-xml <path>` and `.venv/Scripts/python.exe -m pytest -v`

## 1. What "MyoHand" is at the pinned commit (differs from the project README)

The project README assumes a static MyoHand XML. **That is no longer true.**

- The static `hand/myohand.xml` (model "MyoHand_v0.1.7") was **deleted in upstream release 0.2.0** (`d237c6f`, 2026-08-08).
- MyoHand is now **generated at runtime** with `mujoco.MjSpec` by `myo_sim/build/hand.py::prune_arm_spec_to_hand`. The builder:
  1. builds the full MyoArm (38 DoF, 63 muscles);
  2. deletes the 24 non-hand actuators and their tendons;
  3. sets the shoulder to a fixed "preview pose" (`elv_angle=1.57`, `shoulder_elv=1.57`, `shoulder_rot=-1.57` rad);
  4. deletes 15 shoulder/elbow joints, which welds clavicle, scapula, humerus and ulna rigidly to the world;
  5. replaces the `[1,1,1] kg·m²` "phantom" inertias with 1e-4 (upstream issue #128).
- There are two upstream entry points:

| Variant | Call | nbody | ngeom | nsite | nmesh | npair | Build warnings |
|---|---|---|---|---|---|---|---|
| **fragment** (primary candidate) | `myo_sim.FRAGMENT_SPEC_BUILDERS["myohand_r"]()` | 38 | 91 | 491 | 33 | 0 | **none** |
| composed (upstream `load("myohand")`) | `myo_sim.load_spec("myohand_r")` | 53 | 136 | 1169 | 57 | 3 | `Attaching a spec with an empty prefix may prevent the model from being saved correctly.` |

Both variants have **identical** joints, actuators, tendons, joint ranges and muscle parameters (`p1_model_audit_output.json → composed_vs_fragment`). The composed variant only adds 15 torso/spine/head bodies and 3 contact pairs (`radius_coll_r` against `thorax_coll1..3`).

**Legacy cross-check.** The last static XML (commit `33f3ded`, v0.1.7, extracted to a scratch directory, not into the repo) compiles to the same 23 joints and 39 actuators. Names match after stripping the `_r` suffix (`legacy names match: True True`).

## 2. Counts: README claims vs parsed model

| Quantity | Upstream README_hand.md | Parsed (fragment) | Agreement |
|---|---|---|---|
| Degrees of freedom | 23 | nv = nq = 23 (all hinge, all limited) | ✔ |
| Muscle actuators | 39 | nu = 39 (all `dyntype=muscle`, `gaintype=muscle`, tendon transmission) | ✔ |
| Tendons | — | 39 spatial (one per actuator), 315 path elements | — |
| Bodies | lists 8 carpals, 5 MCs, phalanges, radius, ulna | 38 incl. world, model root, clavicle, scapula, humerus and 4 phantom bodies | partial — see §3 |
| Sensors | — | **0** | — |
| Equality constraints | — | **0** | — |
| Keyframes | — | **0** (qpos0 = all zeros, which lies inside every range) | — |
| Contact pairs / excludes | — | 0 / 5 | — |

MyoSuite paper (arXiv:2205.13600, §3.1) gives "29 bones, 23 joints, and 39 muscles-tendon units". The `REFERENCES.md` scaffold note about Mirakhorlo (43 muscles) refers to a **different model**; MyoHand is not derived from it (§ gap analysis).

## 3. Segments (bodies)

| Group | Bodies | Articulation in model |
|---|---|---|
| Proximal scaffold (not hand) | `myoarm_r_root`, `clavicle_r`, `clavphant_r`, `scapula_r`, `scapphant_r`, `humphant_r`, `humphant1_r`, `humerus_r` | welded (joints pruned) |
| Forearm | `ulna_r` (welded), `radius_r` | `pro_sup_r` on radius |
| Carpus | `lunate_r` + `scaphoid_r`, `pisiform_r`, `triquetrum_r`, `capitate_r`, `trapezium_r`, `trapezoid_r`, `hamate_r` | **Only `lunate_r` moves**: `deviation_r` and `flexion_r` are two hinges at the same anchor. The other 7 carpals are welded to lunate/capitate, so the carpus is **one rigid segment** with 8 meshes |
| Thumb | `firstmc_r`, `proximal_thumb_r`, `distal_thumb_r` | CMC 2 DoF, MP 1 DoF, IP 1 DoF |
| Digits 2–5 | `secondmc_r`…`fifthmc_r` (welded to capitate), `{2..5}proxph_r`, `midph{2..5}_r`, `distph{2..5}_r` | MCP 2 DoF, PIP 1 DoF, DIP 1 DoF each |

Metacarpals II–V are welded to `capitate_r`, so there is **no CMC motion for digits 2–5** (no palmar arching).

## 4. Joints / DoF (range shown in degrees for display; stored in radians)

| Joint | Body | Range (deg) | Damping (N·m·s/rad) |
|---|---|---|---|
| pro_sup_r | radius_r | −90 … 90 | 0.5 |
| deviation_r | lunate_r | −10 … 25 | 0.25 |
| flexion_r | lunate_r | −45 … 45 | 0.25 |
| cmc_flexion_r | firstmc_r | −44.69 … 40.11 | 0.05 |
| cmc_abduction_r | firstmc_r | −28.65 … 44.69 | 0.05 |
| mp_flexion_r | proximal_thumb_r | −45 … 40 | 0.05 |
| ip_flexion_r | distal_thumb_r | −75 … 25 | 0.05 |
| mcp{2–5}_flexion_r | {n}proxph_r | 0 … 90 | 0.05 |
| mcp{2–5}_abduction_r | {n}proxph_r | −15 … 15 | 0.05 |
| pm{2–5}_flexion_r | midph{n}_r | 0 … 90 | 0.05 |
| md{2–5}_flexion_r | distph{n}_r | 0 … 90 | 0.05 |

All 23 joints: stiffness 0, armature 1e-4 kg·m², frictionloss 0. Joint axes (body frame) are in the inventory. The thumb axes are oblique and non-orthogonal.

**Sign conventions inferred from the model (derived, at qpos0).** These come from the sign of the tendon dL/dq. With MuJoCo's convention, a negative dL/dq means contraction drives the joint positive.

| Joint | Positive direction | Evidence |
|---|---|---|
| `flexion_r` | wrist flexion | FCR/FCU/PL/FDS/FDP all negative; ECRB/ECU/EDC positive |
| `mcp*/pm*/md*_flexion_r` | flexion | FDP and FDS negative |
| `pro_sup_r` | pronation | PT −11.6 mm and PQ −7.0 mm |

These are moment-arm signs at one pose only. P3/P4 must confirm them over the full range.

## 5. Muscle actuators (39)

Full per-actuator data is in `data/model_inventory.json → actuators`: tendon, peak force, lengthrange, raw MuJoCo params, derived L0 and spanned DoFs.

| Group | Actuators | Expansion |
|---|---|---|
| Wrist | ECRL, ECRB, ECU, FCR, FCU, PL | extensor carpi radialis longus/brevis, extensor carpi ulnaris, flexor carpi radialis/ulnaris, palmaris longus |
| Forearm rotation | PT, PQ | pronator teres, pronator quadratus |
| Extrinsic finger | FDS2–5, FDP2–5, EDC2–5, EDM, EIP | flexor digitorum superficialis/profundus, extensor digitorum (communis), extensor digiti minimi, extensor indicis (proprius) |
| Extrinsic thumb | EPL, EPB, FPL, APL | extensor pollicis longus/brevis, flexor pollicis longus, abductor pollicis longus |
| Intrinsic thumb | OP | opponens pollicis (MyoSuite paper: added by the MyoSuite authors) |
| Intrinsic finger | RI2–5, LU_RB2–5, UI_UB2–5 | RI = "radial interosseous" (Lee et al. 2015: "dorsal interosseous or radial interosseous"). UI = "palmar interosseous or ulnar interosseous". **LU_RB / UI_UB are single actuators whose names suggest lumbrical + radial band and ulnar interosseous + ulnar band. This is a reading of the names, not upstream documentation.** Lee 2015 has a 2022 correction to its interosseous terminology; its text could not be retrieved (PMC bot-check) |

**MuJoCo muscle semantics (from compiled values):**

- Every actuator uses MuJoCo's built-in muscle model: activation dynamics plus force–length–velocity curves inside `gainprm`/`biasprm`. Any BioHand muscle code must **not** add a second Hill model on top.
- **Only two quantities differ between muscles:** peak force `gainprm[2]` (46.0 N for EPB to 557.2 N for PT) and `lengthrange`. Every other parameter is identical for all 39 actuators:
  - `range = [0.75, 1.05]`
  - `lmin = 0.5`, `lmax = 1.6`
  - `vmax = 1.5 L0/s`
  - `fpmax = 1.3`, `fvmax = 1.2`
  - τ_act = 0.01 s, τ_deact = 0.04 s
- These are the `myoarm_muscle` class defaults, **not muscle-specific physiology**.
- Tendon stiffness is 0 for all tendons (inextensible tendon). There is no pennation angle and no separate tendon slack length.
- The derived `L0 = Δlengthrange / Δrange` is a MuJoCo normalisation constant, **not an anatomical optimal fibre length**. For example, FDS gives 0.19–0.23 m.
- Intrinsic muscles share peak forces within each digit:
  - digit 2: RI2, LU_RB2 and UI_UB2 are all 61.2 N
  - digit 3: LU_RB3 and UI_UB3 are 98.0 N
  - digit 4: LU_RB4 and UI_UB4 are 76.9 N
  - digit 5: LU_RB5 and UI_UB5 are 47.9 N

  The origin of these shared values is undocumented.

## 6. Sites, geoms, meshes, contacts, sensors

- **Sites (491, fragment):** tendon via-points plus a few markers. Each site's name, body and body-frame position are in the inventory.
- **Geoms (91):**

  | Group | Count | Type | Role |
  |---|---|---|---|
  | 0 | 32 | mesh | bone visuals |
  | 3 | 30 | 18 cylinders + 12 spheres | tendon wrapping surfaces |
  | 4 | 29 | 24 capsules + 5 ellipsoids | collision proxies |

- **Collision:** the 29 collision geoms have `contype=1, conaffinity=0`, so **no hand self-contact can occur** in the fragment. They only collide with external geoms that have `conaffinity` set. The 5 excluded body pairs are ulna–radius, radius–firstmc and adjacent proximal phalanges.
- **Meshes (33):** STL bone meshes, hashed in `myohand_file_hashes.json`.
- **Sensors: none.** Every telemetry channel (joint state, tendon length, actuator force) must come from `MjData`, not model sensors.

## 7. Inertial properties — flagged

`data/model_inventory.json → bodies` lists every value. Notable points:

- Carpals other than lunate/capitate, metacarpals II–V and all finger phalanges have explicit masses of 0.001–0.02 kg with principal inertia **1e-4 kg·m²**. That equals the compiler `boundinertia` floor. For a 0.01 kg proximal phalanx it implies a radius of gyration of about 0.1 m, which is physically implausible for a finger bone. **These are numerical-conditioning values, not anatomical inertias.**
- `lunate_r` (0.1496 kg) and `capitate_r` (0.0018 kg) have no explicit inertial. Their values come from mesh volume at MuJoCo's default density.
- Consequence: finger dynamics (accelerations, response time) are not anatomically grounded. Kinematics, moment arms and quasi-static force are less affected. This must be stated in every P5 result.

## 8. Numerical smoke tests (not validation)

Fragment variant, timestep 0.002 s, default solver settings, 2 s per run:

| Policy | All states finite | Max joint-limit violation | MuJoCo warnings |
|---|---|---|---|
| zero control | yes | 0.0042 rad | none |
| full activation (u=1 all) | yes | **0.154 rad** (cmc_flexion_r); 16 joints > 0.01 rad | none |
| seeded random u∈[0,1] (seed 20261031) | yes | 0.110 rad | none |

- Seeded random control over 500 steps, run twice, gives **bitwise-identical** qpos.
- Joint limits are soft constraints, so co-contraction drives several joints past their ranges by up to 8.8°. The DIP of digits 3–5 and the PIP of digit 5 also go *negative*, i.e. hyperextension below the 0 limit.
- P3 must decide whether to report, tune (`solref`/`solimp` as a documented project override), or accept this.

## 9. Test results

`reports/P1/test_run_output.txt`: **20 passed**, ruff clean, mypy clean.

- 7 unit tests: dL/dq against analytic fixed-tendon coefficients and an analytic spatial-tendon formula at 3 angles; muscle L0 formula; counts; welded flag.
- 10 integration tests:
  - pinned and clean checkout;
  - fragment builds with no warnings;
  - 23 DoF / 39 actuators;
  - exact joint name order and limits;
  - each actuator is a muscle on its own tendon;
  - carpals lumped on lunate;
  - zero-control stability;
  - determinism;
  - inventory consistent with a fresh build;
  - all 73 upstream file hashes unchanged.
- 3 pre-existing scaffold tests.

## 10. Acceptance criteria (ROADMAP P1)

| Criterion | Status |
|---|---|
| Model loads in MuJoCo without warnings/errors | **PASS for the fragment variant.** The composed (upstream default) variant emits an upstream attach warning |
| Machine-readable inventory of bodies, joints, DoFs, actuators, tendons, sites, geoms | **PASS** — `data/model_inventory.json` |
| Provenance record complete | **PASS.** Licences verified, derivation from MoBL-ARMS demonstrated, and intended use declared non-commercial (research/education) on 2026-10-31 (`docs/P1_LICENSE_AND_PROVENANCE.md`) |
| No external asset copied into repository without permission | **PASS.** `third_party/` is gitignored and BioHand ships no upstream files. Upstream code is only executed from the pinned, verified checkout |

**P1 status: COMPLETE (2026-10-31).** All four ROADMAP P1 criteria are met for the fragment variant. Known limitations and anomalies (gap analysis S1–X1, T1–T9) are documented, not resolved. Resolving them is work for P3/P4.

Addendum (2026-10-31): source traceability against the original OpenSim models is in `docs/P1_SCIENTIFIC_GAP_ANALYSIS.md` §3.6 (findings T1–T9).
