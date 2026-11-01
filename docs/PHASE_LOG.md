# Phase Log

| Phase | Status | Evidence / report | Notes |
|---|---|---|---|
| P0 | Complete (2026-10-31) | `reports/P0/P0_ENVIRONMENT_AUDIT.md`, `reports/P0/environment_audit.json`, `requirements-lock.txt` | All ROADMAP P0 criteria met. Godot open check was headless only. Open: no git repo, no CI |
| P1 | Complete (2026-10-31) | `reports/P1/P1_REFERENCE_MODEL_AUDIT.md`, `data/model_inventory.json`, `data/reference_osim_inventory.json`, `reports/P1/osim_vs_myohand.json`, `docs/P1_LICENSE_AND_PROVENANCE.md`, `docs/P1_SCIENTIFIC_GAP_ANALYSIS.md`, `reports/P1/test_run_output.txt` | Licences verified (MoBL-ARMS: non-commercial BSD-3; 2nd-Hand: MIT). Traceability to OpenSim sources done. Intended use: non-commercial (research/education). Limitations documented in the gap analysis |
| P2 | Complete (2026-10-31) | `reports/P2/P2_REPORT.md`, `reports/P2/test_run_output.txt`, `data/anatomy/glossary.yaml`, generated `docs/{JOINTS_AND_DOF,MUSCLE_MODEL,ANATOMICAL_MAPPING,COORDINATE_SYSTEM}.md`, `blender/visual_to_physics_mapping.csv` | 53/53 tests. Names mostly provisional (anatomy reference pending); joint semantics checked at qpos0 only; no git repo |
| P3 | Complete (2026-10-31) | `reports/P3/P3_REPORT.md`, `reports/P3/test_run_output.txt`, `reports/P3/*.json`, `reports/P3/fingertip_trajectories.csv`, `data/patches/wrist_rom_v1.yaml` | Owner chose option B: wrist patch restores MoBL ranges (provisional until P4). Residual: deviation ROM below ADL (source value). New: T11 lengthrange anomalies (RI5, FDP5, FDS4) |
| P4 | Complete (2026-10-31) | `reports/P4/P4_REPORT.md`, `reports/P4/test_run_output.txt`, `reports/P4/*.json`, `docs/MUSCLE_MODEL.md` | 39/39 isolated-activation directions correct; ablation API; 6 non-functional muscles (T11′), RI2 path break, wrist muscle-path validity envelope ±45°/−10…+10° |
| P5 | Complete (2026-10-31) | `reports/P5/P5_REPORT.md`, `reports/P5/p5_tasks_summary.json`, `reports/P5/tasks/*`, `reports/P5/test_run_output.txt` | Limit stiffness repaired (38° → 1.5°). 8 configured tasks: 6 pass, 2 fail for documented model reasons (pinch not actuatable — M1; rest droop — J3). Inverse dynamics consistent (7.5e-15); static optimization verified |
| P6 | Complete (2026-10-31) | `reports/P6/P6_REPORT.md`, `reports/P6/reproducibility.json`, `schemas/*.schema.json`, `reports/P6/test_run_output.txt` | Session API + CLI; schema v1.0.0 (1.1.0 since P7); all 8 tasks reproduce bitwise; API = runner bitwise; cross-platform not tested |
| P7 | Complete (2026-11-01) | `reports/P7/P7_REPORT.md`, `reports/P7/viewer_check_output.txt`, `reports/P7/viewer_power_grip.png`, `tests/test_p7_viewer.py` | Pose match ≤ 1.2e-7 m; no physics nodes; playback only; no hands-on use yet |
| P8 | Complete (2026-11-01) | `reports/P8/P8_REPORT.md`, `reports/P8/*.json`, `reports/P8/clean_install_output.txt` | All references compared, failures reported: wrist 8/12, index 10/12. No force validation possible. Criteria are about reporting, not about the model passing |
| Fixes | Done 2026-11-01, with caveats | `reports/fixes/FIXES_REPORT.md` | 2 new patches; 8 of 8 tasks pass with caveats; P5 causal claim corrected; moment-arm validation unchanged (8/12, 10/12); grip inertia effect traced to contact softness; viewer used with real input, 4 defects addressed; other OS still untested |
| UI | Done 2026-11-01 | `docs/VIEWER_UI.md`, `tests/test_p7_viewer.py`, `reports/P7/viewer_pinch.png` | Viewer redesigned; no person has used it; no screen-reader test |
| P9 | **Documents done — not complete** | `reports/P9/P9_REPORT.md`, `README.md`, `docs/INSTALL_WINDOWS.md`, `docs/DEMO_SCENARIOS.md`, `docs/LIMITATIONS.md`, `docs/RELEASE_NOTES.md`, `CITATION.cff` | Open: install on a separate clean Windows machine; git repository and release tag (owner decision) |

## 2026-10-31 — P0 + P1 session
- Created `.venv` (Python 3.11.4) and pinned `mujoco==3.17.0`.
- Cloned MyoSim at `93b0ca8f` into gitignored `third_party/`.
- Found that MyoHand is now built at runtime by pruning MyoArm. The static XML was removed upstream in 0.2.0.
- Added `simulation/myosim_source.py` (pinned and verified loader), `simulation/inventory.py`, `scripts/p0_environment_audit.py`, `scripts/p1_model_audit.py`, and unit and integration tests.
- No upstream file has been modified or copied into the project.

## 2026-10-31 — P1 licence + source traceability
- Owner supplied the SimTK archives (MoBL-ARMS bimanual, 2nd-Hand) and the licence texts. They were extracted into gitignored `third_party/simtk_*`, and the licence texts archived in `data/provenance/licenses/`.
- Added `scripts/p1_osim_inventory.py` and `tests/test_osim_sources.py`.
- Findings:
  - MyoHand extrinsic forces come from MoBL-ARMS;
  - the EDC5 force is +100 N versus the source;
  - the intrinsic muscle parameters in the source are placeholders;
  - wrist flexion range was narrowed from ±70° to ±45°;
  - the two-row wrist and the moving via-points of the source were removed.
- Separately: four ONNX models were audited and not accepted (audit not published).

## 2026-10-31 — P1 closed
- The owner declared the intended use as non-commercial (research and education), which satisfies the MoBL-ARMS terms.
- All P1 acceptance criteria are met. 23/23 tests pass.

## 2026-10-31 — P2
- Rigid segments, glossary, naming, frames, kinematic semantics and UI contract implemented.
- P2 docs are now generated, with a drift test.
- Local visual pipeline: MuJoCo → Blender → GLB → Godot, verified end to end (≤ 1.4 µm).
- New findings:
  - thumb "flexion" joints are positive-extension;
  - MCP "abduction" is positive-radial for all digits;
  - 27/38 bodies have their inertial centre at the joint origin.

## 2026-10-31 — P3
- Kinematics API and an independent FK cross-check (1e-15) are implemented. Sweeps, a penetration check, a ROM comparison and a limit-compliance test were run.
- Findings:
  - T10: wrist deviation sign inverted vs MoBL-ARMS;
  - J1′: wrist ROM below ADL (Ryu 1991);
  - J5′: soft limits overrun by up to 38° under maximal agonist torque;
  - C3: adjacent finger proxies overlap.
- Correction to the P2 note: the deviation range is numerically inherited, but its meaning is inverted.
- Option B was applied with patch `wrist_rom_v1`:
  - the values are derived and validated from the MoBL-ARMS source;
  - the model is stable, and the patch puts no new muscle outside the active window;
  - finding T11 (RI5, FDP5, FDS4 `lengthrange`) is pre-existing and recorded.

  P3 is closed with documented residuals.

## 2026-10-31 — P4
- Added `simulation/muscles.py`: moment arms (r = −dL/dq), muscle state with MuJoCo's own active/passive split, and ablation.
- Added the muscle-path validity envelope to the wrist patch.
- Findings:
  - T11′: six non-functional muscles;
  - M7′: RI2 path break;
  - W1: muscle paths invalid beyond the upstream wrist range;
  - M12: PT reversal near full pronation.
- `mj_setLengthRange` was unstable on this model; geometric sampling was used instead.

## 2026-10-31 — Muscle repairs (owner request)
- Added patch `muscle_fixes_v1`:
  - EDC5 force set to its source value;
  - RI2 path made consistent with RI3–RI5;
  - 8 `lengthrange` values re-derived by MuJoCo's definition (median error on healthy muscles 0.78 mm).
- All muscles are functional, RI2 is smooth, and 39/39 directions are still correct.
- The BioHand reference model is now `wrist_rom_v1` + `muscle_fixes_v1`.

## 2026-10-31 — P5
- Added patch `limit_stiffness_v1` (numerical setting).
- Added the task runner, 8 task configs, joint brace and contact fixtures, telemetry CSV, static optimization, inverse dynamics, and determinism and Δt checks.
- Pinch failed with 4 policies; this is recorded as model finding P1, not tuned away. (Corrected 2026-11-01: the cause given then was wrong; see `reports/fixes/FIXES_REPORT.md`.)

## 2026-10-31 — P6
- Added `simulation/api.py` (session, frames, CSV/JSON export, Godot playback, fingerprint), `schemas/` v1.0.0 and `scripts/biohand.py`.
- Added jsonschema as a dependency.

## 2026-11-01 — P7, P8, P9
- P7: viewer completed and checked headlessly; tests found and fixed the cylinder orientation.
- P8: 100 published moment-arm values compared (12 measured, 12 attachment-derived, 76 source-model), sensitivity, performance and a clean-environment install.
- P9: release documents written. Two criteria stay open: a separate clean machine, and a release tag (needs git).
