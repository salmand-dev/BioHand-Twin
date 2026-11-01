# Validation Report P8 — Moment arms, sensitivity, reproducibility, performance

- Report ID: P8-2026-11-01
- Date: 2026-11-01
- Git commit: none (no repository yet)
- Model source / commit: MyoSim `93b0ca8f4ec90c9899ee7f05fee561e9911da91b`; hashes in `data/provenance/myohand_file_hashes.json`
- Model variant / entry point: MyoHand fragment `FRAGMENT_SPEC_BUILDERS["myohand_r"]`
- Build/compile warnings: none
- Project-side overrides of upstream values: `wrist_rom_v1`, `muscle_fixes_v1`, `limit_stiffness_v1`, and since the fixes `thumb_intrinsics_v1`, `passive_stiffness_v1` (`data/patches/`)
- Python / MuJoCo / Godot / Blender: 3.11.4 / 3.17.0 / 4.7.2 / 3.6.23
- OS / hardware: Windows 10; Intel i7-4610M; 8.5 GB RAM; AMD Radeon HD 8690M
- Solver / timestep: Newton, Euler integrator, 100 iterations, 0.002 s
- Random seed: 20261031 where used

This validates the listed comparisons only. It is **not** a clinical validation, and **"validated" must not be used for the model as a whole**.

## 1. Objective

Compare model moment arms with published data wherever a numeric reference could be retrieved, report all comparisons, and measure how much the task outcomes depend on uncertain parameters.

## 2. Reference sources and comparability

Reference values and pass criteria were written to `data/validation/reference_moment_arms.yaml` **before** the comparison was run.

| Set | Type | Source | Comparability limits |
|---|---|---|---|
| Wrist, 12 values | **Measured**, cadaver, tendon excursion, n = 10 | Garland et al. 2018, Table 1 (CC BY) | Reference is a mean over each specimen's own range (range not reported); the model mean uses the validity envelope. Model `pro_sup = 0` is assumed to be mid-prone. Cadaver group vs. a generic model |
| Index MCP, 12 values | **Derived** from measured attachments (An 1979), not direct excursion | Lee et al. 2015, Table 2 | The direct An 1983 values are only in figures, and were not digitised |
| All digits, 76 values | **Source model, not a measurement** | Lee et al. 2015, Tables 3–6 (2nd-Hand OpenSim model) | Tests conversion fidelity only; says nothing about anatomy. The fidelity set uses ±15° for ab/adduction (the file's coordinate range) where the paper lists a 30° span |

## 3. Method

Model value = mean of r = −dL/dq over the stated joint range (91 samples), with other joints at qpos0. Reference deviation uses ulnar +, while the model uses radial +, so the sign is flipped before comparing.

## 4. Results

### 4.1 Wrist vs. measured data: 8 of 12 within the criterion (sign correct and within ±2 SD)

| Axis | Muscle | Reference (mm) | Model (mm) | z | Result |
|---|---|---|---|---|---|
| Flexion–extension | FCR | 12.2 ± 2.1 | 17.08 | +2.3 | **FAIL** |
| | FCU | 11.3 ± 2.1 | 17.62 | +3.0 | **FAIL** |
| | ECRL | −6.6 ± 3.0 | −10.96 | −1.4 | pass |
| | ECRB | −10.8 ± 2.8 | −14.84 | −1.4 | pass |
| | ECU | −4.0 ± 0.9 | −7.24 | −3.6 | **FAIL** |
| | APL | 6.8 ± 1.9 | 6.44 | −0.2 | pass |
| Deviation | FCR | −10.2 ± 2.5 | −3.94 | +2.5 | **FAIL** |
| | FCU | 11.7 ± 4.2 | 10.54 | −0.3 | pass |
| | ECRL | −17.2 ± 3.6 | −23.57 | −1.8 | pass |
| | ECRB | −10.0 ± 2.2 | −14.17 | −1.9 | pass |
| | ECU | 23.7 ± 3.3 | 27.77 | +1.2 | pass |
| | APL | −23.6 ± 2.5 | −26.66 | −1.2 | pass |

RMSE 4.42 mm, Pearson r 0.979, sign agreement 12/12.

- **Pattern:** 9 of the 12 model magnitudes are larger than the reference; FCR deviation is much smaller. So the direction of action is right, but the size is biased.
- **Sign check:** the deviation signs here also independently support the P3 finding that positive `deviation_r` is radial in MyoHand.
- **Sensitivity to range:** averaging over the full ±70° instead of the envelope gives FCR 13.47 and FCU 15.24 mm. FCR would then be inside the band, but the range was fixed in advance, so the result stays FAIL.

### 4.2 Index MCP vs. values derived from measured attachments: 10 of 12

RMSE 1.61 mm, Pearson r 0.963. Flexion–extension passes 6/6.

Both failures are ab/adduction **sign** disagreements on small moment arms: FDP (reference −2.8, model +0.69 mm) and EDC (reference +2.5, model −0.41 mm). The 2nd-Hand source model has the same signs as MyoHand here (+1.3, −1.0), so MyoHand inherited them.

The RI2 repair moved its MCP flexion moment arm from 5.01 to 6.09 mm (reference 6.6). This supports the repair on one number; it does not prove it.

### 4.3 Conversion fidelity vs. 2nd-Hand (76 values)

RMSE 1.63 mm, Pearson r 0.959, sign agreement 94.7 %, 82.9 % within 2.1 mm, largest error 6.2 mm.

The largest differences include FDS5 ab/adduction (+2.2 vs −4.0 mm) and LU_RB3 ab/adduction (3.9 vs 8.35 mm).

These errors are far larger than the 0.38 ± 0.57 % moment-arm difference the 2022 MyoSuite paper reports for its conversion. The metrics and model versions differ, so these are not directly comparable, and the published figure should not be assumed to apply to the pinned model.

### 4.4 Functional tasks

| Comparison | Status |
|---|---|
| Grip force vs. measured grip strength | **Not performed: not comparable.** The output is a sum of contact normal forces on an illustrative fixture at chosen activations, not a dynamometer measurement |
| Pinch force | **Plausibility only.** Task force is about 0.8 N, while a quasi-static capacity estimate reaches the order of measured tip pinch. Not validation (`reports/fixes/FIXES_REPORT.md`, section 2) |
| Joint kinematics vs. measured trajectories | **Not performed:** no compatible reference trajectory was retrieved. ROM comparison is in P3 |
| EMG trends | **Not performed:** no compatible data, and activations are chosen by the task design |

## 5. Sensitivity (`reports/P8/sensitivity.json`)

Nine perturbations were applied to all 8 tasks: F_max ×0.8/×1.2, damping ×0.5/×2, hand inertia ×0.1/×10, upstream limit stiffness, contact stiffness, and timestep 1 ms.

- **Verdict changes:** only `pinch` changes, in 4 of 9 perturbations (F_max x1.2, joint damping x0.5, hand inertia x0.1, hand inertia x10), all against the 0.5 N threshold. All runs stayed finite with no warnings.
- **Kinematic task outcomes are insensitive.** Saturated closing and opening outcomes reach joint limits, so they can't reveal sensitivity.
- **Grip force is sensitive:** 213 N (F_max ×0.8) to 347 N (F_max ×1.2), against 273 N at baseline. It also reaches 339 N with hand inertia ×0.1 and 245 N with inertia ×10. The inertia effect was later traced to MuJoCo's mass-scaled contact softness (`reports/fixes/FIXES_REPORT.md`, section 5), so it is numerical.

## 6. Reproducibility and performance

| Check | Result |
|---|---|
| Same configuration twice, all 8 tasks | Bitwise identical (`reports/P6/reproducibility.json`) |
| Timestep 2 ms vs. 1 ms | Same verdict on all tasks |
| Clean environment from `requirements-lock.txt`, project copy without model files | 0 failures: 50 passed/92 skipped with no model, 127 passed/15 skipped with pinned MyoSim (`reports/P8/clean_install_output.txt`). Same machine |
| Other OS or CPU | **Not tested** |
| Simulation speed | 0.29 ms per step, 7.0× real time in the latest run (7× to 20× across runs today, depending on machine load; same laptop, so treat as approximate) |
| Telemetry frame with all data | 17.6 ms in the latest run (about 6–18 ms across runs today) |
| Viewer | about 45-48 FPS rendered at 1600x900 on an AMD HD 8690M, 3.2 ms per applied frame, after the UI redesign (60 FPS and 2.1 ms before it; the richer muscle and plot panels cost this) |

## 7. Failures and deviations

- 4 of 12 wrist comparisons and 2 of 12 index comparisons fail, as listed above. The added thumb muscles and joint springs do not change these results. A fit to the same data is reported separately and is not validation (`reports/fixes/FIXES_REPORT.md`, section 4).
- Functional force validation was not possible.
- The direct An 1983 and Horii/Loren values were not obtained numerically.

## 8. Limitations

- Only 6 wrist tendons have measured reference values here. No measured reference was compared for the thumb, the other fingers, or the 12 intrinsic actuators beyond index MCP. Intrinsic forces remain placeholders.
- Comparisons are means over a range, so they can't show errors in curve shape.
- Moment-arm comparisons were run on the wrist envelope with the other joints at zero. Pronation dependence, which Garland et al. flag for ECU, was not tested.
- A generic model compared with a small cadaver group: no subject matching or scaling.
- Numbers were read from retrieved papers; Lee 2015 by text extraction, and Garland's signs from the page image.

## 9. Conclusion (scope-limited)

For wrist tendons, the model's direction of action agrees with cadaver data for all 12 values tested, but 4 magnitudes fall outside 2 SD and most are overestimated. Index MCP flexion moment arms agree with attachment-derived values. Functional force outputs are unvalidated and are sensitive to non-physiological parameters. **No clinical claim is supported.**

## 10. Reproduction

```bash
.venv/Scripts/python.exe scripts/p8_moment_arm_validation.py
```

```bash
.venv/Scripts/python.exe scripts/p8_sensitivity_performance.py
```
