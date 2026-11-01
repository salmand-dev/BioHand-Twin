# P4 — Muscle–Tendon Actuator Integration: Report

- Date: 2026-10-31
- Model: BioHand reference model = MyoHand fragment (MyoSim `93b0ca8f`) + patch `wrist_rom_v1`
- Toolchain: MuJoCo 3.17.0, Python 3.11.4
- Git commit: none (no repository yet)
- Evidence:
  - `reports/P4/test_run_output.txt`
  - `p4_muscle_audit.json`
  - `moment_arm_curves.json` (329 curves)
  - `p4_muscle_audit_console.txt`

## Implementation

| Item | File |
|---|---|
| Moment-arm matrix in the biomechanical convention (r = −dL/dq, m/rad: r > 0 drives the joint positive). Muscle state: activation, length, velocity, normalised length, tension, **active/passive split computed by MuJoCo's own `mju_muscleGain`/`mju_muscleBias`** (no second Hill model). Ablation via `without_actuator` (gear 0 on a model copy, so no active or passive force) | `simulation/muscles.py` |
| `muscle_validity_envelope` and `outside_muscle_validity` | `simulation/patches.py` |
| Audit: moment-arm curves over the full BioHand range on two posture backgrounds; activation dynamics; isolated activation of each muscle; ablation of each muscle on two tasks; geometric length sampling (20 000 poses) | `scripts/p4_muscle_audit.py` |
| Generated `docs/MUSCLE_MODEL.md` now also lists each actuator's **actions at the neutral pose** in anatomical terms, plus the validity limits | generator |
| 9 new tests | `tests/test_p4_muscles.py` |

## Verification results

| Check | Result |
|---|---|
| Moment arms vs finite differences of actuator length | ≤ 1e-8 m/rad (a random pose after 150 steps) |
| Tension = MuJoCo active + passive | exact (≤ 1e-9 N); tension ≥ 0 |
| Joint torque = rᵀ·T equals MuJoCo `qfrc_actuator` | ≤ 1e-8 N·m |
| Activation dynamics (FDP3 step) | 63 % rise in 8 ms, 63 % fall in 28 ms (τ_act 0.01 s, τ_deact 0.04 s) |
| **Isolated activation** (u = 1 for 0.2 s, gravity off, vs. all-zero baseline) | **39/39** muscles move their primary free joint in the direction predicted by r. Free-joint agreement is 100 % |
| **Ablation** (each of 39 muscles removed; 2 tasks × 1 s) | Quantified for all 39. Largest: FDP4 (106 mm fingertip change, flexor task; 133 mm random task), FDS5 (77 mm), EDC5 (109 mm random), OP (100 mm random) |
| Moment-arm continuity inside the validity envelope (wrist flexion ±45°, 0.25° steps, all 39 muscles) | every step < 1 mm |

## Findings

1. **Six muscles are effectively non-functional (T11′).** FDS4, FDP5, RI5, LU_RB4, UI_UB4 and LU_RB5 have upstream `lengthrange` values that do not match their geometry. RI5 reaches a non-physical normalised length (−0.48). Ablation confirms it: removing FDS4 or LU_RB4 changes the fingertips by < 0.1 mm. This is an upstream defect.
2. **RI2 path break (M7′).** The moment arm jumps by 10 mm near 10° of MCP II flexion, and RI2 carries 11.9 N of passive tension at rest. Upstream defect.
3. **Wrist patch: muscle paths are not valid in the extended range (W1).** There are path breaks (FDS2/FDS3 at about −66°) and reversals (FCR, FCU, ECRL near the extremes).
   - Decision implemented as metadata, with no behaviour change: kinematics keep the source ROM (±70°, −25…+10°), but muscle forces and torques are claimed only inside the **validity envelope**, flexion ±45° and deviation −10…+10°.
   - The envelope is the intersection of the upstream and BioHand ranges, both source values.
   - P6 telemetry must flag poses outside it.
4. **PT reverses near full pronation (M12, [CHECK]).** Its moment arm is negative from +65° to +90° of pronation.
5. **MuJoCo `mj_setLengthRange` instability.** It was numerically unstable on this model (NaN in QACC at DOF 0), so geometric sampling was used as the evidence for finding 1.

## Acceptance criteria (ROADMAP P4)

| Criterion | Status |
|---|---|
| Each actuator has its source and semantics documented | **PASS** — `docs/MUSCLE_MODEL.md`: provenance (inherited/modified/added), MuJoCo semantics, derived actions per actuator, validity limits; glossary names |
| Activation changes produce the expected sign/direction in isolated tests | **PASS** — 39/39 |
| No unsupported origin/insertion or physiological parameter fabricated | **PASS** — no muscle parameter changed. The validity envelope uses only source ranges, and defective muscles are flagged, not "fixed" |
| Ablation can disable one actuator and quantify the change | **PASS** — `without_actuator`; all 39 quantified on 2 tasks |

**P4 status: COMPLETE (2026-10-31).** The following carry forward:
- 6 non-functional muscles (T11′) and the RI2 path break (M7′): flagged; any correction would be an owner decision;
- the wrist muscle-path validity envelope (W1);
- the PT reversal (M12);
- soft joint limits (J5′) → P5.

## Not done in P4 (by design)

- Comparing moment arms with published cadaver data (e.g. An 1983; Lee 2015 index data) belongs to P8 validation and needs those data.
- Correcting defective muscles: options would be recomputed `lengthrange` (derived) or exclusion. Each is an explicit owner decision with documented consequences.

## Reproduction

```bash
.venv/Scripts/python.exe scripts/p4_muscle_audit.py
```

```bash
.venv/Scripts/python.exe scripts/p2_generate_docs.py
```

```bash
.venv/Scripts/python.exe -m pytest -v
```

## Addendum — repairs (owner request "fix the defects", 2026-10-31)

Patch `muscle_fixes_v1` (`data/patches/muscle_fixes_v1.yaml`, generated by `scripts/p4_derive_muscle_fixes.py`, status: provisional) is part of the BioHand reference model.

| Defect | Repair | Basis |
|---|---|---|
| EDC5 force 139.4 N | 39.4 N | MoBL-ARMS source value (validated against the parsed .osim) |
| RI2 path break | Path rebuilt without the MCP cylinder | Same as converted siblings RI3–RI5. Upstream `lengthrange` matches the no-cylinder geometry (0.004 mm) |
| Broken `lengthrange` (8 muscles) | Re-derived: geometric min/max over the upstream joint ranges | MuJoCo's definition. The method reproduces the 31 healthy muscles with a median error of 0.78 mm |

Verification (`reports/P4/p4_muscle_audit.json` re-run on the repaired model; pre-fix evidence kept in `*_pre_fix.json`; `tests/test_muscle_fixes.py`):

| Check | Before | After |
|---|---|---|
| Minimum share of a muscle's geometric span inside its `lengthrange` | 0 % | 59 % |
| Poses outside the active window | up to 83 % (RI5) | 0 % for every muscle |
| FDS4 removal effect | 0.09 mm | 33 mm |
| FDP5 removal effect | 2.5 mm | 58.6 mm |
| FDP2–5 effects in the flexor task | 1.4–106 mm | 24–26 mm (balanced) |
| RI2 passive tension at rest | 11.9 N | 0.07 N |
| RI2 moment arm | breaks near 10° | smooth MCP flexor, max step 0.07 mm |

- Isolated activation is still **39/39**.
- The model stays stable (finite, no warnings).
- Full-activation limit overshoot rose from 0.15 to 0.21 rad, because the repaired muscles now produce force. This is handled under soft limits (P5).
- Independent re-sampling reproduces every derived range.

**Not repaired:**
- W1 (paths beyond the upstream wrist range) — no source geometry exists, so it stays mitigated by the validity envelope;
- M12 (PT reversal);
- T3 (intrinsic placeholder forces);
- I1 (inertias);
- missing muscles.

None of these has a source-backed fix available.
