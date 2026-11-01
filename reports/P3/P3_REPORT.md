# P3 — Kinematics and Joint Constraints: Report

- Date: 2026-10-31
- Model: MyoHand fragment, MyoSim `93b0ca8f4ec90c9899ee7f05fee561e9911da91b`
- Toolchain: MuJoCo 3.17.0, Python 3.11.4
- Git commit: none (no repository yet)
- Evidence:
  - `reports/P3/test_run_output.txt` — **70 passed, 0 skipped**; ruff and mypy clean
  - `p3_kinematics_output.json`
  - `fingertip_trajectories.csv`
  - `wrist_convention_check.json`
  - `axis_convention_check.json`
  - `*_console.txt`

## Implementation

| Item | File |
|---|---|
| Rigid-body math: Rodrigues rotation, homogeneous transforms, hinge about an anchor | `biomechanics/rigid.py` |
| Kinematics API. Radians only, with a range-validated `qpos_from_angles`. `forward` uses pure kinematics and never steps the dynamics. Also: fingertip sites (`THtip_r`, `IFtip_r`, `MFtip_r`, `RFtip_r`, `LFtip_r`, all upstream), site Jacobians, full-range sweeps | `simulation/kinematics.py` |
| Audit: sweeps, composite flexion, penetration, ROM comparison, limit compliance, reproducibility | `scripts/p3_kinematics_audit.py` |
| Source-convention checks against MoBL-ARMS and 2nd-Hand | `scripts/p3_wrist_convention_check.py`, `scripts/p3_axis_convention_check.py` |
| 17 new tests | `tests/test_p3_kinematics.py` |

## Verification results

| Check | Result |
|---|---|
| Independent numpy FK vs MuJoCo (30 random in-range poses, all 38 bodies) | max error 1.2e-15 (positions and rotations). Confirms the transforms, the hinge order within a body, and radians |
| Rodrigues rotation vs `mju_axisAngle2Quat`/`mju_quat2Mat` | ≤ 1e-12 |
| Fingertip Jacobian (`mj_jacSite`) vs central finite differences, 5 tips | ≤ 1e-7 m/rad |
| Angle units | Passing 90 (degrees) where radians are expected is rejected as out of range. Degrees exist only in `to_display_degrees` |
| Neutral pose reproducibility | Two independent builds give bitwise-identical FK |
| Full-range sweeps (23 joints × 181 samples, all 5 fingertips) | All within model range; all continuous (each step ≤ lever arm × Δθ) |
| Composite finger flexion (all MCP/PIP/DIP from 0 → 90°) | Fingertip travel: index 0.109 m, middle 0.120 m, ring 0.111 m, little 0.093 m (CSV with units and metadata) |
| Coupled coordinates / equality constraints | None in the model (neq = 0). The source's 79 couplers are not represented (gaps T6, T7) |

## Findings

### 1. Wrist deviation sign is inverted relative to the source (T10, high)

- Using MoBL-ARMS's own spline and coupler definitions, +deviation moves the fingertip **ulnarly**. In MyoHand, +deviation moves it **radially**. The cosine between the two directions is −0.994.
- The source range −10…+25° therefore means 10° radial / 25° ulnar. MyoHand kept the same numbers with the opposite sign, so it permits **25° radial / 10° ulnar**.
- Wrist flexion direction agrees (cos 0.9996). Of the 22 hand joints compared with 2nd-Hand, **21 follow the source convention**, the thumb included.
- Note: in P2 I described the deviation range as "inherited (2nd-Hand)". Numerically that is true, but semantically it is wrong because of the sign inversion.

### 2. Wrist ROM is below ADL requirements (J1′, high for wrist tasks)

| Motion | MyoHand | ADL requirement (Ryu et al. 1991) | Covered? |
|---|---|---|---|
| Wrist flexion | 45° | 54° | ✘ |
| Wrist extension | 45° | 60° | ✘ |
| Ulnar deviation | 10° | 40° | ✘ |
| Radial deviation | 25° | 17° | ✔ |
| MCP / PIP / DIP flexion (0–90°) | — | Bain et al. 2015 functional ranges: MCP 19–71°, PIP 23–87°, DIP 10–64° | ✔ all 4 digits |

### 3. Joint limits are soft relative to muscle strength (J5′, high near limits)

- The test applied each joint's maximal agonist muscle torque at its limit: Σ F_max·|moment arm|, no antagonists, gravity off, 3 s.
- Resulting penetrations: wrist flexion 31.5°, MCP III flexion 36.2°, thumb CMC (negative side) 38.4°.
- The limits use the MuJoCo defaults, solref (0.02, 1) and solimp (0.9, 0.95, 0.001).
- This is a worst case. With co-contraction (P1), the overshoot was ≤ 8.8°.

### 4. Collision proxies of adjacent fingers overlap (C3, medium)

- Adjacent-finger capsules interpenetrate by 7–15 mm in flexion and in finger-convergence poses, and already overlap up to 10 mm at qpos0. The fragment has no self-collision.
- No fingertip-into-palm penetration appears in the poses tested.
- Thumb full flexion adds no new penetration.

### 5. Hand orientation and units

The zero pose is a fixed upstream preview pose: forearm along +X, palm down (P2).

## Owner decision: option B — BioHand patch `wrist_rom_v1` (applied 2026-10-31)

- **Mechanism.** `data/patches/wrist_rom_v1.yaml` is applied to the MjSpec at build time by `simulation/patches.py`. `build_biohand_model()` now builds the **BioHand reference model**. Upstream files are untouched: hash and clean-checkout tests still pass.
- **No hand-typed numbers.** Each patched value is recomputed by `Patch.validate` from the MoBL-ARMS source coordinate range (`data/reference_osim_inventory.json`) and the declared sign mapping. It is refused if it does not match, or if the upstream value has changed. Tests cover tampered values, the wrong sign and a stale upstream.
- **Changes:**
  - `deviation_r` −10…+25° → **−25…+10°**, i.e. 25° ulnar / 10° radial, the MoBL meaning;
  - `flexion_r` ±45° → **±70°** (MoBL).

  Nothing else changes: a test compares forces, length ranges, masses, geometry and damping.
- **Verified** (`reports/P3/patch_check.json`, `tests/test_patches.py`):
  - −25° deviation moves the middle fingertip > 1 cm ulnarly.
  - The patched model is stable in all three 2 s smoke runs: finite, no MuJoCo warnings, limit overshoot ≈ upstream.
  - Muscle operating range: the patch puts **no new muscle** outside MuJoCo's active force window `[lmin, lmax] = [0.5, 1.6]`. Wrist-crossing muscles stay at normalised lengths 0.63–1.22.
  - 21 muscles spend more time outside their upstream-calibrated `lengthrange` (up to 30 %, ECRL). That is an extrapolation of the upstream calibration, so **status: provisional until P4** verifies moment arms and forces over the new range.
- **New pre-existing finding T11** (independent of the patch): RI5, FDP5 and FDS4 are outside their calibrated `lengthrange` in 97–100 % of postures even with the upstream ranges. RI5 reaches non-physical normalised lengths (−0.48).

### ROM against ADL requirements (Ryu et al. 1991)

| Motion | Upstream | BioHand (patched) | ADL max | Covered after patch |
|---|---|---|---|---|
| Flexion | 45° | 70° | 54° | ✔ |
| Extension | 45° | 70° | 60° | ✔ |
| Ulnar deviation | 10° | 25° | 40° | ✘ |
| Radial deviation | 25° | 10° | 17° | ✘ |

The remaining deviation shortfall is **the source model's own range** (MoBL-ARMS). Extending it would require a new sourced ROM value, so it stays a documented limitation, not a patch.

## Acceptance criteria (ROADMAP P3)

| Criterion | Status |
|---|---|
| Neutral pose loads reproducibly | **PASS** |
| Joint sweeps stay within model-defined limits | **PASS** for kinematic sweeps and API inputs. Dynamic limits are soft (finding 3), handed over to P5 |
| Fingertip trajectories continuous | **PASS** |
| Fingertip trajectories anatomically plausible | **PASS with a documented residual.** Directions now agree with the source for all wrist and hand joints. Flexion/extension ROM covers ADL; finger ROM covers functional ranges. Residual: deviation ROM equals the source model and is below ADL maxima |
| Numerical tests verify transforms and angle units | **PASS** |

**P3 status: COMPLETE (2026-10-31)**, carrying forward these items:
- the residual deviation ROM limitation;
- the provisional wrist patch (P4 must verify muscles over the new range);
- soft limits (P5);
- finger proxy overlap (C3);
- muscle `lengthrange` anomalies (T11).

## Limitations

- ROM references were retrieved at abstract level via search summaries, not full text.
- The source-convention checks are first-order, at the neutral pose only.
- The penetration check uses the model's crude capsule proxies, not the bone meshes.
- The limit-compliance test is a static worst case, not a physiological task.

## Reproduction

```bash
.venv/Scripts/python.exe scripts/p3_kinematics_audit.py
```

```bash
.venv/Scripts/python.exe scripts/p3_wrist_convention_check.py
```

```bash
.venv/Scripts/python.exe scripts/p3_axis_convention_check.py
```

```bash
.venv/Scripts/python.exe -m pytest -v
```
