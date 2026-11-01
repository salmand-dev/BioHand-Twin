# P5 — Dynamics and Functional Tasks: Report

- Date: 2026-10-31
- Model: BioHand reference model = MyoHand fragment (MyoSim `93b0ca8f`) + `wrist_rom_v1` + `muscle_fixes_v1` + `limit_stiffness_v1`
- Solver settings (upstream, unchanged): timestep 0.002 s; Euler integrator; Newton solver; 100 iterations
- Toolchain: MuJoCo 3.17.0, Python 3.11.4; no git repository
- Evidence: `reports/P5/`
  - `p5_tasks_summary.json`
  - `tasks/<id>/{timeseries.csv, summary.json}`
  - `limit_stiffness_check.json`
  - `pinch_target*.json`, `pinch_policy_opt*.json`
  - `test_run_output.txt`

## 1. Soft joint limits (gap J5′) — repaired by patch `limit_stiffness_v1`

- **The fix.** Every joint-limit time constant is set to **2 × timestep = 0.004 s** (MuJoCo's lower bound for stable soft constraints). The damping ratio is unchanged.
- **Classification.** This is a *numerical solver setting* ("tuned"), not physiology. The validator enforces the 2 × timestep rule.
- **Result** (`limit_stiffness_check.json`, all 23 joints × both limits under maximal agonist muscle torque):

| | Before | After |
|---|---|---|
| Worst static penetration | 38.4° | **1.5°** |
| Random-control overshoot | 9.8° | **0.65°** |
| Numerical health | stable | stable; kinetic energy practically unchanged; no warnings |

## 2. Task framework

- **Runner.** `simulation/tasks.py` executes YAML configs from `data/experiments/tasks/`. Each config defines:
  - an initial state (rad);
  - a control policy: open-loop activation phases, or closed-loop pose tracking;
  - a stopping condition (duration);
  - numerical success criteria, defined in advance from the task definition;
  - scenario fixtures (apparatus, recorded in metadata).
- **Fixtures:**
  - **Joint brace:** forearm/wrist held by MuJoCo joint equality constraints with the same 2 × timestep stiffness. It holds to ≤ 0.21°.
  - **Explicit contact pairs.**
  - **Static grip cylinder:** illustrative, 40 mm diameter. The runner **refuses** to start if a fixture touches the hand at t = 0.
- **Telemetry** (CSV with units and metadata, 50 Hz): joint angles, activations, muscle tensions, tendon lengths, fingertip positions, contact normal force, and a flag for being outside the muscle-path validity envelope.
- **Per-task health:** NaN check, MuJoCo warnings, maximum limit violation, maximum kinetic energy, bitwise repeatability, and a re-run at Δt = 1 ms.
- **Inverse dynamics** (`simulation/inverse.py`, MuJoCo `mj_inverse`): equals the forward-simulation forces to **7.5e-15 N·m**, including brace constraints.
- **Static optimization** (`simulation/static_opt.py`): bounded least squares on MuJoCo's own affine muscle force (gain·a + bias).
  - The affine form reproduces MuJoCo forces exactly.
  - A known feasible torque is recovered to ≤ 1e-6 N·m.

> **Superseded 2026-11-01.** Task results below describe the model at P5. The pinch cause stated in section 4 was wrong: a 2x2 experiment shows joint stiffness, not the missing muscles, decided contact. Current results: `reports/fixes/FIXES_REPORT.md`. The JSON files in `reports/P5/` now hold regenerated results for the current model.

## 3. Results (8 tasks)

| Task | Result | Key metric (criterion) | Notes |
|---|---|---|---|
| hand_closing | **PASS** | flexion fraction 1.00 (≥ 0.5) | braced wrist |
| hand_opening | **PASS** | flexion fraction −0.0007 (≤ 0.25) | from 60° flexed start |
| thumb_opposition | **PASS** | thumb–index distance ratio 0.55 (≤ 0.6) | |
| wrist_flexion_extension | **PASS** | +1.159 / −1.227 rad (±0.3) | **80 % of samples outside the muscle validity envelope**, so kinematics only; no force claims |
| wrist_deviation | **PASS** | +0.176 / −0.317 rad (±0.1) | **93 % outside the envelope** (same caveat) |
| power_grip_proxy | **PASS** | 13 hand bodies in contact (≥ 3); mean normal force 276 N (≥ 1 N) | Force magnitude is a policy-dependent model output at 50 % activation: **not validated, no physiological claim** |
| rest_neutral | **FAIL** | final speed 0.25 rad/s (< 0.05) | Palm-down neutral pose + no passive joint stiffness (gap J3): the unbraced wrist droops toward its limit under gravity and has not settled in 2 s |
| pinch | **FAIL** | contact force 0 N (> 0, ≥ 0.5 N) | See §4 |

All 8 tasks are numerically healthy (finite, no warnings), bitwise deterministic, and give the **same verdict at Δt = 1 ms**.

## 4. Pinch: not achievable with this muscle set (new finding P1)

Four independent policies were tried. The success criteria were never changed.

| Policy | Method | Result |
|---|---|---|
| v1 | open loop FPL + FDS2 + FDP2 | no contact |
| v2 | open loop OP + FDS2 | no contact. In this model **OP drives the thumb into full CMC abduction (+45°)**, away from the index |
| v3 | closed-loop pose tracking with static optimization to a kinematically reachable pad-pinch pose (`pinch_target.json`; the pose itself is reachable, with capsule overlap −2 mm) | no contact. The static optimization cannot generate the torques: median residual ≈ 1.9 N·m. Thumb MP flexion with the IP near neutral requires the short thenar muscles (FPB/APB/adductor), which are **absent** (gap M1). FPL always flexes the IP more |
| v4 | forward-dynamics optimization of 12 thumb/index activations (cf. McFarland et al. 2023), objective on contact-enabled pairs and contact force | best gap 16.5 mm, no contact |

- A first v4 objective used plain geometric distance. The optimizer **exploited** interpenetration with a segment that had no contact pair, giving a false "contact". This was caught and is kept as `pinch_policy_opt_v1_exploited.json`.
- **Conclusion:** a thumb–index pinch is not achievable by this model's actuators under these conditions. This is a model limitation, not a tuning gap. It means **no pinch-force output is available** from BioHand Twin at present.

## 5. Acceptance criteria (ROADMAP P5)

| Criterion | Status |
|---|---|
| Simulation stable under documented solver/timestep settings | **PASS.** All tasks are finite with no warnings; verdicts are identical at Δt = 2 ms and 1 ms |
| No unexplained energy explosion or NaN/Inf | **PASS.** Maximum kinetic energy ≤ 0.13 J across tasks |
| Outputs include joint angles, activations/forces, tendon lengths, external contact forces | **PASS.** Per-task CSV telemetry with units |
| Task success criteria numerical and repeatable | **PASS.** Bitwise deterministic. 6/8 tasks pass; the 2 failures are explained model limitations, not instability |

**P5 status: COMPLETE (2026-10-31).** The phase criteria concern framework, stability and repeatability, and they pass. Two functional tasks fail for documented model reasons:
- pinch: missing thenar muscles and OP behaviour;
- rest: no passive joint stiffness.

These are carried forward. They must not be "fixed" by altering anatomy without sources.

## 6. Limitations

- Fixture parameters are illustrative apparatus: the cylinder size and placement, and default MuJoCo contact parameters. Grip force magnitudes are therefore not comparable with measured grip strength.
- Activation levels are task-design choices, not physiological recruitment.
- Inertias remain non-anatomical (I1). This affects timing and dynamic magnitudes.
- Wrist tasks move outside the muscle-path validity envelope. Use their kinematics only.

## Reproduction

```bash
.venv/Scripts/python.exe scripts/p5_limit_stiffness_check.py
```

```bash
.venv/Scripts/python.exe scripts/p5_run_tasks.py
```

```bash
.venv/Scripts/python.exe -m pytest -v
```
