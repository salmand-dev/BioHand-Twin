# Fixes Report: failed criteria and untested items

- Date: 2026-11-01. Requested by the project owner after the P9 report.
- Reference model is now MyoHand fragment + `wrist_rom_v1`, `muscle_fixes_v1`, `limit_stiffness_v1`, `thumb_intrinsics_v1`, `passive_stiffness_v1`.
- Optional, not in the reference model: `moment_arm_calibration_v1`.
- Evidence: `reports/fixes/*.json`, `reports/fixes/viewer_pinch_manual.jpg`, `reports/P7/viewer_pinch.png`, `reports/P9/test_run_output.txt`.

This report states what changed and what the evidence supports. **No clinical validity is claimed.**

## 1. Correction of an earlier conclusion

P5 concluded that pinch failed **because** APB, FPB and ADP are missing (finding P1). **That causal claim was wrong.** A 2x2 experiment with the same task and policy shows:

| Same task and policy | No joint springs | With joint springs |
|---|---|---|
| Without new muscles | 0.056 N, fail | 1.133 N, pass |
| With new muscles | 0.067 N, fail | 0.794 N, pass |

Joint springs decide whether contact is sustained. Without them, the thumb and wrist-free joints have nothing holding them in the target. The new muscles do not. What the new muscles change is force capacity (section 2.3).

## 2. Pinch

### 2.1 New muscles (`thumb_intrinsics_v1`)
APB, FPB (2 heads) and ADP (2 heads) are absent from MyoHand, MoBL-ARMS and 2nd-Hand, so they come from Mirakhorlo et al. 2016 (CC BY 4.0): attachment points digitized on **one cadaver specimen**, and PCSA.

- **Mapping to the model, by rule:** a least-squares similarity over 6 joint centres for the palm, and a bone frame for the thumb phalanx. Palm residual 6.96 mm RMS, 11.45 mm at worst. This is the size of the geometric uncertainty.
- **Force:** PCSA x 0.55 N/mm2 (O'Brien et al. 2010, measured on the quadriceps, not on hand muscles): APB 24.1 N, FPBs 11.6 N, FPBd 9.1 N, ADPo 71.1 N, ADPt 29.3 N.
- **Checks fixed before running:** APB abducts; FPB flexes the MCP; ADP adducts and flexes (as summarised from Smutz et al. 1998). All passed at the neutral pose. The numeric moment arms were not compared with data.
- **Not physiological muscles:** they use the same generic MuJoCo curves as all other actuators, with straight paths and no sesamoids. OP keeps its 180 N source value, much larger than its PCSA here would give.

### 2.2 Task and policy
- **Criterion unchanged:** contact in the last 0.5 s, mean at least 0.5 N. **The task was made stricter:** only distal phalanx pairs count.
- **Rejected policy v5:** an open-loop optimizer passed with the old pairs, but with the index curled to its limits, the thumb MCP at its extension limit and tips 6.4 cm apart. That is not this task, so it was discarded. The first optimizer window (1 s) also found contact that was lost by 1.5 s.
- **Policy of record v6:** pose tracking plus static optimization, to the pad-to-pad target of `reports/P5/pinch_target_pad.json`.

| Result, 2 s task | Value |
|---|---|
| Force in last 0.5 s, mean / min | 0.794 / 0.596 N |
| Contact geoms at the end | distal_thumb_coll_r, distph2_coll_r |
| Joints within 3 deg of a limit | 0 |
| Deepest penetration (first impact) | 5.653 mm |
| 10 s run: in contact, min / mean / max force | 99.8 % of the time; 0.0 / 0.916 / 2.204 N |
| 10 s run: largest pose error at the end | 1.6 deg |

**Weaknesses, stated plainly:**
- The force is small and unsteady, with brief dropouts (0.19 % of a 10 s run).
- The hand is still moving at 2 s; the pose target is reached only by about 8-10 s.
- A step target causes a first impact (about 16 N at 0.1 s). This is not a physiological motion.
- **It is fragile against the 0.5 N threshold:** 9 of 12 cases pass in `pinch_evidence.json`, and 4 of 9 P8 perturbations change the verdict (F_max x1.2, joint damping x0.5, hand inertia x0.1, hand inertia x10). Contact itself persisted in all 12 listed cases.
- The activations come from an optimizer, not from measured muscle patterns.

### 2.3 Force capacity
A linear program gives the largest normal force the muscles can balance, over 500 sampled postures with pad contact.

| N | Median | 95th percentile | Max | Feasible postures |
|---|---|---:|---:|---:|
| With new muscles, friction 1 | 5.3 | 41.39 | 71.3 | 80 % |
| Without, friction 1 | 1.44 | 18.35 | 61.4 | 57 % |
| With new muscles, no friction | 0.84 | 12.84 | 57.11 | 62 % |
| Without, no friction | 0.0 | 4.46 | 31.73 | 42 % |

Measured tip pinch means are 38.2 N (women) and 60.8 N (men) (Park et al. 1998, n = 1018). The model's best sampled posture reaches the same order. **This is a plausibility check, not validation:** it takes the best of many postures, assumes one contact point, is quasi-static, and depends on OP's large force. The dynamic task produces about 1 N because the controller only tracks a pose; no pinch-strength task was simulated.

## 3. Rest (`passive_stiffness_v1`)

| Joint | Target total (N m/rad) | Evidence |
|---|---|---|
| `flexion_r` | 0.7875 | measured |
| `deviation_r` | 1.4775 | measured |
| `pro_sup_r` | 0.213 | measured |
| `mcp2_flexion_r` | 0.0391 | measured |
| `cmc_flexion_r` | 0.15 | assumed_in_published_model |
| `cmc_abduction_r` | 0.15 | assumed_in_published_model |
| `mp_flexion_r` | 0.1 | assumed_in_published_model |
| `ip_flexion_r` | 0.05 | assumed_in_published_model |
| `mcp3_flexion_r` | 0.0391 | estimated_by_analogy_with_index_mcp |
| `mcp4_flexion_r` | 0.0391 | estimated_by_analogy_with_index_mcp |
| `mcp5_flexion_r` | 0.0391 | estimated_by_analogy_with_index_mcp |

- **Measured:** wrist and forearm (Formica et al. 2012, n = 10), index MCP (Shi et al. 2020, n = 4). The value is the mean of the two directions, because a MuJoCo joint spring is linear.
- **Not measured:** thumb values are assumed inputs of a published simulation (Wu et al. 2009). MCP 3-5 are estimated from the index. No measured value was found for them.
- **Double counting avoided:** spring = target - stiffness of the model's passive muscles at neutral. That muscle term is about 0 at the wrist, so the model's passive muscle does not produce the stiffness Formica et al. attribute mostly to muscle. This is a separate model finding.
- **Not covered:** MCP abduction, PIP, DIP keep k = 0.

**Result:** the wrist no longer sinks to its limit (about 9 deg of flexion droop). The forearm rotates about 28 deg under the hand's weight.

**Declared test change:** the horizon went from 2 s to 4 s with the 0.05 rad/s threshold unchanged. **At the original 2 s the check still fails** (0.091 rad/s), because the upstream forearm damping gives a time constant of about 2.3 s. At 4 s it passes (0.026 rad/s). The project owner reviewed and accepted the 4 s horizon on 2026-11-01. It is a judgement, not a measured fact.

## 4. Moment arms: not fixed in the reference model, on purpose

Moving tendon paths until they match Garland et al. and then counting that as agreement would be circular. So:

| Set | Validation result (unfitted, reference model) | After fitting to the same data (optional patch) |
|---|---|---|
| Wrist, measured (Garland 2018) | **8 of 12** | 11 of 12 |
| Index MCP, derived (Lee 2015) | **10 of 12** | 12 of 12 |

- **The left column remains the validation result.** The right column is a fit.
- `moment_arm_calibration_v1` moves 10 path sites, by up to 12.73 mm, and is labelled TUNED. It is not in `DEFAULT_PATCHES`. No held-out data exist to test it.
- FCR deviation still fails after fitting (reference -10.2 mm, fitted -5.17 mm); site shifts near the wrist do not move it enough.

## 5. Grip force

| Test | Result |
|---|---|
| Baseline, settled (8 s) | 272.34 N |
| Hand inertia x0.1 / x10, settled | 340.03 / 244.1 N |
| Inertia effect after settling | 95.93 N, so it is **not** a transient |
| Same, absolute contact parameters | 4.67-10.93 N |
| Activation 0.25 / 0.5 / 0.75 / 1.0 | 127.22 / 272.32 / 486.52 / 658.41 N |
| F_max x0.8 / x1.2 | 212.27 / 347.89 N |
| Measured grip (Park 1998) | 266.7 N women, 467.8 N men |

- **Cause found:** MuJoCo's default contact softness scales with effective mass. With absolute contact parameters, the inertia effect falls by about 90 %. The dependence is numerical, not physiological. The default contact settings were left as they are, because no measured contact stiffness was retrieved.
- **The measured values are not comparable:** the output is a sum of contact normal forces on a fixed 40 mm cylinder; a dynamometer reads a single handle force. Full activation gives 658.41 N, above the male mean, which is another reason not to read this as grip strength. **Grip force remains unvalidated.**

## 6. Viewer: first real use

Used with real mouse and keyboard input through the desktop, on Windows, not by a person's own hands and with no usability study.

| Tested | Result |
|---|---|
| Run dropdown, Play/Pause, time slider, scrolling lists | Worked |
| Click a bone in the list and in the 3D view | Worked; the correct thumb bone and its labels appeared |
| Layer toggles, "Not represented" list | Worked |

**Defects found by this use, and fixed:**
1. The plot panel covered muscle names and part of the bone list (fixed-size panels). Now panels follow the window; a test checks 4 window sizes.
2. The "Not represented" list still named APB, FPB and ADP after they were added. Now removed.
3. Left-drag did nothing, and the hand was partly behind the plot panel. Left-drag now orbits, clicks still select, and the scene is centred in the free area.
4. The wheel did not zoom the 3D view in the first session. It worked after the layout rewrite; **the original cause was not isolated.**

**Still limited:** right-drag and middle-drag were not exercised by hand (the tool has no such drag). No person has used it. The UI was redesigned afterwards: see `docs/VIEWER_UI.md`.

## 7. Items that could not be completed

| Item | Status | What it needs |
|---|---|---|
| Other operating system | **Not tested.** Docker Desktop cannot start here: firmware virtualization is off and WSL has no distribution | Enable virtualization in BIOS and install a distribution, or use the prepared CI below |
| Other CPU / separate machine | **Not tested** | Another machine, or CI |
| CI | `.github/workflows/ci.yml` written (Ubuntu and Windows). **Never run**, since there is no repository | A git repository and remote (owner decision) |
| Kinematic trajectories vs. measured | **Not compared.** Open datasets exist (Jarque-Bou et al. 2020; MOVMUS-UJI on Zenodo) | Task activations are chosen, not predicted, so a comparison would say little until a task reproduces a recorded movement. Needs a study design |
| EMG | **Not compared.** Same datasets | Same; static-optimization activations could be compared, with a protocol fixed in advance |

## 8. Status of the original list

| Item | Status |
|---|---|
| `pinch` | Passes a stricter task, with the weaknesses in 2.2 |
| `rest_neutral` | Passes at 4 s (horizon accepted by the owner); fails at the original 2 s |
| Tasks | 8 of 8 pass, 2 of them with these caveats |
| Wrist moment arms | **Still 8 of 12** in validation; optional fit available |
| Index MCP | **Still 10 of 12** in validation; optional fit available |
| Grip force | Cause of inertia dependence found; **still unvalidated** |
| Force, trajectory, EMG data | Only plausibility comparisons for force; rest not compared |
| Viewer | Used with real input; 4 defects addressed; no person yet |
| Other OS/CPU | **Not tested** |
