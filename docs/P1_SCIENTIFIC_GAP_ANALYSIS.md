# P1 — Scientific Gap Analysis: MyoHand vs Literature and Anatomy

Scope: MyoHand fragment variant @ MyoSim `93b0ca8f` (see `reports/P1/P1_REFERENCE_MODEL_AUDIT.md`). Every model fact below is read from `data/model_inventory.json`.

Literature statements come from the cited abstracts or full texts that were actually retrieved, as noted in each row. **No value in this document has been added to the model.**

Labels:
- **[MODEL]** — read from the compiled model
- **[LIT]** — stated in a retrieved source
- **[ANAT]** — general anatomical knowledge. It must be backed by a citation from a standard anatomy text in the P2 glossary before it appears in UI or docs
- **[CHECK]** — anomaly or hypothesis that needs investigation; not a conclusion

## 1. Literature compared

| # | Source | Retrieved | Key content used |
|---|---|---|---|
| L1 | Caggiano V, Wang H, Durandau G, Sartori M, Kumar V. *MyoSuite.* arXiv:2205.13600 (2022) | full text (HTML) §3.1, App. A | MyoHand lineage; "29 bones, 23 joints, 39 muscle-tendon units"; OP added; conversion fidelity: moment-arm RMS 0.38±0.57 %, force RMS 4.1±2.0 % of F_max **vs the OpenSim originals** |
| L2 | Holzbaur KRS, Murray WM, Delp SL. Ann Biomed Eng 2005; doi:10.1007/s10439-005-3320-7 | cited by upstream; not retrieved | MoBL base model. Per L4, it has 15 DoF and 50 muscle elements; only thumb and index have active hand joints |
| L3 | Saul KR et al. Comput Methods Biomech Biomed Eng 2015;18(13):1445–58; doi:10.1080/10255842.2014.916698 | abstract | MoBL-ARMS benchmark model: 7 DoF, 50 musculotendon actuators (shoulder → wrist) |
| L4 | Lee JH, Asakawa DS, Dennerlein JT, Jindrich DL. *Finger Muscle Attachments for an OpenSim Upper-Extremity Model.* PLoS ONE 2015;10(4):e0121712; doi:10.1371/journal.pone.0121712 (+ correction, PMC9020702) | full text via journals.plos.org; correction **not** retrieved | 2nd-Hand (details below) |
| L5 | Mirakhorlo M et al. Comput Methods Biomech Biomed Eng 2018;21(9):548–557; doi:10.1080/10255842.2018.1490952 | abstract | Independent OpenSim hand-wrist model: 23 DoF, 43 muscles, 19 segments, carpals as one segment; single-cadaver data (masses, inertias, joint axes, PCSA, fibre and tendon lengths); moment arms vs literature at index MCP and wrist r = 0.88, RMSD 23 %; pinch FDP/FDS forces within 1 SD of in-vivo |
| L6 | McFarland DC et al. IEEE TBME 2023;70(5):1424–1435; doi:10.1109/TBME.2022.3217722 | abstract | Open-source OpenSim hand-wrist model for functional tasks: grip/pinch forward dynamics; 7/8 muscles within EMG variability in pinch; tenodesis-like coupling |
| L7 | MyoConverter documentation (myoconverter.readthedocs.io) | web | Converted models inherit OpenSim errors; wrapping is approximated; MuJoCo tendons are inextensible; contacts are not converted |

Details of L4 (2nd-Hand):
- Per finger: MCP 2 DoF (universal joint), PIP and DIP 1 DoF each.
- Muscles: TE, ES, RB, UB, RI, LU, UI, FDP, FDS and EDC per finger.
- Attachments from An 1979, for a hypothetical 50th-percentile male.
- Moment-arm reference data (An 1983) exist for the **index only**; mean VAF 75.5 %.
- The authors state they "did not conduct experiments to validate the model at a functional level" and that the model was not yet dynamic.

## 2. Lineage and what upstream validation does — and does not — cover

1. MyoHand = MoBL (L2/L3) + 2nd-Hand (L4), converted and optimised, then **manually adjusted without itemisation** (`README_hand.md`).
2. The L1 fidelity numbers measure **agreement with the OpenSim originals**. They are not agreement with human data. They were also published in 2022. The pinned model has since been refactored: muscle parameters were updated on 2026-05-27 ("updated muscle parameters" in the arm changelog), and the hand is now derived by pruning the arm (2026-05/06). **No published fidelity figure applies to the pinned commit.**
3. Anatomical evidence for the underlying paths is limited to the index finger (L4). The model has no functional validation (L4) other than what MyoSuite tasks implicitly exercise.
4. MyoHand is **not** derived from L5 or L6. Those are independent models and serve only as *comparison* references. The project scaffold's `REFERENCES.md` implied a relation to L5; that has been corrected.
5. Upstream `README_hand.md` states "Known limitations: None currently tracked". This audit finds the substantive limitations below.

## 3. Gap table

Severity reflects the impact on BioHand's stated goals: **H** blocks a planned claim or feature; **M** must be labelled in UI/reports; **L** is minor.

### 3.1 Skeleton and segments

| ID | Gap | Evidence | Sev. | Consequence for BioHand |
|---|---|---|---|---|
| S1 | The 8 carpal bones form **one rigid segment**: only `lunate_r` carries joints, and the other 7 are welded | [MODEL] `test_carpals_are_lumped_into_one_wrist_segment` | H | The UI must not show independent carpal motion. All wrist motion happens at a single point (L5 also lumps carpals [LIT]) |
| S2 | Wrist flexion and deviation are two serial hinges **at one anchor** on lunate. There is no radiocarpal/midcarpal split and no equality coupling (neq = 0) | [MODEL] | M | Wrist kinematics are simplified; P3 must report the axis locations |
| S3 | Metacarpals II–V are welded to capitate: **no CMC mobility for digits 4–5** and no palmar arch | [MODEL] | M | Power-grip and cupping fidelity is limited [ANAT: CMC 4/5 are mobile] |
| S4 | Ulna, humerus, scapula and clavicle are welded in a fixed preview pose. **The elbow is fixed at its qpos0** (`elbow_flexion_r = 0`) | [MODEL] `hand.py` | M | Muscles crossing the elbow (ECRL, ECRB, ECU, FCR, FCU, PL, PT, FDS4–5, EDC, EDM) have lengths and wrist moment arms that depend on that fixed elbow angle |
| S5 | Forearm rotation is a single hinge on the radius about a fixed axis; the ulna is static | [MODEL] | L–M | No DRUJ/PRUJ translation |
| S6 | Thumb MCP has 1 DoF (flexion only) | [MODEL] (`mp_flexion_r`) | M | No MCP abduction/rotation. Opposition depends on CMC alone |
| S7 | No sesamoid bones; no ligaments, capsules, volar plates or pulleys as structures | [MODEL] | M | Pulleys appear only implicitly through via-points/wrap geoms |

### 3.2 Joints and ranges

| ID | Gap | Evidence | Sev. |
|---|---|---|---|
| J1 | Wrist flexion range is ±45°; deviation is −10…25° | [MODEL] | M — P3 must compare against published ROM with citations; no ROM value is assumed here |
| J2 | MCP 2–5 range starts at 0°: **no MCP hyperextension**. Abduction is ±15° for all digits; PIP/DIP are 0–90° for all digits | [MODEL] | M |
| J3 | No passive joint stiffness (k = 0 on all joints). Passive behaviour comes only from muscle passive force (`fpmax=1.3`), joint damping, armature (1e-4) and soft limits | [MODEL] | M — passive hand-opening tasks (cf. L6) are not comparable without justification |
| J4 | No PIP–DIP coupling constraint | [MODEL] neq = 0 | M |
| J5 | Soft limits are exceeded under co-contraction: 0.154 rad at full activation; 16/23 joints > 0.01 rad | [MODEL] smoke run | M — P3 decision required |

### 3.3 Muscle–tendon units

| ID | Gap | Evidence | Sev. |
|---|---|---|---|
| M1 | **No thenar intrinsics except OP**: no APB, FPB or adductor pollicis | [MODEL] actuator list; [ANAT] | H — pinch and opposition tasks lack major thumb muscles; results must be labelled |
| M2 | **No hypothenar muscles** (ADM, FDM brevis, ODM) | [MODEL]; [ANAT] | M |
| M3 | **No dedicated supinator**: SUP, BIClong/short and BRD exist in MyoArm but are pruned from the hand. Supination torque comes only from small incidental moment arms (ECRL +3.6 mm, ECRB +1.6 mm, …) | [MODEL] arm muscle XML vs `RIGHT_HAND_ACTUATORS` | H for forearm-rotation tasks |
| M4 | The extensor mechanism is not a network. L4's TE, ES, RB and UB are reduced to combined actuators (LU_RB*, UI_UB*), and EDC/EIP/EDM run as single paths to the distal phalanx | [MODEL] tendon endpoints; [LIT L4] | M — "extensor hood" must not be claimed |
| M5 | Interossei are modelled per side and digit (RI2–5, UI_UB2–5) rather than as anatomical dorsal/palmar muscles. L4 equates RI with dorsal and UI with palmar; a 2022 correction revised this terminology (text not retrieved) | [LIT L4] | M — the canonical-name mapping in P2 must stay "unresolved" until the correction is read |
| M6 | **[CHECK]** The first via-point body of RI4, RI5, LU_RB4/5 and UI_UB4/5 is `thirdmc_r`. Anatomically, lumbricals arise from FDP tendons and interossei from adjacent metacarpals [ANAT] | [MODEL] `first_site_body` | M — verify against L4 attachment tables and the MyoSim chain XML before any origin is displayed |
| M7 | **[CHECK]** RI2 has dL/dq = +4.23 mm/rad at `mcp2_flexion_r`, i.e. it *extends* MCP2 at qpos0. RI3–RI5 are negative (flex) | [MODEL] | M — investigate in P4 over the full range; may be pose-dependent wrapping |
| M8 | Muscle physiology is generic. All 39 actuators share `range`, `lmin`, `lmax`, `vmax`, `fpmax`, `fvmax` and τ_act/τ_deact (0.01/0.04 s) | [MODEL] | H for any claim about force–length/velocity physiology. Only F_max and lengthrange are muscle-specific |
| M9 | Rigid (inextensible) tendons, no pennation, no tendon slack length; L0 is a normalisation constant, not a fibre length | [MODEL]; [LIT L7] | M |
| M10 | Intrinsic peak forces are shared within each digit (e.g. 61.2 N ×3 for digit 2) | [MODEL] | L–M — origin undocumented |
| M11 | Wrapping uses spheres/cylinders that approximate OpenSim ellipsoids; moment-arm discontinuities are possible | [LIT L7] | M — P4 moment-arm sweeps must check continuity |

### 3.4 Inertial and contact properties

| ID | Gap | Evidence | Sev. |
|---|---|---|---|
| I1 | Most hand-segment inertias equal the 1e-4 kg·m² floor; phalanx masses are 0.001–0.01 kg. Implied radii of gyration (~0.1 m) are non-anatomical | [MODEL] | H for dynamic claims (accelerations, timing). L5 has single-cadaver inertias [LIT] but mixing parameter sets needs a documented reconciliation (PROJECT_SPEC) |
| I2 | Lunate/capitate inertia comes from mesh volume at the default density | [MODEL] | L |
| C1 | Fragment variant: collision capsules have conaffinity 0, so there is **no self-contact** and contact only with external objects that have conaffinity set | [MODEL] | M — grasp tasks need an explicit, documented contact setup |
| C2 | No soft-tissue/skin compliance; contact parameters were "optimized for contact-rich behaviors" without published values | [MODEL]; `README_hand.md` | M |

### 3.5 Instrumentation

| ID | Gap | Evidence | Sev. |
|---|---|---|---|
| X1 | No sensors and no keyframes | [MODEL] | L — telemetry must read MjData directly; the neutral pose must be defined by BioHand and documented |

### 3.6 Traceability to the original OpenSim sources (added 2026-10-31)

Sources compared:
- **2nd-Hand** `2nd_Hand_Model_Mass.osim`: 61 bodies, 106 coordinates, 62 muscles (Thelen2003 and Schutte1993), 79 coupler constraints, all body masses 0.
- **MoBL-ARMS bimanual** `MoBL_ARMS_bimanual_6_2_21.osim`: 24 bodies, 100 Millard2012 muscles, 26 couplers. The hand is a single rigid `hand_r` with no finger joints.

Evidence: `data/reference_osim_inventory.json`, `reports/P1/osim_vs_myohand.json`, produced by `scripts/p1_osim_inventory.py`.

| ID | Finding | Evidence | Sev. |
|---|---|---|---|
| T1 | MyoHand's extrinsic peak forces come from **MoBL-ARMS**, not from 2nd-Hand. 24/26 match exactly and ECRB is 252.2 vs 252.5 N. The 2nd-Hand file holds different, mostly smaller values (e.g. FCR 73.96 N) | [MODEL] vs [LIT-file] | — (provenance established) |
| T2 | **[CHECK] EDC5 = 139.4 N in MyoHand vs 39.4 N in MoBL-ARMS (EDCL_r)** — exactly +100 N. The value is present in MyoSim since at least 2023-06 (`babbf81`). 2nd-Hand has 13.11 N. Likely a transcription error (hypothesis). It inflates little-finger and wrist extension | [MODEL] vs [LIT-file] | **H** — report upstream; any BioHand override must be a documented patch |
| T3 | **Intrinsic muscles have placeholder parameters in the source.** All 12 intrinsic actuators in 2nd-Hand share F = 61.24 N, L_opt = 0.0835 m, tendon slack = 0.275 m and pennation = 0.1047 rad. These are identical to 2nd-Hand's FDS2 values. A 0.275 m tendon slack length is longer than the hand itself, so these are not intrinsic-muscle physiology. This agrees with L4 ("not yet dynamic"). MyoHand keeps 61.2 N for 6 actuators and changes the other 6 (98.0, 97.0, 76.9, 47.9 N) **with no documented source** | [LIT-file], [MODEL] | **H** — no intrinsic-force claim is supportable |
| T4 | The sources provide per-muscle **optimal fibre length, tendon slack length and pennation** (MoBL Millard2012). MyoHand discards them: generic MuJoCo FL/FV parameters and inextensible tendons (see M8, M9) | [LIT-file], [MODEL] | M — possible P4 improvement path, but only via a documented reconciliation, never silent replacement |
| T5 | **Joint ranges changed without documentation:**<br>• wrist `flexion` ±70° in both sources → ±45° in MyoHand<br>• `cmc_flexion` −15…45° → −44.7…40.1°<br>• `cmc_abduction` −25…25° → −28.6…44.7°<br>The other 20 ranges are identical to 2nd-Hand | [MODEL] vs [LIT-file] | M — P3 must report both and choose with justification |
| T6 | **Two-row wrist lost.** Both sources move a proximal carpal row and the distal hand through spline couplers driven by `flexion` and `deviation`:<br>• 2nd-Hand: `proximal_distal_r1/r3`<br>• MoBL: `proximal_row_r` + `wrist_hand_r1/r3`<br>MyoHand places both rotations at one joint on `lunate_r` with no coupling (neq = 0) | [LIT-file], [MODEL] | M — upgrades S2. The wrist centre of rotation differs from the sources |
| T7 | **Moving muscle via-points lost.** In 2nd-Hand, 7 phantom bodies (`bicpt2`, `ptpt`, `FCUpt`, `EDCpt`, `APLpt`, `FPLpt`, `FPLpt2`) move muscle path points through 20 spline couplers driven by `pro_sup`, `flexion` and `cmc_flexion`. MyoHand uses fixed sites and wrap geoms instead | [LIT-file] | M — P4 must check moment arms of PT, FCU, EDC, APL and FPL across pronation/supination and wrist flexion |
| T8 | **Masses:** 2nd-Hand gives 0 kg for every body. MoBL gives one `hand_r` of 0.5819 kg. MyoHand's radius (0.23359 kg) and ulna (1.1053 kg) equal MoBL, but its per-bone hand masses (total 0.4303 kg) come from **neither source** | [LIT-file], [MODEL] | H for dynamics (confirms I1) |
| T9 | OP (opponens pollicis) is in neither source, consistent with L1 (added by MyoSuite) | [LIT-file] | L — its parameters need their own source (L1 ref. [30]) |

### 3.7 Kinematic findings from P3 (added 2026-10-31)

Evidence: `reports/P3/` (`p3_kinematics_output.json`, `wrist_convention_check.json`, `axis_convention_check.json`).

| ID | Finding | Sev. |
|---|---|---|
| T10 | **Wrist deviation sign inverted relative to MoBL-ARMS.** A first-order kinematic comparison using the source's own spline/coupler definitions gives a cosine of −0.994 between the fingertip directions for +deviation. In MoBL, + = ulnar, so −10…+25° means 10° radial / 25° ulnar. MyoHand kept the numbers with + = radial, so it allows **25° radial / 10° ulnar**. Wrist flexion matches (cos 0.9996). Of the 22 hand joints compared with 2nd-Hand, 21 follow the source convention (thumb included) | **H** |
| J1′ | **Wrist ROM below ADL requirements** (Ryu et al. 1991, abstract: up to 54° flexion, 60° extension, 40° ulnar, 17° radial): MyoHand gives 45°, 45°, 10° and 25°. Finger flexion ranges (0–90°) cover Bain et al. 2015 functional ranges for MCP, PIP and DIP | H for wrist tasks |
| J5′ | **Soft joint limits are weak relative to muscle strength.** Under each joint's maximal agonist muscle torque (sum of F_max·\|moment arm\| at the limit, no antagonists, no gravity), penetration reaches 31° (wrist flexion), 36° (MCP III flexion) and 38° (thumb CMC, negative side). The defaults are solref (0.02, 1) and solimp (0.9, 0.95, 0.001). With co-contraction (P1), the observed overshoot was ≤ 8.8° | H for force/pose claims near limits |
| T11 | **[CHECK] Upstream `lengthrange` inconsistent with geometry for RI5, FDP5 and FDS4.** Over a grid of wrist × forearm × finger postures, these muscles lie outside their calibrated `lengthrange` in 97–100 % of samples, even with the *upstream* ranges. RI5 reaches normalised fibre lengths of −0.48 (non-physical), and FDP5/FDS4 go down to 0.42–0.44 (below lmin 0.5, so no active force). Pre-existing; not caused by the wrist patch (`reports/P3/patch_check.json`) | H for these muscles' forces |
| C3 | **Adjacent finger proxies overlap.** Collision capsules of neighbouring fingers interpenetrate by 7–15 mm in flexion and in finger-convergence poses, and already overlap by up to 10 mm at qpos0. Self-collision is inactive in the fragment (conaffinity 0), so nothing prevents it | M |

### 3.8 Muscle–tendon findings from P4 (added 2026-10-31)

Evidence: `reports/P4/` (`p4_muscle_audit.json`, `moment_arm_curves.json`).

| ID | Finding | Sev. |
|---|---|---|
| T11′ | **`lengthrange` inconsistent with geometry (extended from P3).** In 20 000 random in-range poses, less than half of the sampled length span lies inside the specified `lengthrange` for **FDS4, FDP5, RI5, LU_RB4, UI_UB4 and LU_RB5**. RI5 normalised length reaches −0.48 and lies outside the active window in 83 % of samples. Ablation confirms the consequence: removing FDS4 or LU_RB4 moves the fingertips by < 0.1 mm in a random-control task, so these muscles are effectively non-functional. MuJoCo's own `mj_setLengthRange` (default-style options) became numerically unstable on this model (NaN in QACC at DOF 0), so the geometric sampling is the evidence used | **H** |
| M7′ | **RI2 path break confirmed.** The moment arm at MCP II jumps from −5.6 to +4.4 mm near 10° flexion, inside the upstream range. RI2 also carries 11.9 N passive tension at qpos0, while every other muscle has ≤ 0.2 N | H for index intrinsics |
| W1 | **Muscle paths invalid beyond the upstream wrist range.**<br>• FDS2/FDS3 wrist moment arms break (24 mm in one 0.25° step) near −66° extension.<br>• FCR and FCU lose and reverse their flexor action beyond +59°/+69°.<br>• ECRL reverses near −70°.<br>Mitigation: muscle-path validity envelope (flexion ±45°, deviation −10…+10°); kinematics keep the source ROM | H outside the envelope |
| M12 | **[CHECK] PT acts as a supinator near full pronation.** Its moment arm about `pro_sup_r` turns negative from +65° to +90° (−6.4 mm at 90°). Several wrist and finger muscles also change sign about the forearm axis over ±90°, all with small magnitudes (≤ 6 mm) | M |

### 3.9 Repair status (patch `muscle_fixes_v1`, 2026-10-31)

| Gap | Status | How |
|---|---|---|
| T2 (EDC5 force) | **Repaired** | Restored to the MoBL-ARMS source value, 39.4 N |
| M7′ (RI2 path break) | **Repaired** | MCP cylinder removed, consistent with RI3–RI5. Independent evidence: the upstream `lengthrange` of RI2 equals the geometric range of the path *without* the cylinder (to 0.004 mm), so the cylinder was not part of the calibrated path |
| T11′ (lengthrange) | **Repaired** | Re-derived with MuJoCo's definition, the same method that reproduces the 31 healthy upstream muscles with a median error of 0.78 mm. Covers FDS4, FDP5, APL, RI2, LU_RB4, UI_UB4, RI5 and LU_RB5. Afterwards every muscle has ≥ 59 % of its span inside its range and none leaves the active window |
| T10, T5 (wrist sign/range) | Repaired (`wrist_rom_v1`) | — |
| W1 (paths beyond the upstream wrist range) | **Mitigated, not repaired** | Validity envelope. A repair would require new path geometry, which is not available from any source |
| M12 (PT reversal), T3 (intrinsic placeholder forces), I1 (inertias), J5′ (soft limits), C3 (proxies), M1–M4 (missing muscles) | Open | No source-backed repair available (J5′ handled in P5) |

### 3.10 Functional findings from P5 (added 2026-10-31)

| ID | Finding | Sev. |
|---|---|---|
| J5′ | **Repaired** (`limit_stiffness_v1`, numerical setting): worst limit penetration falls from 38.4° to 1.5° | — |
| P1 | **Thumb–index pinch is not achievable with the model's actuators.** Four policies fail: two open-loop patterns, pose tracking with static optimization, and forward-dynamics optimization. Causes: OP drives full CMC abduction; there are no FPB/APB/ADP, so thumb MP flexion with the IP near neutral cannot be produced (M1). No pinch-force output is available | **H** |
| J3′ | **The unbraced hand droops under gravity at rest.** With the palm down at qpos0 and zero passive joint stiffness, the wrist sinks toward its limit and has not settled within 2 s | M |
| G1 | Power-grip proxy forces (≈ 276 N at 50 % activation) depend on the illustrative fixture and default contact parameters, so they are not comparable with measured grip strength | M |

### 3.11 Validation findings from P8 (added 2026-11-01)

Evidence: `reports/P8/P8_REPORT.md`.

| ID | Finding | Sev. |
|---|---|---|
| V1 | **Wrist moment arms vs. cadaver data (Garland et al. 2018):** sign correct 12/12, Pearson r 0.98, RMSE 4.4 mm; 8/12 within ±2 SD. FCR, FCU and ECU flexion–extension are too large, and FCR deviation too small | H for wrist forces |
| V2 | **Index MCP ab/adduction signs** of FDP and EDC disagree with attachment-derived values. Inherited from 2nd-Hand; small magnitudes | L–M |
| V3 | **Conversion differences from 2nd-Hand** are RMSE 1.63 mm, up to 6.2 mm over 76 values, at the pinned commit | M |
| V4 | **Grip-proxy force depends on non-anatomical inertia** (248–343 N for inertia ×10 / ×0.1), as well as F_max and contact settings | H for any force claim |

### 3.12 Fixes (added 2026-11-01; evidence: `reports/fixes/FIXES_REPORT.md`)

| ID | Status | Note |
|---|---|---|
| M1 | **Partly addressed** by `thumb_intrinsics_v1` | One specimen, mapped by rule, generic curves. Raises force capacity; not validated |
| J3, J3′ | **Partly addressed** by `passive_stiffness_v1` | Measured for wrist, forearm, index MCP; assumed for the thumb; estimated for MCP 3–5 |
| P1 | **Claim corrected** | The cause given in 3.10 was wrong. Joint stiffness decided contact. Pinch force stays small and fragile |
| New: G1 | Open | Grip force depends on effective mass through MuJoCo's default contact softness; numerical, not physiological |
| New: J6 | Open | Model passive muscle gives about 0 stiffness at the wrist near neutral, while Formica et al. attribute measured stiffness mostly to muscle |
| Moment arms (3.11) | **Unchanged** in the reference model | An optional tuned patch exists; it is a fit to the same data |

## 4. Implications and recommendation

1. **Usable now** (with labels):
   - kinematic visualisation of 23 DoF;
   - muscle path display;
   - tendon length and moment-arm extraction (P4);
   - qualitative actuation demos.
2. **Not supportable without further work or new sources:**
   - physiological force–length/velocity claims (M8);
   - dynamic timing (I1);
   - thumb pinch/opposition force (M1);
   - supination tasks (M3);
   - independent carpal motion (S1);
   - extensor-hood mechanics (M4).
3. MyoHand remains the most practical **MuJoCo-native** reference, subject to the licence resolution. L5 (Mirakhorlo) and L6 (McFarland) should be evaluated as *comparison* data sources for P8. If L6's model is openly licensed, it is a candidate for task-level validation references (grip/pinch).
4. None of this permits the words "validated" or "clinically accurate" for BioHand outputs.

5. Because the original OpenSim files are now available locally, every MyoHand value can be classified as one of:
   - **inherited**: equal to a source value;
   - **modified**: differs from the source (T2, T3, T5);
   - **added**: no source value (T8, T9).

   P2 (WP5) should carry this classification per parameter.

## 5. Literature not yet retrieved (needed before P3/P4 claims)

- Holzbaur 2005 full text (joint definitions, ROM, wrist kinematics).
- Lee 2015 correction (PMC9020702).
- An et al. 1979/1983 (index moment-arm reference data).
- A standard anatomy reference for [ANAT] statements and the P2 glossary.
- Published wrist/finger ROM sources for J1/J2.
