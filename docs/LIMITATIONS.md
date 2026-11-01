# Known Limitations (release summary)

Full detail and evidence: `docs/P1_SCIENTIFIC_GAP_ANALYSIS.md` and `reports/`.

## What this is
A research and education tool: a generic model, built from MoBL-ARMS and 2nd-Hand through MyoSim, with five documented BioHand patches. `reports/fixes/FIXES_REPORT.md` covers the latest two. **Not a medical device, not patient-specific and not clinically validated.**

## Anatomy
- The 8 carpal bones and metacarpals II–V move as **one rigid block**. There is no two-row wrist and no palmar arching.
- The elbow and shoulder are fixed. The forearm rotates by one hinge.
- **Added from one cadaver specimen (Mirakhorlo et al. 2016), mapped by rule:** abductor and flexor pollicis brevis, adductor pollicis. Straight paths, generic curves, about 7 mm mapping uncertainty, force from a specific tension measured on the quadriceps.
- **Missing muscles:** all hypothenar muscles, and the supinator, biceps and brachioradialis (removed from the hand model). Supination force comes only from small incidental moment arms.
- No extensor-hood network, ligaments, capsules, pulleys, sesamoids or skin. Some actuators combine structures (`LU_RB*`, `UI_UB*`).
- Most anatomical names are **provisional** (no anatomy reference chosen). The 12 intrinsic actuators are **unresolved**.

## Muscles and forces
- All muscles share the same generic force curves and time constants. Only peak force and length range differ.
- Tendons do not stretch; there is no pennation.
- **Intrinsic peak forces are placeholders** in the source.
- OP drives the thumb into abduction rather than toward the fingers.
- PT reverses its action near full pronation.
- The `lengthrange` of 8 muscles was re-derived and 1 tendon path was changed by patch (provisional).

## Wrist
- Kinematic range: ±70° flexion, 25° ulnar and 10° radial deviation. **Deviation is below published activity needs.**
- **Forces and torques are only claimed inside the validity envelope (flexion ±45°, deviation ±10°).** Beyond it, muscle paths break or reverse. Telemetry and the viewer flag this.

## Dynamics
- Most hand inertias are numerical floors, not anatomy, so **timing and accelerations are not physiological**.
- Passive joint stiffness is linear and direction-independent. Wrist, forearm and index MCP targets are measured; **thumb values are assumed, not measured; MCP 3-5 are estimated from the index;** MCP abduction, PIP and DIP have none. At rest the forearm still rotates about 28 degrees under the hand's weight.
- Limit stiffness is a tuned numerical setting.

## Functional outputs
- **Thumb–index pinch passes only a small-force criterion (about 0.8 N).** It is unsteady, fragile against small parameter changes, and uses optimizer-derived activations. Maximum-force estimates are from a quasi-static calculation over sampled postures and are not validated.
- Grip-proxy force is not grip strength and changes considerably with F_max and contact settings. Its dependence on inertia comes from MuJoCo's mass-scaled contact softness, so it is numerical.
- Collision capsules of adjacent fingers overlap, and hand self-contact is off unless a task adds it.

## Validation
- Only the comparisons in `reports/P8/P8_REPORT.md` exist. Of 12 measured wrist values, 4 fall outside the 2 SD criterion, and most are overestimated. The optional `moment_arm_calibration_v1` is a fit to those same data, so it is not validation and is off by default.
- No measured validation for the thumb (including the added muscles), the other fingers' intrinsics, forces, EMG or trajectories.

## Software
- CI runs the Python tests from a clean install on Windows and Linux. Results agree within tolerances, not bit for bit, and pinch force has a wide tolerance (50 %). macOS is untested. The Blender build and the viewer are tested on one Windows machine only.
- The viewer is playback-only. It was exercised with real mouse input through desktop automation, but no person has used it, and there is no usability study. See `docs/VIEWER_UI.md`.
