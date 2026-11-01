# Implementation Roadmap and Acceptance Gates

## P0 — Repository and toolchain audit
Tasks: inspect OS/tool versions; create environment; lint/test pipeline; record exact dependency versions.
Acceptance:
- `pytest` runs from a clean environment.
- dependency lock/export exists.
- Godot project opens without errors.
- report includes tested versions.

## P1 — Reference model and provenance
Tasks: inspect MyoHand files; verify model license and dependencies; inspect body/joint/actuator/tendon counts; compare model assumptions with peer-reviewed literature.
Acceptance:
- model loads in MuJoCo without warnings/errors.
- machine-readable inventory lists bodies, joints, DoFs, actuators, tendons, sites, geoms.
- provenance record is complete.
- no external asset is copied into repository without redistribution permission.

## P2 — Anatomy hierarchy and coordinate convention
Tasks: map segments to canonical anatomical names; establish right-hand orientation and frame conventions; create visual-to-physics mapping.
Acceptance:
- every visual segment maps to a documented MuJoCo body or is explicitly visual-only.
- coordinate conversion tests pass.
- UI does not claim individual motion for fused/lumped segments.

## P3 — Kinematics and constraints
Tasks: inspect joint types/axes/ranges; implement forward-kinematics inspection; verify coupled coordinates and constraints.
Acceptance:
- neutral pose loads reproducibly.
- joint sweeps stay within model-defined limits.
- fingertip trajectories are continuous and anatomically plausible.
- numerical tests verify transforms and angle units.

## P4 — Muscle-tendon system
Tasks: inventory actuators; map names to canonical muscle/tendon labels; inspect actuator transmission, activation dynamics, force generation, tendon paths and wrapping.
Acceptance:
- each actuator has source and semantics documented.
- activation changes produce expected sign/direction in isolated tests.
- no unsupported origin/insertion or physiological parameter is fabricated.
- ablation test can disable one actuator and quantify resulting change.

## P5 — Dynamics and functional tasks
Tasks: create repeatable tasks for opening, closing, pinch, grasp proxy, opposition, wrist motion; add inverse/static optimization only after forward model is verified.
Acceptance:
- simulation is stable under documented solver/timestep settings.
- no unexplained energy explosion or NaN/Inf.
- outputs include joint angles, actuator activations/forces, tendon lengths and external contact forces when available.
- task success criteria are numerical and repeatable.

## P6 — Python analysis and telemetry
Tasks: expose model inventory, reset, step, set controls, run task, export results.
Acceptance:
- versioned JSON schema.
- CSV export has units and metadata.
- same experiment configuration reproduces results within documented tolerance.

## P7 — Godot visualization
Tasks: import licensed GLB assets; map visual bones to model bodies; implement layer toggles, selection, muscle activation panel, joint plots and playback.
Acceptance:
- visualization pose matches MuJoCo state within defined transform tolerance.
- no Godot physics node drives the same joints.
- UI labels distinguish simulated, visual-only, and unavailable structures.

## P8 — Scientific validation
Tasks: compare kinematics, moment arms, grip/pinch outputs or EMG trends only where comparable reference data exist; sensitivity analysis; parameter uncertainty.
Acceptance:
- validation report identifies reference, metric, protocol, error, limitations.
- no cherry-picked metrics; failed comparisons are reported.
- numerical tolerances are justified from source/model precision.

## P9 — Release
Tasks: clean install, demo scenarios, docs, screenshots, citation metadata, license audit, release tag.
Acceptance:
- installation works on a clean Windows machine (and Linux if claimed).
- README contains demo, architecture, limitations, citations, and reproducibility instructions.
- third-party notices and asset licenses are complete.
