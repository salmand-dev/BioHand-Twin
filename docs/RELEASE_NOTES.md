# Release Notes

## 0.1.0 — pre-release (2026-11-01)

Python tests pass from a clean install on Windows and Linux in CI.

### Included
- Build of the reference model: pinned MyoSim commit plus five explicit patches, with values validated against their sources or derivation rules.
- Kinematics, muscle inspection (moment arms, active/passive force), ablation, static optimization and inverse dynamics.
- Eight configured tasks with numerical criteria and full telemetry.
- Python session API, CLI and versioned telemetry schema 1.1.0.
- Godot 4 playback viewer: layers, bone and muscle selection with filters, view presets, joint plot with seeking, keyboard control, task caveats and validity warnings (`docs/VIEWER_UI.md`).
- Validation against published moment-arm data, sensitivity analysis, and reports for every phase.

### Known failures and caveats
- 4 of 12 measured wrist moment-arm comparisons and 2 of 12 index comparisons fall outside their criteria. An optional tuned patch exists; it is a fit, not validation.
- `pinch` passes at about 0.8 N and is fragile. `rest_neutral` passes at 4 s and fails at the original 2 s.
- The P5 claim that pinch failed because of the missing muscles was wrong; see `reports/fixes/FIXES_REPORT.md`.

### Not verified
- Building the 3D file and running the viewer on a second machine; macOS.
- Use of the viewer by a person (it was exercised with real input through desktop automation).
- Forces, timing, EMG and trajectories against measured data.

See [LIMITATIONS.md](LIMITATIONS.md).
