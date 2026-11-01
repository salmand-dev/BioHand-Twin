# P6 — Python Telemetry and API: Report

- Date: 2026-10-31
- Model: BioHand reference model = MyoHand fragment + `wrist_rom_v1` + `muscle_fixes_v1` + `limit_stiffness_v1`
- Toolchain: MuJoCo 3.17.0, Python 3.11.4, jsonschema 4.26
- Git commit: none
- Evidence:
  - `reports/P6/reproducibility.json`
  - `reports/P6/test_run_output.txt`

## Implementation

| Component | File |
|---|---|
| Session API `BioHandSession`. **Inspection and export:** `inventory()`, `metadata()`, `frame(include_bodies)`, `export_json()`, `export_csv()`. **Control:** `reset(angles)` (range-checked, radians), `set_controls()` (name- or vector-based, validated to [0, 1]), `step(n)`, `record(duration, every, controller)`. `simulate_task(id)` runs configured tasks with the same control policy as the P5 runner | `simulation/api.py` |
| Versioned schemas (JSON Schema 2020-12, version **1.0.0**; 1.1.0 since P7 adds optional muscle paths) | `schemas/telemetry_frame.schema.json`, `schemas/session_metadata.schema.json` |
| Model fingerprint: SHA-256 over 34 compiled arrays (geometry, kinematics, actuation, solver settings) plus the timestep | `simulation.api.model_fingerprint` |
| CLI: `inventory`, `list-tasks`, `run-task <id>` → `frames.csv` and `playback.json` (gitignored `build/runs/`) | `scripts/biohand.py` |
| Reproducibility evidence | `scripts/p6_reproducibility.py` |
| 12 new tests | `tests/test_p6_api.py` |

## Telemetry content (per frame)

- **Joints:** q (rad) and qvel (rad/s).
- **Muscles:** control, activation, tension, active tension and passive tension (N; split by MuJoCo's own functions), length (m), normalised length, active-window flag.
- **Fingertips:** site positions (m, MuJoCo world).
- **Contact:** contact normal force (N).
- **Flags:** outside the muscle-validity envelope, maximum limit violation, finiteness.
- **Optional body poses in Godot coordinates** (position, quaternion x,y,z,w). They are produced only by the single conversion module `biomechanics/frames.py`; a test checks this.
- **Every export carries:** schema version, software versions, MyoSim commit, patch list, model fingerprint, solver settings, task fixtures, units, licence notice (non-commercial MoBL-ARMS) and a disclaimer (not a medical device, not clinically validated).

## Verification

| Check | Result |
|---|---|
| Schemas valid (Draft 2020-12 meta-schema) | ✔ |
| Frames of all 8 tasks validate against the frame schema | ✔ 733 frames |
| Unknown fields rejected (`additionalProperties: false`) | ✔ |
| Invalid controls / unknown actuators / out-of-range angles rejected | ✔ |
| **API vs P5 runner** (two independent code paths) | **bitwise identical** joint angles for every frame of the tested tasks |
| Same configuration run twice (all 8 tasks) | **identical SHA-256** of the full frame stream |
| Fingerprint | stable across builds; changes with the patch set and with the timestep |
| CSV | metadata header lines; unit-suffixed columns; row count = frames |
| JSON export | round-trips; metadata validates |

## Acceptance criteria (ROADMAP P6)

| Criterion | Status |
|---|---|
| Versioned JSON schema | **PASS** — v1.0.0 for frames and metadata, enforced in tests |
| CSV export has units and metadata | **PASS** |
| Same experiment configuration reproduces results within documented tolerance | **PASS** — documented tolerance: bitwise identity on the same platform, MuJoCo version and model fingerprint. Cross-timestep verdicts: P5. **Cross-platform: bitwise identity does not hold and is not claimed** (the fingerprint differs between Linux and Windows); `tests/test_cross_platform.py` compares verdicts and metrics instead |

**P6 status: COMPLETE (2026-10-31).**

## Limitations

- No live IPC yet (WebSocket/TCP). By design (`docs/ARCHITECTURE.md`), the first Godot integration (P7) uses the offline `playback.json`.
- Playback JSON is about 16 KB per frame with body poses. That is fine for recorded tasks of a few seconds, but a binary or streamed format would be needed for long sessions.
- Telemetry values inherit every model limitation listed in `docs/P1_SCIENTIFIC_GAP_ANALYSIS.md`.

## Reproduction

```bash
.venv/Scripts/python.exe scripts/p6_reproducibility.py
```

```bash
.venv/Scripts/python.exe scripts/biohand.py run-task hand_closing
```

```bash
.venv/Scripts/python.exe -m pytest -v
```
