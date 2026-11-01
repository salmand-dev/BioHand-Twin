# P7 — Godot Visualization and Interaction: Report

- Date: 2026-11-01
- Toolchain: Godot 4.7.2 (compatibility renderer), Blender 3.6.23, MuJoCo 3.17.0
- Git commit: none (no repository yet)
- Evidence:
  - `reports/P7/viewer_check_output.txt` (headless check of the real scene)
  - `reports/P7/viewer_power_grip.png` (rendered screenshot)
  - `tests/test_p7_viewer.py`

## What was built

| Item | File |
|---|---|
| Viewer scene: everything is created by `main.gd` | `godot_app/scenes/main.tscn` |
| Playback, layers, selection, panels | `godot_app/scripts/main.gd` |
| Orbit camera, joint plot | `godot_app/scripts/orbit_camera.gd`, `joint_plot.gd` |
| Names and classifications for the UI (no geometry; committed; drift-tested) | `godot_app/data/biohand_inventory.json` |
| Telemetry schema 1.1.0: optional muscle path points (via-sites and wrap tangent points; wrap arcs drawn as chords; polyline length ≤ 0.25 mm from tendon length) | `schemas/`, `simulation/api.py` |
| Headless check, screenshot and performance tools | `godot_app/tools/` |

Features:
- recorded-run playback with a time slider and speed control;
- layers: bones, muscle paths coloured by activation, fingertip markers, proximal scaffold (off by default);
- task fixtures, such as the grip cylinder, drawn from run metadata;
- muscle panel: activation bars and tension in N, with a warning mark on unresolved names;
- joint plot with the joint range and a time cursor.

Selection works from the list or by clicking. Selecting any bone highlights its whole motion unit.

## Acceptance criteria (ROADMAP P7)

| Criterion | Status | Evidence |
|---|---|---|
| Visualization pose matches MuJoCo state within a defined tolerance (10 µm, justified by float32 storage) | **PASS** | All 8 runs checked: position error ≤ 1.2e-7 m and basis error ≤ 2.4e-7, for all body nodes at 3 frames per run. The grip cylinder pose also matches MuJoCo's geom pose |
| No Godot physics node drives the same joints | **PASS** | No physics body, joint, area or collision node in the live scene tree, and none in the sources (test). Picking uses a ray vs. mesh-AABB test |
| UI labels distinguish simulated, visual-only and unavailable structures | **PASS** with a note | Labels exist for simulated, lumped ("LUMPED — this bone cannot move on its own"), fixed, simulation-helper and not-represented structures. **No visual-only structures exist in this release**, so that label has nothing to apply to |

**P7 status: COMPLETE (2026-11-01).**

## Tests caught two real viewer bugs

- The grip cylinder was first drawn along the hand's dorsal axis. The test comparing it with MuJoCo's geom pose found this, and the fix removed a wrong extra rotation.
- The run selector did not follow runs loaded from code.

## Limitations

- **Playback only.** The viewer plays recorded runs; there is no live link to MuJoCo and no way to change activations from the UI.
- **Interaction testing.** Click picking, playback and layers were first tested by calling functions in a headless run. On 2026-11-01 they were also exercised with real mouse input through desktop automation, which found 4 defects (`reports/fixes/FIXES_REPORT.md`, section 6). No person has yet used the UI.
- **Picking is approximate.** It uses bounding boxes, so a click near overlapping bones may select a neighbour. The structures list is exact.
- **Muscles are drawn as thin lines**, not volumes. There are no ligament or skin layers because the model has no such data.
- **Local assets.** Bone geometry and runs live in gitignored `godot_app/local_assets/` (non-commercial, MoBL-derived). Without them the viewer shows a message explaining how to build them.
- The UI was redesigned on 2026-11-01 (`docs/VIEWER_UI.md`). Panels no longer overlap the view (checked at 4 window sizes).

## Reproduction

```bash
.venv/Scripts/python.exe scripts/biohand.py run-task power_grip_proxy
```

```bash
.venv/Scripts/python.exe -m pytest tests/test_p7_viewer.py -v
```
