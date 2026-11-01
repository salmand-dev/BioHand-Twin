# Demo Scenarios

Run any task, then choose it in the viewer's selector:

```bash
.venv/Scripts/python.exe scripts/biohand.py run-task hand_closing
```

Every task configuration is in `data/experiments/tasks/`.

| Task | What it shows | Result | What not to conclude |
|---|---|---|---|
| `hand_closing` | Finger flexors close the fingers (wrist braced) | Pass | Activation levels are chosen, not measured |
| `hand_opening` | Extensors open the fingers from 60° flexion | Pass | No extensor-hood mechanics |
| `thumb_opposition` | Thumb tip approaches the index tip | Pass (distance falls 45 %) | The tips do not touch. This is not a pinch |
| `wrist_flexion_extension` | Wrist flexors, then extensors | Pass | Mostly outside the muscle validity envelope: use kinematics only, no forces |
| `wrist_deviation` | Radial, then ulnar deviation | Pass | Same limit |
| `power_grip_proxy` | Fingers around a static cylinder | Pass | Force depends on the fixture and is not grip strength |
| `pinch` | Thumb–index pad contact with pose tracking | Pass, about 0.8 N | Small, unsteady and fragile force; optimizer-derived activations. Not a measure of pinch strength |
| `rest_neutral` | No activation, gravity on, 4 s | Pass at 4 s; **fails at the original 2 s** | The forearm settles slowly and rotates about 28 degrees. Thumb stiffness is assumed, MCP 3–5 estimated |

## Things to try in the viewer

1. Select the trapezoid in the list. The whole 12-bone unit lights up and the panel says it is lumped.
2. Press "Not represented in model…" to see what is missing.
3. Play `wrist_flexion_extension` and watch the warning in the status line when the wrist leaves the muscle validity envelope.
4. Pick `flexion_r` in the joint plot to see the angle against its limits.

## Screenshot and video checklist

- [ ] Status line visible, with the task name, time, model fingerprint and disclaimer
- [ ] Non-commercial licence notice included in any publication, plus the citations in `CITATION.cff`
- [ ] No caption claims clinical validity, patient specificity or physiological force values
- [ ] Failed tasks are shown as failed if shown at all
- [ ] A video states its playback speed (the viewer default is 0.5×)

## Demo video

`docs/media/biohand_demo.gif` shows real recorded runs, with no staged data; only the camera is scripted. Regenerate it with `godot_app/tools/demo_video.gd` (usage is in the file).
