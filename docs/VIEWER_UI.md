# Viewer UI

The Godot viewer plays recorded MuJoCo runs. It runs no physics. Design tokens are in `godot_app/scripts/ui_theme.gd`.

## Layout
- **Top bar:** task selector with a one-line caveat for that task, and Help.
- **Left:** layers, and a filterable bone list grouped by digit and region.
- **Centre:** 3D view with view presets, contact force, activation legend and a warning banner.
- **Bottom:** playback controls and the joint plot. Click or drag the plot to seek.
- **Right:** details of the selected bone or muscle, and a filterable muscle list grouped by region.
- **Status bar:** disclaimer, model fingerprint and telemetry schema.

Panels sit in containers, so they cannot overlap, and the 3D view is centred in the free area.

## Controls
| Input | Action |
|---|---|
| Left- or right-drag | Rotate |
| Middle-drag | Pan |
| Wheel | Zoom |
| Click | Select a bone (empty space clears the selection) |
| 1, 2, 3 | Dorsal, palmar, radial view |
| F | Frame the recorded motion |
| Space | Play or pause |
| Left, Right | One frame back or forward |
| Home, End | First or last frame |
| Esc | Clear the selection, or close Help |
| F1 | Help |
| Ctrl and +, -, 0 | Text size |

## Design decisions
- **Colours:** a neutral dark theme, so the hand and data carry the colour. The generated design-system palette (neon green and red on black) was rejected, because it does not suit a scientific tool and red-green pairs fail with colour blindness.
- **Activation ramp:** blue to orange, readable with red-green colour blindness. The value is also shown as a number.
- **Meaning is never colour alone:** limits are dashed, warnings have an icon and text, unverified names have a question mark and a label, and selected rows are labelled in the details panel.
- **Scientific honesty in the UI:** each task shows its caveat. The validity warning appears on the frames it applies to. Simplified actuators are labelled, and lumped bones say they cannot move alone.
- **Font:** Godot's default font. Fira Sans, as the design system suggested, would need a bundled font file; this was left out of the first release.

## Checks
Automated in `tests/test_p7_viewer.py` through `godot_app/tools/check_viewer.gd`: no panel overlap at 4 window sizes, a usable 3D area, text contrast of at least 4.5:1, keyboard controls, filters, and linked selection.

Used with real mouse and keyboard input through desktop automation on Windows.

## Performance
About 45-48 FPS at 1600x900 with all layers on an AMD HD 8690M (60 FPS before the redesign). The first redesigned version dropped to 15 FPS because the muscle panel re-laid out on every frame; a lightweight bar and a separate plot cursor layer fixed most of that.

## Not tested or not included
- No person has used it, and there is no usability study.
- Not tested with a screen reader, on a touch screen, on high-DPI displays, or outside Windows.
- Right- and middle-drag were not exercised with real input.
- English only.
- Bone picking uses bounding boxes, so a click near a bone may select it or a neighbour. The list is exact.
- Windows smaller than 1024x640 are not supported.
