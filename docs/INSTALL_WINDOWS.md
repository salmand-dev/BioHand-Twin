# Installation on Windows

Tested on the development machine only (Windows 10 Pro 10.0.19045, Python 3.11.4) in a fresh virtual environment. **It has not been tested on a separate clean Windows machine, and Linux is not tested.**

This software is for non-commercial research and education. The model data derive from MoBL-ARMS, whose licence forbids commercial use without a commercial licence.

## 1. Tools

| Tool | Version tested | Needed for |
|---|---|---|
| Python | 3.11.4 (`>=3.10,<3.13`) | everything |
| Git | 2.56 | fetching MyoSim |
| Blender | 3.6.23 | building the local 3D hand file (viewer only) |
| Godot | 4.7.2 | the viewer only |

## 2. Python environment

Run these from the project folder:

```bash
py -3.11 -m venv .venv
```

```bash
.venv/Scripts/python.exe -m pip install -r requirements-lock.txt
```

```bash
.venv/Scripts/python.exe -m pip install --no-deps -e .
```

## 3. Reference model (not bundled)

```bash
git clone https://github.com/MyoHub/myo_sim.git third_party/myo_sim
```

```bash
git -C third_party/myo_sim checkout 93b0ca8f4ec90c9899ee7f05fee561e9911da91b
```

The source OpenSim models are only needed to regenerate the source comparison. Patches are validated at build time against `data/reference_osim_inventory.json`, which is part of the project. To regenerate that file, download these from SimTK and accept their agreements yourself:
- MoBL-ARMS (https://simtk.org/projects/upexdyn): extract to `third_party/simtk_mobl_arms_bimanual/`
- 2nd-Hand (https://simtk.org/projects/hand_muscle): extract to `third_party/simtk_2nd_hand/`

## 4. Run the tests

```bash
.venv/Scripts/python.exe -m pytest -v
```

Tests that need `third_party/`, the local 3D file or Godot **skip** when those are missing. A run with skips does not reproduce the full evidence.

## 5. Viewer (optional)

Build the local 3D hand file. The geometry is non-commercial; never commit or redistribute it.

```bash
.venv/Scripts/python.exe scripts/p2_export_visual_scene.py
```

```bash
"C:/Program Files/Blender Foundation/Blender 3.6/blender.exe" --background --factory-startup --python blender/scripts/build_glb.py -- build/visual/scene build/visual/myohand_fragment.glb
```

Copy `build/visual/myohand_fragment.glb` into `godot_app/local_assets/`. Then record the runs:

```bash
.venv/Scripts/python.exe scripts/reproduce_all.py --with-visual
```

Open `godot_app/project.godot` in Godot 4.7 and run it. If Godot is not in the default location, set the `BIOHAND_GODOT` environment variable for the tests.
