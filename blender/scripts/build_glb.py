"""Blender (headless) script: build a GLB from scripts/p2_export_visual_scene.py output.

Run with Blender 3.6:
  blender --background --factory-startup --python blender/scripts/build_glb.py -- <scene_dir> <out.glb>

Scene structure: one Empty per MuJoCo body (named exactly as the body, parented as in
MuJoCo, local transform = body pose relative to parent at qpos0); one mesh object per
MuJoCo mesh geom, named "<geom>__visual" (geom and body names can coincide, and object
names must be unique; "." and ":" are avoided because Godot forbids them), parented
to its body Empty. Coordinates
are written unchanged (MuJoCo and Blender are both right-handed, +Z up); the glTF
exporter's +Y-up conversion then yields the project's MuJoCo->Godot transform.
No geometry is edited. Output must stay local (non-commercial MoBL-ARMS geometry).
"""

import json
import sys
from pathlib import Path

import bpy
import numpy as np

VISUAL_SUFFIX = "__visual"


def main() -> None:
    argv = sys.argv[sys.argv.index("--") + 1:]
    scene_dir, out_path = Path(argv[0]), Path(argv[1])
    scene = json.loads((scene_dir / "scene.json").read_text())

    bpy.ops.wm.read_factory_settings(use_empty=True)
    objects = {}
    for b in scene["bodies"]:
        obj = bpy.data.objects.new(b["name"], None)
        assert obj.name == b["name"], f"name collision for {b['name']}"
        obj.empty_display_type = "ARROWS"
        obj.empty_display_size = 0.01
        bpy.context.scene.collection.objects.link(obj)
        if b["parent"] is not None:
            obj.parent = objects[b["parent"]]
        obj.rotation_mode = "QUATERNION"
        obj.location = b["pos"]
        obj.rotation_quaternion = b["quat_wxyz"]
        objects[b["name"]] = obj

    for g in scene["geoms"]:
        data = np.load(scene_dir / g["file"])
        node = g["name"] + VISUAL_SUFFIX
        mesh = bpy.data.meshes.new(node)
        mesh.from_pydata(data["vertices"].tolist(), [], data["faces"].tolist())
        mesh.validate()
        mesh.update()
        obj = bpy.data.objects.new(node, mesh)
        assert obj.name == node, f"name collision for {node}"
        bpy.context.scene.collection.objects.link(obj)
        obj.parent = objects[g["body"]]
        obj.rotation_mode = "QUATERNION"
        obj.location = g["pos"]
        obj.rotation_quaternion = g["quat_wxyz"]

    out_path.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.export_scene.gltf(filepath=str(out_path), export_format="GLB", export_yup=True,
                              export_apply=False, export_extras=False)
    print(f"BUILD_GLB_OK bodies={len(scene['bodies'])} geoms={len(scene['geoms'])} out={out_path}")


main()
