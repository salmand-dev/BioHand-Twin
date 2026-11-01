"""Export the MyoHand visual scene for Blender (local build only; never committed).

Writes build/visual/scene/ (gitignored):
  scene.json            body hierarchy (pose at qpos0, relative to parent) and mesh geoms
  meshes/<geom>.npz     compiled MuJoCo mesh vertices (geom frame, m) and triangle faces

Vertices come from the compiled model (model.mesh_vert), which are expressed in
the geom frame; geom_pos/geom_quat place them in the body frame. Using compiled
vertices avoids any file-importer axis convention.

Licence: geometry is MoBL-ARMS-derived (non-commercial). Outputs stay local.

Usage: .venv/Scripts/python.exe scripts/p2_export_visual_scene.py
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from simulation.myosim_source import MYOSIM_PINNED_COMMIT, build_myohand

OUT = ROOT / "build" / "visual" / "scene"


def export(out: Path = OUT) -> Path:
    model = build_myohand("fragment").model
    (out / "meshes").mkdir(parents=True, exist_ok=True)

    def name(obj: mujoco.mjtObj, i: int) -> str | None:
        return mujoco.mj_id2name(model, obj, i)

    bodies = [{
        "name": name(mujoco.mjtObj.mjOBJ_BODY, b),
        "parent": name(mujoco.mjtObj.mjOBJ_BODY, int(model.body_parentid[b])) if b else None,
        "pos": model.body_pos[b].tolist(), "quat_wxyz": model.body_quat[b].tolist(),
    } for b in range(model.nbody)]
    if np.any(model.qpos0 != 0) or np.any(model.qpos_spring != 0):
        raise RuntimeError("body_pos/body_quat equal the qpos0 pose only if all hinge qpos0 are 0")
    geoms = []
    for g in range(model.ngeom):
        if model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mid = int(model.geom_dataid[g])
        va, vn = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
        fa, fn = model.mesh_faceadr[mid], model.mesh_facenum[mid]
        gname = name(mujoco.mjtObj.mjOBJ_GEOM, g)
        np.savez(out / "meshes" / f"{gname}.npz",
                 vertices=model.mesh_vert[va:va + vn].astype(np.float64),
                 faces=model.mesh_face[fa:fa + fn].astype(np.int64))
        geoms.append({"name": gname, "body": name(mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[g])),
                      "pos": model.geom_pos[g].tolist(), "quat_wxyz": model.geom_quat[g].tolist(),
                      "file": f"meshes/{gname}.npz"})
    (out / "scene.json").write_text(json.dumps({
        "model_commit": MYOSIM_PINNED_COMMIT, "pose": "qpos0", "units": "m",
        "frame": "MuJoCo (right-handed, +Z up)",
        "licence_note": "MoBL-ARMS-derived geometry: non-commercial use only; do not commit or redistribute",
        "bodies": bodies, "geoms": geoms}, indent=1))
    return out


if __name__ == "__main__":
    print("exported", export())
