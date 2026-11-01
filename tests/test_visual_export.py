"""Verify the locally built GLB against MuJoCo through biomechanics.frames.

Skipped when build/visual/myohand_fragment.glb has not been built (it is a local,
gitignored, non-commercial artefact). Build it with:
  .venv/Scripts/python.exe scripts/p2_export_visual_scene.py
  blender --background --factory-startup --python blender/scripts/build_glb.py -- \
      build/visual/scene build/visual/myohand_fragment.glb
"""

import csv
import json
import os
import struct
import subprocess
from pathlib import Path

import mujoco
import numpy as np
import pytest

from biomechanics.frames import mujoco_to_godot_pos, quat_to_matrix_wxyz

ROOT = Path(__file__).resolve().parents[1]
GLB = ROOT / "build" / "visual" / "myohand_fragment.glb"
pytestmark = pytest.mark.skipif(not GLB.is_file(), reason="local GLB not built")

# Blender stores transforms in float32 (relative eps 1.2e-7). At ~0.6 m from the origin and a
# body chain up to 10 levels deep, accumulated rounding reaches ~1-2 um (observed max 1.4 um).
# 10 um is far below any visual or anatomical relevance.
POS_TOL_M = 1e-5
VISUAL_SUFFIX = "__visual"


def read_glb_json(path: Path) -> dict:
    raw = path.read_bytes()
    magic, version, _ = struct.unpack_from("<4sII", raw, 0)
    assert magic == b"glTF" and version == 2
    length, ctype = struct.unpack_from("<I4s", raw, 12)
    assert ctype == b"JSON"
    return json.loads(raw[20:20 + length])


def trs_matrix(node: dict) -> np.ndarray:
    m = np.eye(4)
    if "matrix" in node:
        return np.array(node["matrix"]).reshape(4, 4).T
    x, y, z, w = node.get("rotation", [0, 0, 0, 1])
    m[:3, :3] = quat_to_matrix_wxyz(np.array([w, x, y, z])) @ np.diag(node.get("scale", [1, 1, 1]))
    m[:3, 3] = node.get("translation", [0, 0, 0])
    return m


@pytest.fixture(scope="module")
def gltf_world():
    doc = read_glb_json(GLB)
    nodes = doc["nodes"]
    parent = {c: i for i, n in enumerate(nodes) for c in n.get("children", [])}
    world: dict[str, np.ndarray] = {}

    def wm(i: int) -> np.ndarray:
        m = trs_matrix(nodes[i])
        return wm(parent[i]) @ m if i in parent else m

    for i, n in enumerate(nodes):
        world[n["name"]] = wm(i)
    return doc, world


@pytest.fixture(scope="module")
def mj():
    from simulation.myosim_source import build_myohand

    model = build_myohand("fragment").model
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def test_every_body_and_mesh_geom_is_a_named_node(gltf_world, mj):
    _, world = gltf_world
    model, _ = mj
    for b in range(model.nbody):
        assert mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) in world
    for g in range(model.ngeom):
        if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            assert mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) + VISUAL_SUFFIX in world


def test_body_world_positions_match_mujoco_after_conversion(gltf_world, mj):
    _, world = gltf_world
    model, data = mj
    worst = 0.0
    for b in range(1, model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b)
        err = np.linalg.norm(world[name][:3, 3] - mujoco_to_godot_pos(data.xpos[b]))
        worst = max(worst, err)
    assert worst < POS_TOL_M, worst


def test_geom_orientations_match_mujoco_after_conversion(gltf_world, mj):
    _, world = gltf_world
    model, data = mj
    c = np.array([[1.0, 0, 0], [0, 0, 1], [0, -1, 0]])
    for g in range(model.ngeom):
        if model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) + VISUAL_SUFFIX
        r_expected = c @ data.geom_xmat[g].reshape(3, 3) @ c.T
        assert np.allclose(world[name][:3, :3], r_expected, atol=1e-5), name
        assert np.allclose(world[name][:3, 3], mujoco_to_godot_pos(data.geom_xpos[g]), atol=POS_TOL_M), name


def test_mesh_vertex_bounds_preserved(gltf_world, mj):
    """Accessor min/max (node-local, Godot axes) equal the converted MuJoCo vertex bounds."""
    doc, _ = gltf_world
    model, _ = mj
    by_name = {n["name"]: n for n in doc["nodes"]}
    for g in range(model.ngeom):
        if model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) + VISUAL_SUFFIX
        mid = model.geom_dataid[g]
        v = model.mesh_vert[model.mesh_vertadr[mid]:model.mesh_vertadr[mid] + model.mesh_vertnum[mid]]
        conv = v.astype(float) @ np.array([[1.0, 0, 0], [0, 0, 1], [0, -1, 0]]).T
        prim = doc["meshes"][by_name[name]["mesh"]]["primitives"][0]
        acc = doc["accessors"][prim["attributes"]["POSITION"]]
        assert np.allclose(acc["min"], conv.min(0), atol=1e-6), name
        assert np.allclose(acc["max"], conv.max(0), atol=1e-6), name


def test_mapping_csv_agrees_with_glb(gltf_world):
    _, world = gltf_world
    rows = list(csv.DictReader((ROOT / "blender" / "visual_to_physics_mapping.csv").open()))
    mesh_rows = [r for r in rows if r["representation"] != "debug_only"]
    assert len(mesh_rows) == 32
    for r in mesh_rows:
        assert r["visual_node"] in world and r["mujoco_body"] in world


GODOT = Path(os.environ.get(
    "BIOHAND_GODOT",
    Path.home() / "Downloads" / "Godot_v4.7.2-stable_win64.exe" / "Godot_v4.7.2-stable_win64_console.exe"))
GODOT_GLB = ROOT / "godot_app" / "local_assets" / "myohand_fragment.glb"


@pytest.mark.skipif(not (GODOT.is_file() and GODOT_GLB.is_file()), reason="Godot or local GLB copy absent")
def test_godot_import_positions_match_mujoco(mj):
    """End to end: Godot's own node transforms equal frames.mujoco_to_godot_pos(MuJoCo)."""
    r = subprocess.run([str(GODOT), "--headless", "--path", str(ROOT / "godot_app"),
                        "--script", "res://tools/dump_glb_nodes.gd"],
                       capture_output=True, text=True, timeout=300, check=False)
    line = next(ln for ln in r.stdout.splitlines() if ln.startswith("DUMP_JSON "))
    got = json.loads(line.split(" ", 1)[1])
    model, data = mj
    assert got["__visual_node_count"] == 32
    for name, pos in got.items():
        if name.startswith("__"):
            continue
        expected = mujoco_to_godot_pos(data.xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)])
        assert np.linalg.norm(np.array(pos) - expected) < POS_TOL_M, name
    assert "leaked" not in r.stderr + r.stdout
