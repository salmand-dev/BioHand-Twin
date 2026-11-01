"""P7 tests: Godot viewer contract (no physics, pose match, labels, inventory drift)."""

import importlib.util
import json
import os
import re
import subprocess
from pathlib import Path

import mujoco
import numpy as np
import pytest

from biomechanics.frames import mujoco_to_godot_pos

ROOT = Path(__file__).resolve().parents[1]
GODOT_APP = ROOT / "godot_app"
GODOT = Path(os.environ.get(
    "BIOHAND_GODOT",
    Path.home() / "Downloads" / "Godot_v4.7.2-stable_win64.exe" / "Godot_v4.7.2-stable_win64_console.exe"))
PHYSICS_PATTERN = re.compile(r"\b(RigidBody3D|StaticBody3D|CharacterBody3D|AnimatableBody3D|CollisionShape3D|"
                             r"CollisionPolygon3D|Area3D|PhysicalBone3D|SoftBody3D|VehicleBody3D|\w*Joint3D|"
                             r"PhysicsServer3D|PhysicsDirectSpaceState3D|intersect_ray)\b")
POSE_TOL_M = 1e-5        # float32 transforms in Godot (observed ~1.2e-7 m)
BASIS_TOL = 1e-5


def test_no_physics_in_viewer_sources():
    hits = []
    for p in list(GODOT_APP.rglob("*.gd")) + list(GODOT_APP.rglob("*.tscn")):
        if "local_assets" in p.parts or ".godot" in p.parts:
            continue
        for i, line in enumerate(p.read_text(encoding="utf-8").splitlines(), 1):
            code = re.sub(r'"[^"]*"', '""', line.split("#", 1)[0])   # names inside strings are not nodes
            if PHYSICS_PATTERN.search(code):
                hits.append(f"{p.name}:{i}: {line.strip()}")
    assert hits == [], hits


@pytest.mark.skipif(not (ROOT / "third_party" / "myo_sim" / "myo_sim").is_dir(), reason="third_party/myo_sim not cloned")
def test_godot_inventory_is_current():
    spec = importlib.util.spec_from_file_location("cli", ROOT / "scripts" / "biohand.py")
    cli = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cli)
    fresh = json.dumps(cli.godot_inventory(), indent=1, sort_keys=True) + "\n"
    assert (GODOT_APP / "data" / "biohand_inventory.json").read_text(encoding="utf-8") == fresh


def test_inventory_labels_cover_every_bone_representation():
    inv = json.loads((GODOT_APP / "data" / "biohand_inventory.json").read_text(encoding="utf-8"))
    reps = {b["representation"].split(":")[0] for b in inv["bodies"].values()}
    viewer = (GODOT_APP / "scripts" / "main.gd").read_text(encoding="utf-8")
    for rep in reps:
        assert rep in viewer or rep == "rigid_part_of", rep
    assert "LUMPED" in viewer and "unverified name" in viewer and "Not represented" in viewer


needs_godot = pytest.mark.skipif(
    not (GODOT.is_file() and (GODOT_APP / "local_assets" / "myohand_fragment.glb").is_file()
         and (GODOT_APP / "local_assets" / "runs" / "power_grip_proxy" / "playback.json").is_file()),
    reason="Godot, local GLB or local playback runs absent")


@pytest.fixture(scope="module")
def godot_check():
    r = subprocess.run([str(GODOT), "--headless", "--path", str(GODOT_APP), "--script", "res://tools/check_viewer.gd"],
                       capture_output=True, text=True, timeout=300, check=False)
    line = next(ln for ln in r.stdout.splitlines() if ln.startswith("CHECK_JSON "))
    return json.loads(line.split(" ", 1)[1]), r.stdout + r.stderr


@needs_godot
def test_viewer_scene_has_no_physics_nodes_and_loads_runs(godot_check):
    out, log = godot_check
    assert out["physics_nodes"] == []
    assert out["body_nodes"] == 38 and out["visual_nodes"] == 32
    assert len(out["runs"]) >= 1 and all(r["loaded"] for r in out["runs"].values())
    assert "SCRIPT ERROR" not in log


@needs_godot
def test_viewer_pose_matches_mujoco_telemetry(godot_check):
    out, _ = godot_check
    for run, r in out["runs"].items():
        assert r["bodies_checked"] > 0, run
        assert r["max_pos_err_m"] < POSE_TOL_M, run
        assert r["max_basis_err"] < BASIS_TOL, run


@needs_godot
def test_lumped_bone_selection_selects_whole_unit(godot_check):
    out, _ = godot_check
    assert out["select_trapezoid_unit"] == "lunate_r" and out["select_trapezoid_info_mentions_lumped"]


@needs_godot
def test_grip_fixture_drawn_at_mujoco_pose(godot_check):
    from simulation.tasks import build_task_model, load_config

    out, _ = godot_check
    m, _ = build_task_model(load_config("power_grip_proxy"))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    g = m.geom("grip_cylinder").id
    centre = mujoco_to_godot_pos(d.geom_xpos[g])
    axis = mujoco_to_godot_pos(d.geom_xmat[g].reshape(3, 3)[:, 2])
    got = out["grip_fixture_godot"]
    assert np.allclose(got["origin"], centre, atol=1e-5)
    assert abs(abs(np.dot(got["axis"], axis)) - 1.0) < 1e-5


@needs_godot
def test_click_picking_returns_a_hand_bone(godot_check):
    out, _ = godot_check
    # ray vs mesh AABB is approximate: the nearest box may be a neighbouring segment, so require a bone body
    inv = json.loads((GODOT_APP / "data" / "biohand_inventory.json").read_text(encoding="utf-8"))
    assert inv["bodies"][out["pick_at_proxph3_centre"]]["structure_type"] == "bone"


@needs_godot
def test_playback_advances_only_while_playing(godot_check):
    out, _ = godot_check
    assert out["playback"]["frame_when_paused"] == 0
    assert abs(out["playback"]["time_after_0p5s"] - 0.5) < 0.03


@needs_godot
def test_scaffold_layer_hidden_by_default(godot_check):
    assert godot_check[0]["scaffold_hidden_by_default"] is True


@needs_godot
def test_panels_do_not_overlap_at_common_window_sizes(godot_check):
    overlaps = godot_check[0]["panel_overlap_by_window_size"]
    assert len(overlaps) == 4
    assert not any(overlaps.values())


@needs_godot
def test_click_selects_but_drag_orbits(godot_check):
    out = godot_check[0]
    assert out["left_click_selects"] is True
    assert out["left_drag_selects"] is False
    assert out["wheel_zoom_ratio"] == pytest.approx(0.9)


@needs_godot
def test_text_contrast_meets_wcag_aa(godot_check):
    out = godot_check[0]
    assert out["worst_text_contrast"] >= 4.5
    assert out["on_accent_contrast"] >= 4.5
    assert out["on_warn_contrast"] >= 4.5


@needs_godot
def test_keyboard_controls(godot_check):
    k = godot_check[0]["keyboard"]
    assert k == {"space_plays": True, "right_steps_to": 1, "f1_opens_help": True, "esc_closes_help": True}


@needs_godot
def test_filters_and_selection_links(godot_check):
    out = godot_check[0]
    assert 0 < out["bone_filter"]["thumb"] < out["bone_filter"]["all"]
    assert out["muscle_filter_pollicis_rows"] >= 7
    assert out["plot_follows_bone"] == "ip_flexion_r"
    assert out["muscle_info_title"] == "abductor pollicis brevis"
    assert out["telemetry_muscles_without_row"] == 0


@needs_godot
def test_free_view_stays_usable(godot_check):
    for w, h in godot_check[0]["free_view_size_by_window_size"].values():
        assert w >= 300 and h >= 200
