"""BioHand Twin command-line interface (P6).

  biohand.py inventory                         print model inventory JSON
  biohand.py run-task <id> [--out DIR]         run a task; write frames.csv and playback.json
                                               (default: godot_app/local_assets/runs/<id>, gitignored)
  biohand.py export-godot-inventory            write godot_app/data/biohand_inventory.json
  biohand.py list-tasks                        list configured tasks

Usage: .venv/Scripts/python.exe scripts/biohand.py <command> ...
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from simulation.api import SCHEMA_VERSION, BioHandSession, simulate_task
from simulation.tasks import TASK_DIR


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="biohand")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("inventory")
    sub.add_parser("list-tasks")
    sub.add_parser("export-godot-inventory")
    rt = sub.add_parser("run-task")
    rt.add_argument("task_id")
    rt.add_argument("--out", type=Path, default=ROOT / "godot_app" / "local_assets" / "runs")
    rt.add_argument("--every", type=int, default=10, help="record every N simulation steps")
    args = ap.parse_args(argv)
    if args.cmd == "inventory":
        print(json.dumps(BioHandSession().inventory(), indent=1))
    elif args.cmd == "export-godot-inventory":
        path = ROOT / "godot_app" / "data" / "biohand_inventory.json"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(godot_inventory(), indent=1, sort_keys=True) + "\n", encoding="utf-8")
        print("wrote", path)
    elif args.cmd == "list-tasks":
        for p in sorted(TASK_DIR.glob("*.yaml")):
            print(p.stem)
    else:
        session, frames = simulate_task(args.task_id, args.every, include_bodies=True, include_paths=True)
        out = args.out / args.task_id
        out.mkdir(parents=True, exist_ok=True)
        (args.out / ".gdignore").write_text("", encoding="utf-8")   # Godot: do not import run files
        (out / "frames.csv").write_text(session.export_csv(frames), encoding="utf-8")
        (out / "playback.json").write_text(session.export_json(frames), encoding="utf-8")
        print(f"wrote {len(frames)} frames to {out}")
    return 0


def godot_inventory() -> dict:
    """Names and classifications for the viewer (no geometry; safe to commit). Deterministic."""
    s = BioHandSession()
    inv = s.inventory()
    g = s.glossary
    unit_of = {b: u["root_body"] for u in inv["motion_units"] for b in u["bodies"]}
    rows = list(csv.DictReader((ROOT / "blender" / "visual_to_physics_mapping.csv").open(encoding="utf-8")))
    bodies = {}
    for e in g.entries("bodies"):
        b = e["model_name"]
        bodies[b] = {"canonical_name": e.get("canonical_name"), "representation": e["representation"],
                     "status": e["status"], "structure_type": e["structure_type"], "notes": e.get("notes", ""),
                     "motion_unit": unit_of[b],
                     "visual_nodes": [r["visual_node"] for r in rows
                                      if r["mujoco_body"] == b and r["representation"] != "debug_only"]}
    meta = inv["metadata"]
    return {"schema_version": SCHEMA_VERSION, "model_fingerprint_sha256": meta["model"]["fingerprint_sha256"],
            "licence_notice": meta["model"]["licence_notice"], "disclaimer": meta["disclaimer"],
            "bodies": bodies, "joints": inv["joints"], "actuators": inv["actuators"],
            "motion_units": inv["motion_units"], "not_represented": inv["not_represented"]}


if __name__ == "__main__":
    raise SystemExit(main())
