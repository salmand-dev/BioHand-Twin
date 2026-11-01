"""P6 reproducibility evidence: every task twice through the API; compare full-export hashes; validate frames.

Documented tolerance: bitwise identity for the same model fingerprint, MuJoCo version and platform
(tested here). Cross-timestep agreement is in reports/P5 (same success verdicts at dt = 1 ms).
Cross-platform reproducibility is NOT tested.
Output: reports/P6/reproducibility.json
"""

from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

import jsonschema

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from simulation.api import load_schema, simulate_task
from simulation.tasks import TASK_DIR


def main() -> None:
    frame_schema, meta_schema = load_schema("telemetry_frame"), load_schema("session_metadata")
    results = {}
    for path in sorted(TASK_DIR.glob("*.yaml")):
        digests, n_frames, fingerprint = [], 0, ""
        for _ in range(2):
            session, frames = simulate_task(path.stem, every_n_steps=10, include_bodies=True)
            for f in frames:
                jsonschema.validate(f, frame_schema)
            jsonschema.validate(session.metadata(), meta_schema)
            digests.append(hashlib.sha256(json.dumps(frames, sort_keys=True).encode()).hexdigest())
            n_frames, fingerprint = len(frames), session.fingerprint
        results[path.stem] = {"frames": n_frames, "schema_valid": True, "identical": digests[0] == digests[1],
                              "frames_sha256": digests[0], "model_fingerprint": fingerprint}
        print(path.stem, results[path.stem]["identical"], n_frames, digests[0][:16])
    out = ROOT / "reports" / "P6"
    out.mkdir(parents=True, exist_ok=True)
    (out / "reproducibility.json").write_text(json.dumps({
        "tolerance": "bitwise identity on the same platform, MuJoCo version and model fingerprint",
        "not_tested": "cross-platform (other OS/CPU) reproducibility",
        "tasks": results}, indent=1) + "\n")


if __name__ == "__main__":
    main()
