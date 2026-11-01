"""Run all P5 functional tasks; write telemetry, summaries, determinism and timestep-sensitivity evidence.

Outputs reports/P5/tasks/<id>/timeseries.csv and summary.json, plus reports/P5/p5_tasks_summary.json.
Usage: .venv/Scripts/python.exe scripts/p5_run_tasks.py
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from simulation.tasks import TASK_DIR, load_config, run_task

FINE_DT = 0.001


def main() -> None:
    out_dir = ROOT / "reports" / "P5" / "tasks"
    summary = {}
    for path in sorted(TASK_DIR.glob("*.yaml")):
        cfg = load_config(path.stem)
        res = run_task(cfg)
        again = run_task(cfg)
        fine = run_task(cfg, timestep=FINE_DT)
        task_dir = out_dir / cfg["id"]
        task_dir.mkdir(parents=True, exist_ok=True)
        (task_dir / "timeseries.csv").write_text(res.to_csv(), encoding="utf-8")
        keys = [k for k, v in res.metrics.items() if isinstance(v, (int, float)) and not isinstance(v, bool)]
        sens = {k: {"dt_0p002": res.metrics[k], "dt_0p001": fine.metrics[k],
                    "abs_diff": round(abs(float(res.metrics[k]) - float(fine.metrics[k])), 6)} for k in keys}
        record = {"id": cfg["id"], "description": cfg["description"], "config": f"data/experiments/tasks/{path.name}",
                  "meta": res.meta, "metrics": res.metrics, "success": res.success,
                  "deterministic_repeat": bool(np.array_equal(res.rows, again.rows)),
                  "timestep_sensitivity": sens,
                  "fine_dt_success": fine.success["all_passed"]}
        (task_dir / "summary.json").write_text(json.dumps(record, indent=1) + "\n")
        summary[cfg["id"]] = {"passed": res.success["all_passed"], "healthy": res.success["numerically_healthy"],
                              "deterministic": record["deterministic_repeat"],
                              "fine_dt_passed": record["fine_dt_success"],
                              "checks": [(c["metric"], c["op"], c["value"], c["observed"], c["passed"])
                                         for c in res.success["checks"]],
                              "max_limit_violation_deg": round(float(np.degrees(res.metrics["max_limit_violation_rad"])), 2),
                              "outside_envelope_fraction": res.metrics["fraction_samples_outside_muscle_validity_envelope"],
                              "max_KE_J": res.metrics["max_kinetic_energy_J"]}
        print(cfg["id"], json.dumps(summary[cfg["id"]]))
    (ROOT / "reports" / "P5" / "p5_tasks_summary.json").write_text(json.dumps(summary, indent=1) + "\n")


if __name__ == "__main__":
    main()
