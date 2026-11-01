"""P8: sensitivity of task outcomes to uncertain parameters, and performance on this machine.

Each perturbation is applied to one parameter group of the compiled model, for this analysis only; no patch
or reference value changes. Output: reports/P8/sensitivity.json, reports/P8/performance.json
"""

from __future__ import annotations

import json
import platform
import sys
import time
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from simulation.api import BioHandSession
from simulation.tasks import load_config, run_task


def scale_force(k: float) -> Any:
    def hook(m: mujoco.MjModel) -> None:
        m.actuator_gainprm[:, 2] *= k
        m.actuator_biasprm[:, 2] *= k
    return hook


def scale_damping(k: float) -> Any:
    def hook(m: mujoco.MjModel) -> None:
        m.dof_damping[:] *= k
    return hook


def scale_hand_inertia(k: float) -> Any:
    """Scale inertia of bodies distal to the wrist (their values are non-anatomical; gap I1)."""
    def hook(m: mujoco.MjModel) -> None:
        wrist = m.body("lunate_r").id
        for b in range(wrist, m.nbody):
            m.body_inertia[b] *= k
        mujoco.mj_setConst(m, mujoco.MjData(m))
    return hook


def limit_timeconst(tc: float) -> Any:
    def hook(m: mujoco.MjModel) -> None:
        m.jnt_solref[:, 0] = tc
    return hook


def contact_timeconst(tc: float) -> Any:
    def hook(m: mujoco.MjModel) -> None:
        m.geom_solref[:, 0] = tc
    return hook


PERTURBATIONS: dict[str, tuple[Any, float | None]] = {
    "baseline": (None, None),
    "F_max x0.8": (scale_force(0.8), None), "F_max x1.2": (scale_force(1.2), None),
    "joint damping x0.5": (scale_damping(0.5), None), "joint damping x2": (scale_damping(2.0), None),
    "hand inertia x0.1": (scale_hand_inertia(0.1), None), "hand inertia x10": (scale_hand_inertia(10.0), None),
    "limit timeconst upstream 0.02 s": (limit_timeconst(0.02), None),
    "contact timeconst 0.01 s": (contact_timeconst(0.01), None),
    "timestep 0.001 s": (None, 0.001),
}


def time_to_half_flexion(res: Any) -> float:
    q = np.mean([res.column(f"q_mcp{d}_flexion_r_rad") for d in (2, 3, 4, 5)], axis=0)
    idx = np.where(q >= 0.5 * np.pi / 2)[0]
    return float(res.column("time_s")[idx[0]]) if idx.size else float("nan")


def sensitivity() -> dict[str, Any]:
    tasks = {"hand_closing": "flexion_fraction_final", "hand_opening": "flexion_fraction_final",
             "thumb_opposition": "thumb_index_min_distance_ratio", "wrist_flexion_extension": "flexion_max_phase1",
             "wrist_deviation": "deviation_min_phase2", "power_grip_proxy": "grip_force_mean_last_0p5s",
             "pinch": "pinch_force_mean_last_0p5s", "rest_neutral": "final_max_abs_qvel_rad_s"}
    out: dict[str, Any] = {}
    for tid, metric in tasks.items():
        cfg = load_config(tid)
        rows = {}
        for name, (hook, dt) in PERTURBATIONS.items():
            r = run_task(cfg, timestep=dt, model_hook=hook)
            rows[name] = {"metric_value": r.metrics[metric], "passed": r.success["all_passed"],
                          "finite": r.metrics["finite"], "warnings": r.metrics["warnings"]}
            if tid == "hand_closing":
                rows[name]["time_to_half_MCP_flexion_s"] = round(time_to_half_flexion(r), 3)
            print(tid, name, rows[name], flush=True)
        base = rows["baseline"]
        out[tid] = {"metric": metric, "rows": rows,
                    "verdict_changes": [n for n, v in rows.items() if v["passed"] != base["passed"]],
                    "max_abs_change": round(max(abs(v["metric_value"] - base["metric_value"]) for v in rows.values()), 4)}
    return out


def performance() -> dict[str, Any]:
    s = BioHandSession()
    rng = np.random.default_rng(0)
    u = rng.uniform(0, 1, s.model.nu)
    s.set_controls(u)
    t = time.perf_counter()
    n = 5000
    s.step(n)
    step_s = (time.perf_counter() - t) / n
    t = time.perf_counter()
    for _ in range(50):
        s.frame(include_bodies=True, include_paths=True)
    frame_s = (time.perf_counter() - t) / 50
    return {"cpu": platform.processor(), "steps": n, "ms_per_step": round(step_s * 1000, 4),
            "realtime_factor": round(float(s.model.opt.timestep) / step_s, 1),
            "ms_per_full_telemetry_frame": round(frame_s * 1000, 3),
            "note": "single thread, one run on the development laptop, not a benchmark claim"}


def main() -> None:
    path = ROOT / "reports" / "P8"
    path.mkdir(parents=True, exist_ok=True)
    perf = performance()
    print(perf, flush=True)
    (path / "performance.json").write_text(json.dumps(perf, indent=1) + "\n")
    (path / "sensitivity.json").write_text(json.dumps(sensitivity(), indent=1) + "\n")


if __name__ == "__main__":
    main()
