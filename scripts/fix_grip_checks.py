"""Tests of the grip force proxy, which P8 found sensitive to F_max and to a non-anatomical inertia.

1. Settling: is the 2 s reading a settled value? Repeat at 2, 4 and 8 s, at baseline and at hand inertia x0.1 and x10.
   If the inertia effect vanishes as the grip settles, it was a transient, not a static dependence.
2. Force scaling: grip against activation 0.1 to 1.0 and against F_max.
   Absolute contact parameters test whether a settled inertia effect comes from mass-scaled contact softness.
3. Comparison with dynamometer norms (Park et al. 1998). The model output sums contact normal forces on a fixed 40 mm
   cylinder. A dynamometer reads one force on its handle, so these are different quantities; this is a plausibility
   check, not validation.

Writes reports/fixes/grip_checks.json.
"""

from __future__ import annotations

import copy
import json
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from p8_sensitivity_performance import scale_force, scale_hand_inertia

from simulation.tasks import load_config, run_task

MEASURED = {"men_mean_N": round(47.7 * 9.80665, 1), "women_mean_N": round(27.2 * 9.80665, 1),
            "source": "Park et al. 1998, doi:10.35371/kjoem.1998.10.3.362, n = 1018, Jamar dynamometer, right hand"}
METRIC = "grip_force_mean_last_0p5s"


def grip(duration: float = 2.0, activation: float = 0.5, hook: Any = None) -> dict[str, Any]:
    cfg = copy.deepcopy(load_config("power_grip_proxy"))
    cfg["duration_s"] = duration
    cfg["phases"][0]["end_s"] = duration
    cfg["phases"][0]["activations"] = {"finger_flexors": activation, "thumb_flexors": activation}
    for m in cfg["metrics"]:
        if m.get("name") == METRIC:
            m["from_s"], m["to_s"] = duration - 0.5, duration
    r = run_task(cfg, model_hook=hook)
    return {"force_N": round(float(r.metrics[METRIC]), 2),
            "final_qvel_rad_s": round(float(r.metrics["final_max_abs_qvel_rad_s"]), 4)}


def absolute_contact(stiffness: float, damping: float, inner: Any = None) -> Any:
    """Contact softness as absolute stiffness and damping (MuJoCo negative solref), independent of effective mass."""
    def hook(m: Any) -> None:
        if inner is not None:
            inner(m)
        m.pair_solref[:] = [-stiffness, -damping]
        m.geom_solref[:] = [-stiffness, -damping]
    return hook


def contact_hypothesis() -> dict[str, Any]:
    """If mass-scaled contact softness causes the static inertia effect, absolute contact parameters must remove it."""
    out = {}
    for k_contact, d_contact in ((2.0e4, 200.0), (1.0e5, 400.0)):
        rows = {label: grip(8.0, hook=absolute_contact(k_contact, d_contact, inner)) for label, inner in
                (("baseline", None), ("hand inertia x0.1", scale_hand_inertia(0.1)),
                 ("hand inertia x10", scale_hand_inertia(10.0)))}
        forces = [r["force_N"] for r in rows.values()]
        out[f"k={k_contact:.0f} N/m, d={d_contact:.0f} N s/m"] = {
            "rows": rows, "inertia_spread_N": round(max(forces) - min(forces), 2)}
    return out


def main() -> None:
    settling = {}
    for label, hook in (("baseline", None), ("hand inertia x0.1", scale_hand_inertia(0.1)),
                        ("hand inertia x10", scale_hand_inertia(10.0))):
        settling[label] = {f"{t:.0f} s": grip(t, hook=hook) for t in (2.0, 4.0, 8.0)}
    spread = {}
    for t in ("2 s", "4 s", "8 s"):
        forces = [v[t]["force_N"] for v in settling.values()]
        spread[t] = round(max(forces) - min(forces), 2)
    activation = {f"{a:.2f}": grip(4.0, activation=a) for a in (0.1, 0.25, 0.5, 0.75, 1.0)}
    fmax = {f"F_max x{k}": grip(4.0, hook=scale_force(k)) for k in (0.8, 1.0, 1.2)}
    out = {"settling": settling, "inertia_spread_N": spread, "contact_hypothesis_8s": contact_hypothesis(),
           "activation_sweep_4s": activation, "fmax_sweep_4s": fmax,
           "measured_grip": MEASURED, "comparability": "not the same quantity; plausibility check only"}
    (ROOT / "reports" / "fixes" / "grip_checks.json").write_text(json.dumps(out, indent=1) + "\n", encoding="utf-8")
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
