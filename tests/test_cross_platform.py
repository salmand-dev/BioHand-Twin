"""Run every task and compare it with the results recorded on the development machine (Windows).

Bitwise identity is only claimed on one platform (P6). Across platforms, verdicts must match and metrics must agree
within tolerances set here. Pinch is contact-driven and fragile, so its force gets a wider tolerance.
"""

import json
from pathlib import Path

import pytest

from simulation.myosim_source import DEFAULT_MYOSIM_DIR
from simulation.tasks import load_config, run_task

ROOT = Path(__file__).resolve().parents[1]
RECORDED = json.loads((ROOT / "reports" / "P5" / "p5_tasks_summary.json").read_text(encoding="utf-8"))
TOLERANCE = {"pinch": 0.5}
DEFAULT_TOLERANCE = 0.02

pytestmark = pytest.mark.skipif(not (DEFAULT_MYOSIM_DIR / "myo_sim").is_dir(), reason="third_party/myo_sim not cloned")


@pytest.mark.parametrize("task", sorted(RECORDED))
def test_task_matches_recorded_result(task):
    result = run_task(load_config(task))
    assert result.success["all_passed"] == RECORDED[task]["passed"]
    tolerance = TOLERANCE.get(task, DEFAULT_TOLERANCE)
    observed = [c["observed"] for c in result.success["checks"]]
    expected = [c[3] for c in RECORDED[task]["checks"]]
    print(task, "observed", observed, "recorded", expected)
    for now, then in zip(observed, expected, strict=True):
        assert now == pytest.approx(then, rel=tolerance, abs=0.01)
