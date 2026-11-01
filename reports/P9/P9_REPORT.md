# P9 — Release Documentation and Demo: Report

- Date: 2026-11-01
- Release candidate: 0.1.0 (not tagged)
- Evidence:
  - `reports/P9/test_run_output.txt`
  - `reports/P8/clean_install_output.txt`
  - `tests/test_p9_release.py`

## Delivered

| Item | File |
|---|---|
| Overview, architecture, limits, citations, reproduction | `README.md` |
| Windows installation (Linux not claimed) | `docs/INSTALL_WINDOWS.md` |
| Demo scenarios and screenshot/video checklist | `docs/DEMO_SCENARIOS.md` |
| Limitations summary | `docs/LIMITATIONS.md` |
| Future extension interface (no AI included) | `docs/AI_EXTENSION_INTERFACE.md`, `simulation/extensions.py` |
| Citation metadata | `CITATION.cff` |
| Third-party notices and licence texts | `docs/THIRD_PARTY_NOTICES.md`, `data/provenance/licenses/` |
| Release notes | `docs/RELEASE_NOTES.md` |
| One command to regenerate the evidence | `scripts/reproduce_all.py` |

## Install checks actually performed

All on the development machine, from a project copy that has no `third_party/`, `build/` or local assets, using a new virtual environment built from `requirements-lock.txt`:

| Check | Result |
|---|---|
| Installed packages equal the lock file | identical |
| Tests with no model files | 50 passed, 92 skipped, 0 failed |
| Tests after adding pinned MyoSim as in the README | 127 passed, 15 skipped, 0 failed |
| CLI `run-task` in that copy | works |

- This check found one test failing instead of skipping without model files; it was fixed.
- MyoSim came from a local clone of the pinned commit, not from GitHub, to avoid a long download. The same commit was checked by the test suite.
- The new environment used the Python already on this machine.

## Acceptance criteria (ROADMAP P9)

| Criterion | Status |
|---|---|
| Installation works on a clean Windows machine | **NOT VERIFIED.** Only the isolated checks above on the development machine, and those ran before the fixes. A separate machine has not been tried |
| README contains demo, architecture, limitations, citations, reproducibility | **PASS** (test) |
| Third-party notices and asset licences complete | **PASS** |
| Release tag | **NOT DONE.** A local git repository exists; the tag waits until CI has run after the first push |

**P9 status: documents complete; phase NOT complete.** Two items need the project owner:
1. Run the installation on another Windows machine.
2. Push to GitHub (owner decision), let CI run, then tag `v0.1.0`.

A CI workflow for Ubuntu and Windows is prepared (`.github/workflows/ci.yml`) but has never run, because nothing has been pushed yet.

Also still open, though not P9 criteria: choosing an anatomy reference for the provisional names, and use of the viewer by a person.

The fixes after this report are in `reports/fixes/FIXES_REPORT.md`.
