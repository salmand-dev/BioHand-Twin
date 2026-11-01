# P0 — Repository and Toolchain Audit

- Date: 2026-10-31
- Evidence: `reports/P0/environment_audit.json` (machine-readable), `reports/P0/p0_environment_audit_console.txt`
- Reproduce: `.venv/Scripts/python.exe scripts/p0_environment_audit.py`

## 1. Tool versions (measured, not assumed)

| Item | Result | How verified |
|---|---|---|
| OS | Windows 10 Pro 10.0.19045 (build 3086), x86_64 | `platform`, `ver` |
| Python (system) | 3.11.4 at `%LOCALAPPDATA%\Programs\Python\Python311` | `python --version` |
| `python` on PATH | **Resolves to the Microsoft Store stub, not 3.11** — use `py -3.11` or the venv | `which -a python` |
| Project venv | `.venv` (Python 3.11.4), created fresh this session | `sys.prefix != sys.base_prefix` |
| MuJoCo (Python bindings) | 3.17.0 (package and `mj_versionString()` agree) | import in venv |
| MuJoCo (standalone, user-supplied) | 3.17.0, `Downloads\mujoco-3.17.0-windows-x86_64`, Apache-2.0 | `mujoco.dll` FileVersion |
| Godot | 4.7.2.stable.official.ed1daf0bf | `--version` (console exe) |
| Godot executable path | `Downloads\Godot_v4.7.2-stable_win64.exe\` is a **folder** containing `Godot_v4.7.2-stable_win64.exe` and `..._console.exe` | `ls` |
| Blender | 3.6.23 (build 2025-06-17) | `blender --version` |
| Git | 2.56.0.windows.2 | `git --version` |
| NumPy / SciPy / PyYAML | 2.4.6 / 1.17.1 / 6.0.3 | `pip freeze` |
| pytest / ruff / mypy | 9.2.0 / 0.17.3 / 2.4.1 | `pip freeze` |

Full pinned environment: `requirements-lock.txt` (24 packages, `pip freeze --exclude-editable`).

## 2. Hardware

| Item | Value |
|---|---|
| CPU | Intel Core i7-4610M @ 3.00 GHz, 4 logical processors |
| RAM | 8.5 GB (8 524 095 488 bytes) |
| GPU | Intel HD Graphics 4600 (driver 20.19.15.4835) + AMD Radeon HD 8690M (driver 15.201.1101.0) |
| Disk C: | 536 GB total, ~90 GB free |

Implication: the Godot project already uses `gl_compatibility`. Keep it — Forward+/Mobile need Vulkan, and Vulkan support on this 2013-era GPU pair is not verified. P7 performance budgets must be set on this hardware.

## 3. Repository state

- **Not a git repository** (`git status` → rc 128, "not a git repository"). No commits or history exist. Nothing has been initialised, because creating a repository is the owner's decision. Recommended next step: `git init`, then commit the scaffold plus P0/P1 outputs.
- README references a `.claude/` directory with agent briefs. **It does not exist.**
- Files inspected: all 33 scaffold files (docs, `biomechanics/`, `simulation/`, `godot_app/`, `blender/`, `data/`, `tests/`).
- Pre-existing scaffold tests: 3/3 passed. `ruff` reported 3 import-order findings in the scaffold; these were auto-fixed (formatting only).

## 4. Changes made in P0

| File | Change |
|---|---|
| `.venv/` | New virtual environment (gitignored) |
| `requirements-lock.txt` | New exact dependency lock |
| `pyproject.toml` | `mujoco==3.17.0` pinned; `pyyaml` added; package discovery restricted to `biomechanics*`, `simulation*`; pytest `pythonpath` and `integration` marker |
| `.gitignore` | New (venv, caches, `third_party/`, Godot cache, `MUJOCO_LOG.TXT`) |
| `scripts/p0_environment_audit.py` | New audit script |
| `biomechanics/*.py`, `tests/test_scaffold.py` | Import order only (ruff `--fix`) |

## 5. Acceptance criteria (ROADMAP P0)

| Criterion | Status | Evidence |
|---|---|---|
| `pytest` runs from a clean environment | **PASS** — fresh venv, `pip install -e ".[dev]"` succeeded, 20/20 tests passed | `reports/P1/test_run_output.txt` |
| Dependency lock/export exists | **PASS** | `requirements-lock.txt` |
| Godot project opens without errors | **PASS (headless)** — `--headless --path godot_app --quit` gives rc 0 with empty stderr. Not checked: opening the project in the GUI editor | `environment_audit.json → godot.headless_project_open` |
| Report includes tested versions | **PASS** | this file |

**P0 status: complete for the listed acceptance criteria.** Open items that do not block the gate:

1. No git repository (owner decision).
2. No CI pipeline. Lint, type-check and tests run locally only.
3. `python` on PATH points to the Store stub. Always use `.venv\Scripts\python.exe`.
