"""P0 environment audit: record tool versions and hardware as JSON evidence.

Tool locations default to the paths on the development machine and can be
overridden with BIOHAND_GODOT, BIOHAND_BLENDER and BIOHAND_MUJOCO_DIR.
A tool that cannot be found or run is reported as such, never assumed present.

Usage:
  .venv/Scripts/python.exe scripts/p0_environment_audit.py
"""

from __future__ import annotations

import datetime as dt
import importlib.metadata as md
import json
import os
import platform
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
DOWNLOADS = Path.home() / "Downloads"
GODOT = Path(os.environ.get(
    "BIOHAND_GODOT",
    DOWNLOADS / "Godot_v4.7.2-stable_win64.exe" / "Godot_v4.7.2-stable_win64_console.exe",
))
BLENDER = Path(os.environ.get(
    "BIOHAND_BLENDER", r"C:\Program Files\Blender Foundation\Blender 3.6\blender.exe"
))
MUJOCO_DIR = Path(os.environ.get("BIOHAND_MUJOCO_DIR", DOWNLOADS / "mujoco-3.17.0-windows-x86_64"))


def run(cmd: list[str], timeout: float = 120.0, cwd: Path | None = None) -> dict[str, Any]:
    try:
        r = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout, cwd=cwd, check=False
        )
        return {"cmd": cmd, "returncode": r.returncode,
                "stdout": r.stdout.strip(), "stderr": r.stderr.strip()}
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"cmd": cmd, "returncode": None, "error": repr(exc)}


def powershell(expr: str) -> Any:
    r = run(["powershell", "-NoProfile", "-Command", f"{expr} | ConvertTo-Json -Compress"])
    try:
        return json.loads(r["stdout"])
    except (KeyError, json.JSONDecodeError):
        return {"error": r}


def packages() -> dict[str, str]:
    return {d.metadata["Name"]: d.version for d in md.distributions()}


def main() -> None:
    import mujoco

    report: dict[str, Any] = {
        "generated_utc": dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds"),
        "generator": "scripts/p0_environment_audit.py",
        "os": {"platform": platform.platform(), "version": platform.version(),
               "machine": platform.machine()},
        "python": {"version": platform.python_version(), "executable": sys.executable,
                   "in_venv": sys.prefix != sys.base_prefix},
        "python_packages": dict(sorted(packages().items(), key=lambda kv: kv[0].lower())),
        "mujoco_python": {"package": mujoco.__version__, "library": mujoco.mj_versionString()},
        "git": run(["git", "--version"]),
        "project_git_status": run(["git", "status", "--short"], cwd=ROOT),
    }

    dll = MUJOCO_DIR / "bin" / "mujoco.dll"
    report["mujoco_standalone"] = {
        "dir": str(MUJOCO_DIR), "exists": MUJOCO_DIR.is_dir(),
        "mujoco_dll_exists": dll.is_file(),
        "dll_file_version": powershell(f"(Get-Item '{dll}').VersionInfo.FileVersion")
        if dll.is_file() else None,
        "license_first_line": (MUJOCO_DIR / "LICENSE").read_text().strip().splitlines()[0]
        if (MUJOCO_DIR / "LICENSE").is_file() else None,
    }

    godot: dict[str, Any] = {"path": str(GODOT), "exists": GODOT.is_file()}
    if GODOT.is_file():
        godot["version"] = run([str(GODOT), "--version"])
        godot["headless_project_open"] = run(
            [str(GODOT), "--headless", "--path", str(ROOT / "godot_app"), "--quit"], timeout=300
        )
    report["godot"] = godot

    blender: dict[str, Any] = {"path": str(BLENDER), "exists": BLENDER.is_file()}
    if BLENDER.is_file():
        blender["version"] = run([str(BLENDER), "--version"])
    report["blender"] = blender

    report["hardware"] = {
        "computer": powershell(
            "Get-CimInstance Win32_ComputerSystem | "
            "Select-Object TotalPhysicalMemory,NumberOfLogicalProcessors"),
        "cpu": powershell("Get-CimInstance Win32_Processor | Select-Object Name"),
        "gpu": powershell(
            "Get-CimInstance Win32_VideoController | Select-Object Name,DriverVersion"),
        "disk_c": dict(zip(("total", "used", "free"), shutil.disk_usage("C:\\"), strict=True)),
    }

    out = ROOT / "reports" / "P0"
    out.mkdir(parents=True, exist_ok=True)
    (out / "environment_audit.json").write_text(json.dumps(report, indent=1))

    def first_line(r: dict[str, Any] | None) -> str:
        if not r:
            return "NOT RUN"
        text = (r.get("stdout") or r.get("error") or "").splitlines()
        return text[0] if text else f"(no output, rc={r.get('returncode')})"

    print("OS:", report["os"]["platform"])
    print("Python:", report["python"]["version"], "venv:", report["python"]["in_venv"])
    print("MuJoCo (python):", report["mujoco_python"])
    print("MuJoCo standalone:", report["mujoco_standalone"]["dll_file_version"],
          "exists:", report["mujoco_standalone"]["exists"])
    print("Godot:", first_line(godot.get("version")) if godot["exists"] else "NOT FOUND")
    if godot.get("headless_project_open"):
        h = godot["headless_project_open"]
        print("Godot headless open rc:", h.get("returncode"), "stderr:", h.get("stderr") or "<empty>")
    print("Blender:", first_line(blender.get("version")) if blender["exists"] else "NOT FOUND")
    print("Git:", first_line(report["git"]))
    print("Project git status:", first_line(report["project_git_status"]),
          "rc:", report["project_git_status"].get("returncode"))
    print("Hardware:", json.dumps(report["hardware"]))


if __name__ == "__main__":
    main()
