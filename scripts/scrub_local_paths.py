"""Replace absolute local paths in recorded evidence, so reports do not expose a user name when published.

Only the path prefix changes; the recorded results stay as they were. Usage: python scripts/scrub_local_paths.py
"""

from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PROJECT = re.compile(r"[A-Za-z]:[\\/]+Users[\\/]+[^\\/\s\"']+[\\/]+Desktop[\\/]+BioHand-Twin-Starter[\\/]+BioHand-Twin")
HOME = re.compile(r"[A-Za-z]:[\\/]+Users[\\/]+[^\\/\s\"']+")
SUFFIXES = {".txt", ".json", ".md", ".csv"}


def scrub(text: str) -> str:
    return HOME.sub("<HOME>", PROJECT.sub("<PROJECT>", text))


def main() -> None:
    changed = 0
    for path in (ROOT / "reports").rglob("*"):
        if path.suffix not in SUFFIXES or not path.is_file():
            continue
        text = path.read_text(encoding="utf-8", errors="surrogateescape")
        clean = scrub(text)
        if clean != text:
            path.write_text(clean, encoding="utf-8", errors="surrogateescape", newline="")
            changed += 1
    print(f"scrubbed {changed} files")


if __name__ == "__main__":
    main()
