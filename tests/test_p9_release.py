"""P9 tests: release documents exist and public text makes no unsupported claims."""

import re
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
PUBLIC = ["README.md", "docs/INSTALL_WINDOWS.md", "docs/DEMO_SCENARIOS.md", "docs/LIMITATIONS.md",
          "docs/RELEASE_NOTES.md", "godot_app/scripts/main.gd"]
# A risky phrase is allowed only in a sentence that also negates it.
RISKY = re.compile(r"clinically (validated|accurate)|patient-specific|medical device|research-grade|"
                   r"anatomically accurate", re.IGNORECASE)
NEGATION = re.compile(r"\bnot\b|\bno\b|without|never", re.IGNORECASE)


@pytest.mark.parametrize("name", PUBLIC)
def test_no_unsupported_claims(name):
    text = (ROOT / name).read_text(encoding="utf-8")
    for sentence in re.split(r"(?<=[.!?\n])\s+", text):
        if RISKY.search(sentence):
            assert NEGATION.search(sentence), f"{name}: {sentence.strip()}"


def test_readme_has_required_content():
    text = (ROOT / "README.md").read_text(encoding="utf-8")
    for needle in ("non-commercial", "Saul et al. 2015", "McFarland et al. 2019", "Architecture", "LIMITATIONS.md",
                   "93b0ca8f", "pinch", "INSTALL_WINDOWS.md"):
        assert needle in text, needle


def test_readme_links_resolve():
    text = (ROOT / "README.md").read_text(encoding="utf-8")
    for target in re.findall(r"\]\(((?!http)[^)#]+)\)", text):
        assert (ROOT / target).exists(), target


def test_citation_file_is_valid_yaml_with_required_citations():
    data = yaml.safe_load((ROOT / "CITATION.cff").read_text(encoding="utf-8"))
    dois = {r.get("doi") for r in data["references"]}
    assert {"10.1080/10255842.2014.916698", "10.1115/1.4043035"} <= dois


def test_nothing_licensed_is_trackable():
    ignore = (ROOT / ".gitignore").read_text(encoding="utf-8")
    for entry in ("third_party/", "build/", "godot_app/local_assets/"):
        assert entry in ignore
    binaries = [p for p in ROOT.rglob("*") if p.suffix.lower() in {".stl", ".glb", ".vtp", ".osim", ".onnx"}
                and not {"third_party", "build", "local_assets", ".venv", ".godot"} & set(p.parts)]
    assert binaries == []


def test_licence_texts_present():
    for name in ("MoBL-ARMS_use_agreement.txt", "2nd-Hand_MIT_license.txt"):
        assert (ROOT / "data" / "provenance" / "licenses" / name).is_file()
