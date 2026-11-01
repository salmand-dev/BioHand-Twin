"""Canonical anatomical naming backed by data/anatomy/glossary.yaml."""

from __future__ import annotations

from dataclasses import dataclass
from functools import cached_property
from pathlib import Path
from typing import Any

import yaml

GLOSSARY_PATH = Path(__file__).resolve().parents[1] / "data" / "anatomy" / "glossary.yaml"
SECTIONS = ("bodies", "joints", "actuators")
NAMED_STATUSES = {"verified", "provisional", "unresolved"}


class GlossaryError(ValueError):
    """Raised when the glossary violates its schema."""


@dataclass(frozen=True)
class Glossary:
    """Validated glossary with lookups by model name and alias."""

    data: dict[str, Any]

    @classmethod
    def load(cls, path: Path = GLOSSARY_PATH) -> Glossary:
        glossary = cls(yaml.safe_load(path.read_text(encoding="utf-8")))
        glossary.validate()
        return glossary

    def entries(self, section: str) -> list[dict[str, Any]]:
        return list(self.data.get(section) or [])

    @cached_property
    def _by_model_name(self) -> dict[str, dict[str, Any]]:
        return {e["model_name"]: e for s in SECTIONS for e in self.entries(s)}

    def entry(self, model_name: str) -> dict[str, Any]:
        try:
            return self._by_model_name[model_name]
        except KeyError as exc:
            raise KeyError(f"{model_name!r} is not in the glossary") from exc

    def canonical(self, model_name: str) -> str | None:
        """Canonical name of a model element (None for non-anatomical elements)."""
        return self.entry(model_name).get("canonical_name")

    def resolve(self, name: str) -> str:
        """Return the model name for a model name or alias."""
        if name in self._by_model_name:
            return name
        hits = [e["model_name"] for e in self._by_model_name.values() if name in (e.get("aliases") or [])]
        if len(hits) != 1:
            raise KeyError(f"alias {name!r} matches {len(hits)} entries")
        return hits[0]

    def validate(self) -> None:
        seen_model: set[str] = set()
        seen_alias: dict[str, str] = {}
        canon: dict[tuple[str, str], str] = {}
        valid_status = set(self.data.get("status_values", {}))
        for section in SECTIONS:
            for e in self.entries(section):
                name = e.get("model_name")
                if not name or name in seen_model:
                    raise GlossaryError(f"missing or duplicate model_name {name!r}")
                seen_model.add(name)
                if e.get("status") not in valid_status:
                    raise GlossaryError(f"{name}: unknown status {e.get('status')!r}")
                c = e.get("canonical_name")
                if c is not None:
                    key = (section, c)
                    if key in canon:
                        raise GlossaryError(f"duplicate canonical name {c!r} ({canon[key]}, {name})")
                    canon[key] = name
                if e.get("status") in NAMED_STATUSES and not e.get("name_source") and section != "joints":
                    raise GlossaryError(f"{name}: status {e['status']} requires name_source")
                for a in e.get("aliases") or []:
                    if a in seen_alias and seen_alias[a] != name:
                        raise GlossaryError(f"alias {a!r} used by {seen_alias[a]} and {name}")
                    seen_alias[a] = name
        clash = seen_model & set(seen_alias)
        if clash:
            raise GlossaryError(f"aliases collide with model names: {sorted(clash)}")
