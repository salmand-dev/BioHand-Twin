"""What the UI may present as independently movable (P2 WP7).

The UI must never pose, animate or describe a lumped bone (e.g. trapezoid) as
moving on its own. It consumes these motion units instead; selecting any bone
resolves to the unit that contains it.
"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco

from simulation.naming import Glossary
from simulation.segments import rigid_segments


@dataclass(frozen=True)
class MotionUnit:
    """A rigid segment as the UI should present it."""

    unit_id: str  # root body name; stable across sessions
    display_name: str
    member_bodies: tuple[str, ...]
    member_display_names: tuple[str, ...]
    joints: tuple[str, ...]
    movable: bool
    lumped: bool  # True if more than one anatomical bone moves as one
    note: str


def motion_units(model: mujoco.MjModel, glossary: Glossary) -> list[MotionUnit]:
    """Return one MotionUnit per rigid segment of ``model``."""
    units = []
    for seg in rigid_segments(model):
        bones = [b for b in seg.bodies
                 if glossary.entry(b).get("structure_type") == "bone"]
        names = tuple(glossary.canonical(b) or b for b in bones)
        lumped = len(bones) > 1
        root_name = glossary.canonical(seg.root_body) or seg.root_body
        if seg.is_world_fixed:
            display, note = "fixed proximal scaffold", "fixed to the world in this model; does not move"
        elif lumped:
            display = f"{root_name} segment ({len(bones)} bones moving as one)"
            note = "these bones are lumped in the reference model and cannot move relative to each other"
        else:
            display, note = root_name, ""
        units.append(MotionUnit(
            unit_id=seg.root_body, display_name=display, member_bodies=tuple(seg.bodies),
            member_display_names=names, joints=tuple(seg.joints),
            movable=not seg.is_world_fixed, lumped=lumped, note=note,
        ))
    return units


def unit_for_body(units: list[MotionUnit], body: str) -> MotionUnit:
    """The motion unit that contains ``body`` (what a click on that bone selects)."""
    for u in units:
        if body in u.member_bodies:
            return u
    raise KeyError(body)
