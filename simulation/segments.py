"""Rigid (functional) segments derived from a compiled MuJoCo model.

A body without joints is welded to its parent, so it can never move relative
to it. Grouping every jointless body with its parent yields the units that can
actually move independently. These units -- not individual bones -- are what
the UI may pose, animate or describe as moving (see simulation.ui_contract).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import mujoco


@dataclass
class RigidSegment:
    """A set of bodies rigidly attached to one root body."""

    root_body: str
    bodies: list[str] = field(default_factory=list)
    mesh_geoms: list[str] = field(default_factory=list)
    joints: list[str] = field(default_factory=list)
    parent_segment: str | None = None

    @property
    def n_dof(self) -> int:
        return len(self.joints)

    @property
    def is_world_fixed(self) -> bool:
        return self.parent_segment is None


def _name(model: mujoco.MjModel, obj: mujoco.mjtObj, idx: int) -> str:
    name = mujoco.mj_id2name(model, obj, idx)
    return name if name else f"<unnamed {obj.name} {idx}>"


def segment_root_ids(model: mujoco.MjModel) -> list[int]:
    """Return, for each body id, the id of the root body of its rigid segment.

    Body 0 (world) is its own root. A body with at least one joint starts a new
    segment; a jointless body inherits its parent's root.
    """
    roots = [0] * model.nbody
    for b in range(1, model.nbody):  # MuJoCo guarantees parent id < child id
        roots[b] = b if model.body_jntnum[b] > 0 else roots[model.body_parentid[b]]
    return roots


def rigid_segments(model: mujoco.MjModel) -> list[RigidSegment]:
    """Group bodies into rigid segments, ordered by root body id."""
    roots = segment_root_ids(model)
    segments: dict[int, RigidSegment] = {}
    for b in range(model.nbody):
        r = roots[b]
        if r not in segments:
            parent = None if r == 0 else _name(
                model, mujoco.mjtObj.mjOBJ_BODY, roots[model.body_parentid[r]])
            segments[r] = RigidSegment(
                root_body=_name(model, mujoco.mjtObj.mjOBJ_BODY, r), parent_segment=parent)
        segments[r].bodies.append(_name(model, mujoco.mjtObj.mjOBJ_BODY, b))
    for j in range(model.njnt):
        segments[roots[model.jnt_bodyid[j]]].joints.append(
            _name(model, mujoco.mjtObj.mjOBJ_JOINT, j))
    for g in range(model.ngeom):
        if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            segments[roots[model.geom_bodyid[g]]].mesh_geoms.append(
                _name(model, mujoco.mjtObj.mjOBJ_GEOM, g))
    return [segments[r] for r in sorted(segments)]


def segment_of_body(model: mujoco.MjModel) -> dict[str, str]:
    """Map every body name to the root-body name of its rigid segment."""
    return {b: seg.root_body for seg in rigid_segments(model) for b in seg.bodies}
