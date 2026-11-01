"""Forward-kinematics inspection for the reference model (P3).

All angles are radians. Degrees exist only in ``to_display_degrees`` for UI/report output.
Kinematic queries never step the dynamics: they set qpos and call ``mj_kinematics`` /
``mj_comPos`` only, so they cannot violate or soften joint limits.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

import mujoco
import numpy as np

#: Fingertip landmark sites present in the upstream model (on the distal phalanges).
FINGERTIP_SITES: dict[str, str] = {
    "thumb": "THtip_r", "index": "IFtip_r", "middle": "MFtip_r", "ring": "RFtip_r", "little": "LFtip_r",
}


class JointLimitError(ValueError):
    """Raised when a requested joint angle lies outside the model-defined range."""


def to_display_degrees(angle_rad: float | np.ndarray) -> float | np.ndarray:
    """Convert radians to degrees for display only."""
    return np.degrees(angle_rad)


def joint_id(model: mujoco.MjModel, name: str) -> int:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if jid < 0:
        raise KeyError(f"unknown joint {name!r}")
    return jid


def qpos_from_angles(model: mujoco.MjModel, angles_rad: Mapping[str, float],
                     base: np.ndarray | None = None, tol_rad: float = 0.0) -> np.ndarray:
    """Build a qpos vector from named hinge angles (rad), validating model ranges.

    Joints not named keep their value in ``base`` (default: model.qpos0).
    Raises JointLimitError for any angle outside [range_lo - tol, range_hi + tol].
    """
    qpos = (model.qpos0 if base is None else np.asarray(base, dtype=float)).copy()
    for name, value in angles_rad.items():
        j = joint_id(model, name)
        if model.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE:
            raise ValueError(f"{name} is not a hinge joint")
        lo, hi = model.jnt_range[j]
        if not np.isfinite(value) or (model.jnt_limited[j] and not lo - tol_rad <= value <= hi + tol_rad):
            raise JointLimitError(f"{name}={value!r} rad outside [{lo}, {hi}]")
        qpos[model.jnt_qposadr[j]] = value
    return qpos


def forward(model: mujoco.MjModel, qpos: np.ndarray, data: mujoco.MjData | None = None) -> mujoco.MjData:
    """Pure kinematics at ``qpos`` (positions, orientations, sites, tendon lengths)."""
    d = mujoco.MjData(model) if data is None else data
    d.qpos[:] = qpos
    mujoco.mj_kinematics(model, d)
    mujoco.mj_comPos(model, d)
    mujoco.mj_tendon(model, d)
    return d


def site_positions(model: mujoco.MjModel, data: mujoco.MjData, sites: Mapping[str, str]) -> dict[str, np.ndarray]:
    out = {}
    for key, name in sites.items():
        sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
        if sid < 0:
            raise KeyError(f"unknown site {name!r}")
        out[key] = data.site_xpos[sid].copy()
    return out


def site_jacobian(model: mujoco.MjModel, data: mujoco.MjData, site: str) -> np.ndarray:
    """Translational Jacobian d(site position)/d(qvel), shape (3, nv), m/rad."""
    jacp = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, jacp, None, mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site))
    return jacp


@dataclass
class SweepResult:
    joint: str
    angles_rad: np.ndarray            # (n,)
    tips_m: dict[str, np.ndarray]     # key -> (n, 3) world positions

    def max_step_m(self, key: str) -> float:
        return float(np.max(np.linalg.norm(np.diff(self.tips_m[key], axis=0), axis=1)))


def sweep_joint(model: mujoco.MjModel, joint: str, n: int = 181, base: np.ndarray | None = None,
                sites: Mapping[str, str] = FINGERTIP_SITES) -> SweepResult:
    """Kinematic sweep of one joint over its full model range (other joints at ``base``)."""
    j = joint_id(model, joint)
    lo, hi = model.jnt_range[j]
    angles = np.linspace(lo, hi, n)
    data = mujoco.MjData(model)
    tips = {k: np.zeros((n, 3)) for k in sites}
    for i, a in enumerate(angles):
        forward(model, qpos_from_angles(model, {joint: float(a)}, base), data)
        for k, p in site_positions(model, data, sites).items():
            tips[k][i] = p
    return SweepResult(joint=joint, angles_rad=angles, tips_m=tips)
