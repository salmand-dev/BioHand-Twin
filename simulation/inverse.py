"""Inverse dynamics via MuJoCo (P5).

``inverse_dynamics`` returns the generalized force that, applied together with the model's passive
forces and constraint (limit, contact, equality) forces, produces the given acceleration: MuJoCo's
``qfrc_inverse``. For a state produced by forward simulation it equals qfrc_actuator + qfrc_applied
(verified in tests/test_p5_tasks.py to machine precision).
"""

from __future__ import annotations

import mujoco
import numpy as np


def inverse_dynamics(model: mujoco.MjModel, qpos: np.ndarray, qvel: np.ndarray, qacc: np.ndarray,
                     act: np.ndarray | None = None) -> np.ndarray:
    """Generalized force (N*m for hinge DoFs) required for ``qacc`` at (qpos, qvel)."""
    d = mujoco.MjData(model)
    d.qpos[:], d.qvel[:], d.qacc[:] = qpos, qvel, qacc
    if act is not None and model.na:
        d.act[:] = act
    mujoco.mj_inverse(model, d)
    return d.qfrc_inverse.copy()
