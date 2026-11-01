"""Muscle-tendon inspection using MuJoCo's own muscle semantics (P4).

Conventions
-----------
* MuJoCo's actuator moment is ``gear * dL/dq`` and a muscle's actuator force is
  negative (tension pulls). BioHand reports the biomechanical moment arm
  ``r = -dL/dq`` (m/rad): r > 0 means that muscle tension drives the joint in
  its positive direction. Joint torque from tension T >= 0 is ``r * T``.
* Muscle tension is reported as ``T = -actuator_force`` (N, >= 0 for muscles).
* Active/passive decomposition calls MuJoCo's ``mju_muscleGain`` and
  ``mju_muscleBias``; no second Hill model is implemented here.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass

import mujoco
import numpy as np


def moment_arm_matrix(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    """Biomechanical moment arms r = -dL/dq, shape (nu, nv), m/rad (requires mj_forward)."""
    dense = np.zeros((model.nu, model.nv))
    mujoco.mju_sparse2dense(dense, data.actuator_moment, data.moment_rownnz,
                            data.moment_rowadr, data.moment_colind)
    return -dense / model.actuator_gear[:, :1]


def normalisation(model: mujoco.MjModel) -> tuple[np.ndarray, np.ndarray]:
    """MuJoCo muscle length normalisation constants (L0, LT) per actuator, in m."""
    gp, lr = model.actuator_gainprm, model.actuator_lengthrange
    l0 = (lr[:, 1] - lr[:, 0]) / (gp[:, 1] - gp[:, 0])
    return l0, lr[:, 0] - gp[:, 0] * l0


@dataclass
class MuscleState:
    activation: np.ndarray            # (nu,) dimensionless, 0..1
    control: np.ndarray               # (nu,)
    length_m: np.ndarray              # actuator (tendon) length
    velocity_m_s: np.ndarray
    normalised_length: np.ndarray     # (L - LT) / L0
    tension_N: np.ndarray             # -actuator_force
    active_tension_N: np.ndarray      # a * gain (from mju_muscleGain), >= 0
    passive_tension_N: np.ndarray     # from mju_muscleBias, >= 0
    in_active_window: np.ndarray      # lmin <= normalised_length <= lmax


def muscle_state(model: mujoco.MjModel, data: mujoco.MjData) -> MuscleState:
    """Muscle state after mj_forward/mj_step. Active + passive reproduces MuJoCo's force."""
    l0, lt = normalisation(model)
    act = data.act[model.actuator_actadr] if model.na else np.zeros(model.nu)
    gain = np.zeros(model.nu)
    bias = np.zeros(model.nu)
    for a in range(model.nu):
        lr = model.actuator_lengthrange[a]
        gain[a] = mujoco.mju_muscleGain(data.actuator_length[a], data.actuator_velocity[a], lr,
                                        model.actuator_acc0[a], model.actuator_gainprm[a, :9])
        bias[a] = mujoco.mju_muscleBias(data.actuator_length[a], lr,
                                        model.actuator_acc0[a], model.actuator_biasprm[a, :9])
    lnorm = (data.actuator_length - lt) / l0
    return MuscleState(
        activation=act.copy(), control=data.ctrl.copy(),
        length_m=data.actuator_length.copy(), velocity_m_s=data.actuator_velocity.copy(),
        normalised_length=lnorm, tension_N=-data.actuator_force.copy(),
        active_tension_N=-gain * act, passive_tension_N=-bias,
        in_active_window=(lnorm >= model.actuator_gainprm[:, 4]) & (lnorm <= model.actuator_gainprm[:, 5]),
    )


def without_actuator(model: mujoco.MjModel, actuator: str) -> mujoco.MjModel:
    """Copy of ``model`` in which ``actuator`` produces no force at all (active or passive).

    Implemented by zeroing the actuator's gear on a copy, so its transmission and force
    vanish; all other elements are untouched. The original model is not modified.
    """
    a = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator)
    if a < 0:
        raise KeyError(actuator)
    ablated = copy.copy(model)
    ablated.actuator_gear[a, :] = 0.0
    return ablated
