"""Static optimization of muscle activations (P5).

At a given state, MuJoCo muscle actuator force is affine in activation:
    f_a = gain_a(L, V) * act_a + bias_a(L)        (mju_muscleGain / mju_muscleBias; both <= 0)
and the generalized actuator force is qfrc = Mᵀ f with M the dense actuator moment (gear*dL/dq).
Given a desired generalized force tau, solve
    min_a  || W (Mᵀ (g ⊙ a + b) - tau) ||² + lam ||a||²     s.t. 0 <= a <= 1
(bounded linear least squares). Activation dynamics are not inverted: the solution is used as the
control signal and MuJoCo's activation dynamics (tau_act 10 ms) apply.
"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np
from scipy.optimize import lsq_linear


@dataclass
class StaticOptResult:
    activation: np.ndarray       # (nu,) in [0, 1]
    achieved_qfrc: np.ndarray    # (nv,) N*m
    residual_norm: float         # weighted, N*m
    success: bool


def muscle_affine_terms(model: mujoco.MjModel, data: mujoco.MjData) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Dense actuator moment M (nu x nv) and per-actuator gain g, bias b at the current state."""
    moment = np.zeros((model.nu, model.nv))
    mujoco.mju_sparse2dense(moment, data.actuator_moment, data.moment_rownnz, data.moment_rowadr, data.moment_colind)
    g = np.zeros(model.nu)
    b = np.zeros(model.nu)
    for a in range(model.nu):
        lr = model.actuator_lengthrange[a]
        g[a] = mujoco.mju_muscleGain(data.actuator_length[a], data.actuator_velocity[a], lr,
                                     model.actuator_acc0[a], model.actuator_gainprm[a, :9])
        b[a] = mujoco.mju_muscleBias(data.actuator_length[a], lr, model.actuator_acc0[a], model.actuator_biasprm[a, :9])
    return moment, g, b


def solve_activations(model: mujoco.MjModel, data: mujoco.MjData, tau_des: np.ndarray,
                      dof_weights: np.ndarray | None = None, lam: float = 1e-3) -> StaticOptResult:
    """Bounded least-squares activations reproducing ``tau_des`` (requires mj_forward on data)."""
    moment, g, b = muscle_affine_terms(model, data)
    w = np.ones(model.nv) if dof_weights is None else np.asarray(dof_weights, dtype=float)
    a_mat = (moment.T * g) * w[:, None]                  # (nv, nu)
    rhs = w * (tau_des - moment.T @ b)
    a_aug = np.vstack([a_mat, np.sqrt(lam) * np.eye(model.nu)])
    rhs_aug = np.concatenate([rhs, np.zeros(model.nu)])
    sol = lsq_linear(a_aug, rhs_aug, bounds=(0.0, 1.0), method="bvls")
    act = np.clip(sol.x, 0.0, 1.0)
    achieved = moment.T @ (g * act + b)
    return StaticOptResult(activation=act, achieved_qfrc=achieved,
                           residual_norm=float(np.linalg.norm(w * (achieved - tau_des))), success=bool(sol.success))
