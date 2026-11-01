"""Biomechanics equations and analysis helpers."""
from __future__ import annotations

import numpy as np


def generalized_muscle_torque(moment_arm_matrix: np.ndarray, muscle_forces: np.ndarray) -> np.ndarray:
    """Compute tau = R @ F after validating compatible dimensions.

    Convention: R has shape (n_dof, n_muscles), in metres; F is in Newtons;
    output generalized torque is in N*m. Sign convention must match the model.
    """
    r = np.asarray(moment_arm_matrix, dtype=float)
    f = np.asarray(muscle_forces, dtype=float)
    if r.ndim != 2 or f.ndim != 1 or r.shape[1] != f.shape[0]:
        raise ValueError("Incompatible moment-arm matrix and force vector shapes")
    if not np.isfinite(r).all() or not np.isfinite(f).all():
        raise ValueError("Inputs must be finite")
    return r @ f
