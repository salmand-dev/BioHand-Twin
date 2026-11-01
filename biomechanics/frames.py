"""Coordinate-frame conversions between MuJoCo and Godot/glTF.

Conventions (see docs/COORDINATE_SYSTEM.md):

* MuJoCo world: right-handed, +Z up, metres. Quaternions ordered (w, x, y, z).
* Godot / glTF: right-handed, +Y up, metres. Godot quaternions ordered (x, y, z, w).

The basis change C maps MuJoCo coordinates to Godot coordinates:
``(x, y, z)_mj -> (x, z, -y)_godot``. C is a proper rotation (-90 deg about +X,
det = +1), so handedness is preserved. Rotations transform as R_g = C R_mj C^T.
This module is the only place in the project where this conversion may live.
"""

from __future__ import annotations

import numpy as np

from biomechanics.coordinates import validate_vector3

#: Basis change from MuJoCo (Z-up) to Godot/glTF (Y-up) coordinates.
MJ_TO_GODOT = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]])
#: Quaternion (w, x, y, z) of MJ_TO_GODOT: rotation of -90 deg about +X.
_C_QUAT_WXYZ = np.array([np.sqrt(0.5), -np.sqrt(0.5), 0.0, 0.0])


def _validate_quat_wxyz(q: np.ndarray) -> np.ndarray:
    arr = np.asarray(q, dtype=np.float64)
    if arr.shape != (4,) or not np.isfinite(arr).all():
        raise ValueError("Expected a finite quaternion of shape (4,)")
    norm = np.linalg.norm(arr)
    if not np.isclose(norm, 1.0, atol=1e-6):
        raise ValueError(f"Quaternion is not unit length (|q| = {norm})")
    return arr / norm


def quat_multiply_wxyz(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product a*b of (w, x, y, z) quaternions."""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_to_matrix_wxyz(q: np.ndarray) -> np.ndarray:
    """Rotation matrix of a unit (w, x, y, z) quaternion."""
    w, x, y, z = _validate_quat_wxyz(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def mujoco_to_godot_pos(p_mj: np.ndarray) -> np.ndarray:
    """Convert a MuJoCo position (m) to Godot coordinates (m)."""
    return MJ_TO_GODOT @ validate_vector3(p_mj)


def godot_to_mujoco_pos(p_g: np.ndarray) -> np.ndarray:
    """Convert a Godot position (m) to MuJoCo coordinates (m)."""
    return MJ_TO_GODOT.T @ validate_vector3(p_g)


def mujoco_to_godot_quat(q_mj_wxyz: np.ndarray) -> np.ndarray:
    """Convert a MuJoCo orientation (w, x, y, z) to a Godot quaternion (x, y, z, w).

    Implements R_g = C R_mj C^T as q_g = q_C * q_mj * conj(q_C).
    """
    q = _validate_quat_wxyz(q_mj_wxyz)
    c_conj = _C_QUAT_WXYZ * np.array([1.0, -1.0, -1.0, -1.0])
    w, x, y, z = quat_multiply_wxyz(quat_multiply_wxyz(_C_QUAT_WXYZ, q), c_conj)
    out = np.array([x, y, z, w])
    return out if out[3] >= 0 else -out  # canonical sign (q and -q are the same rotation)


def godot_to_mujoco_quat(q_g_xyzw: np.ndarray) -> np.ndarray:
    """Inverse of mujoco_to_godot_quat: Godot (x, y, z, w) -> MuJoCo (w, x, y, z)."""
    x, y, z, w = np.asarray(q_g_xyzw, dtype=np.float64)
    q = _validate_quat_wxyz(np.array([w, x, y, z]))
    c_conj = _C_QUAT_WXYZ * np.array([1.0, -1.0, -1.0, -1.0])
    out = quat_multiply_wxyz(quat_multiply_wxyz(c_conj, q), _C_QUAT_WXYZ)
    return out if out[0] >= 0 else -out
