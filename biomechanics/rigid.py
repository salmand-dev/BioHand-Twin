"""Rigid-body transform helpers (SI units, radians)."""

from __future__ import annotations

import numpy as np

from biomechanics.coordinates import validate_vector3
from biomechanics.frames import quat_to_matrix_wxyz


def axis_angle_matrix(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    """Rotation matrix for a right-handed rotation of ``angle_rad`` about ``axis`` (Rodrigues)."""
    a = validate_vector3(axis)
    n = np.linalg.norm(a)
    if n == 0:
        raise ValueError("Rotation axis must be non-zero")
    if not np.isfinite(angle_rad):
        raise ValueError("Angle must be finite")
    x, y, z = a / n
    k = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + np.sin(angle_rad) * k + (1 - np.cos(angle_rad)) * (k @ k)


def homogeneous(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    """4x4 homogeneous transform from a 3x3 rotation and a translation (m)."""
    t = np.eye(4)
    t[:3, :3] = rotation
    t[:3, 3] = validate_vector3(translation)
    return t


def pose_from_pos_quat(pos: np.ndarray, quat_wxyz: np.ndarray) -> np.ndarray:
    """4x4 transform from a position (m) and a (w, x, y, z) quaternion."""
    return homogeneous(quat_to_matrix_wxyz(quat_wxyz), pos)


def hinge_transform(anchor: np.ndarray, axis: np.ndarray, angle_rad: float) -> np.ndarray:
    """Transform of a rotation by ``angle_rad`` about ``axis`` through point ``anchor``."""
    r = axis_angle_matrix(axis, angle_rad)
    p = validate_vector3(anchor)
    return homogeneous(r, p - r @ p)
