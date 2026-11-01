"""Coordinate helpers. SI units and radians are used internally."""
from __future__ import annotations

import numpy as np


def validate_vector3(value: np.ndarray) -> np.ndarray:
    """Return a finite float64 3-vector or raise ValueError."""
    arr = np.asarray(value, dtype=np.float64)
    if arr.shape != (3,) or not np.isfinite(arr).all():
        raise ValueError("Expected a finite 3D vector")
    return arr
