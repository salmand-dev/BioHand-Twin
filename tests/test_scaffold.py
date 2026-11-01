import numpy as np
import pytest

from biomechanics.coordinates import validate_vector3
from biomechanics.dynamics import generalized_muscle_torque


def test_validate_vector3():
    assert np.allclose(validate_vector3([1, 2, 3]), [1, 2, 3])

def test_validate_vector3_rejects_bad_shape():
    with pytest.raises(ValueError):
        validate_vector3([1, 2])

def test_generalized_muscle_torque():
    r = np.array([[0.02, -0.01], [0.03, 0.04]])
    f = np.array([10.0, 5.0])
    assert np.allclose(generalized_muscle_torque(r, f), [0.15, 0.50])
