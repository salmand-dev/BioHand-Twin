"""Unit tests for biomechanics.frames (MuJoCo <-> Godot conversion)."""

import mujoco
import numpy as np
import pytest

from biomechanics.frames import (
    MJ_TO_GODOT,
    godot_to_mujoco_pos,
    godot_to_mujoco_quat,
    mujoco_to_godot_pos,
    mujoco_to_godot_quat,
    quat_multiply_wxyz,
    quat_to_matrix_wxyz,
)


def random_quats(n: int, seed: int = 0) -> np.ndarray:
    q = np.random.default_rng(seed).normal(size=(n, 4))
    return q / np.linalg.norm(q, axis=1, keepdims=True)


def xyzw_to_matrix(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    return quat_to_matrix_wxyz(np.array([w, x, y, z]))


def test_basis_is_proper_rotation():
    assert np.allclose(MJ_TO_GODOT @ MJ_TO_GODOT.T, np.eye(3))
    assert np.linalg.det(MJ_TO_GODOT) == pytest.approx(1.0)


@pytest.mark.parametrize(
    ("mj", "godot"),
    [((1, 0, 0), (1, 0, 0)), ((0, 1, 0), (0, 0, -1)), ((0, 0, 1), (0, 1, 0))],
)
def test_basis_vectors(mj, godot):
    assert np.allclose(mujoco_to_godot_pos(np.array(mj, float)), godot)


def test_up_axis_maps_to_up_axis():
    gravity_mj = np.array([0.0, 0.0, -9.81])
    assert np.allclose(mujoco_to_godot_pos(gravity_mj), [0.0, -9.81, 0.0])


def test_position_round_trip():
    p = np.random.default_rng(1).normal(size=(100, 3))
    for v in p:
        assert np.allclose(godot_to_mujoco_pos(mujoco_to_godot_pos(v)), v, atol=1e-12)


def test_quat_matrix_matches_mujoco():
    for q in random_quats(50):
        ref = np.zeros(9)
        mujoco.mju_quat2Mat(ref, q)
        assert np.allclose(quat_to_matrix_wxyz(q), ref.reshape(3, 3), atol=1e-12)


def test_quat_multiply_matches_mujoco():
    qs = random_quats(20, seed=3)
    for a, b in zip(qs[:10], qs[10:], strict=True):
        ref = np.zeros(4)
        mujoco.mju_mulQuat(ref, a, b)
        assert np.allclose(quat_multiply_wxyz(a, b), ref, atol=1e-12)


def test_quaternion_conversion_equals_basis_change_of_matrix():
    for q in random_quats(200, seed=2):
        r_mj = quat_to_matrix_wxyz(q)
        r_g = xyzw_to_matrix(mujoco_to_godot_quat(q))
        assert np.allclose(r_g, MJ_TO_GODOT @ r_mj @ MJ_TO_GODOT.T, atol=1e-12)


def test_quaternion_round_trip_and_order():
    for q in random_quats(200, seed=4):
        back = godot_to_mujoco_quat(mujoco_to_godot_quat(q))
        assert np.allclose(back, q, atol=1e-12) or np.allclose(back, -q, atol=1e-12)
    # identity: MuJoCo (1,0,0,0) -> Godot (0,0,0,1)
    assert np.allclose(mujoco_to_godot_quat(np.array([1.0, 0, 0, 0])), [0, 0, 0, 1])


def test_rotated_point_is_consistent_between_frames():
    rng = np.random.default_rng(5)
    for q in random_quats(50, seed=6):
        p = rng.normal(size=3)
        lhs = mujoco_to_godot_pos(quat_to_matrix_wxyz(q) @ p)
        rhs = xyzw_to_matrix(mujoco_to_godot_quat(q)) @ mujoco_to_godot_pos(p)
        assert np.allclose(lhs, rhs, atol=1e-12)


@pytest.mark.parametrize("bad", [np.array([1.0, 0, 0]), np.array([2.0, 0, 0, 0]),
                                 np.array([np.nan, 0, 0, 1.0])])
def test_rejects_invalid_quaternions(bad):
    with pytest.raises(ValueError):
        mujoco_to_godot_quat(bad)
