"""Unit tests for simulation.inventory on small analytic MJCF models."""

import mujoco
import numpy as np
import pytest

from simulation.inventory import build_inventory, counts, tendon_length_jacobian

# Fixed tendon L = 0.02*q0 - 0.03*q1, so dL/dq = [0.02, -0.03] exactly.
FIXED_TENDON_XML = """
<mujoco>
  <worldbody>
    <body name="a"><joint name="j0" type="hinge" axis="0 0 1" range="-1 1"/>
      <geom type="sphere" size="0.01" mass="0.1"/>
      <body name="b" pos="0.1 0 0"><joint name="j1" type="hinge" axis="0 0 1" range="-1 1"/>
        <geom type="sphere" size="0.01" mass="0.1"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <fixed name="t"><joint joint="j0" coef="0.02"/><joint joint="j1" coef="-0.03"/></fixed>
  </tendon>
</mujoco>
"""

# Spatial tendon from a world site at (0, r, 0) to a site at (L, 0, 0) on a link that
# rotates about z at the origin. Length l(q) = |p(q) - s|, p(q) = L*(cos q, sin q, 0).
# Analytic dl/dq = -L*r*cos(q) / l(q).
R_OFFSET, LINK = 0.03, 0.2
SPATIAL_XML = f"""
<mujoco>
  <worldbody>
    <site name="origin" pos="0 {R_OFFSET} 0"/>
    <body name="link"><joint name="hinge" type="hinge" axis="0 0 1" range="-1 1"/>
      <geom type="capsule" fromto="0 0 0 {LINK} 0 0" size="0.005" mass="0.1"/>
      <site name="insertion" pos="{LINK} 0 0"/>
    </body>
  </worldbody>
  <tendon><spatial name="s"><site site="origin"/><site site="insertion"/></spatial></tendon>
  <actuator>
    <muscle name="m" tendon="s" lengthrange="0.15 0.25" range="0.75 1.05" force="100"/>
  </actuator>
</mujoco>
"""


def test_fixed_tendon_jacobian_matches_coefficients():
    model = mujoco.MjModel.from_xml_string(FIXED_TENDON_XML)
    jac = tendon_length_jacobian(model, np.array([0.3, -0.2]))
    assert np.allclose(jac, [[0.02, -0.03]], atol=1e-9)


@pytest.mark.parametrize("q", [-0.5, 0.0, 0.4])
def test_spatial_tendon_jacobian_matches_analytic(q: float):
    model = mujoco.MjModel.from_xml_string(SPATIAL_XML)
    jac = tendon_length_jacobian(model, np.array([q]))
    p = LINK * np.array([np.cos(q), np.sin(q)])
    length = np.linalg.norm(p - np.array([0.0, R_OFFSET]))
    expected = -LINK * R_OFFSET * np.cos(q) / length
    assert jac[0, 0] == pytest.approx(expected, rel=1e-6)


def test_jacobian_rejects_free_joint():
    model = mujoco.MjModel.from_xml_string(
        "<mujoco><worldbody><body><freejoint/><geom size='0.1'/></body></worldbody></mujoco>"
    )
    with pytest.raises(ValueError):
        tendon_length_jacobian(model, model.qpos0)


def test_inventory_reports_muscle_l0_formula():
    model = mujoco.MjModel.from_xml_string(SPATIAL_XML)
    inv = build_inventory(model)
    act = inv["actuators"][0]
    assert act["gaintype"] == "muscle"
    assert act["mujoco_muscle_params"]["peak_active_force_N"] == pytest.approx(100.0)
    # L0 = (0.25 - 0.15) / (1.05 - 0.75)
    assert act["derived_L0_equivalent_m"]["value"] == pytest.approx(0.1 / 0.3)
    assert inv["tendons"][0]["kind"] == "spatial"
    assert inv["tendons"][0]["first_site_body"] == "world"
    assert inv["tendons"][0]["last_site_body"] == "link"


def test_inventory_counts_and_welded_flag():
    model = mujoco.MjModel.from_xml_string(FIXED_TENDON_XML)
    c = counts(model)
    assert (c["nq"], c["njnt"], c["ntendon"], c["nu"]) == (2, 2, 1, 0)
    bodies = {b["name"]: b for b in build_inventory(model)["bodies"]}
    assert bodies["a"]["is_welded_to_parent"] is False
    assert bodies["a"]["joints"] == ["j0"]
