"""Minimal smoke test: the gravity compensation model loads and predicts."""

import numpy as np

from agilex_control import gravity_compensation

# A tiny two-link pendulum, horizontal under gravity, so predict() returns a
# non-zero torque.
_TWO_LINK_MODEL = """
<mujoco>
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <body name="link1" pos="0 0 1">
      <joint name="joint1" type="hinge" axis="1 0 0"/>
      <geom type="capsule" fromto="0 0 0 0 0.2 0" size="0.02" mass="1"/>
      <body name="link2" pos="0 0.2 0">
        <joint name="joint2" type="hinge" axis="1 0 0"/>
        <geom type="capsule" fromto="0 0 0 0 0.2 0" size="0.02" mass="1"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


def test_predict_runs(tmp_path):
  model_path = tmp_path / "two_link.xml"
  model_path.write_text(_TWO_LINK_MODEL)

  model = gravity_compensation.GravityCompensationModel(
      model_path, joint_names=("joint1", "joint2")
  )
  tau = model.predict(np.zeros(2))

  assert tau.shape == (2,)
  assert np.all(np.isfinite(tau))
  assert np.any(tau != 0.0)  # gravity actually loads the joints
