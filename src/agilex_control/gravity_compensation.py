"""Gravity compensation model using MuJoCo simulation.

Predicts the per-joint physical torque needed to counteract gravity at a given
configuration.
"""

import pathlib
from collections.abc import Sequence

import mujoco as mj
import numpy as np

# These are the joint names in the default MuJoCo model for the arm.
DEFAULT_JOINT_NAMES = (
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
)


class GravityCompensationModel:
  """Predicts gravity compensation torques using MuJoCo.

  The compiled model is read-only here and may be shared with other consumers;
  each instance owns its ``MjData``.
  """

  def __init__(
      self,
      model: mj.MjModel,
      joint_names: Sequence[str] = DEFAULT_JOINT_NAMES,
  ):
    self._model = model
    self._data = mj.MjData(model)
    self._joint_names = tuple(joint_names)

    joint_indices = [self._model.joint(name).id for name in self._joint_names]
    self.qpos_indices = self._model.jnt_qposadr[joint_indices]
    self.qvel_indices = self._model.jnt_dofadr[joint_indices]

  @classmethod
  def from_xml_path(
      cls,
      model_path: str | pathlib.Path,
      joint_names: Sequence[str] = DEFAULT_JOINT_NAMES,
  ) -> "GravityCompensationModel":
    """Compiles the model from an MJCF file."""
    return cls(mj.MjModel.from_xml_path(str(model_path)), joint_names)

  def _calculate_sim_tau(self, qpos):
    self._data.qpos[self.qpos_indices] = qpos
    mj.mj_forward(self._model, self._data)
    return self._data.qfrc_bias[self.qvel_indices]

  def predict(self, qpos) -> np.ndarray:
    """Return the physical gravity-comp torque (Nm) per joint at qpos."""
    return np.asarray(self._calculate_sim_tau(qpos))
