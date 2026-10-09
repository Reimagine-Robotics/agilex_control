"""Arm interface that reads and commands joints in a calibrated frame.

An AgileX arm's joint readings can shift at power-on or on a reset. Joint
zeroing measures a calibration offset per joint and sets it here, so everything
above the interface (controllers, gravity compensation, clients) works in the
true joint frame.
"""

from collections.abc import Sequence

import numpy as np

from agilex_control import agilex_interface


class CalibratedArmInterface(agilex_interface.ArmInterface):
  """ArmInterface with software calibration offsets on every joint.

  A calibration offset is the raw reading of a joint at its true 0 rad, so
  calibrated = raw - offset. Until offsets are set the interface reports raw
  angles and ``is_calibrated`` is False. ``raw=True`` bypasses the offsets.

  The offsets live only in this object; nothing is written to the arm. They are
  valid only until the arm's next power-on or reset, so callers clear them
  whenever the arm leaves the enabled state.
  """

  def __init__(self, *args, **kwargs) -> None:
    super().__init__(*args, **kwargs)
    self._calibration_offsets: np.ndarray | None = None

  @property
  def is_calibrated(self) -> bool:
    """Whether calibration offsets have been set."""
    return self._calibration_offsets is not None

  @property
  def calibration_offsets(self) -> list[float] | None:
    """The per-joint offsets in radians, or None when uncalibrated."""
    if self._calibration_offsets is None:
      return None
    return self._calibration_offsets.tolist()

  def set_calibration_offsets(self, offsets: Sequence[float]) -> None:
    """Sets the per-joint offsets (raw reading at each joint's true 0 rad).

    Args:
      offsets: One offset in radians per joint; length must match the arm.

    Raises:
      ValueError: If the number of offsets does not match the joint count.
    """
    num_joints = self.get_num_joints()
    if len(offsets) != num_joints:
      raise ValueError(f"Expected {num_joints} offsets, got {offsets!r}")
    self._calibration_offsets = np.asarray(offsets, dtype=float)

  def clear_calibration_offsets(self) -> None:
    """Drops the offsets, so the interface reports the raw joint frame again."""
    self._calibration_offsets = None

  def get_joint_positions(self, raw: bool = False) -> list[float]:
    positions = super().get_joint_positions()
    if raw or self._calibration_offsets is None:
      return positions
    return (np.asarray(positions) - self._calibration_offsets).tolist()

  def command_joint_position_mit(
      self,
      joint_index: int,
      *,
      position: float,
      kp: float,
      kd: float,
      torque_ff: float = 0.0,
      velocity: float = 0.0,
      raw: bool = False,
  ) -> None:
    if not raw and self._calibration_offsets is not None:
      position += float(self._calibration_offsets[joint_index])
    super().command_joint_position_mit(
        joint_index,
        position=position,
        kp=kp,
        kd=kd,
        torque_ff=torque_ff,
        velocity=velocity,
    )
