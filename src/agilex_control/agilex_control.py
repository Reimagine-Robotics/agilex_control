"""High-level controllers for AgileX arms.

These controllers are provided as a convenient high-level way of controlling the
AgileX robotic arms. They provide two main benefits:
- a simplified usage interface that hides much of the underlying pyAgxArm
  complexity,
- a context manager (or an explicit stop()) that parks the arm in a safe rest
  position on exit.
"""

from __future__ import annotations

import dataclasses
import time
from collections.abc import Sequence
from typing import TYPE_CHECKING

import numpy as np

from agilex_control import agilex_interface

if TYPE_CHECKING:
  from typing import Self


@dataclasses.dataclass(frozen=True)
class ArmOrientation:
  """Represents an arm mounting orientation with associated data."""

  name: str
  rest_position: tuple[float, ...]  # Joint angles in radians
  mounting_quaternion: tuple[float, float, float, float]  # [w, i, j, k]


# TODO: Need to add 7 joint versions of these for Nero.
@dataclasses.dataclass(frozen=True)
class ArmOrientations:
  """Registry of standard arm orientations.

  Coordinate system: Default upright arm has +x forward, +y left, +z up.
  """

  upright: ArmOrientation = ArmOrientation(
      name="upright",
      rest_position=(0.0, 0.0, 0.0, 0.02, 0.5, 0.0),
      mounting_quaternion=(1.0, 0.0, 0.0, 0.0),  # Identity - no rotation
  )

  left: ArmOrientation = ArmOrientation(
      name="left",
      rest_position=(1.71, 2.96, -2.65, 1.41, -0.081, -0.190),
      mounting_quaternion=(0.7071068, -0.7071068, 0.0, 0.0),  # -90 deg around X
  )

  right: ArmOrientation = ArmOrientation(
      name="right",
      rest_position=(-1.66, 2.91, -2.74, 0.0545, -0.271, 0.0979),
      mounting_quaternion=(0.7071068, 0.7071068, 0.0, 0.0),  # +90 deg around X
  )

  @classmethod
  def from_string(cls, orientation_name: str) -> ArmOrientation:
    """Get ArmOrientation instance from string name.

    Args:
      orientation_name: Name of the orientation ('upright', 'left', 'right').

    Returns:
      ArmOrientation instance.

    Raises:
      ValueError: If orientation_name is not recognized.
    """
    orientation_name = orientation_name.lower()
    for orientation in cls.__dict__.values():
      if (
          isinstance(orientation, ArmOrientation)
          and orientation.name.lower() == orientation_name
      ):
        return orientation

    available = [
        o.name for o in cls.__dict__.values() if isinstance(o, ArmOrientation)
    ]
    raise ValueError(
        f"Unknown arm orientation: {orientation_name}. Available: {available}"
    )


# Control rate in Hz. Frequency at which to send joint commands to the robot.
_CONTROL_RATE = 200.0

# Allowed gain ranges for the MIT controller, matching what pyAgxArm's move_mit
# accepts (it raises outside these), so we fail early with a clear message.
_MIN_KP_GAIN = 0.0
_MAX_KP_GAIN = 500.0
_MIN_KD_GAIN = -5.0
_MAX_KD_GAIN = 5.0


def _joints_within_target_threshold(
    cur_joints: Sequence[float],
    target: Sequence[float],
    threshold: Sequence[float] | float = 0.001,
) -> bool:
  assert len(cur_joints) == len(target)
  diffs = np.abs(np.array(cur_joints) - np.array(target))
  return bool(np.all(diffs < np.array(threshold)))


class MitJointPositionController:
  """Joint position controller that uses MIT-mode commands.

  By using MIT mode we can specify the P and D gains for the underlying
  controller.
  """

  def __init__(
      self,
      arm: agilex_interface.ArmInterface,
      kp_gains: Sequence[float] | float,
      kd_gains: Sequence[float] | float,
      rest_position: (
          Sequence[float] | None
      ) = ArmOrientations.upright.rest_position,
  ):
    """Controller constructor.

    Args:
      arm: The arm interface.
      kp_gains: Either one p-gain per joint, or a single shared p-gain.
      kd_gains: Either one d-gain per joint, or a single shared d-gain.
      rest_position: An optional per-joint set of angles in radians that the
        robot will go to upon stopping. If None, the rest behaviour is not
        executed.
    """
    self._arm = arm
    self._num_joints = arm.get_num_joints()

    if isinstance(kp_gains, (int, float)):
      self._kp_gains = (float(kp_gains),) * self._num_joints
    else:
      self._kp_gains = tuple(kp_gains)
      assert len(self._kp_gains) == self._num_joints

    if isinstance(kd_gains, (int, float)):
      self._kd_gains = (float(kd_gains),) * self._num_joints
    else:
      self._kd_gains = tuple(kd_gains)
      assert len(self._kd_gains) == self._num_joints

    if any(p < _MIN_KP_GAIN or p > _MAX_KP_GAIN for p in self._kp_gains):
      raise ValueError(f"KP gains outside valid range: {self._kp_gains}")

    if any(d < _MIN_KD_GAIN or d > _MAX_KD_GAIN for d in self._kd_gains):
      raise ValueError(f"KD gains outside valid range: {self._kd_gains}")

    self._rest_position = rest_position
    # Read the limits once now to warm the interface's cache and fail fast if
    # they are unavailable. command_joints re-reads them (cheaply, from that
    # cache) so a later set_joint_limits is always reflected.
    arm.get_joint_limits()

  def __enter__(self) -> Self:
    self.start()
    return self

  def __exit__(self, exit_type, value, traceback) -> None:
    del exit_type, value, traceback
    self.stop()

  def start(self) -> None:
    self._arm.set_mit_mode()

  def stop(self) -> None:
    # Move to the rest position if one is specified.
    if self._rest_position:
      self._smoothly_move_to_position(
          self._rest_position,
          threshold=0.1,  # No need to be precise.
          timeout=2.0,
      )

    # Over a few seconds relax all of the joints.
    self.relax_joints(2.0)

  def command_joints(
      self,
      target: Sequence[float],
      kp_gains: Sequence[float] | None = None,
      kd_gains: Sequence[float] | None = None,
      torques_ff: Sequence[float] | None = None,
      velocities: Sequence[float] | None = None,
  ) -> None:
    if kp_gains is None or len(kp_gains) == 0:
      kp_gains = self._kp_gains
    if kd_gains is None or len(kd_gains) == 0:
      kd_gains = self._kd_gains
    if torques_ff is None or len(torques_ff) == 0:
      torques_ff = (0.0,) * self._num_joints
    if velocities is None or len(velocities) == 0:
      velocities = (0.0,) * self._num_joints

    assert len(target) == self._num_joints
    assert len(kp_gains) == self._num_joints
    assert len(kd_gains) == self._num_joints
    assert len(torques_ff) == self._num_joints
    assert len(velocities) == self._num_joints

    # Re-read the limits each call (cheaply, interface-cached) so a later
    # set_joint_limits is reflected rather than going stale.
    joint_limits = self._arm.get_joint_limits()

    for ji, pos in enumerate(target):
      # Clip the position to limits so we don't send invalid commands.
      min_rad = joint_limits["min"][ji]
      max_rad = joint_limits["max"][ji]
      pos = min(max(pos, min_rad), max_rad)

      # pyAgxArm applies the per-model/firmware torque scaling and limit
      # internally, so torque_ff is passed through as raw physical Nm.
      self._arm.command_joint_position_mit(
          ji,
          position=pos,
          kp=kp_gains[ji],
          kd=kd_gains[ji],
          torque_ff=torques_ff[ji],
          velocity=velocities[ji],
      )

  def relax_joints(self, timeout: float) -> None:
    """Relaxes joints, using MIT mode, over a number of seconds.

    This can be useful to "rest" the arm just prior to shutting down.
    """
    num_steps = round(timeout * _CONTROL_RATE)
    kp_gains = np.geomspace(2.0, 0.01, num_steps)
    kd_gains = np.geomspace(1.0, 0.01, num_steps)
    num_joints = self._num_joints

    # Relax by ramping the gains toward zero. last_known holds only an actually
    # measured position - never an invented one.
    last_known = None
    for i in range(num_steps):
      try:
        last_known = self._arm.get_joint_positions()
      except RuntimeError:
        pass  # reuse the last measured position; a short/stale gap is safe

      if last_known is not None:
        # We know where we are: hold it with the decaying gains.
        self.command_joints(
            last_known,
            kp_gains=[kp_gains[i]] * num_joints,
            kd_gains=[kd_gains[i]] * num_joints,
        )
      else:
        # Position never measured. Do NOT pull toward any pose: kp=0 removes the
        # position term (p_des is irrelevant), leaving only kd velocity damping
        # - a gentle limp - while the gains still ramp to ~0 so nothing snaps on
        # disable.
        self.command_joints(
            [0.0] * num_joints,  # filler; ignored because kp=0
            kp_gains=[0.0] * num_joints,
            kd_gains=[kd_gains[i]] * num_joints,
        )
      time.sleep(1.0 / _CONTROL_RATE)

  def command_torques(self, torques: Sequence[float | None]) -> None:
    assert len(torques) == self._num_joints

    for ji, torque in enumerate(torques):
      if torque is not None:
        self._arm.command_joint_torque_mit(ji, torque)

  def _smoothly_move_to_position(
      self,
      target: Sequence[float],
      threshold: Sequence[float] | float = 0.001,
      timeout: float = 1.0,
  ) -> bool:
    assert len(target) == self._num_joints

    ramp_steps = round(timeout * _CONTROL_RATE)
    p_gains = np.geomspace(0.5, 5.0, ramp_steps)

    for step_idx in range(ramp_steps):
      self.command_joints(
          target, kp_gains=[p_gains[step_idx]] * self._num_joints
      )
      try:
        cur_joints = self._arm.get_joint_positions()
      except RuntimeError:
        time.sleep(1.0 / _CONTROL_RATE)
        continue

      if _joints_within_target_threshold(cur_joints, target, threshold):
        return True

      time.sleep(1.0 / _CONTROL_RATE)

    return False
