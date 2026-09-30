"""Smoke tests for ArmInterface's MIT command mapping (no hardware).

Uses a fake driver and bypasses connect() so we can assert what the command
wrappers forward to pyAgxArm's move_mit: the zero-to-one-based joint index and
the per-argument values.
"""

# pylint: disable=protected-access

import types

from agilex_control import agilex_interface


class _FakeDriver:
  """Records move_mit calls so tests can assert the interface's mapping."""

  def __init__(self):
    self.calls = []

  def move_mit(
      self, joint_index, p_des=0.0, v_des=0.0, kp=10.0, kd=0.8, t_ff=0.0
  ):
    """Record a move_mit call."""
    self.calls.append(
        {
            "joint_index": joint_index,
            "p_des": p_des,
            "v_des": v_des,
            "kp": kp,
            "kd": kd,
            "t_ff": t_ff,
        }
    )


def _interface_with_fake() -> agilex_interface.ArmInterface:
  """Build an ArmInterface without connecting, backed by a fake driver."""
  arm = agilex_interface.ArmInterface.__new__(agilex_interface.ArmInterface)
  arm._arm = _FakeDriver()
  return arm


def test_command_joint_position_mit_maps_to_move_mit():
  arm = _interface_with_fake()
  arm.command_joint_position_mit(
      0, position=0.5, kp=10.0, kd=0.8, torque_ff=1.0, velocity=0.2
  )
  assert arm._arm.calls == [
      {
          "joint_index": 1,  # zero-based -> one-based
          "p_des": 0.5,
          "v_des": 0.2,
          "kp": 10.0,
          "kd": 0.8,
          "t_ff": 1.0,
      }
  ]


def test_command_joint_torque_mit_maps_to_move_mit():
  arm = _interface_with_fake()
  arm.command_joint_torque_mit(5, 2.5)
  assert arm._arm.calls == [
      {
          "joint_index": 6,  # zero-based -> one-based
          "p_des": 0.0,
          "v_des": 0.0,
          "kp": 0.0,  # pure torque: zero PD gains
          "kd": 0.0,
          "t_ff": 2.5,
      }
  ]


class _FakeGripper:
  """Fake effector recording move_gripper_m and returning canned feedback."""

  def __init__(self, position=0.03, force=1.5, max_opening=0.08):
    self._position = position
    self._force = force
    self._max_opening = max_opening
    self.move_calls = []

  def move_gripper_m(self, value=0.0, force=1.0):
    """Record a gripper move."""
    self.move_calls.append((value, force))

  def get_gripper_status(self):
    """Return canned (position, force) feedback, shaped like pyAgxArm's."""
    return types.SimpleNamespace(
        msg=types.SimpleNamespace(value=self._position, force=self._force)
    )

  def get_gripper_teaching_pendant_param(self, timeout=1.0, min_interval=1.0):
    """Return the canned configured max range."""
    del timeout, min_interval
    return types.SimpleNamespace(
        msg=types.SimpleNamespace(max_range_config=self._max_opening)
    )


def _interface_with_fake_gripper(gripper) -> agilex_interface.ArmInterface:
  """Build an ArmInterface without connecting, backed by a fake gripper."""
  arm = agilex_interface.ArmInterface.__new__(agilex_interface.ArmInterface)
  arm._gripper = gripper
  arm._gripper_max_opening = None
  return arm


def test_command_gripper_clips_position_to_max_opening():
  gripper = _FakeGripper(max_opening=0.08)
  arm = _interface_with_fake_gripper(gripper)
  arm.command_gripper(position=0.2, force=1.0)  # exceeds the 0.08 max
  assert gripper.move_calls == [(0.08, 1.0)]


def test_command_gripper_none_keeps_current_position_and_force():
  gripper = _FakeGripper(position=0.03, force=1.5)
  arm = _interface_with_fake_gripper(gripper)
  arm.command_gripper(position=None, force=None)
  assert gripper.move_calls == [(0.03, 1.5)]
