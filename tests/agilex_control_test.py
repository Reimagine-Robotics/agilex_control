"""Smoke tests for MitJointPositionController (no hardware).

Uses a fake arm interface that records the commands the controller forwards, so
we can assert the clipping, torque forwarding, and gain validation without a
robot.
"""

import pytest

from agilex_control import agilex_control


class _FakeArm:
  """Records the interface calls the controller makes."""

  def __init__(self, limits=None):
    self._limits = limits or {"min": [-2.0] * 6, "max": [2.0] * 6}
    self.mit_mode_calls = 0
    self.position_cmds = []
    self.torque_cmds = []

  def get_num_joints(self):
    """Return the number of joints."""
    return len(self._limits["min"])

  def get_joint_limits(self):
    """Return a copy of the fake joint limits."""
    return {"min": list(self._limits["min"]), "max": list(self._limits["max"])}

  def set_mit_mode(self):
    """Record a set_mit_mode call."""
    self.mit_mode_calls += 1

  def command_joint_position_mit(
      self, joint_index, *, position, kp, kd, torque_ff=0.0, velocity=0.0
  ):
    """Record a position command."""
    self.position_cmds.append(
        (joint_index, position, kp, kd, torque_ff, velocity)
    )

  def command_joint_torque_mit(self, joint_index, torque):
    """Record a torque command."""
    self.torque_cmds.append((joint_index, torque))

  def get_joint_positions(self):
    """Return a stationary pose."""
    return [0.0] * 6


def test_start_sets_mit_mode():
  arm = _FakeArm()
  controller = agilex_control.MitJointPositionController(
      arm, kp_gains=5.0, kd_gains=0.8
  )
  controller.start()
  assert arm.mit_mode_calls == 1


def test_context_manager_starts_and_stops():
  arm = _FakeArm()
  controller = agilex_control.MitJointPositionController(
      arm, kp_gains=5.0, kd_gains=0.8
  )
  # Spy start/stop so we assert the protocol without running the real (slow)
  # rest-and-relax loops.
  calls = []
  controller.start = lambda: calls.append("start")
  controller.stop = lambda: calls.append("stop")
  with controller as entered:
    assert entered is controller
    assert calls == ["start"]
  assert calls == ["start", "stop"]


def test_command_joints_clips_to_limits_and_forwards():
  arm = _FakeArm(limits={"min": [-1.0] * 6, "max": [1.0] * 6})
  controller = agilex_control.MitJointPositionController(
      arm, kp_gains=5.0, kd_gains=0.8, rest_position=None
  )
  # J1/J2 targets exceed the ±1.0 limits and should be clipped; J3 is in range.
  controller.command_joints(
      [2.0, -2.0, 0.5, 0.0, 0.0, 0.0],
      torques_ff=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
      velocities=[0.1] * 6,
  )
  assert len(arm.position_cmds) == 6
  # (joint_index, position, kp, kd, torque_ff, velocity)
  assert arm.position_cmds[0] == (0, 1.0, 5.0, 0.8, 1.0, 0.1)
  assert arm.position_cmds[1] == (1, -1.0, 5.0, 0.8, 2.0, 0.1)
  assert arm.position_cmds[2] == (2, 0.5, 5.0, 0.8, 3.0, 0.1)


def test_command_torques_forwards_non_none_only():
  arm = _FakeArm()
  controller = agilex_control.MitJointPositionController(
      arm, kp_gains=5.0, kd_gains=0.8, rest_position=None
  )
  controller.command_torques([1.0, None, 2.0, None, None, 3.0])
  assert arm.torque_cmds == [(0, 1.0), (2, 2.0), (5, 3.0)]


def test_rejects_out_of_range_gains():
  arm = _FakeArm()
  with pytest.raises(ValueError):
    agilex_control.MitJointPositionController(
        arm, kp_gains=1000.0, kd_gains=0.8
    )  # kp > 500
  with pytest.raises(ValueError):
    agilex_control.MitJointPositionController(
        arm, kp_gains=5.0, kd_gains=10.0
    )  # kd > 5


def test_move_to_position_returns_true_when_reached():
  arm = _FakeArm()  # get_joint_positions reports a stationary [0]*6 pose.
  controller = agilex_control.MitJointPositionController(
      arm, kp_gains=5.0, kd_gains=0.8, rest_position=None
  )
  reached = controller.move_to_position([0.0] * 6, threshold=0.1, timeout=1.0)
  assert reached
  assert len(arm.position_cmds) >= 1


def test_move_to_position_returns_false_on_timeout():
  arm = (
      _FakeArm()
  )  # Always reports [0]*6, so a nonzero target is never reached.
  controller = agilex_control.MitJointPositionController(
      arm, kp_gains=5.0, kd_gains=0.8, rest_position=None
  )
  reached = controller.move_to_position([1.0] * 6, threshold=0.01, timeout=0.05)
  assert not reached
