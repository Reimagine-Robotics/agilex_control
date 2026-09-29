"""Smoke tests for ArmInterface's MIT command mapping (no hardware).

Uses a fake driver and bypasses connect() so we can assert what the command
wrappers forward to pyAgxArm's move_mit: the zero-to-one-based joint index and
the per-argument values.
"""

# pylint: disable=protected-access

from agilex_control.agilex_interface import ArmInterface


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


def _interface_with_fake() -> ArmInterface:
  """Build an ArmInterface without connecting, backed by a fake driver."""
  arm = ArmInterface.__new__(ArmInterface)
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
