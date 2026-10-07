"""Smoke tests for ArmInterface's MIT command mapping (no hardware).

Uses a fake driver and bypasses connect() so we can assert what the command
wrappers forward to pyAgxArm's move_mit: the zero-to-one-based joint index and
the per-argument values.
"""

# pylint: disable=protected-access

import types

import pyAgxArm
import pytest

from agilex_control import agilex_interface


class _FakeDriver:
  """Records move_mit calls so tests can assert the interface's mapping."""

  def __init__(self, joint_nums=6):
    self.calls = []
    self.calibrate_calls = []
    self.joint_nums = joint_nums

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

  def calibrate_joint(self, joint_index):
    """Record a calibrate_joint call."""
    self.calibrate_calls.append(joint_index)


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


def test_set_joint_zero_positions_maps_to_one_based_calibrate():
  arm = _interface_with_fake()
  arm.set_joint_zero_positions([0, 5])
  # Zero-based indices map to pyAgxArm's one-based calibrate_joint.
  assert arm._arm.calibrate_calls == [1, 6]


def test_set_joint_zero_positions_rejects_out_of_range():
  arm = _interface_with_fake()  # 6-joint arm
  with pytest.raises(ValueError):
    arm.set_joint_zero_positions([0, 6])  # 6 is out of range [0, 5]
  # Validation runs before any zeroing, so nothing was calibrated.
  assert arm._arm.calibrate_calls == []


class _FakeGripper:
  """Fake effector recording move_gripper_m and returning canned feedback."""

  def __init__(self, position=0.03, force=1.5, max_opening=0.08, mode="width"):
    self._position = position
    self._force = force
    self._max_opening = max_opening
    self._mode = mode
    self.move_calls = []

  def move_gripper_m(self, value=0.0, force=1.0):
    """Record a gripper move."""
    self.move_calls.append((value, force))

  def get_gripper_status(self):
    """Return canned (position, force, mode) feedback like pyAgxArm's."""
    return types.SimpleNamespace(
        msg=types.SimpleNamespace(
            value=self._position, force=self._force, mode=self._mode
        )
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


def test_get_gripper_state_raises_in_angle_mode():
  # We only support width mode; angle mode would report degrees, not metres.
  arm = _interface_with_fake_gripper(_FakeGripper(mode="angle"))
  with pytest.raises(RuntimeError):
    arm.get_gripper_state()


def _arm_with_config(arm_type) -> agilex_interface.ArmInterface:
  """ArmInterface (no connect) with the config for arm_type set."""
  arm = agilex_interface.ArmInterface.__new__(agilex_interface.ArmInterface)
  arm._arm_type = arm_type
  arm._firmware_version = "S-V1.8-6"  # modern driver (> 1.8.post2)
  arm._config = pyAgxArm.create_agx_arm_config(
      robot=agilex_interface._ARM_MODEL[arm_type]
  )
  arm._arm = types.SimpleNamespace(
      joint_nums=len(arm._config["joint_torque_k"])
  )
  return arm


def test_direct_scaling_factors_base_piper_is_identity():
  # Base Piper is the reference, so every joint scales by 1.0.
  arm = _arm_with_config(agilex_interface.ArmType.PIPER)
  assert arm.direct_scaling_factors() == pytest.approx([1.0] * 6)


def test_direct_scaling_factors_piper_h_scales_wrist_and_j2():
  # piperH differs from base Piper on j2 (k and b) and j4/j5 (b), so those get
  # the base(k*b)/arm(k*b) ratio; j1/j3/j6 match base and stay 1.0.
  arm = _arm_with_config(agilex_interface.ArmType.PIPER_H)
  assert arm.direct_scaling_factors() == pytest.approx(
      [1.0, 1.143, 1.0, 0.588, 0.588, 1.0], abs=1e-3
  )


def test_direct_scaling_factors_nero_is_unscaled():
  # Nero is a different arm family (7 joints), not defined relative to base
  # Piper, so it is left unscaled.
  arm = _arm_with_config(agilex_interface.ArmType.NERO)
  scale = arm.direct_scaling_factors()
  assert scale == pytest.approx([1.0] * 7)
  assert len(scale) == 7


def test_direct_scaling_factors_rejects_legacy_firmware():
  # The legacy default driver (<= 1.8.post2) divides by b*c, so a non-base arm
  # needs a different base(k)/arm(k) correction there that is not implemented --
  # it should raise NotImplementedError rather than return a wrong scale.
  arm = _arm_with_config(agilex_interface.ArmType.PIPER_H)
  arm._firmware_version = "S-V1.8-2"
  with pytest.raises(NotImplementedError):
    arm.direct_scaling_factors()


def test_direct_scaling_factors_base_piper_identity_on_legacy_firmware():
  # Base Piper's ratio is 1.0 (no correction) on any firmware, so legacy
  # firmware must not raise -- base Piper works fine on <= 1.8.post2.
  arm = _arm_with_config(agilex_interface.ArmType.PIPER)
  arm._firmware_version = "S-V1.8-2"
  assert arm.direct_scaling_factors() == pytest.approx([1.0] * 6)


def test_compute_direct_scaling_factors_offline_matches_models():
  # Offline helper (no live arm) reproduces the per-model ratios.
  piper = agilex_interface.compute_direct_scaling_factors(
      agilex_interface.ArmType.PIPER, "1.8.post6"
  )
  assert piper == pytest.approx([1.0] * 6)
  piper_h = agilex_interface.compute_direct_scaling_factors(
      agilex_interface.ArmType.PIPER_H, "1.8.post6"
  )
  assert piper_h == pytest.approx(
      [1.0, 1.143, 1.0, 0.588, 0.588, 1.0], abs=1e-3
  )
  nero = agilex_interface.compute_direct_scaling_factors(
      agilex_interface.ArmType.NERO, "1.8.post6"
  )
  assert nero == pytest.approx([1.0] * 7)


def test_compute_direct_scaling_factors_base_piper_identity_on_legacy():
  # Base Piper is identity on any firmware, so legacy must not raise.
  piper = agilex_interface.compute_direct_scaling_factors(
      agilex_interface.ArmType.PIPER, "1.8.post2"
  )
  assert piper == pytest.approx([1.0] * 6)


def test_compute_direct_scaling_factors_rejects_legacy_non_base():
  with pytest.raises(NotImplementedError):
    agilex_interface.compute_direct_scaling_factors(
        agilex_interface.ArmType.PIPER_H, "1.8.post2"
    )


def test_joint_torque_coefficients_offline_returns_kbc():
  k, b, c = agilex_interface.joint_torque_coefficients(
      agilex_interface.ArmType.PIPER_H
  )
  assert len(k) == len(b) == len(c) == 6
  # piperH wrist c: j4/j5 ~ 0.757, j6 ~ 1.287 (per-model, c=1 elsewhere).
  assert c[3] == pytest.approx(0.757, abs=1e-3)
  assert c[5] == pytest.approx(1.287, abs=1e-3)
