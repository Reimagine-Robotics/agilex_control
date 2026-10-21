"""Tests for the calibrated Agilex interface."""

# pylint: disable=protected-access

import types
from unittest import mock

import numpy as np
import pytest

from agilex_control import (
    agilex_control,
    agilex_interface,
    calibrated_agilex_interface,
)

_RAW_ANGLES = [1.0, 1.1, 1.2, 1.3, 1.4, 1.5]
_OFFSETS = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]


class _FakeDriver:
  """Serves canned joint angles and records move_mit calls."""

  def __init__(self, angles=tuple(_RAW_ANGLES)):
    self.joint_nums = len(angles)
    self._angles = list(angles)
    self.calls = []

  def get_joint_angles(self):
    return types.SimpleNamespace(msg=list(self._angles))

  def get_joint_angle_vel_limits(self, joint_index: int):
    del joint_index
    return types.SimpleNamespace(
        msg=types.SimpleNamespace(min_angle_limit=-2.0, max_angle_limit=2.0)
    )

  def move_mit(
      self, joint_index, p_des=0.0, v_des=0.0, kp=10.0, kd=0.8, t_ff=0.0
  ):
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


def _calibrated_with_fake(
    limits: dict[str, list[float]] | None = None,
) -> calibrated_agilex_interface.CalibratedArmInterface:
  """Build a CalibratedArmInterface without connecting, backed by a fake."""

  def connect(arm: agilex_interface.ArmInterface, timeout: float) -> None:
    del timeout
    arm._arm = _FakeDriver()

  with mock.patch.object(agilex_interface.ArmInterface, "_connect", connect):
    return calibrated_agilex_interface.CalibratedArmInterface(
        calibrated_joint_limits=limits
    )


def test_reports_raw_positions_until_calibrated():
  arm = _calibrated_with_fake()
  assert not arm.is_calibrated
  assert arm.calibration_offsets is None
  assert arm.get_joint_positions() == pytest.approx(_RAW_ANGLES)


def test_reports_positions_relative_to_calibration_offsets():
  arm = _calibrated_with_fake()
  arm.set_calibration_offsets(_OFFSETS)
  assert arm.is_calibrated
  assert arm.calibration_offsets == pytest.approx(_OFFSETS)
  expected = (np.asarray(_RAW_ANGLES) - np.asarray(_OFFSETS)).tolist()
  assert arm.get_joint_positions() == pytest.approx(expected)
  # raw=True bypasses the offsets.
  assert arm.get_joint_positions(raw=True) == pytest.approx(_RAW_ANGLES)


def test_clearing_offsets_restores_raw_positions():
  arm = _calibrated_with_fake()
  arm.set_calibration_offsets(_OFFSETS)
  arm.clear_calibration_offsets()
  assert not arm.is_calibrated
  assert arm.get_joint_positions() == pytest.approx(_RAW_ANGLES)


def test_mit_command_is_shifted_into_the_raw_frame():
  arm = _calibrated_with_fake()
  arm.set_calibration_offsets(_OFFSETS)
  for joint in range(len(_OFFSETS)):
    arm.command_joint_position_mit(joint, position=0.0, kp=10.0, kd=0.8)
  # Calibrated target 0.0 reaches the arm as the raw offset, one-based index.
  assert [call["joint_index"] for call in arm._arm.calls] == [
      joint + 1 for joint in range(len(_OFFSETS))
  ]
  assert [call["p_des"] for call in arm._arm.calls] == pytest.approx(_OFFSETS)


def test_raw_mit_command_bypasses_the_offsets():
  arm = _calibrated_with_fake()
  arm.set_calibration_offsets(_OFFSETS)
  arm.command_joint_position_mit(2, position=0.5, kp=10.0, kd=0.8, raw=True)
  assert arm._arm.calls[0]["p_des"] == pytest.approx(0.5)


def test_set_calibration_offsets_copies_input():
  arm = _calibrated_with_fake()
  offsets = list(_OFFSETS)
  arm.set_calibration_offsets(offsets)
  offsets[0] = 99.0  # mutating the caller's array must not shift calibration
  assert arm.calibration_offsets == pytest.approx(_OFFSETS)


def test_rejects_wrong_number_of_offsets():
  arm = _calibrated_with_fake()  # 6-joint arm
  with pytest.raises(ValueError):
    arm.set_calibration_offsets([0.1, 0.2, 0.3])
  assert not arm.is_calibrated


@pytest.mark.parametrize(
    "bad",
    [
        [float("nan")] * 6,
        [float("inf")] * 6,
        [[0.1]] * 6,  # (6, 1): right outer length, wrong shape
    ],
)
def test_rejects_non_finite_or_misshaped_offsets(bad):
  arm = _calibrated_with_fake()
  with pytest.raises(ValueError):
    arm.set_calibration_offsets(bad)
  assert not arm.is_calibrated


def test_controller_limits_follow_calibration_changes() -> None:
  arm = _calibrated_with_fake({"min": [-0.5] * 6, "max": [0.5] * 6})
  controller = agilex_control.MitJointPositionController(
      arm, kp_gains=5.0, kd_gains=0.8
  )
  controller.command_joints([1.5] * 6)
  assert [c["p_des"] for c in arm._arm.calls[-6:]] == [1.5] * 6

  arm.set_calibration_offsets([0.2] * 6)
  controller.command_joints([1.5] * 6)
  assert [c["p_des"] for c in arm._arm.calls[-6:]] == pytest.approx([0.7] * 6)

  arm.clear_calibration_offsets()
  controller.command_joints([1.5] * 6)
  assert [c["p_des"] for c in arm._arm.calls[-6:]] == [1.5] * 6

  arm.set_calibration_offsets([-0.1] * 6)
  controller.command_joints([-1.5] * 6)
  assert [c["p_des"] for c in arm._arm.calls[-6:]] == pytest.approx([-0.6] * 6)


def test_unconfigured_limits_preserve_firmware_bounds() -> None:
  arm = _calibrated_with_fake()
  firmware_limits = arm.get_joint_limits()
  arm.set_calibration_offsets(_OFFSETS)
  assert arm.get_joint_limits() == firmware_limits
  arm.clear_calibration_offsets()
  assert arm.get_joint_limits() == firmware_limits


def test_calibrated_limits_copy_inputs_and_outputs() -> None:
  limits = {"min": [-0.1] * 6, "max": [1.1] * 6}
  arm = _calibrated_with_fake(limits)
  arm.set_calibration_offsets(_OFFSETS)
  limits["min"][0] = -10.0
  arm.get_joint_limits()["max"][1] = 10.0
  assert arm.get_joint_limits() == {"min": [-0.1] * 6, "max": [1.1] * 6}


@pytest.mark.parametrize(
    "limits",
    [
        {"min": [0.0] * 5, "max": [1.0] * 5},
        {"min": [2.0] * 6, "max": [1.0] * 6},
        {"min": [float("nan")] * 6, "max": [1.0] * 6},
        {"min": [0.0] * 6, "max": [float("inf")] * 6},
    ],
)
def test_rejects_invalid_calibrated_limits(
    limits: dict[str, list[float]],
) -> None:
  with pytest.raises(ValueError, match="finite, ordered"):
    _calibrated_with_fake(limits)
