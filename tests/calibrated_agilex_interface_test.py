"""Tests for the calibrated Agilex interface."""

# pylint: disable=protected-access

import types

import numpy as np
import pytest

from agilex_control import calibrated_agilex_interface

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


def _calibrated_with_fake() -> (
    calibrated_agilex_interface.CalibratedArmInterface
):
  """Build a CalibratedArmInterface without connecting, backed by a fake."""
  cls = calibrated_agilex_interface.CalibratedArmInterface
  arm = cls.__new__(cls)
  arm._arm = _FakeDriver()
  arm._calibration_offsets = None
  return arm


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


def test_rejects_wrong_number_of_offsets():
  arm = _calibrated_with_fake()  # 6-joint arm
  with pytest.raises(ValueError):
    arm.set_calibration_offsets([0.1, 0.2, 0.3])
  assert not arm.is_calibrated
