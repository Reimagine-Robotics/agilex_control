"""Functions for enabling, disabling, and resetting the arm and gripper.

These helper methods are blocking calls as they perform multiple retries and
sleeps, using piper_interface under the hood.
"""

import time
from collections.abc import Callable

from agilex_control import agilex_interface

_SHORT_WAIT = 0.1
_LONG_WAIT = 0.5

# Minimum gap between resets, comfortably past the motors-off delay below:
# re-sending while the motors are still cutting out restarts the wait.
_MIN_RESET_INTERVAL_SEC = 2.0


def _create_timeout(
    seconds: float,
    message: str = "Timeout",
) -> Callable[[], None]:
  """Creates a timeout trigger that raises a TimeoutError after a given time."""
  start_time = time.time()

  def timeout_trigger() -> None:
    if time.time() - start_time > seconds:
      raise TimeoutError(message)

  return timeout_trigger


def disable_gripper(
    arm_interface: agilex_interface.ArmInterface,
    *,
    timeout_seconds: float = 10.0,
) -> None:
  """
  Disables the gripper.
  """
  timeout_trigger = _create_timeout(timeout_seconds, "disable_gripper")
  while True:
    arm_interface.disable_gripper()
    time.sleep(_SHORT_WAIT)
    if not arm_interface.is_gripper_enabled():
      break

    timeout_trigger()
    time.sleep(_LONG_WAIT)


def enable_gripper(
    arm_interface: agilex_interface.ArmInterface,
    *,
    timeout_seconds: float = 10.0,
) -> None:
  """
  Enables the gripper.
  """
  deadline = time.time() + timeout_seconds
  while not arm_interface.is_gripper_enabled():
    # Commanding the gripper to hold its current state enables it implicitly.
    value, force = arm_interface.get_gripper_state()
    arm_interface.command_gripper(position=value, force=force)
    if time.time() >= deadline:
      raise TimeoutError("Timed out while trying to enable the gripper")
    time.sleep(0.01)


def disable_arm(
    arm: agilex_interface.ArmInterface,
    *,
    timeout_seconds: float = 10.0,
) -> None:
  """
  Disables the arm.

  Returns once the joint motors have actually switched off, not merely once the
  arm reports STANDBY: the arm reports STANDBY within ~20ms of the reset but its
  motors only cut out 722-730ms later (measured across 20 resets).

  WARNING: This powers down the arm and it will drop if not supported.
  """
  timeout_trigger = _create_timeout(timeout_seconds, "disable_arm")
  while True:
    arm.disable_arm()
    time.sleep(_SHORT_WAIT)
    if not arm.is_arm_enabled():
      break

    timeout_trigger()
    time.sleep(_LONG_WAIT)


def enable_arm(
    arm: agilex_interface.ArmInterface,
    timeout_seconds: float = 10.0,
):
  """
  Enables the arm.
  """
  deadline = time.time() + timeout_seconds
  while not arm.enable_arm():
    if time.time() >= deadline:
      raise TimeoutError("Timed out while trying to enable the arm")
    time.sleep(0.01)


def clear_joint_errors(
    arm: agilex_interface.ArmInterface,
    *,
    timeout_seconds: float = 10.0,
) -> None:
  """Clears joint error codes; re-enables only if arm was enabled.

  Lighter than reset_arm (no power cycle). Use after firmware trips a joint
  on collision or tracking error.

  Args:
    joint: clears all errors on all joints
  """
  was_enabled = arm.is_arm_enabled()
  arm.clear_joint_errors()
  if was_enabled:
    enable_arm(arm, timeout_seconds=timeout_seconds)


def reset_arm(
    arm: agilex_interface.ArmInterface,
    timeout_seconds: float = 10.0,
) -> None:
  """
  Resets the arm.

  WARNING: This depowers the arm and it will drop if not supported.

  Args:
    arm_controller (ArmController): The arm controller to use when enabling.
    move_mode (MoveMode): The move mode to use when enabling.
  """
  disable_arm(arm, timeout_seconds=timeout_seconds)
  enable_arm(arm, timeout_seconds=timeout_seconds)
