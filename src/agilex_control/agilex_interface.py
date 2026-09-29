"""Wrapper around pyAgxArm for controlling AgileX arms."""

import enum
import time
from typing import Any

import pyAgxArm
from packaging import version as packaging_version


class ArmType(enum.Enum):
  """Different models of arm."""

  PIPER = "Piper"
  PIPER_H = "Piper H"
  PIPER_X = "Piper X"
  PIPER_L = "Piper L"


class GripperType(enum.Enum):
  """Different models of gripper."""

  V1 = "V1 7cm gripper"

  # Most arms now ship with the V2 gripper.
  V2 = "V2 10cm gripper"


class ArmInstallationPos(enum.Enum):
  """Arm mounting orientation.

  The values are our own identifiers. The interface maps these to pyAgxArm's own
  installation-pos constants (``arm.OPTIONS.INSTALLATION_POS.*``) at the call
  boundary, rather than hardcoding pyAgxArm's string values here.
  """

  UPRIGHT = "upright"

  # Side mount left. In this orientation, the rear cable is facing backward and
  # the green LED is above the cable socket.
  LEFT = "left"

  # Side mount right. In this orientation, the rear cable is facing backward and
  # the green LED is below the cable socket.
  RIGHT = "right"


# Maps our ArmType to pyAgxArm's ArmModel constant (the robot identifier passed
# to create_agx_arm_config).
_ARM_MODEL = {
    ArmType.PIPER: pyAgxArm.ArmModel.PIPER,
    ArmType.PIPER_H: pyAgxArm.ArmModel.PIPER_H,
    ArmType.PIPER_X: pyAgxArm.ArmModel.PIPER_X,
    ArmType.PIPER_L: pyAgxArm.ArmModel.PIPER_L,
}

# create_agx_arm_config's default driver profile. resolve_firmware_profile
# returns this for firmwares the default driver already handles, letting us skip
# reconnecting with a firmware-specific driver.
_DEFAULT_FIRMWARE_PROFILE = "default"


class ArmInterface:
  """A thin wrapper around pyAgxArm for a single AgileX arm.

  This class provides a nicer bridge to the underlying API. It abstracts away
  differences between arm models and firmware versions, e.g. Piper vs Nero.

  Constructing the interface connects to the arm, auto-detecting its firmware to
  select the matching pyAgxArm driver. Call disconnect() when done.
  """

  def __init__(
      self,
      can_port: str = "can0",
      arm_type: ArmType = ArmType.PIPER,
      can_interface: str = "socketcan",
      timeout: float = 5.0,
  ) -> None:
    self._can_port = can_port
    self._arm_type = arm_type
    self._can_interface = can_interface
    # pyAgxArm's driver has no importable public type; set by _connect().
    self._arm: Any = None
    self._firmware_version: str | None = None
    self._connect(timeout)

  def _connect(self, timeout: float) -> None:
    """Connect to the arm, selecting the driver for its firmware.

    pyAgxArm ships a per-firmware driver. We connect with the default driver to
    read the firmware version and resolve its profile. If the default driver
    already matches, we keep that connection; otherwise we disconnect and
    reconnect with the firmware-specific driver.
    """
    robot = _ARM_MODEL[self._arm_type]

    self._arm = pyAgxArm.AgxArmFactory.create_arm(
        pyAgxArm.create_agx_arm_config(
            robot=robot,
            firmeware_version=_DEFAULT_FIRMWARE_PROFILE,
            interface=self._can_interface,
            channel=self._can_port,
        )
    )
    self._arm.connect()
    deadline = time.time() + timeout
    # TODO: The Nero version in the examples also calls self._arm.enable()
    # inside this wait loop. Confirm whether it is needed in order to obtain
    # firmware when adding Nero support. See:
    # https://github.com/agilexrobotics/pyAgxArm/blob/841a625/pyAgxArm/demos/detect_nero_series.py
    firmware = self._arm.get_firmware()
    while firmware is None:
      if time.time() >= deadline:
        self._arm.disconnect()
        raise TimeoutError(
            f"Timed out waiting for firmware on {self._can_port}."
        )
      time.sleep(0.5)
      firmware = self._arm.get_firmware()
    software_version = firmware["software_version"]
    self._firmware_version = software_version

    firmware_profile = pyAgxArm.resolve_firmware_profile(
        robot, software_version
    )
    if firmware_profile == _DEFAULT_FIRMWARE_PROFILE:
      # The default driver already matches the firmware; keep this connection.
      return

    # The firmware needs a specific driver: reconnect with it.
    self._arm.disconnect()
    self._arm = pyAgxArm.AgxArmFactory.create_arm(
        pyAgxArm.create_agx_arm_config(
            robot=robot,
            firmeware_version=firmware_profile,  # pyAgxArm's spelling.
            interface=self._can_interface,
            channel=self._can_port,
        )
    )
    self._arm.connect()

  def disconnect(self) -> None:
    if self._arm is not None:
      self._arm.disconnect()

  def get_firmware_version(self) -> str | None:
    """Return the arm's firmware version, normalized (e.g. "1.8.post6").

    Falls back to the raw string if it does not parse.
    """
    if self._firmware_version is None:
      return None
    try:
      raw = self._firmware_version
      return str(packaging_version.parse(raw[raw.index("V") :].strip()))
    except (packaging_version.InvalidVersion, ValueError):
      # Just return the raw string if parsing fails
      return self._firmware_version

  def get_joint_positions(self) -> list[float]:
    """
    Returns the current joint positions as a sequence of floats (radians).

    Returns:
      Sequence[float]: Joint positions in radians.
    """
    msg = self._arm.get_joint_angles()
    if msg is None:
      raise RuntimeError("No joint angle feedback available.")
    return list(msg.msg)
