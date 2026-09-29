"""Wrapper around pyAgxArm for controlling AgileX arms."""

import enum


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
