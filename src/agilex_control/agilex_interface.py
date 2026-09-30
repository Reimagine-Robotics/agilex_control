"""Wrapper around pyAgxArm for controlling AgileX arms."""

import enum
import time
from collections.abc import Sequence
from typing import Any

import pyAgxArm
from packaging import version as packaging_version


class ArmType(enum.Enum):
    """Different models of arm."""

    PIPER = "Piper"
    PIPER_H = "Piper H"
    PIPER_X = "Piper X"
    PIPER_L = "Piper L"
    NERO = "Nero"


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
    ArmType.NERO: pyAgxArm.ArmModel.Nero,
}

# create_agx_arm_config's default driver profile. resolve_firmware_profile
# returns this for firmwares the default driver already handles, letting us skip
# reconnecting with a firmware-specific driver.
_DEFAULT_FIRMWARE_PROFILE = "default"

# Max gripper force in Newtons. AgileX's gripper docs give a force range of
# [0.0, 3.0] N; unlike the max opening, it is not queryable from the arm, so it
# is a constant here. See get_gripper_status in:
# https://github.com/agilexrobotics/pyAgxArm/blob/841a625/docs/effector/agx_gripper/agx_gripper_api.md
_GRIPPER_FORCE_MAX = 3.0


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
        # pyAgxArm's driver and gripper effector have no importable public type;
        # both are set by _connect().
        self._arm: Any = None
        self._gripper: Any = None
        self._firmware_version: str | None = None
        # Cached per-joint angle limits (static config; read from the arm on first
        # get_joint_limits, invalidated by set_joint_limits).
        self._joint_limits: dict[str, list[float]] | None = None
        # Cached gripper max opening in metres (static config; read from the arm on
        # first get_gripper_max_opening).
        self._gripper_max_opening: float | None = None
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
        self._init_gripper()
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

        firmware_profile = pyAgxArm.resolve_firmware_profile(robot, software_version)
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
        self._init_gripper()
        self._arm.connect()

    def _init_gripper(self) -> None:
        """Initializes the gripper effector on the current driver.

        Must be called before connect() (per pyAgxArm's effector docs, so the
        effector's feedback is parsed from the start of the read thread) and only
        once per driver instance -- a reconnect creates a new driver and re-inits.
        See the init_effector notes in:
        https://github.com/agilexrobotics/pyAgxArm/blob/841a625/docs/effector/agx_gripper/agx_gripper_api.md
        """
        self._gripper = self._arm.init_effector(self._arm.OPTIONS.EFFECTOR.AGX_GRIPPER)

    def disconnect(self) -> None:
        if self._arm is not None:
            self._arm.disconnect()

    def enable_arm(self) -> bool:
        """Enables all joint motors, returning whether they report enabled."""
        return self._arm.enable()

    def disable_arm(self) -> bool:
        """Disables all joint motors, returning whether they report disabled.

        WARNING: this powers down the joints; an unsupported arm will drop.
        """
        return self._arm.disable()

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

    def get_num_joints(self) -> int:
        """Returns the number of joints on the arm."""
        return self._arm.joint_nums

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

    def get_joint_velocities(self) -> list[float]:
        """
        Returns the current joint velocities as a sequence of floats (rad/s).

        Returns:
          Sequence[float]: Joint velocities in radians per second.
        """
        states = [
            self._arm.get_motor_states(i) for i in range(1, self.get_num_joints() + 1)
        ]
        if any(state is None for state in states):
            raise RuntimeError("No motor state feedback available.")
        return [state.msg.velocity for state in states]

    def get_joint_limits(self) -> dict[str, list[float]]:
        """
        Returns the per-joint angle limits (radians) read from the arm.

        Performs a one-time read from the arm and caches the result (limits are
        static config); set_joint_limits refreshes the cache.

        Returns:
          dict[str, list[float]]: A dictionary with 'min' and 'max' keys containing
        lists of joint limits in radians, e.g. {"min": [...], "max": [...]}.
        """
        if self._joint_limits is None:
            mins: list[float] = []
            maxs: list[float] = []
            for i in range(1, self._arm.joint_nums + 1):
                limit = self._arm.get_joint_angle_vel_limits(i)
                if limit is None:
                    raise RuntimeError(f"No joint limit feedback for joint {i}.")
                mins.append(limit.msg.min_angle_limit)
                maxs.append(limit.msg.max_angle_limit)
            self._joint_limits = {"min": mins, "max": maxs}
        # Return copies so callers can't mutate the cache.
        return {
            "min": list(self._joint_limits["min"]),
            "max": list(self._joint_limits["max"]),
        }

    def set_joint_limits(
        self,
        min_angles: Sequence[float],
        max_angles: Sequence[float],
    ) -> None:
        """
        Overrides the arm's per-joint angle limits (radians).

        Both sequences must have length joint_nums. Invalidates the cached limits so
        the next joint_limits access re-reads them from the arm.

        Args:
          min_angles (Sequence[float]): Per-joint minimum angle limits in radians.
          max_angles (Sequence[float]): Per-joint maximum angle limits in radians.
        """
        num_joints = self._arm.joint_nums
        if len(min_angles) != num_joints or len(max_angles) != num_joints:
            raise ValueError(
                f"Expected {num_joints} limits, got {len(min_angles)} min /"
                f" {len(max_angles)} max."
            )
        # Invalidate the cache before writing, so a mid-loop failure can't leave a
        # stale cache while the arm has already been partially changed.
        self._joint_limits = None
        for i, (min_angle, max_angle) in enumerate(
            zip(min_angles, max_angles), start=1
        ):
            self._arm.set_joint_angle_vel_limits(
                i, min_angle_limit=min_angle, max_angle_limit=max_angle
            )

    def set_installation_pos(
        self, installation_pos: ArmInstallationPos = ArmInstallationPos.UPRIGHT
    ) -> None:
        """Sets the arm's mounting orientation. Call right after connecting.

        Maps our ArmInstallationPos to pyAgxArm's own installation-pos constants
        rather than hardcoding its string values.
        """
        options = self._arm.OPTIONS.INSTALLATION_POS
        installation_pos_map = {
            ArmInstallationPos.UPRIGHT: options.HORIZONTAL,
            ArmInstallationPos.LEFT: options.LEFT,
            ArmInstallationPos.RIGHT: options.RIGHT,
        }
        self._arm.set_installation_pos(installation_pos_map[installation_pos])

    def set_mit_mode(self) -> None:
        """Switches the arm to MIT mode for move_mit commands.

        Also disables pyAgxArm's automatic per-call motion-mode frames, so a loop of
        move_mit calls does not re-send the mode every tick.
        """
        self._arm.set_motion_mode(self._arm.OPTIONS.MOTION_MODE.MIT)
        self._arm.set_auto_set_motion_mode_enabled(False)

    def command_joint_position_mit(
        self,
        joint_index: int,
        *,
        position: float,
        kp: float,
        kd: float,
        torque_ff: float = 0.0,
        velocity: float = 0.0,
    ) -> None:
        """
        Commands a single joint via MIT control to a given angle.

        Requires MIT mode (see set_mit_mode). pyAgxArm applies the per-model b/c
        scaling and the per-firmware wire torque limit internally, so torque_ff is
        passed as raw physical Nm.

        Args:
          joint_index (int): Zero-based joint index (0 to joint_nums - 1).
          position (float): Desired position in radians.
          kp (float): Proportional gain.
          kd (float): Derivative gain.
          torque_ff (float): Feed-forward torque in Nm.
          velocity (float): Desired velocity in radians per second.
        """
        self._arm.move_mit(
            joint_index + 1,
            p_des=position,
            v_des=velocity,
            kp=kp,
            kd=kd,
            t_ff=torque_ff,
        )

    def command_joint_torque_mit(self, joint_index: int, torque: float) -> None:
        """
        Commands a single joint via pure MIT torque (zero PD gains).

        Requires MIT mode (see set_mit_mode).

        Args:
          joint_index (int): Zero-based joint index (0 to joint_nums - 1).
          torque (float): The feed-forward torque command in Nm.
        """
        self._arm.move_mit(
            joint_index + 1, p_des=0.0, v_des=0.0, kp=0.0, kd=0.0, t_ff=torque
        )

    def get_gripper_max_opening(self) -> float:
        """
        Returns the gripper's maximum opening in metres, read from the arm.

        Performs a one-time read of the gripper's configured max range and caches
        it (static config), matching how get_joint_limits reads from the arm.

        Raises:
          RuntimeError: If the gripper parameter feedback is not available.
        """
        max_opening = self._gripper_max_opening
        if max_opening is None:
            param = self._gripper.get_gripper_teaching_pendant_param()
            if param is None:
                raise RuntimeError("No gripper parameter feedback available.")
            max_opening = param.msg.max_range_config
            self._gripper_max_opening = max_opening
        return max_opening

    def get_gripper_state(self) -> tuple[float, float]:
        """
        Returns the current gripper state as a tuple of (position in metres, force
        in Newtons).

        Returns:
          tuple[float, float]: (gripper position, gripper force)

        Raises:
          RuntimeError: If no gripper feedback is available, or the gripper is in
            angle mode (we only command width mode, so its value would be degrees,
            not metres).
        """
        status = self._gripper.get_gripper_status()
        if status is None:
            raise RuntimeError("No gripper feedback available.")
        # We only ever command via move_gripper_m (width mode), so value is in
        # metres. Fail loudly rather than silently treat an angle (degrees) as
        # metres if the gripper is somehow in angle mode.
        if status.msg.mode != "width":
            raise RuntimeError(
                f"Gripper is in {status.msg.mode!r} mode; only width mode "
                "(metres) is supported."
            )
        return status.msg.value, status.msg.force

    def command_gripper(
        self, position: float | None = None, force: float | None = None
    ) -> None:
        """
        Commands the gripper to an opening width with a given force.

        pyAgxArm's move command enables the gripper implicitly, so no separate
        enable step is needed.

        Args:
          position (float | None): Desired gripper opening in metres, clipped to
            [0, get_gripper_max_opening()]. If None, the current position is kept.
          force (float | None): Desired gripper force in Newtons, clipped to
            [0, _GRIPPER_FORCE_MAX]. If None, the current force is kept.
        """
        if position is None or force is None:
            current_position, current_force = self.get_gripper_state()
            position = current_position if position is None else position
            force = current_force if force is None else force
        position = min(max(position, 0.0), self.get_gripper_max_opening())
        force = min(max(force, 0.0), _GRIPPER_FORCE_MAX)
        self._gripper.move_gripper_m(value=position, force=force)

    def disable_gripper(self) -> None:
        """Disables the gripper. WARNING: it will go limp and may drop its load."""
        self._gripper.disable_gripper()
