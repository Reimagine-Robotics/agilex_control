# Migrating from `piper_control` to `agilex_control`

`agilex_control` wraps AgileX's `pyAgxArm` SDK; `piper_control` wrapped
`piper_sdk`. This guide maps the API and calls out behavioural differences for
code moving over.

## Quick start

Most code moving over needs only these four changes:

1. **Rename imports / classes** — `PiperInterface` → `ArmInterface`,
   `piper_connect` → `can_utils`, `piper_init` → `agilex_init`, and drop the
   `Piper` enum prefix (`PiperArmType` → `ArmType`). See [API mapping](#api-mapping).
2. **`get_joint_efforts` → `get_joint_torques`** — same N·m on the base joints,
   but the wrist value now carries `c` (see
   [the `c` coefficient](#joint-torque-and-the-c-coefficient)).
3. **Joint limits may have changed** — `get_joint_limits()` now returns what the
   arm reports, not piper_control's hardcoded tables. Nothing errors if you skip
   this; if you rely on specific limits, call `set_joint_limits(min, max)` after
   connecting — see [Joint limits](#joint-limits).
4. **Catch `RuntimeError` on reads** — `get_joint_positions` /
   `get_joint_velocities` / `get_joint_torques` raise before the first CAN frame
   (piper_sdk returned zeros); skip the tick in your control loop. See
   [Reads can raise](#reads-can-raise-before-the-first-frame).

The same minimal control loop, before and after:

```python
# Before — piper_control
from piper_control import piper_connect, piper_init, piper_interface

piper_connect.activate()
robot = piper_interface.PiperInterface(can_port="can0")
piper_init.enable_arm(robot)
while running:
    pos = robot.get_joint_positions()   # returns [0, 0, ...] until data arrives
    ...
```

```python
# After — agilex_control
from agilex_control import agilex_init, agilex_interface, can_utils

can_utils.activate()
arm = agilex_interface.ArmInterface(
    can_port="can0", arm_type=agilex_interface.ArmType.PIPER
)
agilex_init.enable_arm(arm)                     # enable (same retry/timeout as before)
arm.set_joint_limits(min_angles, max_angles)    # arm's limits differ — set yours
while running:
    try:
        pos = arm.get_joint_positions()         # raises until the first CAN frame
    except RuntimeError:
        continue                                # no data this tick; skip it
    ...
```

## API mapping

**Modules and enums**

| `piper_control` | `agilex_control` | Behaviour | Notes |
|---|---|---|---|
| `piper_connect` | `can_utils` | same | CAN discovery/activation. |
| `piper_init` | `agilex_init` | same | `enable_arm` / `disable_arm` / `reset_arm` helpers. |
| `PiperArmType` | `ArmType` | same | Member names and string values unchanged (`ArmType.PIPER == "Piper"`). |
| `PiperGripperType` | `GripperType` | same | Member names and string values unchanged (`GripperType.V1 == "V1 7cm gripper"`). |
| `ArmInstallationPos` | `ArmInstallationPos` | changed | Values are now our own identifiers (`"upright"`/`"left"`/`"right"`), not the old `piper_sdk` int codes. |

**Classes**

| `piper_control` | `agilex_control` | Behaviour | Notes |
|---|---|---|---|
| `piper_interface.PiperInterface` | `agilex_interface.ArmInterface` | same | Connects in `__init__`; auto-detects the firmware and driver. |
| `MitJointPositionController` | `MitJointPositionController` | changed | Now the only controller; `start()` uses `set_mit_mode()`, torque is fed raw. See [Controllers](#controllers). |
| `GravityCompensationModel` | `GravityCompensationModel` | changed | `model_path` is now required. See [Gravity compensation](#gravity-compensation). |
| — | `CalibratedArmInterface` | new | Software calibration offsets. See [Calibration offsets](#calibration-offsets). |
| `JointPositionController` | — | removed | MIT-only now. |
| `BuiltinJointPositionController` | — | removed | MIT-only now. |

**Methods**

| `piper_control` | `agilex_control` | Behaviour | Notes |
|---|---|---|---|
| `get_piper_firmware_version()` | `get_firmware_version()` | same | Still normalized to the PEP440 form (`"1.8.post6"`). |
| `enable_arm(arm, arm_controller, move_mode)` | `enable_arm(arm)` | changed | MIT-only, so the `arm_controller` / `move_mode` args are gone; the retry/timeout is unchanged. |
| `get_joint_positions()` | `get_joint_positions()` | changed | **Raises** if no feedback yet — see [Reads can raise](#reads-can-raise-before-the-first-frame). |
| `get_joint_velocities()` | `get_joint_velocities()` | changed | Raises if no feedback yet. |
| `get_joint_efforts()` | `get_joint_torques()` | changed | Renamed `effort` → `torque`; same N·m on base joints, wrist now carries `c`. See [Joint torque and the `c` coefficient](#joint-torque-and-the-c-coefficient). |
| `joint_torque_coefficients(arm_type)` → `(k, b)` | `get_joint_torque_coefficients()` → `(k, b, c)` | changed | Per-model coefficients; agilex also returns `c` (its feedback decodes `current·k·b·c`). See [Joint torque and the `c` coefficient](#joint-torque-and-the-c-coefficient). |
| `direct_scaling_factors(...)` (function) | `direct_scaling_factors()` (method) | changed | Now a method on the arm (was a module function). Per-joint `base(k·b)/arm(k·b)`, computed for firmware `> 1.8.post2`; the legacy `≤ 1.8.post2` path raises `NotImplementedError`. `c` is applied separately. See [Joint torque and the `c` coefficient](#joint-torque-and-the-c-coefficient). |
| `joint_limits` (property) | `get_joint_limits()` (method) | changed | Now **read from the arm** (cached), not hardcoded per model — a method, not a property. Override with `set_joint_limits(...)`; see [Joint limits](#joint-limits). |
| `set_arm_mode(MIT, MIT)` | `set_mit_mode()` | changed | **MIT-only** (single controller). Also disables `pyAgxArm`'s per-call motion-mode frames. |
| `command_joint_position_mit(...)` | `command_joint_position_mit(...)` | same | `torque_ff` is passed through; scale by `direct_scaling_factors` for non-base arms — see [Joint torque and the `c` coefficient](#joint-torque-and-the-c-coefficient). |
| `command_joint_torque_mit(...)` | `command_joint_torque_mit(...)` | same | Torque goes through the same `c` / scaling treatment — see [Joint torque and the `c` coefficient](#joint-torque-and-the-c-coefficient). |
| `command_gripper(position, effort)` | `command_gripper(position, force)` | changed | `effort` (N·m) → `force` (N) — re-check tuned values. See [Gripper](#gripper). |
| `get_gripper_status()` | `get_gripper_state()` | changed | Returns `(position_m, force_N)`. See [Gripper](#gripper). |

## Reads can raise before the first frame

**What changed:** `get_joint_positions`, `get_joint_velocities`, and
`get_joint_torques` raise `RuntimeError` until the first CAN frame arrives. In the
same situation, `piper_sdk` returned `[0, 0, 0, 0, 0, 0]`.

**What to do:** in a control loop, catch it and skip the tick (matches AgileX's
own demos). It's mainly a startup window — once frames flow, the values are cached.

## Joint torque and the `c` coefficient

**What changed:** `agilex_control` keeps a per-model wrist coefficient `c` that
`piper_control` dropped — on both the read and the command side.

**Reads:** `get_joint_torques` returns `effort · c`:

| source | formula |
|---|---|
| `piper_control.get_joint_efforts` | `current · k · b` |
| `agilex_control.get_joint_torques` | `current · k · b · c` |

The first three joints have `c = 1`; only the wrist differs. On the base Piper,
`c ≈ 0.813` for j4–6; on `piper_h`, it's `0.757` for j4/j5 and `1.287` for j6. So
anything recorded under `piper_control` from wrist efforts — observations, datasets,
gravity-comp calibrations — is in the old frame; re-check it.

**Commands:** `move_mit` divides `t_ff` by `c` and clamps it to the wire limit for
you, so `_clamp_wire_torque` is gone. But the same `t_ff` now maps to a different
torque, so re-check tuned values before copying them across. Non-base arms also need
a per-joint scale (`arm.direct_scaling_factors()`) for gravity comp — see
[Gravity compensation](#gravity-compensation). (Why that scale is `k·b`, not
`k·b·c`, is in the `direct_scaling_factors` docstring.)

## Joint limits

**What changed:** `get_joint_limits()` returns what the *arm* reports (cached on
the first read), not `piper_control`'s hardcoded per-model tables — so the values
can differ from the ones your code assumed.

**What to do:** `agilex_control` doesn't auto-apply piper_control's limits. To keep
specific limits (e.g. piper_control's), call `set_joint_limits(min, max)` right
after connecting; otherwise you get whatever the arm reports.

## Gravity compensation

`GravityCompensationModel` and `predict()` are used as before — two things changed:

- `model_path` is now **required** (no bundled model or `get_default_model_path()`).
- `predict()` returns **raw** physics torque. piper_control baked the command
  scale into the model; now you apply it yourself before commanding:

  ```python
  tau = model.predict(qpos) * np.array(arm.direct_scaling_factors())
  ```

  `direct_scaling_factors()` is 1.0 on base Piper and the per-joint ratio on
  `piper_h` (whose wrist floats without it); non-base arms need firmware
  `> 1.8.post2` (older raises). For backwards compatibility with a
  piper_control-era model (fit without `c`), pass `--compensate-c` to
  `scripts/run_gravity_compensation.py`.

## Controllers

`MitJointPositionController` is the only controller now. Its API is unchanged, but
it's constructed with an `ArmInterface`, `start()` calls `set_mit_mode()`, and
torque is fed raw. Two behaviour changes matter when porting:

- **Gain ranges differ.** `move_mit` takes `kd ∈ [-5, 5]` (piper_control: `[0, 10]`)
  and `kp ∈ [0, 500]`. The controller validates up front and **raises** `ValueError`
  out of range, so ported gains fail fast instead of silently clamping.
- It re-reads `get_joint_limits()` each command, so a `set_joint_limits` after
  construction takes effect.

## Gripper

The gripper changed in several ways, whether you drive it through
`GripperController` or command the arm directly:

- **`effort` → `force`, and N·m → N.** piper_control's `effort` was in N·m; agilex's
  `force` is in Newtons. The default is still `1.0`, but the unit changed — a tuned
  N·m value now means that many Newtons. Rename `effort` → `force` and re-check the
  values.
- **Max opening is read from the arm, not hardcoded.** `get_gripper_max_opening()`
  reflects the actual gripper (0.07 m / 0.10 m) where piper_control hardcoded 0.07,
  so `command_open()` on a 10 cm gripper now opens further.
- **Force clamp changed** from `[0, 2.0]` N·m to `[0, 3.0]` N — a constant, not
  queryable from the arm.
- **No separate enable / reset.** `reset_gripper` / `enable_gripper` are gone; the
  move command enables the gripper implicitly.
- `get_gripper_status()` is now `get_gripper_state()` → `(position_m, force_N)`.
  `disable_gripper()` is kept (WARNING: the gripper goes limp and may drop its load).

## Calibration offsets

`CalibratedArmInterface` (a subclass of `ArmInterface`) ports piper_control's
calibrated interface / `j0_calibration_offset` — a software offset applied on read
and command, now per joint rather than just j0. Readings can shift at power-on or
reset; measuring an offset per joint lets code on the interface see true joint
angles instead of the shifted raw ones.

- `set_calibration_offsets(offsets)` takes one offset per joint: the raw angle a
  joint reports when it's actually at 0 rad. The interface subtracts it, so
  `calibrated = raw − offset`.
- `get_joint_positions()` returns the calibrated frame; pass `raw=True` to bypass.
  `command_joint_position_mit(...)` shifts the target back into the raw frame before
  sending (also with a `raw=True` escape hatch).
- `is_calibrated` and `calibration_offsets` expose the state;
  `clear_calibration_offsets()` drops them.
- Offsets live only in the object and hold until the arm's next power-on or reset,
  so clear them whenever the arm leaves the enabled state. Until they're set, it
  behaves exactly like `ArmInterface` and reports raw angles.

## Deliberately dropped (do not port)

These have no `agilex_control` equivalent.

- The MIT joint flip map and the runtime wire-torque **clamp**
  (`_clamp_wire_torque`) — `pyAgxArm`'s per-firmware driver clamps the wire frame
  itself. (The limit *value* is still available as `mit_wire_torque_limit`, and
  `direct_scaling_factors` is still provided — see
  [Joint torque and the `c` coefficient](#joint-torque-and-the-c-coefficient).)
- Position and cartesian moves (`move_j`, `move_p`, `move_l`, ...) —
  `agilex_control` is MIT-only.
- `reset_gripper` — pyAgxArm's `move_gripper_m` enables the gripper in place, so
  the disable+enable reset cycle isn't needed.
- `collision_checking.py` — not ported.

## Not yet ported

- `set_payload`, `get_flange_pose`, and `get_driver_states` / `foc_status`
  exposure — not on the current production path. (The on-prem monitoring collector
  reads `foc_status` from piper_sdk directly today, so this isn't blocking.)
