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
   arm reports, not piper_control's hardcoded tables, so they may differ from the
   ones your code assumed. Nothing errors if you skip this; if you rely on specific
   limits, call `set_joint_limits(min, max)` right after connecting — see
   [Joint limits](#joint-limits).
4. **Catch `RuntimeError` on reads** — `get_joint_positions` /
   `get_joint_velocities` / `get_joint_torques` raise before the first CAN frame
   (piper_sdk returned zeros); skip the tick in your control loop. See
   [Reads can raise](#reads-can-raise-before-the-first-frame).

Here is the same minimal control loop before and after:

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
agilex_init.enable_arm(arm)                     # robust enable (retries + timeout)
arm.set_joint_limits(min_angles, max_angles)    # arm's limits differ — set yours
while running:
    try:
        pos = arm.get_joint_positions()         # raises until the first CAN frame
    except RuntimeError:
        continue                                # no data this tick; skip it
    ...
```

Those four numbered points are the only required changes. The rest of the guide
opens with the API-mapping tables — a scannable **reference layer** for looking up
what to rename or call now — then gives one short section per changed area, each
listing only what changed from `piper_control`; anything not mentioned behaves the
same.

## API mapping

Each table maps an old name to its new one. The **Behaviour** column is **same**
(works as before), **changed** (behaves differently — the matching section below
explains how), **new** (no `piper_control` equivalent), or **removed**. Other
dropped APIs and not-yet-ported ones are listed at the end.

**Modules and enums**

| `piper_control` | `agilex_control` | Behaviour | Notes |
|---|---|---|---|
| `piper_connect` | `can_utils` | same | CAN discovery/activation. |
| `piper_init` | `agilex_init` | changed | `enable_arm` / `disable_arm` / `reset_arm`; `enable_arm` now retries with a timeout. |
| `PiperArmType` | `ArmType` | same | Member names and string values unchanged (`ArmType.PIPER == "Piper"`). |
| `PiperGripperType` | `GripperType` | same | Values unchanged. |
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
| `get_joint_positions()` | `get_joint_positions()` | changed | **Raises** if no feedback yet — see [Reads can raise](#reads-can-raise-before-the-first-frame). |
| `get_joint_velocities()` | `get_joint_velocities()` | changed | Raises if no feedback yet. |
| `get_joint_efforts()` | `get_joint_torques()` | changed | Renamed `effort` → `torque`; same N·m on base joints, wrist now carries `c`. See [Joint torque and the `c` coefficient](#joint-torque-and-the-c-coefficient). |
| `joint_torque_coefficients(arm_type)` → `(k, b)` | `get_joint_torque_coefficients()` → `(k, b, c)` | changed | Per-model coefficients; agilex also returns `c` (its feedback decodes `current·k·b·c`). See [Joint torque and the `c` coefficient](#joint-torque-and-the-c-coefficient). |
| `direct_scaling_factors(...)` (function) | `direct_scaling_factors()` (method) | changed | Now a method on the arm (was a module function). Per-joint `base(k·b)/arm(k·b)`, computed for firmware `> 1.8.post2`; the legacy `≤ 1.8.post2` path raises `NotImplementedError`. `c` is applied separately, not folded in. See [Joint torque and the `c` coefficient](#joint-torque-and-the-c-coefficient). |
| `set_installation_pos(pos)` | `set_installation_pos(pos)` | same | Maps to `pyAgxArm`'s `OPTIONS.INSTALLATION_POS.*`. |
| `joint_limits` (property) | `get_joint_limits()` (method) | changed | Now **read from the arm** (cached), not hardcoded per model — a method, not a property. Override with `set_joint_limits(...)`; see [Joint limits](#joint-limits). |
| `set_arm_mode(MIT, MIT)` | `set_mit_mode()` | changed | **MIT-only** (single controller). Also disables `pyAgxArm`'s per-call motion-mode frames. |
| `command_joint_position_mit(...)` | `command_joint_position_mit(...)` | same | Via `move_mit`; `torque_ff` is passed through (scale by `direct_scaling_factors` for non-base arms — see [Joint torque and the `c` coefficient](#joint-torque-and-the-c-coefficient)). |
| `command_joint_torque_mit(...)` | `command_joint_torque_mit(...)` | same | Via `move_mit`. |
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

The first three joints all have `c = 1`; only the wrist differs. On the base
Piper, `c ≈ 0.813` for j4–6; on `piper_h`, it's `0.757` for j4/j5 and `1.287` for
j6. So anything recorded under `piper_control` that used wrist efforts —
observations, datasets, gravity-comp calibrations — is in the old frame; re-check
it.

**Commands:** `move_mit` divides `t_ff` by `c` and clamps it to the wire limit for
you, so piper_control's `_clamp_wire_torque` is no longer needed. But the same
`t_ff` now maps to a different torque than it did under piper_control, so don't
copy a tuned value across without re-checking it. Non-base arms additionally need a
per-joint scale (`arm.direct_scaling_factors()`) for gravity comp — see
[Gravity compensation](#gravity-compensation). (Why that scale is `k·b` and not
`k·b·c` lives in the `direct_scaling_factors` docstring.)

## Joint limits

**What changed:** `get_joint_limits()` returns what the *arm* reports (cached on
the first read), not `piper_control`'s hardcoded per-model tables — so the values
can differ from the ones your code assumed.

**What to do:** `agilex_control` doesn't auto-apply piper_control's limits. If you
want specific limits — for example to keep piper_control's — call
`set_joint_limits(min, max)` once right after connecting; otherwise you get
whatever the arm reports.

## Gravity compensation

`GravityCompensationModel` and `predict()` are used as before — two things changed:

- `model_path` is now **required** (no bundled model or `get_default_model_path()`).
- `predict()` returns **raw** physics torque. piper_control baked the command
  scale into the model; now you apply it yourself before commanding:

  ```python
  tau = model.predict(qpos) * np.array(arm.direct_scaling_factors())
  ```

  `direct_scaling_factors()` is 1.0 on base Piper and the per-joint ratio on
  `piper_h` (whose wrist floats without it); non-base arms need firmware newer than
  1.8.post2, and on anything older `direct_scaling_factors()` raises. Reusing a
  piper_control-era model (fit without `c`)? Pass `--compensate-c` to the
  gravity-compensation script.

## Controllers

`MitJointPositionController` works much as before — construct it with an
`ArmInterface`:

```python
with agilex_control.MitJointPositionController(
    arm, kp_gains=5.0, kd_gains=0.8
) as controller:
    controller.move_to_position(target, threshold=0.01, timeout=5.0)
    # also: controller.command_joints(target), controller.command_torques(tau)
# leaving the context parks the arm at its rest pose and relaxes it
```

- Its constructor takes an `ArmInterface` (not a `PiperInterface`); otherwise the
  API — `start`, `move_to_position`, `command_joints`, `command_torques`,
  `relax_joints`, `stop`, and the context-manager — is unchanged.
- `start()` now calls `set_mit_mode()` instead of `set_arm_mode(MIT, MIT)`, and
  torque is fed raw.
- **Gain ranges differ.** `move_mit` takes `kd ∈ [-5, 5]` (vs piper_control's
  `[0, 10]`) and `kp ∈ [0, 500]`, clamping and warning out of range; the controller
  also validates up front and **raises** `ValueError`, so you never unknowingly run
  clamped.
- It re-reads `get_joint_limits()` each command (interface-cached) for clipping,
  so a `set_joint_limits` after construction is reflected. Set the limits before
  commanding if you want values other than what the arm reports.
- `relax_joints` (used by `stop()`) copes with the new feedback gaps. Through a
  dropout it reuses the last *measured* position; if a position was never measured,
  it relaxes with `kp=0` (velocity damping only, so the arm goes limp without
  lunging toward a guessed pose) before disabling. This is **additive**: with
  feedback flowing it behaves exactly as piper_control did. It only exists because
  `get_joint_positions()` can now raise (piper_sdk always returned zeros).

## Gripper

Drive it through the controller, or command the arm directly:

```python
with agilex_control.GripperController(arm) as gripper:
    gripper.command_open()
    gripper.command_close()

arm.command_gripper(position=0.04, force=1.0)  # metres, newtons (not N·m!)
```

- `GripperController`'s API is unchanged — `command_open`, `command_close`,
  `command_position`, and the context-manager protocol (`start`/`stop` are no-ops,
  as before). Its constructor takes an `ArmInterface`.
- **`effort` → `force`, and N·m → N.** `piper_control` used `effort` in **N·m**
  (`command_gripper(position, effort)`, `DEFAULT_GRIPPER_EFFORT = 1.0`). AgileX
  uses **`force` in Newtons** (`command_gripper(position, force)`,
  `DEFAULT_GRIPPER_FORCE = 1.0`). The default is coincidentally still `1.0`, but
  the **unit changed** — a ported call that tuned a specific N·m effort now means
  that many Newtons. Rename `effort` → `force` and re-check the values.
- **Max opening is read from the arm, not hardcoded.** `piper_control` hardcoded
  `_GRIPPER_ANGLE_MAX = 0.07`; `agilex_control` reads it via
  `get_gripper_max_opening()` (pyAgxArm's `max_range_config`, cached like
  `get_joint_limits`), so it reflects the actual gripper (0.07 for the 7 cm, 0.10
  for the 10 cm). `command_open()` opens to this value; `command_position` clips
  to `[0, max]`.
- **Force is clamped to a different limit.** `piper_control` clamped effort to
  `[0, 2.0]` N·m; `agilex_control` clamps force to `[0, 3.0]` N (AgileX's
  documented gripper force range, `_GRIPPER_FORCE_MAX`). Unlike the max opening,
  this is **not** queryable from the arm, so it is a constant.
- **No separate enable / reset.** `piper_control` needed `reset_gripper` /
  `enable_gripper` before commanding; `agilex_control` does not — pyAgxArm's move
  command enables the gripper implicitly. `command_gripper(position=None,
  force=None)` still means "leave that field unchanged" (now implemented by
  reading the current gripper state).
- `disable_gripper()` is kept (WARNING: the gripper goes limp and may drop its
  load). `get_gripper_status()` becomes `get_gripper_state()` returning
  `(position_m, force_N)`. `init_effector` is called **before** `connect()` (per
  pyAgxArm's effector docs) and is safe even with no gripper attached.

## Calibration offsets

`CalibratedArmInterface` (a subclass of `ArmInterface`) is the port of
piper_control's calibrated interface / `j0_calibration_offset` — a software offset
applied on read and command, now per joint rather than just j0. An arm's readings
can shift at power-on or reset; joint zeroing measures an offset per joint, so any
code built on the interface sees true joint angles rather than the shifted raw ones.

- `set_calibration_offsets(offsets)` takes one offset per joint: the raw angle a
  joint reports when it's actually at 0 rad. The interface subtracts that offset
  from every reading, so `calibrated = raw − offset`.
- `get_joint_positions()` returns the calibrated frame; pass `raw=True` to bypass.
  `command_joint_position_mit(...)` shifts the target back into the raw frame
  before sending (also with a `raw=True` escape hatch).
- `is_calibrated` and `calibration_offsets` expose the state;
  `clear_calibration_offsets()` drops them.
- The offsets live only in the object and are valid until the arm's next power-on
  or reset, so clear them whenever the arm leaves the enabled state. Until offsets
  are set, it behaves exactly like `ArmInterface` and reports raw angles.

## Deliberately dropped (do not port)

These have no `agilex_control` equivalent. (The `removed` controllers in the tables
above — `JointPositionController` / `BuiltinJointPositionController` — are part of
the same MIT-only consolidation.)

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
