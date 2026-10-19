# Migrating from `piper_control` to `agilex_control`

`agilex_control` wraps AgileX's `pyAgxArm` SDK; `piper_control` wrapped
`piper_sdk`. This guide maps the API and calls out behavioural differences for
code moving over. It is a living document — start with [Quick
start](#quick-start) for the short path, and see [Not yet
ported](#not-yet-ported) for what's still pending.

Import the submodules the same way as `piper_control`:

```python
from agilex_control import agilex_control, agilex_interface
# agilex_interface.ArmInterface(...), agilex_control.MitJointPositionController(...)
```

## Quick start

Most code moving over needs only these four changes:

1. **Rename imports / classes** — `PiperInterface` → `ArmInterface`,
   `piper_connect` → `can_utils`, and drop the `Piper` enum prefix
   (`PiperArmType` → `ArmType`). See [Naming](#naming).
2. **`get_joint_efforts` → `get_joint_torques`** — same N·m on the base joints,
   but the wrist value now carries `c` (see
   [the `c` coefficient](#joint-torque-and-the-c-coefficient)).
3. **Set joint limits at init** — the arm's reported limits differ from
   piper_control's hardcoded tables, so motions **silently clip** unless you call
   `set_joint_limits(min, max)` right after connecting. This is the one that bites
   quietly (no error), so don't skip it.
4. **Catch `RuntimeError` on reads** — `get_joint_positions` /
   `get_joint_velocities` / `get_joint_torques` raise before the first CAN frame
   (piper_sdk returned zeros); skip the tick in your control loop.

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

The only deltas are the four points above: renamed imports, a robust
`enable_arm`, setting joint limits at init, and catching `RuntimeError` on reads.
Everything else is detail below.

## Naming

| `piper_control` | `agilex_control` | Status | Notes |
|---|---|---|---|
| `piper_interface.PiperInterface` | `agilex_interface.ArmInterface` | same | Still connects in `__init__`. |
| `piper_connect` | `can_utils` | same | CAN discovery/activation. |
| `PiperArmType` | `ArmType` | same | Member names and string values unchanged (`ArmType.PIPER == "Piper"`). |
| `PiperGripperType` | `GripperType` | same | Values unchanged. |
| `ArmInstallationPos` | `ArmInstallationPos` | changed | Kept, but values are now our own identifiers (`"upright"`/`"left"`/`"right"`); the old `piper_sdk` int codes are gone. The interface maps them to `pyAgxArm`'s constants at the call boundary. |

The `Piper` prefix is dropped from the enum class names (the arm models are still
`PIPER`, `PIPER_H`, ...). Enum string values are kept identical so existing
config-string parsing is unaffected.

## Interface methods

Status: *same* = direct port, same behaviour · *changed* = ported but behaviour
differs (read the note). Dropped/planned APIs have their own sections.

| `piper_control` | `agilex_control` | Status | Notes |
|---|---|---|---|
| `get_piper_firmware_version()` | `get_firmware_version()` | same | Still normalized to the PEP440 form (`"1.8.post6"`). |
| `get_joint_positions()` | `get_joint_positions()` | changed | **Raises** if no feedback yet (see below). |
| `get_joint_velocities()` | `get_joint_velocities()` | changed | Raises if no feedback yet. |
| `get_joint_efforts()` | `get_joint_torques()` | changed | Renamed (ROS "effort" → pyAgxArm "torque"); same N·m on base joints, wrist now carries `c`. See [Joint torque and the `c` coefficient](#joint-torque-and-the-c-coefficient). |
| `joint_torque_coefficients(arm_type)` → `(k, b)` | `get_joint_torque_coefficients()` → `(k, b, c)` | changed | Per-model coefficients; agilex also returns `c` (its feedback decodes `current·k·b·c`). |
| `direct_scaling_factors(...)` (function) | `direct_scaling_factors()` (method) | same | **Ported** (modern branch). Per-joint `base(k·b)/arm(k·b)`, same `> 1.8.post2` boundary; the legacy `≤ 1.8.post2` branch raises `NotImplementedError`. The `c` correction is applied separately, not folded in here. See [Joint torque and the `c` coefficient](#joint-torque-and-the-c-coefficient). |
| `set_installation_pos(pos)` | `set_installation_pos(pos)` | same | Maps to `pyAgxArm`'s `OPTIONS.INSTALLATION_POS.*`. |
| `joint_limits` (property) | `get_joint_limits()` (method) | changed | Now **read from the arm** (cached), not hardcoded per model — a method, not a property (first call does a one-time read). New `set_joint_limits(min_angles, max_angles)` overrides them. |
| `set_arm_mode(MIT, MIT)` | `set_mit_mode()` | changed | **MIT-only** (single controller). Also disables `pyAgxArm`'s per-call motion-mode frames. |
| `command_joint_position_mit(...)` | `command_joint_position_mit(...)` | same | Via `move_mit`; `torque_ff` is passed through (apply `direct_scaling_factors` for non-base arms — see below). |
| `command_joint_torque_mit(...)` | `command_joint_torque_mit(...)` | same | Via `move_mit`. |

## Behavioural differences to watch

- **No-data reads raise instead of returning zeros.** `piper_sdk` returned
  `[0, 0, 0, 0, 0, 0]` before the first CAN frame; `pyAgxArm` reports "no data"
  explicitly, so `get_joint_positions`/`get_joint_velocities` raise `RuntimeError`
  until the first frame arrives. In a control loop, catch it and skip the tick
  (matches AgileX's own demos). This is mainly a startup window — once frames
  flow, values are cached.
- **MIT feed-forward torque is in agilex units, not piper's.** `move_mit` divides
  `t_ff` by the per-joint `c`, clamps it to the wire limit (`±8·c`, or `±16·c` on
  firmware ≥ S-V1.8-8 for Piper / ≥ v1.11 for Nero), and warns. So piper_control's
  `_clamp_wire_torque` / `mit_wire_torque_limit` clamp is dropped — the driver
  clamps for you. But **agilex torque ≠ piper effort**: don't copy a tuned value
  across blind. See [the `c` coefficient](#joint-torque-and-the-c-coefficient).
- **MIT gain ranges differ.** `move_mit` accepts `kd ∈ [-5, 5]` (vs
  `piper_control`'s `[0, 10]`) and `kp ∈ [0, 500]`, and **clamps + warns** on
  out-of-range values. The controller additionally validates gains up front and
  **raises** `ValueError` (an early fail, so you don't unknowingly run clamped).
- **The MIT joint flip map is gone.** `pyAgxArm`'s sign conventions match
  `piper_control`'s across all joints/firmwares (no action needed).
- **Firmware-matched driver on connect.** Connecting auto-detects the firmware
  and selects the matching `pyAgxArm` driver (skipping the reconnect when the
  default driver already matches, i.e. firmware `< S-V1.8-3`).
- **Joint limits differ — set them at init.** `get_joint_limits()` reports what
  the *arm* says, which isn't `piper_control`'s hardcoded tables (most joints
  tighter, a few at `±π`, j6 wider). Inheriting them blindly clips most joints and
  over-extends j6. For `piper_control`'s limits, **call `set_joint_limits(min,
  max)` once right after connecting** — `agilex_control` doesn't auto-apply them.

## Joint torque and the `c` coefficient

> **Skip this section unless** you run gravity comp on a **non-base** arm (e.g.
> `piper_h`) and it sags or floats, or you're reusing a model calibrated under
> `piper_control`. Base-Piper gravity comp needs none of it.
>
> **If you do need it**, the fix is one line at the command site: scale the model
> torque by `arm.direct_scaling_factors()` (identity on base Piper, the per-joint
> ratio on `piper_h`). Reusing a `piper_control`-era model? Also pass
> `--compensate-c`. Everything below is the *why*.

Two things affect how a commanded torque reaches the joint: the per-arm `k·b`
coefficients and the `c` coefficient. Both matter for gravity comp on non-base
arms.

### `effort` (piper_control) vs `torque` (agilex)

`piper_control` exposes joint actuation as **`effort`** (`get_joint_efforts`);
`agilex_control`/pyAgxArm expose it as **`torque`** (`get_joint_torques`). The
rename is cosmetic, but the values differ on the wrist:

| | formula |
|---|---|
| `piper_control.get_joint_efforts` | `current · k · b` |
| `agilex_control.get_joint_torques` | `current · k · b · c` |

So `get_joint_torques == effort · c`: identical on base joints (`c = 1`),
different on the wrist — base Piper `c ≈ 0.813` on j4–j6; `piper_h` `c = 0.757`
on j4/j5 and `1.287` on j6. `piper_control` drops `c`; pyAgxArm keeps it.

### Why non-base arms need a command scale

The firmware runs every MIT command at **base-Piper's `k·b`**, whatever arm is
connected (piper_sdk Q&A §3.1: "the motor executes 4× the input", base Piper
`b = 4`). So a non-base arm is off by the fixed per-joint ratio
`base_piper(k·b) / arm(k·b)`:

- Base Piper is the reference — ratio 1.0, deploys raw.
- `piper_h`'s wrist (j4/j5, `b = 1.7`) gets ~1.7× too much torque and floats; its
  j2 gets ~14% too little.

`arm.direct_scaling_factors()` is that ratio — a **port** of piper_control's
function of the same name (`> 1.8.post2` firmware only; raises on legacy). Base
Piper and Nero are identity, so they skip it.

**Why the ratio is `k·b`, not `k·b·c`:** the modern driver divides the command by
`c` and the feedback decodes `c`, so the two `c`s cancel — that's why
`direct_scaling_factors` carries no `c`. The driver doesn't divide by `b`; the
firmware applies it, always at base-Piper's `b`. That matches a base Piper but not
a non-base arm, leaving exactly the `k·b` gap. (Inferred from the Q&A plus
hardware — AgileX doesn't document the split. Sources: pyAgxArm `v183/driver.py`
vs `default/driver.py`, feedback `default/parser.py`, piper_sdk Q&A §3.)

> **Legacy firmware (`≤ 1.8.post2`): not ported.** piper_control handled it (the
> old firmware amplifies commands by the base gear ratio, so it divided the ratio
> by `base_b`). We **raise `NotImplementedError`** instead: no non-base arm runs
> legacy firmware in practice, and it wouldn't port cleanly anyway — pyAgxArm's
> legacy driver already divides by the arm's `b` (`t_ff / b / c`), which
> piper_control's piper_sdk target may not, and we have no legacy hardware to
> test. Implement it only if a non-base arm ever ends up on legacy firmware.

## Gravity compensation

The model is pure physics; the caller applies the command-path corrections. A
deploy loop looks like:

```python
model = gravity_compensation.GravityCompensationModel(model_path, joint_names)
command_scale = np.array(arm.direct_scaling_factors())  # 1.0 on base Piper
while running:
    try:
        qpos = arm.get_joint_positions()
    except RuntimeError:
        continue                                        # no data this tick
    tau = model.predict(qpos) * command_scale           # (+ stability/damping)
    controller.command_torques(tau.tolist())
```

- `GravityCompensationModel(model_path, joint_names=...)` — `model_path` is
  **required** (no bundled model or `get_default_model_path()`); the model is pure
  physics.
- `predict()` returns the gravity torque; scale it by `direct_scaling_factors()`
  before commanding (identity on base Piper; the per-joint `k·b` ratio on
  `piper_h`, whose wrist floats without it).
- **Reusing a piper_control-era model** (fit against `k·b` effort, no `c`): also
  pass **`--compensate-c`**, which multiplies the model torque by `c` to bring it
  into agilex's `k·b·c` frame. A model freshly calibrated through agilex_control
  needs no flag. Why both scales exist: [Joint torque and the `c`
  coefficient](#joint-torque-and-the-c-coefficient).

## Controllers

Same `MitJointPositionController` as before — just hand it an `ArmInterface`:

```python
with agilex_control.MitJointPositionController(
    arm, kp_gains=5.0, kd_gains=0.8
) as controller:
    controller.move_to_position(target, threshold=0.01, timeout=5.0)
    # also: controller.command_joints(target), controller.command_torques(tau)
# leaving the context parks the arm at its rest pose and relaxes it
```

- `MitJointPositionController` moves to `agilex_control.agilex_control` (with
  `ArmOrientation` / `ArmOrientations`). Its constructor takes an `ArmInterface`
  instead of a `PiperInterface`; the API (`start`, `command_joints`,
  `command_torques`, `relax_joints`, `stop`, and the context-manager protocol) is
  unchanged.
- It's the **only** joint controller — the `JointPositionController` ABC and
  `BuiltinJointPositionController` are gone. `start()` calls `set_mit_mode()`
  (not `set_arm_mode(MIT, MIT)`), torque is fed raw, the flip map and wire-torque
  clamp are dropped, and gain validation matches pyAgxArm's ranges (kd ∈ [-5, 5]).
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

- `GripperController` moves to `agilex_control.agilex_control`. Its API is
  unchanged — `command_open`, `command_close`, `command_position`, and the
  context-manager protocol (`start`/`stop` are no-ops, as before). Its
  constructor takes an `ArmInterface`.
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
can shift at power-on or reset; joint zeroing measures an offset per joint so
everything above the interface works in the true joint frame.

- `set_calibration_offsets(offsets)` stores one offset per joint — the raw reading
  at each joint's true 0 rad, so `calibrated = raw − offset`.
- `get_joint_positions()` returns the calibrated frame; pass `raw=True` to bypass.
  `command_joint_position_mit(...)` shifts the target back into the raw frame
  before sending (also with a `raw=True` escape hatch).
- `is_calibrated` and `calibration_offsets` expose the state;
  `clear_calibration_offsets()` drops them.
- The offsets live only in the object and are valid until the arm's next power-on
  or reset, so clear them whenever the arm leaves the enabled state. Until offsets
  are set, it behaves exactly like `ArmInterface` and reports raw angles.

## Deliberately dropped (do not port)

- The MIT joint flip map and the runtime wire-torque **clamp**
  (`_clamp_wire_torque`) — `pyAgxArm`'s per-firmware driver clamps the wire frame
  itself. (`mit_wire_torque_limit` the *value* **is** ported, in `agilex_control`
  for sysid out-of-range rejection; and `direct_scaling_factors` **is** ported —
  see [Joint torque and the `c` coefficient](#joint-torque-and-the-c-coefficient).)
- `BuiltinJointPositionController` and position/cartesian moves (`move_j`,
  `move_p`, ...) — unused in production.
- `reset_gripper` — pyAgxArm's `move_gripper_m` enables the gripper in place, so
  the disable+enable reset cycle isn't needed.
- `collision_checking.py` — unused.

## Not yet ported

- `set_payload`, `get_flange_pose`, and `get_driver_states` / `foc_status`
  exposure — not on the current production path. (The on-prem monitoring collector
  reads `foc_status` from piper_sdk directly today, so this isn't blocking.)

Already landed (in the tables above, or as their own wrappers):
`agilex_init` enable/disable/reset loops, joint zeroing
(`set_joint_zero_positions`), gripper zeroing (`set_gripper_zero_position`),
collision-protection get/set, `clear_joint_errors`, `mit_wire_torque_limit`, and
calibration offsets (`CalibratedArmInterface`, see [Calibration
offsets](#calibration-offsets)).

New in `agilex_control` (no `piper_control` equivalent, so nothing to port):
CAN-error reporting via a callback (`can_error_logging.install(handler)`) —
`piper_control` had no comm-error handling.
