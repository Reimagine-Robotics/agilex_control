# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Initial port over from `piper_control`.
- `CalibratedArmInterface`: per-joint software calibration offsets applied on read
  and command (`calibrated = raw − offset`).
- CAN-error reporting via a callback (`can_error_logging.install(handler)`).
- `get_timestamped_joint_positions()`: joint positions together with the CAN
  receive time of that feedback, read from one message so the two always match.
