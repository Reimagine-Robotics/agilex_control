# agilex_control - Library for controlling AgileX arms

## Overview

This repo provides a lightweight wrapper around
[`pyAgxArm`](https://github.com/agilexrobotics/pyAgxArm), AgileX's Python SDK,
for connecting to and controlling AgileX arms (Piper and Nero series).

`pyAgxArm` handles the low-level CAN protocol, per-firmware message encoding,
and SI-unit conversion. `agilex_control` adds a simple, stable abstraction on
top: typed feedback, robust enable/disable/reset lifecycle handling, and the
higher-level controllers our stack depends on.

It is the successor to
[`piper_control`](https://github.com/Reimagine-Robotics/piper_control), which
wrapped the older `piper_sdk`.

## Local development setup

This project uses [`uv`](https://github.com/astral-sh/uv).

```shell
# Create/refresh the .venv and install the project with dev tools.
uv sync --all-extras --dev
```

`can-utils` is required for CAN bus communication on Linux:

```shell
sudo apt install can-utils
```

## Linting

```shell
uv run pre-commit run --all-files
```
