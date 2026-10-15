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

## CAN errors

Install a handler before creating any arms to report CAN send and receive
failures, including errors the SDK recovers from:

```python
import logging

from agilex_control import can_error_logging

logger = logging.getLogger(__name__)


def log_can_error(channel: str, error: Exception) -> None:
  logger.error("CAN I/O failed on %s: %r", channel, error)


can_error_logging.install(log_can_error)
```

This applies to all new CAN connections in the process. The handler runs on
the sending or receiving thread and must be quick and safe to call from either.
The SDK still decides whether to recover or disconnect.

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
