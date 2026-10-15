"""CAN error reporting preserves the SDK's recovery behaviour."""

import errno
from collections.abc import Iterator
from unittest import mock

import can
import pytest
from pyAgxArm.protocols.can_protocol.comms import can_comm, comm_factory

from agilex_control import can_error_logging


@pytest.fixture(name="handler")
def handler_fixture() -> mock.Mock:
  return mock.Mock()


@pytest.fixture(name="comm")
def comm_fixture(handler: mock.Mock) -> Iterator[can_comm.CanCommImpl]:
  factory = comm_factory.CommsFactory
  original = factory.load_class(  # pyright: ignore[reportAttributeAccessIssue]
      "can", "impl"
  )
  try:
    can_error_logging.install(handler)
    can_error_logging.install(handler)
    comm = factory.create_comm(
        "can",
        "impl",
        config=can_comm.create_can_comm_config(
            channel="can7", auto_connect=False, enable_check_can=False
        ),
    )
    assert isinstance(comm, can_comm.CanCommImpl)
    comm.send_bus = comm.recv_bus = mock.Mock()
    try:
      yield comm
    finally:
      comm.close()
  finally:
    factory.register_comm(  # pyright: ignore[reportAttributeAccessIssue]
        "can", "impl", original
    )


@pytest.mark.parametrize(
    "operation,error,recoverable",
    [
        ("send", can.CanOperationError("Buffer full", errno.ENOBUFS), True),
        ("recv", OSError(errno.ENETDOWN, "Network is down"), True),
        ("send", OSError(errno.ENODEV, "No such device"), False),
        ("recv", RuntimeError("Unexpected failure"), False),
    ],
)
def test_error_reporting_preserves_recovery(
    comm: can_comm.CanCommImpl,
    handler: mock.Mock,
    operation: str,
    error: Exception,
    recoverable: bool,
) -> None:
  bus = comm.send_bus
  assert isinstance(bus, mock.Mock)
  getattr(bus, operation).side_effect = error
  call = getattr(comm, operation)
  args = (can.Message(),) if operation == "send" else ()

  if recoverable:
    call(*args)
    bus.shutdown.assert_not_called()
  else:
    with pytest.raises(type(error)) as caught:
      call(*args)
    assert caught.value is error
    bus.shutdown.assert_called()
    assert comm.send_bus is comm.recv_bus is None
  handler.assert_called_once_with("can7", error)
  assert comm.last_error is error


def test_successful_io_is_quiet(
    comm: can_comm.CanCommImpl, handler: mock.Mock
) -> None:
  bus = comm.recv_bus
  assert isinstance(bus, mock.Mock)
  message = can.Message()
  bus.recv.return_value = message
  comm.send(message)
  assert comm.recv() is message
  handler.assert_not_called()


def test_handler_failure_does_not_interrupt_recovery(
    comm: can_comm.CanCommImpl,
    handler: mock.Mock,
    caplog: pytest.LogCaptureFixture,
) -> None:
  bus = comm.recv_bus
  assert isinstance(bus, mock.Mock)
  error = OSError(errno.ENETDOWN, "Network is down")
  bus.recv.side_effect = error
  handler.side_effect = RuntimeError("Cannot log")

  comm.recv()

  assert comm.last_error is error
  bus.shutdown.assert_not_called()
  assert "CAN error handler failed" in caplog.text
