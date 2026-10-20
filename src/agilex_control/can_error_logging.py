"""Report CAN failures before the SDK handles them."""

import logging
from collections.abc import Callable

from pyAgxArm.protocols.can_protocol.comms import can_comm, comm_factory


def install(error_handler: Callable[[str, Exception], None]) -> None:
  """Report failures on CAN connections created after this call.

  Call before creating any arms. The handler receives the channel and original
  exception on the sending or receiving thread, so it must be quick and safe
  to call from either thread. Repeated calls replace the handler for new
  connections.
  """

  class _LoggingCanComm(can_comm.CanCommImpl):

    def _classify_can_error(self, exc: Exception) -> int | None:
      try:
        error_handler(self.get_channel(), exc)
      except Exception:  # pylint: disable=broad-exception-caught
        logging.getLogger(__name__).exception("CAN error handler failed")
      return super()._classify_can_error(exc)

  # The SDK's stub omits this registry method.
  factory = comm_factory.CommsFactory
  factory.register_comm(  # pyright: ignore[reportAttributeAccessIssue]
      "can", "impl", _LoggingCanComm
  )
