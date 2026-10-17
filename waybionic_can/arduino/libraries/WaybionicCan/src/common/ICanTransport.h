#pragma once

#include <stdint.h>

#include "common/Frame.h"

namespace waybionic
{

// Shared by the gateway and the receivers. Implemented by SocketCanTransport on Linux and by
// R4CanTransport (the UNO R4's built-in CAN controller) on the Arduinos.
class ICanTransport
{
public:
  virtual ~ICanTransport() = default;

  // True once the frame is queued on the bus. A CAN ACK only proves that some controller saw a
  // valid frame, never that the addressed node acted: confirm that with its MKS reply.
  virtual bool send(const Frame & frame) = 0;

  // Wait up to timeout_ms (0 polls without blocking) for the next frame; false on timeout.
  virtual bool receive(Frame & frame, uint32_t timeout_ms) = 0;
};

}  // namespace waybionic
