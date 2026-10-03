#pragma once

#include <stdint.h>

#include <string>

#include "common/ICanTransport.h"

namespace waybionic
{

// Linux SocketCAN (vcan0 or can0). Linux headers stay in the .cpp so nothing else sees
// struct can_frame.
class SocketCanTransport : public ICanTransport
{
public:
  SocketCanTransport() = default;
  ~SocketCanTransport() override;
  SocketCanTransport(const SocketCanTransport &) = delete;
  SocketCanTransport & operator=(const SocketCanTransport &) = delete;

  bool open(const std::string & interface_name, std::string & error);
  void close();
  bool isOpen() const {return fd_ >= 0;}

  bool send(const Frame & frame) override;
  // Skips extended, remote and error frames: the MKS protocol only uses standard data frames.
  bool receive(Frame & frame, uint32_t timeout_ms) override;

private:
  int fd_ = -1;
};

}  // namespace waybionic
