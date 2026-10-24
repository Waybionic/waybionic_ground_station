#pragma once

#include <deque>
#include <memory>

#include "common/ICanTransport.h"

namespace waybionic
{

// In-process broadcast bus: every attached port receives every frame except its own, like
// separate SocketCAN sockets on one vcan interface.
class MemoryBus
{
public:
  class Port : public ICanTransport
  {
public:
    explicit Port(MemoryBus & bus)
    : bus_(bus) {}

    bool send(const Frame & frame) override
    {
      bus_.deliver(this, frame);
      return true;
    }

    bool receive(Frame & frame, uint32_t) override
    {
      if (inbox_.empty()) {
        return false;
      }
      frame = inbox_.front();
      inbox_.pop_front();
      return true;
    }

    bool connected = true;

private:
    friend class MemoryBus;
    MemoryBus & bus_;
    std::deque<Frame> inbox_;
  };

  Port & attach()
  {
    ports_.push_back(std::make_unique<Port>(*this));
    return *ports_.back();
  }

private:
  void deliver(const Port * from, const Frame & frame)
  {
    for (const auto & port : ports_) {
      if (port.get() != from && port->connected) {
        port->inbox_.push_back(frame);
      }
    }
  }

  std::deque<std::unique_ptr<Port>> ports_;
};

}  // namespace waybionic
