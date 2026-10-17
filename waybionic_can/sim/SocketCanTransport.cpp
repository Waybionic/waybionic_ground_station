#include "sim/SocketCanTransport.h"

#include <errno.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>

namespace waybionic
{
namespace
{

bool toLinux(const Frame & frame, can_frame & out)
{
  if (!isValidFrame(frame)) {
    return false;
  }
  memset(&out, 0, sizeof(out));
  out.can_id = frame.id;
  out.can_dlc = frame.dlc;
  memcpy(out.data, frame.data, frame.dlc);
  return true;
}

bool fromLinux(const can_frame & frame, Frame & out)
{
  if (frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG) || frame.can_dlc > CAN_MAX_DLEN) {
    return false;
  }
  out = Frame{};
  out.id = static_cast<uint16_t>(frame.can_id & CAN_SFF_MASK);
  out.dlc = frame.can_dlc;
  memcpy(out.data, frame.data, frame.can_dlc);
  return true;
}

}  // namespace

SocketCanTransport::~SocketCanTransport()
{
  close();
}

bool SocketCanTransport::open(const std::string & interface_name, std::string & error)
{
  close();
  const unsigned int index = if_nametoindex(interface_name.c_str());
  if (index == 0) {
    error = "no CAN interface " + interface_name + " (run scripts/setup_vcan.sh)";
    return false;
  }
  fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    error = std::string("socket: ") + strerror(errno);
    return false;
  }
  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = static_cast<int>(index);
  if (bind(fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
    error = std::string("bind ") + interface_name + ": " + strerror(errno);
    close();
    return false;
  }
  return true;
}

void SocketCanTransport::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SocketCanTransport::send(const Frame & frame)
{
  can_frame linux_frame;
  if (fd_ < 0 || !toLinux(frame, linux_frame)) {
    return false;
  }
  return write(fd_, &linux_frame, sizeof(linux_frame)) == static_cast<ssize_t>(sizeof(linux_frame));
}

bool SocketCanTransport::receive(Frame & frame, const uint32_t timeout_ms)
{
  if (fd_ < 0) {
    return false;
  }
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (true) {
    const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now()).count();
    pollfd descriptor{fd_, POLLIN, 0};
    const int ready = poll(&descriptor, 1, left > 0 ? static_cast<int>(left) : 0);
    if (ready <= 0) {  // timeout, or EINTR from a shutdown signal
      return false;
    }
    can_frame linux_frame;
    if (read(fd_, &linux_frame, sizeof(linux_frame)) == static_cast<ssize_t>(sizeof(linux_frame)) &&
      fromLinux(linux_frame, frame))
    {
      return true;
    }
    if (left <= 0) {
      return false;
    }
  }
}

}  // namespace waybionic
