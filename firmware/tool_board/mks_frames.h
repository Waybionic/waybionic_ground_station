// MKS SERVO42D/57D CAN frames used by the tool board (CAN user manual V1.0.9 subset).
// Plain C++ with no Arduino headers, so the frame code can be checked on a computer against
// waybionic_teleop/mks_can.py (see waybionic_teleop/test/test_tool_board_frames.py).

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace mks
{

constexpr uint8_t READ_ENCODER = 0x31;
constexpr uint8_t SET_MODE = 0x82;
constexpr uint8_t SET_RESPONSE = 0x8C;
constexpr uint8_t SET_ZERO = 0x92;
constexpr uint8_t SET_HEARTBEAT = 0x98;
constexpr uint8_t ENABLE = 0xF3;
constexpr uint8_t ABSOLUTE_AXIS = 0xF5;

constexpr uint8_t MODE_SR_VFOC = 0x05;
constexpr int32_t COUNTS_PER_REV = 0x4000;

// F5h replies: the move started, finished, or stopped at a limit.
constexpr uint8_t MOVE_FAILED = 0;
constexpr uint8_t MOVE_RUNNING = 1;
constexpr uint8_t MOVE_COMPLETE = 2;
constexpr uint8_t MOVE_AT_LIMIT = 3;

struct Move
{
  uint16_t rpm;
  uint8_t acc;
  int32_t axis;
};

// The manual's CHECKSUM 8bit: the low byte of the ID plus every data byte.
inline uint8_t checksum(uint16_t id, const uint8_t * bytes, size_t length)
{
  unsigned sum = id;
  for (size_t i = 0; i < length; ++i) {
    sum += bytes[i];
  }
  return static_cast<uint8_t>(sum & 0xFF);
}

// A frame is a function code, its arguments and a checksum.
inline bool valid(uint16_t id, const uint8_t * data, size_t length)
{
  return length >= 2 && length <= 8 && data[length - 1] == checksum(id, data, length - 1);
}

inline uint32_t u32(const uint8_t * bytes)
{
  return (static_cast<uint32_t>(bytes[0]) << 24) | (static_cast<uint32_t>(bytes[1]) << 16) |
         (static_cast<uint32_t>(bytes[2]) << 8) | bytes[3];
}

// F5h arguments: speed in rpm (2 bytes), acceleration (1 byte), axis (signed 3 bytes).
inline Move absolute_axis(const uint8_t * arguments)
{
  int32_t axis = (static_cast<int32_t>(arguments[3]) << 16) |
    (static_cast<int32_t>(arguments[4]) << 8) | arguments[5];
  if (axis & 0x800000) {
    axis -= 0x1000000;
  }
  return {static_cast<uint16_t>((arguments[0] << 8) | arguments[1]), arguments[2], axis};
}

// Writes code, arguments and checksum to out (8 bytes) and returns the frame length.
inline uint8_t reply(
  uint16_t id, uint8_t code, const uint8_t * arguments, uint8_t count, uint8_t * out)
{
  out[0] = code;
  for (uint8_t i = 0; i < count; ++i) {
    out[1 + i] = arguments[i];
  }
  out[1 + count] = checksum(id, out, 1 + count);
  return count + 2;
}

inline uint8_t status_reply(uint16_t id, uint8_t code, uint8_t status, uint8_t * out)
{
  return reply(id, code, &status, 1, out);
}

// 31h reply: the cumulative encoder value as a signed 48-bit number.
inline uint8_t encoder_reply(uint16_t id, int64_t value, uint8_t * out)
{
  uint8_t bytes[6];
  for (int i = 0; i < 6; ++i) {
    bytes[i] = static_cast<uint8_t>(static_cast<uint64_t>(value) >> (8 * (5 - i)));
  }
  return reply(id, READ_ENCODER, bytes, 6, out);
}

}  // namespace mks
