#pragma once

#include <stdint.h>

namespace waybionic
{

constexpr uint16_t kMaxStandardCanId = 0x7FF;
constexpr uint8_t kMaxCanDataBytes = 8;

// Neutral classic CAN data frame. Transports convert to and from their own types
// (Linux struct can_frame, the UNO R4 Arduino_CAN CanMsg) at the boundary.
struct Frame
{
  uint16_t id = 0;  // 11-bit standard identifier
  uint8_t dlc = 0;
  uint8_t data[kMaxCanDataBytes] = {};
};

inline bool isValidFrame(const Frame & frame)
{
  return frame.id <= kMaxStandardCanId && frame.dlc <= kMaxCanDataBytes;
}

}  // namespace waybionic
