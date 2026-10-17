#pragma once

#include <stdint.h>

#include <chrono>

namespace waybionic
{

// Wrapping millisecond counter, like Arduino millis(), so the logic sees the same time type
// on the host and on the boards.
inline uint32_t monotonicMs()
{
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return static_cast<uint32_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
}

}  // namespace waybionic
