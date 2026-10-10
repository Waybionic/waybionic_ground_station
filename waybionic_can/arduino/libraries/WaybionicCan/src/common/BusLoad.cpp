#include "common/BusLoad.h"

#include <stdio.h>

namespace waybionic
{

BusLoad::BusLoad(const uint32_t window_ms)
: window_ms_(window_ms == 0 ? 1 : window_ms)
{
}

void BusLoad::reset(const uint32_t bitrate, const uint32_t now_ms)
{
  bitrate_ = bitrate;
  window_start_ms_ = now_ms;
  current_ = Window{};
  last_ = Window{};
  peak_max_permille_ = 0;
  total_frames_ = 0;
}

uint32_t BusLoad::permille(const uint32_t bits, const uint32_t duration_ms) const
{
  if (bitrate_ == 0 || duration_ms == 0) {
    return 0;
  }
  return static_cast<uint32_t>(
    static_cast<uint64_t>(bits) * 1000000u / (static_cast<uint64_t>(bitrate_) * duration_ms));
}

void BusLoad::update(const uint32_t now_ms)
{
  const uint32_t elapsed = now_ms - window_start_ms_;
  if (elapsed < window_ms_) {
    return;
  }
  // After an idle gap the closed window covers the whole gap, so it reports the average.
  last_ = current_;
  last_.duration_ms = elapsed;
  const uint32_t max_permille = lastMaxPermille();
  if (max_permille > peak_max_permille_) {
    peak_max_permille_ = max_permille;
  }
  current_ = Window{};
  window_start_ms_ = now_ms;
}

void BusLoad::note(const Frame & frame, const uint32_t now_ms)
{
  update(now_ms);
  const uint8_t dlc = frame.dlc > kMaxCanDataBytes ? kMaxCanDataBytes : frame.dlc;
  ++current_.frames;
  current_.min_bits += frameBitsMin(dlc);
  current_.max_bits += frameBitsMax(dlc);
  ++total_frames_;
}

void BusLoad::format(const uint32_t now_ms, char * out, const size_t out_size)
{
  update(now_ms);
  if (!hasWindow()) {
    snprintf(out, out_size, "bus load: no %lu ms window closed yet; %lu frames total",
      static_cast<unsigned long>(window_ms_), static_cast<unsigned long>(total_frames_));
    return;
  }
  const uint32_t low = lastMinPermille();
  const uint32_t high = lastMaxPermille();
  snprintf(out, out_size,
    "bus load last %lu ms: %lu frames, %lu.%lu-%lu.%lu %% of %lu bit/s; peak %lu.%lu %%; "
    "%lu frames total",
    static_cast<unsigned long>(last_.duration_ms), static_cast<unsigned long>(last_.frames),
    static_cast<unsigned long>(low / 10), static_cast<unsigned long>(low % 10),
    static_cast<unsigned long>(high / 10), static_cast<unsigned long>(high % 10),
    static_cast<unsigned long>(bitrate_),
    static_cast<unsigned long>(peak_max_permille_ / 10),
    static_cast<unsigned long>(peak_max_permille_ % 10),
    static_cast<unsigned long>(total_frames_));
}

}  // namespace waybionic
