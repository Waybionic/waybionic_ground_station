#pragma once

#include <stddef.h>
#include <stdint.h>

#include "common/Frame.h"

namespace waybionic
{

// Bus-load estimate from the frames one node sees: the ones it sent and the ones it received.
// Error frames, retransmissions and frames the controller dropped are invisible to it, so the
// result is a lower bound on the real load. Bit counts are for classic CAN data frames with
// 11-bit IDs, including the 3-bit interframe space.
class BusLoad
{
public:
  explicit BusLoad(uint32_t window_ms = 1000);

  void reset(uint32_t bitrate, uint32_t now_ms);
  void note(const Frame & frame, uint32_t now_ms);
  // Closes the current window once window_ms has passed. note() and format() call it too.
  void update(uint32_t now_ms);

  bool hasWindow() const {return last_.duration_ms != 0;}
  uint32_t lastFrames() const {return last_.frames;}
  uint32_t lastDurationMs() const {return last_.duration_ms;}
  // Load in tenths of a percent over the last closed window: without and with worst-case bit
  // stuffing.
  uint32_t lastMinPermille() const {return permille(last_.min_bits, last_.duration_ms);}
  uint32_t lastMaxPermille() const {return permille(last_.max_bits, last_.duration_ms);}
  uint32_t peakMaxPermille() const {return peak_max_permille_;}
  uint32_t totalFrames() const {return total_frames_;}

  // One log line, for example
  // "bus load last 1000 ms: 12 frames, 0.2-0.3 % of 500000 bit/s; peak 0.4 %; 345 frames total"
  void format(uint32_t now_ms, char * out, size_t out_size);

  static uint32_t frameBitsMin(uint8_t dlc) {return 47u + 8u * dlc;}
  static uint32_t frameBitsMax(uint8_t dlc) {return frameBitsMin(dlc) + (33u + 8u * dlc) / 4u;}

private:
  struct Window
  {
    uint32_t frames = 0;
    uint32_t min_bits = 0;
    uint32_t max_bits = 0;
    uint32_t duration_ms = 0;
  };

  uint32_t permille(uint32_t bits, uint32_t duration_ms) const;

  uint32_t window_ms_;
  uint32_t bitrate_ = 0;
  uint32_t window_start_ms_ = 0;
  Window current_;
  Window last_;
  uint32_t peak_max_permille_ = 0;
  uint32_t total_frames_ = 0;
};

}  // namespace waybionic
