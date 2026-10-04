#pragma once

#include <stdint.h>

namespace waybionic
{

// Bench signals the task asks the carrier to report. Electrical has not specified how the
// emergency-stop circuit or the 24 V motor supply reach the Arduino (no pins, divider or
// isolation), so nothing is sampled yet and both always read as unknown.
//
// TODO(hardware-TBD, Yassin/Electrical): once the E-stop sense contact and the supply sense
// circuit are specified, implement readBenchTelemetry() for the board. Never infer the E-stop
// state from CAN traffic: the physical E-stop must work with this code absent.
enum class EstopState : uint8_t
{
  kUnknown,   // not wired / not specified
  kReleased,  // circuit closed, motion permitted by hardware
  kPressed,   // circuit open, drives de-energised by hardware
};

struct BenchTelemetry
{
  EstopState estop = EstopState::kUnknown;
  bool supply_known = false;
  uint32_t supply_millivolts = 0;
};

inline BenchTelemetry readBenchTelemetry()
{
  return BenchTelemetry{};
}

inline const char * estopStateName(const EstopState state)
{
  switch (state) {
    case EstopState::kUnknown: return "UNKNOWN (hardware-TBD)";
    case EstopState::kReleased: return "released";
    case EstopState::kPressed: return "PRESSED";
  }
  return "?";
}

}  // namespace waybionic
