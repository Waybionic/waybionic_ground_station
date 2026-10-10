#pragma once

#include <stdint.h>

#include "common/IActuator.h"

namespace waybionic
{

// Application-state stand-in for a servo: it records what it was told and reports a move as
// finished move_duration_ms later, jumping straight to the target. No motion is modelled.
class SoftwareActuator : public IActuator
{
public:
  explicit SoftwareActuator(uint32_t move_duration_ms = 0);

  void setEnabled(bool on) override;
  void moveTo(int32_t axis, uint16_t speed_rpm, uint8_t acc, uint32_t now_ms) override;
  void stop(uint32_t now_ms) override;
  void update(uint32_t now_ms) override;
  ActuatorState state() const override {return state_;}

  uint16_t speedRpm() const {return speed_rpm_;}
  uint8_t acc() const {return acc_;}
  uint32_t lastCommandMs() const {return last_command_ms_;}
  uint32_t moveCount() const {return moves_;}
  uint32_t stopCount() const {return stops_;}

private:
  uint32_t move_duration_ms_;
  ActuatorState state_;
  uint16_t speed_rpm_ = 0;
  uint8_t acc_ = 0;
  uint32_t move_started_ms_ = 0;
  uint32_t last_command_ms_ = 0;
  uint32_t moves_ = 0;
  uint32_t stops_ = 0;
};

}  // namespace waybionic
