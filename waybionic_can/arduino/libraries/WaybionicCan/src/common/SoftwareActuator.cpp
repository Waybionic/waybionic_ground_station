#include "common/SoftwareActuator.h"

namespace waybionic
{

SoftwareActuator::SoftwareActuator(const uint32_t move_duration_ms)
: move_duration_ms_(move_duration_ms)
{
}

void SoftwareActuator::setEnabled(const bool on)
{
  state_.enabled = on;
  if (!on) {
    state_.moving = false;
  }
}

void SoftwareActuator::moveTo(
  const int32_t axis, const uint16_t speed_rpm, const uint8_t acc, const uint32_t now_ms)
{
  if (!state_.enabled) {
    return;
  }
  state_.target = axis;
  state_.moving = true;
  speed_rpm_ = speed_rpm;
  acc_ = acc;
  move_started_ms_ = now_ms;
  last_command_ms_ = now_ms;
  ++moves_;
}

void SoftwareActuator::stop(const uint32_t now_ms)
{
  // Position stays where it was: without a motion model there is no partial travel.
  state_.moving = false;
  last_command_ms_ = now_ms;
  ++stops_;
}

void SoftwareActuator::update(const uint32_t now_ms)
{
  if (state_.moving && now_ms - move_started_ms_ >= move_duration_ms_) {
    state_.position = state_.target;
    state_.moving = false;
  }
}

}  // namespace waybionic
