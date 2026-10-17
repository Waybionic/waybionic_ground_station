#pragma once

#include <stdint.h>

namespace waybionic
{

// Axis values are MKS encoder counts (0x4000 per turn), as sent in F5h frames.
struct ActuatorState
{
  bool enabled = false;
  bool moving = false;
  int32_t target = 0;
  int32_t position = 0;
};

// What NodeLogic drives. SoftwareActuator implements it for host tests and the bench receivers;
// a physical actuator (for example a hobby servo) can implement it later.
class IActuator
{
public:
  virtual ~IActuator() = default;

  virtual void setEnabled(bool on) = 0;
  // Starting a new move while one is running replaces its target (mks_can.absolute_axis).
  virtual void moveTo(int32_t axis, uint16_t speed_rpm, uint8_t acc, uint32_t now_ms) = 0;
  virtual void stop(uint32_t now_ms) = 0;
  // Called every loop so the actuator can finish moves; must not block.
  virtual void update(uint32_t now_ms) = 0;
  virtual ActuatorState state() const = 0;
};

}  // namespace waybionic
