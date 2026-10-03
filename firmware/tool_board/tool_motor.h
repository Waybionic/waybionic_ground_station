// One tool motor: the MKS commands for its CAN ID, applied to a stepper driver.
// Plain C++ templated on the stepper (AccelStepper on the board, a fake on a computer), so the
// command handling, travel limits and heartbeat stop are tested in waybionic_teleop's tests.

#pragma once

#include <math.h>
#include <stdint.h>

#include "mks_frames.h"

struct MotorConfig
{
  long steps_per_rev;
  long min_steps;
  long max_steps;
  float max_steps_per_s;
  float max_steps_per_s2;
};

template<typename Stepper>
class ToolMotor
{
public:
  ToolMotor(uint16_t id, Stepper & stepper, const MotorConfig & config)
  : id_(id), stepper_(stepper), config_(config)
  {
  }

  uint16_t id() const {return id_;}
  bool enabled() const {return enabled_;}

  // Handle one frame for this motor; write any reply to out (8 bytes) and return its length.
  uint8_t handle(const uint8_t * data, uint8_t length, uint32_t now_ms, uint8_t * out)
  {
    if (!mks::valid(id_, data, length)) {
      return 0;
    }
    last_frame_ms_ = now_ms;
    const uint8_t code = data[0];
    const uint8_t * arguments = data + 1;
    const uint8_t count = length - 2;
    uint8_t status = 1;
    if (code == mks::READ_ENCODER && count == 0) {
      return mks::encoder_reply(id_, counts_from_steps(stepper_.currentPosition()), out);
    } else if (code == mks::SET_MODE && count == 1) {
      mode_ = arguments[0];
    } else if (code == mks::SET_RESPONSE && count == 2) {
      respond_ = arguments[0];
      active_ = arguments[1];
    } else if (code == mks::SET_HEARTBEAT && count == 4) {
      heartbeat_ms_ = mks::u32(arguments);
    } else if (code == mks::SET_ZERO && count == 0) {
      stepper_.setCurrentPosition(0);
      moving_ = false;
    } else if (code == mks::ENABLE && count == 1) {
      enabled_ = arguments[0];
      if (!enabled_) {
        hold();
      }
    } else if (code == mks::ABSOLUTE_AXIS && count == 6) {
      status = move(mks::absolute_axis(arguments));
    } else {
      return 0;
    }
    return respond_ ? mks::status_reply(id_, code, status, out) : 0;
  }

  // Step the motor; write a "move finished" report to out and return its length, or 0.
  uint8_t update(uint32_t now_ms, uint8_t * out)
  {
    stepper_.run();
    if (moving_ && heartbeat_ms_ && now_ms - last_frame_ms_ > heartbeat_ms_) {
      // The host went quiet: stop at once, like an MKS drive's heartbeat protection.
      hold();
      return 0;
    }
    if (moving_ && stepper_.distanceToGo() == 0) {
      moving_ = false;
      if (respond_ && active_) {
        return mks::status_reply(
          id_, mks::ABSOLUTE_AXIS, limited_ ? mks::MOVE_AT_LIMIT : mks::MOVE_COMPLETE, out);
      }
    }
    return 0;
  }

private:
  uint8_t move(const mks::Move & request)
  {
    if (!enabled_ || mode_ != mks::MODE_SR_VFOC) {
      return mks::MOVE_FAILED;
    }
    if (request.rpm == 0) {
      // The manual's stop: slow down with this frame's acceleration, or at once if it is 0.
      if (request.acc == 0) {
        hold();
      } else {
        stepper_.setAcceleration(steps_per_s2(request.acc));
        stepper_.stop();
        moving_ = false;
      }
      return mks::MOVE_RUNNING;
    }
    const long wanted = steps_from_counts(request.axis);
    const long target = wanted < config_.min_steps ? config_.min_steps :
      wanted > config_.max_steps ? config_.max_steps : wanted;
    limited_ = target != wanted;
    const float speed = request.rpm * static_cast<float>(config_.steps_per_rev) / 60.0f;
    stepper_.setMaxSpeed(speed < config_.max_steps_per_s ? speed : config_.max_steps_per_s);
    stepper_.setAcceleration(steps_per_s2(request.acc));
    stepper_.moveTo(target);
    moving_ = true;
    return mks::MOVE_RUNNING;
  }

  void hold()
  {
    stepper_.setCurrentPosition(stepper_.currentPosition());
    moving_ = false;
  }

  // The manual's ramp: 1 rpm every (256 - acc) * 50 us, and no ramp at all when acc is 0.
  float steps_per_s2(uint8_t acc) const
  {
    if (acc == 0) {
      return config_.max_steps_per_s2;
    }
    const float rpm_per_s = 1.0f / ((256 - acc) * 50e-6f);
    const float steps = rpm_per_s * config_.steps_per_rev / 60.0f;
    return steps < config_.max_steps_per_s2 ? steps : config_.max_steps_per_s2;
  }

  long steps_from_counts(int32_t counts) const
  {
    return lround(static_cast<double>(counts) * config_.steps_per_rev / mks::COUNTS_PER_REV);
  }

  int64_t counts_from_steps(long steps) const
  {
    return llround(static_cast<double>(steps) * mks::COUNTS_PER_REV / config_.steps_per_rev);
  }

  uint16_t id_;
  Stepper & stepper_;
  MotorConfig config_;
  uint8_t mode_ = 0;
  bool enabled_ = false;
  bool respond_ = true;
  bool active_ = true;
  uint32_t heartbeat_ms_ = 0;
  uint32_t last_frame_ms_ = 0;
  // A move is under way, and whether its target was cut short at a travel limit.
  bool moving_ = false;
  bool limited_ = false;
};
