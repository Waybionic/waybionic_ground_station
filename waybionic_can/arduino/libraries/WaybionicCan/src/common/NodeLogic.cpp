#include "common/NodeLogic.h"

namespace waybionic
{

const char * nodeActionName(const NodeAction action)
{
  switch (action) {
    case NodeAction::kIgnored: return "ignored";
    case NodeAction::kRejected: return "rejected";
    case NodeAction::kUnsupported: return "unsupported";
    case NodeAction::kAccepted: return "accepted";
  }
  return "unknown";
}

NodeLogic::NodeLogic(const uint16_t can_id, IActuator & actuator, const uint32_t now_ms)
: can_id_(can_id), actuator_(actuator), last_rx_ms_(now_ms)
{
}

NodeEvent NodeLogic::handleFrame(const Frame & frame, const uint32_t now_ms)
{
  NodeEvent event;
  // Every node sees all bus traffic; only frames carrying this node's ID are for it.
  if (frame.id != can_id_) {
    return event;
  }

  mks::Message message;
  event.parse_result = mks::parseFrame(frame, message);
  if (event.parse_result != mks::ParseResult::kOk) {
    event.action = NodeAction::kRejected;
    return event;
  }
  event.code = message.code;
  last_rx_ms_ = now_ms;

  // The encoder reply is data, not an acknowledgement, so it ignores SET_RESPONSE.
  if (message.code == mks::kReadEncoder && message.argument_count == 0) {
    event.action = NodeAction::kAccepted;
    event.has_reply = mks::encoderReply(can_id_, actuator_.state().position, event.reply);
    return event;
  }

  uint8_t status = mks::kStatusFailed;
  if (!handleMessage(message, now_ms, status)) {
    event.action = NodeAction::kUnsupported;
    return event;
  }
  event.action = NodeAction::kAccepted;
  if (respond_) {
    event.has_reply = mks::statusReply(can_id_, message.code, status, event.reply);
  }
  return event;
}

bool NodeLogic::handleMessage(
  const mks::Message & message, const uint32_t now_ms, uint8_t & status)
{
  const uint8_t * a = message.arguments;
  const uint8_t count = message.argument_count;
  status = mks::kStatusOk;

  if (message.code == mks::kSetMode && count == 1) {
    has_mode_ = true;
    mode_ = a[0];
  } else if (message.code == mks::kSetResponse && count == 2) {
    respond_ = a[0] != 0;
    active_ = a[1] != 0;
  } else if (message.code == mks::kSetHeartbeat && count == 4) {
    heartbeat_ms_ = (static_cast<uint32_t>(a[0]) << 24) | (static_cast<uint32_t>(a[1]) << 16) |
      (static_cast<uint32_t>(a[2]) << 8) | a[3];
  } else if (message.code == mks::kEnable && count == 1) {
    if (a[0] == 0) {
      actuator_.stop(now_ms);
      move_pending_ = false;
    }
    actuator_.setEnabled(a[0] != 0);
  } else if (message.code == mks::kAbsoluteAxis && count == 6) {
    mks::AbsoluteAxisArgs move;
    mks::decodeAbsoluteAxis(message, move);
    status = startMove(move, now_ms);
  } else {
    return false;
  }
  return true;
}

uint8_t NodeLogic::startMove(const mks::AbsoluteAxisArgs & move, const uint32_t now_ms)
{
  if (!actuator_.state().enabled || !has_mode_ || mode_ != mks::kModeSrVfoc) {
    return mks::kStatusFailed;
  }
  // Speed 0 is the manual's stop command.
  if (move.speed_rpm == 0) {
    actuator_.stop(now_ms);
    move_pending_ = false;
    return mks::kStatusOk;
  }
  const uint16_t speed = move.speed_rpm > mks::kMaxSpeedRpm ?
    static_cast<uint16_t>(mks::kMaxSpeedRpm) : move.speed_rpm;
  actuator_.moveTo(move.axis, speed, move.acc, now_ms);
  move_pending_ = true;
  return mks::kStatusOk;
}

PollEvent NodeLogic::poll(const uint32_t now_ms)
{
  PollEvent event;
  actuator_.update(now_ms);

  // Unsigned subtraction keeps this correct across millis() wraparound.
  const uint32_t quiet_ms = now_ms - last_rx_ms_;
  if (heartbeat_ms_ != 0 && quiet_ms > heartbeat_ms_ && actuator_.state().moving) {
    actuator_.stop(now_ms);
    move_pending_ = false;
    ++heartbeat_stops_;
    event.heartbeat_stop = true;
    return event;
  }

  if (move_pending_ && !actuator_.state().moving) {
    move_pending_ = false;
    if (respond_ && active_) {
      event.has_frame =
        mks::statusReply(can_id_, mks::kAbsoluteAxis, mks::kRunComplete, event.frame);
    }
  }
  return event;
}

}  // namespace waybionic
