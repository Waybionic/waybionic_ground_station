#pragma once

#include <stdint.h>

#include "common/Frame.h"
#include "common/IActuator.h"
#include "common/MksFrame.h"

namespace waybionic
{

enum class NodeAction : uint8_t
{
  kIgnored,      // addressed to another node
  kRejected,     // addressed here but malformed (DLC or checksum)
  kUnsupported,  // valid frame, unknown code or wrong argument count: no reply, like the drive
  kAccepted,
};

const char * nodeActionName(NodeAction action);

struct NodeEvent
{
  NodeAction action = NodeAction::kIgnored;
  mks::ParseResult parse_result = mks::ParseResult::kOk;
  uint8_t code = 0;
  bool has_reply = false;
  Frame reply;
};

struct PollEvent
{
  bool heartbeat_stop = false;
  bool has_frame = false;  // unprompted F5h run-complete report
  Frame frame;
};

// One receiver node, following SimulatedServo.receive in waybionic_teleop/sim_drives.py
// (PR #24) minus its motion model, which belongs to the IActuator.
class NodeLogic
{
public:
  NodeLogic(uint16_t can_id, IActuator & actuator, uint32_t now_ms);

  NodeEvent handleFrame(const Frame & frame, uint32_t now_ms);
  // Call every loop: updates the actuator, applies the heartbeat stop, reports finished moves.
  PollEvent poll(uint32_t now_ms);

  uint16_t canId() const {return can_id_;}
  bool hasMode() const {return has_mode_;}
  uint8_t mode() const {return mode_;}
  bool respond() const {return respond_;}
  bool active() const {return active_;}
  uint32_t heartbeatMs() const {return heartbeat_ms_;}
  uint32_t heartbeatStops() const {return heartbeat_stops_;}

private:
  bool handleMessage(const mks::Message & message, uint32_t now_ms, uint8_t & status);
  uint8_t startMove(const mks::AbsoluteAxisArgs & move, uint32_t now_ms);

  uint16_t can_id_;
  IActuator & actuator_;
  bool has_mode_ = false;
  uint8_t mode_ = 0;
  bool respond_ = true;
  bool active_ = true;
  uint32_t heartbeat_ms_ = 0;  // 0 disables the heartbeat stop
  uint32_t last_rx_ms_;
  uint32_t heartbeat_stops_ = 0;
  bool move_pending_ = false;
};

}  // namespace waybionic
