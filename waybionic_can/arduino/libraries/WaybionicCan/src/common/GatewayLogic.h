#pragma once

#include <stdint.h>

#include "common/Frame.h"
#include "common/MksFrame.h"

namespace waybionic
{

enum class NodeHealth : uint8_t
{
  kUnknown,  // no reply has been received yet and no request has timed out
  kOnline,   // answered before and no pending request has timed out
  kOffline,  // a request went unanswered for longer than the reply timeout
};

enum class ReplyKind : uint8_t
{
  kNotANode,     // CAN ID is not a configured receiver
  kMalformed,    // bad DLC or checksum
  kReply,        // answers a request this gateway sent
  kRunComplete,  // unprompted F5h report that a move ended
  kUnexpected,   // nothing was pending: a duplicate node ID, or a reply after a newer request
};

const char * nodeHealthName(NodeHealth health);
const char * replyKindName(ReplyKind kind);

struct ReplyEvent
{
  ReplyKind kind = ReplyKind::kNotANode;
  mks::ParseResult parse_result = mks::ParseResult::kOk;
  mks::Message message;
  uint32_t latency_ms = 0;  // for kReply
};

// Tracks which receivers actually answered. Success is only ever the addressed node's MKS
// reply; the CAN ACK behind a successful send() says nothing about which node acted.
class GatewayLogic
{
public:
  static constexpr uint8_t kMaxNodes = 8;
  static constexpr uint8_t kMaxPending = 4;

  GatewayLogic(const uint16_t * node_ids, uint8_t node_count, uint32_t reply_timeout_ms);

  bool isNode(uint16_t can_id) const;
  // Record a command sent to a node so its reply can be matched.
  void noteSent(const Frame & frame, uint32_t now_ms);
  ReplyEvent handleFrame(const Frame & frame, uint32_t now_ms);
  NodeHealth health(uint16_t can_id, uint32_t now_ms) const;
  uint32_t unexpectedReplies(uint16_t can_id) const;

  uint8_t nodeCount() const {return node_count_;}
  uint16_t nodeId(uint8_t index) const {return nodes_[index].id;}

private:
  struct Pending
  {
    uint8_t code;
    uint32_t sent_ms;
  };

  struct Node
  {
    uint16_t id = 0;
    bool answered = false;
    Pending pending[kMaxPending] = {};
    uint8_t pending_count = 0;
    uint32_t unexpected = 0;
  };

  Node * find(uint16_t can_id);
  const Node * find(uint16_t can_id) const;

  Node nodes_[kMaxNodes];
  uint8_t node_count_ = 0;
  uint32_t reply_timeout_ms_;
};

}  // namespace waybionic
