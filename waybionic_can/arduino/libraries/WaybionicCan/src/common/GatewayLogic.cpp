#include "common/GatewayLogic.h"

namespace waybionic
{

const char * nodeHealthName(const NodeHealth health)
{
  switch (health) {
    case NodeHealth::kUnknown: return "UNKNOWN";
    case NodeHealth::kOnline: return "ONLINE";
    case NodeHealth::kOffline: return "OFFLINE";
  }
  return "?";
}

const char * replyKindName(const ReplyKind kind)
{
  switch (kind) {
    case ReplyKind::kNotANode: return "not a node";
    case ReplyKind::kMalformed: return "malformed";
    case ReplyKind::kReply: return "reply";
    case ReplyKind::kRunComplete: return "run complete";
    case ReplyKind::kUnexpected: return "unexpected";
  }
  return "?";
}

GatewayLogic::GatewayLogic(
  const uint16_t * node_ids, const uint8_t node_count, const uint32_t reply_timeout_ms)
: reply_timeout_ms_(reply_timeout_ms)
{
  for (uint8_t i = 0; i < node_count && node_count_ < kMaxNodes; ++i) {
    if (!isNode(node_ids[i])) {
      nodes_[node_count_++].id = node_ids[i];
    }
  }
}

bool GatewayLogic::isNode(const uint16_t can_id) const
{
  return find(can_id) != nullptr;
}

GatewayLogic::Node * GatewayLogic::find(const uint16_t can_id)
{
  for (uint8_t i = 0; i < node_count_; ++i) {
    if (nodes_[i].id == can_id) {
      return &nodes_[i];
    }
  }
  return nullptr;
}

const GatewayLogic::Node * GatewayLogic::find(const uint16_t can_id) const
{
  return const_cast<GatewayLogic *>(this)->find(can_id);
}

void GatewayLogic::noteSent(const Frame & frame, const uint32_t now_ms)
{
  Node * node = find(frame.id);
  if (node == nullptr || frame.dlc == 0) {
    return;
  }
  const uint8_t code = frame.data[0];
  for (uint8_t i = 0; i < node->pending_count; ++i) {
    // Keep the oldest send time so repeated polls cannot hide a silent node.
    if (node->pending[i].code == code) {
      return;
    }
  }
  if (node->pending_count == kMaxPending) {
    for (uint8_t i = 1; i < kMaxPending; ++i) {
      node->pending[i - 1] = node->pending[i];
    }
    --node->pending_count;
  }
  node->pending[node->pending_count++] = Pending{code, now_ms};
}

ReplyEvent GatewayLogic::handleFrame(const Frame & frame, const uint32_t now_ms)
{
  ReplyEvent event;
  Node * node = find(frame.id);
  if (node == nullptr) {
    return event;
  }
  event.parse_result = mks::parseFrame(frame, event.message);
  if (event.parse_result != mks::ParseResult::kOk) {
    event.kind = ReplyKind::kMalformed;
    return event;
  }

  for (uint8_t i = 0; i < node->pending_count; ++i) {
    if (node->pending[i].code != event.message.code) {
      continue;
    }
    event.kind = ReplyKind::kReply;
    event.latency_ms = now_ms - node->pending[i].sent_ms;
    for (uint8_t j = i + 1; j < node->pending_count; ++j) {
      node->pending[j - 1] = node->pending[j];
    }
    --node->pending_count;
    node->answered = true;
    return event;
  }

  uint8_t status = 0;
  if (event.message.code == mks::kAbsoluteAxis && mks::decodeStatus(event.message, status) &&
    (status == mks::kRunComplete || status == mks::kRunEndLimit))
  {
    event.kind = ReplyKind::kRunComplete;
    return event;
  }
  event.kind = ReplyKind::kUnexpected;
  ++node->unexpected;
  return event;
}

NodeHealth GatewayLogic::health(const uint16_t can_id, const uint32_t now_ms) const
{
  const Node * node = find(can_id);
  if (node == nullptr) {
    return NodeHealth::kUnknown;
  }
  for (uint8_t i = 0; i < node->pending_count; ++i) {
    if (now_ms - node->pending[i].sent_ms > reply_timeout_ms_) {
      return NodeHealth::kOffline;
    }
  }
  return node->answered ? NodeHealth::kOnline : NodeHealth::kUnknown;
}

uint32_t GatewayLogic::unexpectedReplies(const uint16_t can_id) const
{
  const Node * node = find(can_id);
  return node == nullptr ? 0 : node->unexpected;
}

}  // namespace waybionic
