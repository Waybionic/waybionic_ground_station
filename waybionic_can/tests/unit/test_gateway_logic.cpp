// Gateway reply tracking, alone and with three NodeLogic receivers on an in-memory bus.

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "common/GatewayLogic.h"
#include "common/NodeLogic.h"
#include "common/SoftwareActuator.h"
#include "MemoryBus.h"

namespace waybionic
{
namespace
{

constexpr uint32_t kTimeoutMs = 50;

Frame built(bool ok, const Frame & frame)
{
  EXPECT_TRUE(ok);
  return frame;
}

struct SimNode
{
  SimNode(MemoryBus & bus, uint16_t id)
  : port(bus.attach()), actuator(20), logic(id, actuator, 0) {}

  void step(uint32_t now_ms)
  {
    if (!port.connected) {
      return;
    }
    Frame frame;
    while (port.receive(frame, 0)) {
      const NodeEvent event = logic.handleFrame(frame, now_ms);
      if (event.has_reply) {
        port.send(event.reply);
      }
    }
    const PollEvent poll = logic.poll(now_ms);
    if (poll.has_frame) {
      port.send(poll.frame);
    }
  }

  MemoryBus::Port & port;
  SoftwareActuator actuator;
  NodeLogic logic;
};

class FourNodeBus : public ::testing::Test
{
protected:
  void SetUp() override
  {
    for (const uint16_t id : kIds) {
      nodes.push_back(std::make_unique<SimNode>(bus, id));
    }
  }

  void transmit(const Frame & frame, uint32_t now_ms)
  {
    ASSERT_TRUE(gateway_port.send(frame));
    gateway.noteSent(frame, now_ms);
  }

  // Let every running receiver act, then collect what the gateway heard.
  std::vector<ReplyEvent> pump(uint32_t now_ms)
  {
    for (auto & node : nodes) {
      node->step(now_ms);
    }
    std::vector<ReplyEvent> events;
    Frame frame;
    while (gateway_port.receive(frame, 0)) {
      events.push_back(gateway.handleFrame(frame, now_ms));
    }
    return events;
  }

  SimNode & node(uint16_t id) {return *nodes[id - 1];}

  const uint16_t kIds[3] = {1, 2, 3};
  MemoryBus bus;
  MemoryBus::Port & gateway_port = bus.attach();
  GatewayLogic gateway{kIds, 3, kTimeoutMs};
  std::vector<std::unique_ptr<SimNode>> nodes;
  Frame f;
};

TEST_F(FourNodeBus, OnlyTheAddressedReceiverActsAndReplies)
{
  transmit(built(mks::enable(2, true, f), f), 0);
  const auto events = pump(3);
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].kind, ReplyKind::kReply);
  EXPECT_EQ(events[0].message.can_id, 2u);
  EXPECT_EQ(events[0].latency_ms, 3u);
  uint8_t status = 0;
  ASSERT_TRUE(mks::decodeStatus(events[0].message, status));
  EXPECT_EQ(status, mks::kStatusOk);

  EXPECT_FALSE(node(1).actuator.state().enabled);
  EXPECT_TRUE(node(2).actuator.state().enabled);
  EXPECT_FALSE(node(3).actuator.state().enabled);
  EXPECT_EQ(gateway.health(2, 3), NodeHealth::kOnline);
  EXPECT_EQ(gateway.health(1, 3), NodeHealth::kUnknown);
}

TEST_F(FourNodeBus, MoveIsConfirmedByTheReplyAndTheCompletionReport)
{
  transmit(built(mks::setMode(3, mks::kModeSrVfoc, f), f), 0);
  transmit(built(mks::enable(3, true, f), f), 0);
  transmit(built(mks::absoluteAxis(3, 0x4000, 600, 2, f), f), 0);
  const auto replies = pump(1);
  ASSERT_EQ(replies.size(), 3u);
  for (const auto & event : replies) {
    EXPECT_EQ(event.kind, ReplyKind::kReply);
  }
  EXPECT_TRUE(pump(10).empty());
  const auto done = pump(21);
  ASSERT_EQ(done.size(), 1u);
  EXPECT_EQ(done[0].kind, ReplyKind::kRunComplete);

  transmit(built(mks::readEncoder(3, f), f), 30);
  const auto encoder = pump(31);
  ASSERT_EQ(encoder.size(), 1u);
  int64_t value = 0;
  ASSERT_TRUE(mks::decodeEncoderValue(encoder[0].message, value));
  EXPECT_EQ(value, 0x4000);
  EXPECT_EQ(node(1).actuator.state().position, 0);
  EXPECT_EQ(node(2).actuator.state().position, 0);
}

TEST_F(FourNodeBus, WrongIdGetsNoReplyAndNoNodeActs)
{
  transmit(built(mks::enable(7, true, f), f), 0);
  EXPECT_TRUE(pump(10).empty());
  for (auto & sim : nodes) {
    EXPECT_FALSE(sim->actuator.state().enabled);
  }
  Frame foreign;
  ASSERT_TRUE(mks::readEncoder(9, foreign));
  EXPECT_EQ(gateway.handleFrame(foreign, 0).kind, ReplyKind::kNotANode);
}

TEST_F(FourNodeBus, BadChecksumGetsNoReplyAndNodeStaysUnknown)
{
  ASSERT_TRUE(mks::enable(2, true, f));
  f.data[f.dlc - 1] ^= 0x01;
  transmit(f, 0);
  EXPECT_TRUE(pump(10).empty());
  EXPECT_FALSE(node(2).actuator.state().enabled);
  EXPECT_EQ(gateway.health(2, kTimeoutMs + 1), NodeHealth::kOffline);  // unanswered request
}

TEST_F(FourNodeBus, MalformedReplyIsReported)
{
  Frame bad;
  bad.id = 2;
  bad.dlc = 1;
  bad.data[0] = mks::kEnable;
  const ReplyEvent event = gateway.handleFrame(bad, 0);
  EXPECT_EQ(event.kind, ReplyKind::kMalformed);
  EXPECT_EQ(event.parse_result, mks::ParseResult::kBadDlc);
}

TEST_F(FourNodeBus, SilentReceiverGoesOfflineExactlyAfterTheTimeout)
{
  node(3).port.connected = false;
  for (const uint16_t id : kIds) {
    transmit(built(mks::readEncoder(id, f), f), 0);
  }
  EXPECT_EQ(pump(1).size(), 2u);
  EXPECT_EQ(gateway.health(1, kTimeoutMs), NodeHealth::kOnline);
  EXPECT_EQ(gateway.health(3, kTimeoutMs), NodeHealth::kUnknown);  // boundary: not yet
  EXPECT_EQ(gateway.health(3, kTimeoutMs + 1), NodeHealth::kOffline);
}

TEST_F(FourNodeBus, RepeatedPollsDoNotHideASilentReceiver)
{
  node(3).port.connected = false;
  transmit(built(mks::readEncoder(3, f), f), 0);
  transmit(built(mks::readEncoder(3, f), f), 40);
  EXPECT_EQ(gateway.health(3, kTimeoutMs + 1), NodeHealth::kOffline);
}

TEST_F(FourNodeBus, DelayedReplyBringsTheNodeBackOnline)
{
  transmit(built(mks::readEncoder(2, f), f), 0);
  EXPECT_EQ(gateway.health(2, 60), NodeHealth::kOffline);
  const auto events = pump(80);  // the receiver answers late
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].kind, ReplyKind::kReply);
  EXPECT_EQ(events[0].latency_ms, 80u);
  EXPECT_EQ(gateway.health(2, 80), NodeHealth::kOnline);
}

TEST_F(FourNodeBus, DuplicateCanIdProducesAnUnexpectedSecondReply)
{
  SimNode twin(bus, 2);
  transmit(built(mks::readEncoder(2, f), f), 0);
  twin.step(1);
  const auto events = pump(1);
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0].kind, ReplyKind::kReply);
  EXPECT_EQ(events[1].kind, ReplyKind::kUnexpected);
  EXPECT_EQ(gateway.unexpectedReplies(2), 1u);
}

TEST_F(FourNodeBus, RestartedReceiverMustBeConfiguredAgain)
{
  transmit(built(mks::setMode(1, mks::kModeSrVfoc, f), f), 0);
  transmit(built(mks::enable(1, true, f), f), 0);
  ASSERT_EQ(pump(1).size(), 2u);

  // A power cycle loses the drive state; the gateway only learns this from the replies.
  node(1).port.connected = false;
  nodes[0] = std::make_unique<SimNode>(bus, 1);
  transmit(built(mks::absoluteAxis(1, 100, 60, 0, f), f), 10);
  auto events = pump(11);
  ASSERT_EQ(events.size(), 1u);
  uint8_t status = 0;
  ASSERT_TRUE(mks::decodeStatus(events[0].message, status));
  EXPECT_EQ(status, mks::kStatusFailed);

  transmit(built(mks::setMode(1, mks::kModeSrVfoc, f), f), 20);
  transmit(built(mks::enable(1, true, f), f), 20);
  transmit(built(mks::absoluteAxis(1, 100, 60, 0, f), f), 20);
  events = pump(21);
  ASSERT_EQ(events.size(), 3u);
  ASSERT_TRUE(mks::decodeStatus(events[2].message, status));
  EXPECT_EQ(status, mks::kStatusOk);
}

TEST_F(FourNodeBus, SpeedZeroStopHaltsOnlyTheAddressedMove)
{
  for (const uint16_t id : {2, 3}) {
    transmit(built(mks::setMode(id, mks::kModeSrVfoc, f), f), 0);
    transmit(built(mks::enable(id, true, f), f), 0);
    transmit(built(mks::absoluteAxis(id, 0x4000, 600, 2, f), f), 0);
  }
  ASSERT_EQ(pump(1).size(), 6u);
  ASSERT_TRUE(node(2).actuator.state().moving);
  ASSERT_TRUE(node(3).actuator.state().moving);

  transmit(built(mks::absoluteAxis(2, 0, 0, 0, f), f), 5);
  const auto stop = pump(6);
  ASSERT_EQ(stop.size(), 1u);
  EXPECT_EQ(stop[0].kind, ReplyKind::kReply);
  EXPECT_EQ(stop[0].message.can_id, 2u);
  uint8_t status = 0;
  ASSERT_TRUE(mks::decodeStatus(stop[0].message, status));
  EXPECT_EQ(status, mks::kStatusOk);  // F5h status 1
  EXPECT_FALSE(node(2).actuator.state().moving);
  EXPECT_TRUE(node(3).actuator.state().moving);

  const auto done = pump(21);
  ASSERT_EQ(done.size(), 1u);  // only node 3 completes; a stopped move reports nothing
  EXPECT_EQ(done[0].kind, ReplyKind::kRunComplete);
  EXPECT_EQ(done[0].message.can_id, 3u);
  EXPECT_TRUE(pump(100).empty());
  EXPECT_EQ(node(2).actuator.state().position, 0);
  EXPECT_EQ(node(3).actuator.state().position, 0x4000);
  EXPECT_FALSE(node(1).actuator.state().enabled);
}

TEST_F(FourNodeBus, EmergencyStopIsPending)
{
  GTEST_SKIP() << "TODO(MKS manual): F7h emergency stop is not defined in "
    "waybionic_teleop/mks_can.py; add it from the manual before testing it";
}

TEST(GatewayLogic, IgnoresDuplicateAndExtraNodeIds)
{
  const uint16_t ids[] = {1, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  GatewayLogic gateway(ids, 10, 50);
  EXPECT_EQ(gateway.nodeCount(), GatewayLogic::kMaxNodes);
  EXPECT_TRUE(gateway.isNode(8));
  EXPECT_FALSE(gateway.isNode(9));
}

}  // namespace
}  // namespace waybionic
