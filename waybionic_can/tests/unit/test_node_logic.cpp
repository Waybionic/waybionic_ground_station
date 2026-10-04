// Receiver behavior, mirroring waybionic_teleop/test/test_sim_drives.py (PR #24) where it
// overlaps, without the motion model.

#include <gtest/gtest.h>

#include "common/NodeLogic.h"
#include "common/SoftwareActuator.h"

namespace waybionic
{
namespace
{

constexpr uint16_t kId = 2;

Frame statusFrame(uint8_t code, uint8_t status)
{
  Frame frame;
  mks::statusReply(kId, code, status, frame);
  return frame;
}

Frame built(bool ok, const Frame & frame)
{
  EXPECT_TRUE(ok);
  return frame;
}

bool sameFrame(const Frame & a, const Frame & b)
{
  if (a.id != b.id || a.dlc != b.dlc) {
    return false;
  }
  for (uint8_t i = 0; i < a.dlc; ++i) {
    if (a.data[i] != b.data[i]) {
      return false;
    }
  }
  return true;
}

class NodeLogicTest : public ::testing::Test
{
protected:
  NodeEvent send(bool ok, const Frame & frame, uint32_t now_ms = 0)
  {
    EXPECT_TRUE(ok);
    return node.handleFrame(frame, now_ms);
  }

  void makeReady()
  {
    ASSERT_TRUE(send(mks::setMode(kId, mks::kModeSrVfoc, f), f).has_reply);
    ASSERT_TRUE(send(mks::enable(kId, true, f), f).has_reply);
  }

  SoftwareActuator actuator{100};
  NodeLogic node{kId, actuator, 0};
  Frame f;
};

TEST_F(NodeLogicTest, IgnoresFramesForOtherNodes)
{
  const NodeEvent event = send(mks::enable(3, true, f), f);
  EXPECT_EQ(event.action, NodeAction::kIgnored);
  EXPECT_FALSE(event.has_reply);
  EXPECT_FALSE(actuator.state().enabled);
}

TEST_F(NodeLogicTest, IgnoresMalformedFramesForOtherNodes)
{
  Frame broken;
  broken.id = 3;
  broken.dlc = 1;
  EXPECT_EQ(node.handleFrame(broken, 0).action, NodeAction::kIgnored);
}

TEST_F(NodeLogicTest, AddressedNodeActsAndReplies)
{
  const NodeEvent event = send(mks::enable(kId, true, f), f);
  EXPECT_EQ(event.action, NodeAction::kAccepted);
  EXPECT_TRUE(actuator.state().enabled);
  ASSERT_TRUE(event.has_reply);
  EXPECT_TRUE(sameFrame(event.reply, statusFrame(mks::kEnable, mks::kStatusOk)));
}

TEST_F(NodeLogicTest, SettingsReplyWithStatusOne)
{
  EXPECT_TRUE(sameFrame(send(mks::setMode(kId, mks::kModeSrVfoc, f), f).reply,
    statusFrame(mks::kSetMode, 1)));
  EXPECT_TRUE(node.hasMode());
  EXPECT_TRUE(sameFrame(send(mks::setHeartbeat(kId, 500, f), f).reply,
    statusFrame(mks::kSetHeartbeat, 1)));
  EXPECT_EQ(node.heartbeatMs(), 500u);
  EXPECT_TRUE(sameFrame(send(mks::setResponse(kId, true, false, f), f).reply,
    statusFrame(mks::kSetResponse, 1)));
  EXPECT_FALSE(node.active());
}

TEST_F(NodeLogicTest, MovesAreRefusedUntilModeAndEnableAreSet)
{
  EXPECT_TRUE(sameFrame(send(mks::absoluteAxis(kId, 0x4000, 600, 0, f), f).reply,
    statusFrame(mks::kAbsoluteAxis, mks::kStatusFailed)));
  send(mks::enable(kId, true, f), f);
  EXPECT_TRUE(sameFrame(send(mks::absoluteAxis(kId, 0x4000, 600, 0, f), f).reply,
    statusFrame(mks::kAbsoluteAxis, mks::kStatusFailed)));
  send(mks::setMode(kId, 0x04, f), f);  // any mode other than SR_vFOC
  EXPECT_TRUE(sameFrame(send(mks::absoluteAxis(kId, 0x4000, 600, 0, f), f).reply,
    statusFrame(mks::kAbsoluteAxis, mks::kStatusFailed)));
  EXPECT_EQ(actuator.moveCount(), 0u);
}

TEST_F(NodeLogicTest, MoveRunsThenReportsCompletion)
{
  makeReady();
  const NodeEvent event = send(mks::absoluteAxis(kId, 0x4000, 600, 2, f), f, 10);
  EXPECT_TRUE(sameFrame(event.reply, statusFrame(mks::kAbsoluteAxis, 1)));
  EXPECT_TRUE(actuator.state().moving);
  EXPECT_EQ(actuator.state().target, 0x4000);
  EXPECT_EQ(actuator.speedRpm(), 600);
  EXPECT_EQ(actuator.acc(), 2);

  EXPECT_FALSE(node.poll(109).has_frame);
  const PollEvent done = node.poll(110);
  ASSERT_TRUE(done.has_frame);
  EXPECT_TRUE(sameFrame(done.frame, statusFrame(mks::kAbsoluteAxis, mks::kRunComplete)));
  EXPECT_EQ(actuator.state().position, 0x4000);
  EXPECT_FALSE(node.poll(200).has_frame);
}

TEST_F(NodeLogicTest, EncoderReplyReportsThePosition)
{
  makeReady();
  send(mks::absoluteAxis(kId, -1234, 300, 0, f), f);
  node.poll(100);
  const NodeEvent event = send(mks::readEncoder(kId, f), f, 100);
  mks::Message message;
  ASSERT_EQ(mks::parseFrame(event.reply, message), mks::ParseResult::kOk);
  int64_t value = 0;
  ASSERT_TRUE(mks::decodeEncoderValue(message, value));
  EXPECT_EQ(value, -1234);
}

TEST_F(NodeLogicTest, NewTargetReplacesARunningMove)
{
  makeReady();
  send(mks::absoluteAxis(kId, 0x4000, 600, 0, f), f, 0);
  node.poll(50);
  EXPECT_TRUE(sameFrame(send(mks::absoluteAxis(kId, -0x4000, 300, 0, f), f, 50).reply,
    statusFrame(mks::kAbsoluteAxis, 1)));
  EXPECT_EQ(actuator.state().target, -0x4000);
  EXPECT_FALSE(node.poll(100).has_frame);  // the timer restarted with the new target
  EXPECT_TRUE(node.poll(150).has_frame);
  EXPECT_EQ(actuator.state().position, -0x4000);
}

TEST_F(NodeLogicTest, SpeedZeroStopsWithoutACompletionReport)
{
  makeReady();
  send(mks::absoluteAxis(kId, 0x4000, 600, 0, f), f);
  EXPECT_TRUE(sameFrame(send(mks::absoluteAxis(kId, 0, 0, 4, f), f, 10).reply,
    statusFrame(mks::kAbsoluteAxis, 1)));
  EXPECT_FALSE(actuator.state().moving);
  EXPECT_EQ(actuator.state().position, 0);
  EXPECT_FALSE(node.poll(500).has_frame);
}

TEST_F(NodeLogicTest, SpeedAboveTheLimitIsClamped)
{
  makeReady();
  const uint8_t args[6] = {0x0F, 0xA0, 0x00, 0x00, 0x40, 0x00};  // 4000 rpm
  ASSERT_TRUE(mks::buildFrame(kId, mks::kAbsoluteAxis, args, 6, f));
  node.handleFrame(f, 0);
  EXPECT_EQ(actuator.speedRpm(), mks::kMaxSpeedRpm);
}

TEST_F(NodeLogicTest, DisableStopsTheMotor)
{
  makeReady();
  send(mks::absoluteAxis(kId, 0x4000, 600, 0, f), f);
  send(mks::enable(kId, false, f), f, 10);
  EXPECT_FALSE(actuator.state().enabled);
  EXPECT_FALSE(actuator.state().moving);
  EXPECT_FALSE(node.poll(500).has_frame);
}

TEST_F(NodeLogicTest, ResponseOffSilencesStatusButNotEncoderData)
{
  makeReady();
  EXPECT_FALSE(send(mks::setResponse(kId, false, true, f), f).has_reply);
  EXPECT_FALSE(send(mks::absoluteAxis(kId, 100, 60, 0, f), f).has_reply);
  EXPECT_TRUE(actuator.state().moving);  // silent, but it still acted
  EXPECT_FALSE(node.poll(100).has_frame);
  EXPECT_TRUE(send(mks::readEncoder(kId, f), f).has_reply);
}

TEST_F(NodeLogicTest, ActiveOffSkipsTheCompletionReport)
{
  makeReady();
  send(mks::setResponse(kId, true, false, f), f);
  EXPECT_TRUE(send(mks::absoluteAxis(kId, 100, 60, 0, f), f).has_reply);
  EXPECT_FALSE(node.poll(100).has_frame);
  EXPECT_EQ(actuator.state().position, 100);
}

TEST_F(NodeLogicTest, BadChecksumAndDlcAreRejectedWithoutReply)
{
  makeReady();
  ASSERT_TRUE(mks::absoluteAxis(kId, 0x4000, 600, 0, f));
  f.data[f.dlc - 1] ^= 0xFF;
  NodeEvent event = node.handleFrame(f, 0);
  EXPECT_EQ(event.action, NodeAction::kRejected);
  EXPECT_EQ(event.parse_result, mks::ParseResult::kBadChecksum);
  EXPECT_FALSE(event.has_reply);

  Frame short_frame;
  short_frame.id = kId;
  short_frame.dlc = 1;
  short_frame.data[0] = mks::kReadEncoder;
  event = node.handleFrame(short_frame, 0);
  EXPECT_EQ(event.parse_result, mks::ParseResult::kBadDlc);
  EXPECT_FALSE(event.has_reply);
  EXPECT_EQ(actuator.moveCount(), 0u);
}

TEST_F(NodeLogicTest, UnsupportedCodesAndWrongArgumentCountsGetNoReply)
{
  // 0x00 stands for any code outside the PR #24 subset; it is not an MKS command here.
  ASSERT_TRUE(mks::buildFrame(kId, 0x00, nullptr, 0, f));
  EXPECT_EQ(node.handleFrame(f, 0).action, NodeAction::kUnsupported);

  const uint8_t two[2] = {1, 1};
  ASSERT_TRUE(mks::buildFrame(kId, mks::kEnable, two, 2, f));
  const NodeEvent event = node.handleFrame(f, 0);
  EXPECT_EQ(event.action, NodeAction::kUnsupported);
  EXPECT_FALSE(event.has_reply);
  EXPECT_FALSE(actuator.state().enabled);

  // A 31h frame with arguments is an encoder reply, never a request.
  ASSERT_TRUE(mks::encoderReply(kId, 5, f));
  EXPECT_EQ(node.handleFrame(f, 0).action, NodeAction::kUnsupported);
}

TEST_F(NodeLogicTest, HeartbeatStopsAMovingMotorOnlyAfterTheTimeout)
{
  SoftwareActuator slow(10000);
  NodeLogic slow_node(kId, slow, 0);
  slow_node.handleFrame(built(mks::setMode(kId, mks::kModeSrVfoc, f), f), 0);
  slow_node.handleFrame(built(mks::enable(kId, true, f), f), 0);
  slow_node.handleFrame(built(mks::setHeartbeat(kId, 100, f), f), 0);
  slow_node.handleFrame(built(mks::absoluteAxis(kId, 0x4000, 60, 0, f), f), 1000);

  EXPECT_FALSE(slow_node.poll(1100).heartbeat_stop);  // exactly the timeout: still running
  EXPECT_TRUE(slow.state().moving);
  EXPECT_TRUE(slow_node.poll(1101).heartbeat_stop);
  EXPECT_FALSE(slow.state().moving);
  EXPECT_EQ(slow_node.heartbeatStops(), 1u);
  EXPECT_FALSE(slow_node.poll(5000).has_frame);  // no completion report after a stop
}

TEST_F(NodeLogicTest, AnyAddressedFrameFeedsTheHeartbeat)
{
  SoftwareActuator slow(10000);
  NodeLogic slow_node(kId, slow, 0);
  slow_node.handleFrame(built(mks::setMode(kId, mks::kModeSrVfoc, f), f), 0);
  slow_node.handleFrame(built(mks::enable(kId, true, f), f), 0);
  slow_node.handleFrame(built(mks::setHeartbeat(kId, 100, f), f), 0);
  slow_node.handleFrame(built(mks::absoluteAxis(kId, 0x4000, 60, 0, f), f), 0);
  for (uint32_t t = 90; t < 1000; t += 90) {
    slow_node.handleFrame(built(mks::readEncoder(kId, f), f), t);
    EXPECT_FALSE(slow_node.poll(t).heartbeat_stop);
  }
  // Frames for other nodes do not count.
  slow_node.handleFrame(built(mks::readEncoder(3, f), f), 1050);
  EXPECT_TRUE(slow_node.poll(1091).heartbeat_stop);
}

TEST_F(NodeLogicTest, HeartbeatIgnoresAnIdleMotorAndZeroDisablesIt)
{
  send(mks::setHeartbeat(kId, 100, f), f);
  EXPECT_FALSE(node.poll(10000).heartbeat_stop);

  SoftwareActuator slow(100000);
  NodeLogic slow_node(kId, slow, 0);
  slow_node.handleFrame(built(mks::setMode(kId, mks::kModeSrVfoc, f), f), 0);
  slow_node.handleFrame(built(mks::enable(kId, true, f), f), 0);
  slow_node.handleFrame(built(mks::setHeartbeat(kId, 0, f), f), 0);
  slow_node.handleFrame(built(mks::absoluteAxis(kId, 0x4000, 60, 0, f), f), 0);
  EXPECT_FALSE(slow_node.poll(50000).heartbeat_stop);
}

TEST_F(NodeLogicTest, NewMoveAfterAHeartbeatStopRecovers)
{
  SoftwareActuator slow(1000);
  NodeLogic slow_node(kId, slow, 0);
  slow_node.handleFrame(built(mks::setMode(kId, mks::kModeSrVfoc, f), f), 0);
  slow_node.handleFrame(built(mks::enable(kId, true, f), f), 0);
  slow_node.handleFrame(built(mks::setHeartbeat(kId, 100, f), f), 0);
  slow_node.handleFrame(built(mks::absoluteAxis(kId, 0x4000, 60, 0, f), f), 0);
  ASSERT_TRUE(slow_node.poll(200).heartbeat_stop);

  // The drive stays enabled and in its mode, so the next command simply starts a new move.
  const NodeEvent event = slow_node.handleFrame(built(mks::absoluteAxis(kId, 0x4000, 60, 0, f), f), 300);
  EXPECT_TRUE(sameFrame(event.reply, statusFrame(mks::kAbsoluteAxis, 1)));
  for (uint32_t t = 350; t < 1300; t += 50) {
    slow_node.handleFrame(built(mks::readEncoder(kId, f), f), t);
    slow_node.poll(t);
  }
  EXPECT_TRUE(slow_node.poll(1300).has_frame);
  EXPECT_EQ(slow.state().position, 0x4000);
}

TEST_F(NodeLogicTest, HeartbeatHandlesClockWraparound)
{
  SoftwareActuator slow(100000);
  const uint32_t start = 0xFFFFFFF0u;
  NodeLogic slow_node(kId, slow, start);
  slow_node.handleFrame(built(mks::setMode(kId, mks::kModeSrVfoc, f), f), start);
  slow_node.handleFrame(built(mks::enable(kId, true, f), f), start);
  slow_node.handleFrame(built(mks::setHeartbeat(kId, 100, f), f), start);
  slow_node.handleFrame(built(mks::absoluteAxis(kId, 0x4000, 60, 0, f), f), start);
  EXPECT_FALSE(slow_node.poll(start + 100).heartbeat_stop);  // wraps past zero
  EXPECT_TRUE(slow_node.poll(start + 101).heartbeat_stop);
}

}  // namespace
}  // namespace waybionic
