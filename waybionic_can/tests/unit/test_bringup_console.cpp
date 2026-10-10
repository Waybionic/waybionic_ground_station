// BringupConsole: the serial command layer of single_drive_bringup.ino and the bench gateway.
// Frame bytes are checked against the PR #24 / manual vectors in test_mks_frame.cpp.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "common/BringupConsole.h"
#include "common/Candump.h"

namespace waybionic
{
namespace
{

class RecordingTransport : public ICanTransport
{
public:
  bool send(const Frame & frame) override
  {
    sent.push_back(frame);
    return accept;
  }
  bool receive(Frame &, uint32_t) override {return false;}

  std::vector<Frame> sent;
  bool accept = true;
};

class RecordingOutput : public ConsoleOutput
{
public:
  void line(const char * text) override {lines.emplace_back(text);}

  bool contains(const std::string & needle) const
  {
    for (const std::string & line : lines) {
      if (line.find(needle) != std::string::npos) {
        return true;
      }
    }
    return false;
  }

  std::vector<std::string> lines;
};

std::string candump(const Frame & frame)
{
  char text[24];
  mks::formatFrame(frame, text, sizeof(text));
  return text;
}

Frame fromCandump(const char * text)
{
  Frame frame;
  EXPECT_TRUE(parseCandump(text, frame)) << text;
  return frame;
}

class ConsoleTest : public ::testing::Test
{
protected:
  ConsoleTest()
  : console(can, out, policy()) {}

  static MotionPolicy policy()
  {
    MotionPolicy p;
    p.max_rpm = 600;  // lets the manual vector F502580200400092 (600 rpm) pass the bench cap
    p.min_acc = 1;
    p.max_acc = 128;
    p.max_step_counts = 0x4000;
    p.encoder_fresh_ms = 10000;
    return p;
  }

  void run(const char * command, uint32_t now_ms = 1000)
  {
    std::string copy = command;
    EXPECT_TRUE(console.handleLine(&copy[0], now_ms)) << command;
  }

  void reply(const char * text, uint32_t now_ms = 1000)
  {
    console.handleFrame(fromCandump(text), now_ms);
  }

  // Enable confirmed and encoder at 0 for drive 1.
  void arm(uint32_t now_ms = 1000)
  {
    run("enable 1", now_ms);
    reply("001#F301F5", now_ms);
    run("read 1", now_ms);
    reply("001#3100000000000032", now_ms);
    can.sent.clear();
  }

  size_t sentCount() const {return can.sent.size();}
  std::string lastSent() const {return can.sent.empty() ? "" : candump(can.sent.back());}

  RecordingTransport can;
  RecordingOutput out;
  BringupConsole console;
};

TEST_F(ConsoleTest, SendsNothingUntilACommandIsTyped)
{
  EXPECT_EQ(sentCount(), 0u);
  run("help");
  run("policy");
  run("drives");
  run("");
  run("# comment");
  EXPECT_EQ(sentCount(), 0u);
}

TEST_F(ConsoleTest, ReadSendsTheManualFrameAndLogsItLikeCandump)
{
  run("read 1", 1234);
  ASSERT_EQ(sentCount(), 1u);
  EXPECT_EQ(lastSent(), "001#3132");
  EXPECT_TRUE(out.contains("TX 001#3132 t=1234"));
}

TEST_F(ConsoleTest, EnableAndDisableMatchTheManual)
{
  run("enable 1");
  EXPECT_EQ(lastSent(), "001#F301F5");
  EXPECT_TRUE(out.contains("MOTION F3h ENABLE id=1"));
  run("disable 1");
  EXPECT_EQ(lastSent(), "001#F300F4");
}

TEST_F(ConsoleTest, SettingCommandsMatchTheManual)
{
  run("mode 1");
  EXPECT_EQ(lastSent(), "001#820588");
  run("response 1 1 0");
  EXPECT_EQ(lastSent(), "001#8C01008E");
  run("heartbeat 1 500");
  EXPECT_EQ(lastSent(), "001#98000001F48E");
}

TEST_F(ConsoleTest, StandardCanIdBoundsAreEnforced)
{
  run("read 0");
  run("read 2048");
  run("read -1");
  run("read 0x800");
  run("read abc");
  EXPECT_EQ(sentCount(), 0u);
  run("read 2047");
  EXPECT_EQ(lastSent(), "7FF#3130");
  run("read 0x7FF");
  EXPECT_EQ(lastSent(), "7FF#3130");
}

TEST_F(ConsoleTest, MoveRequiresAllFourArguments)
{
  arm();
  run("move 1");
  run("move 1 16384");
  run("move 1 16384 600");
  run("move 1 16384 600 2 9");
  EXPECT_EQ(sentCount(), 0u);
  EXPECT_TRUE(out.contains("ERR usage: move"));
}

TEST_F(ConsoleTest, MoveIsRefusedUntilEnableIsConfirmed)
{
  run("read 1");
  reply("001#3100000000000032");
  run("enable 1");
  can.sent.clear();
  run("move 1 16384 600 2");
  EXPECT_EQ(sentCount(), 0u);
  EXPECT_TRUE(out.contains("has not confirmed 'enable 1'"));
  reply("001#F301F5");
  run("read 1");
  reply("001#3100000000000032");
  can.sent.clear();
  run("move 1 16384 600 2");
  EXPECT_EQ(lastSent(), "001#F502580200400092");
}

TEST_F(ConsoleTest, RefusedEnableKeepsMoveLocked)
{
  run("enable 1");
  reply("001#F300F4");  // status 0
  run("read 1");
  reply("001#3100000000000032");
  can.sent.clear();
  run("move 1 16384 600 2");
  EXPECT_EQ(sentCount(), 0u);
  EXPECT_TRUE(out.contains("refused enable"));
  EXPECT_FALSE(console.enabledConfirmed(1));
}

TEST_F(ConsoleTest, MoveRequiresAnEncoderReadingFirst)
{
  run("enable 1");
  reply("001#F301F5");
  can.sent.clear();
  run("move 1 16384 600 2");
  EXPECT_EQ(sentCount(), 0u);
  EXPECT_TRUE(out.contains("run 'read 1' first"));
}

TEST_F(ConsoleTest, ManualVectorsPassThroughMove)
{
  arm();
  run("move 1 16384 600 2");
  EXPECT_EQ(lastSent(), "001#F502580200400092");
  EXPECT_TRUE(out.contains("MOTION F5h ABSOLUTE_AXIS id=1 axis=16384 rpm=600 acc=2"));
  run("read 1");
  reply("001#3100000000000032");
  run("move 1 -16384 600 2");
  EXPECT_EQ(lastSent(), "001#F5025802FFC00011");
}

TEST_F(ConsoleTest, ReadmeFirstMoveExampleBytes)
{
  arm();
  run("move 1 4096 30 2");
  EXPECT_EQ(lastSent(), "001#F5001E0200100026");
}

TEST_F(ConsoleTest, ReadmeStartupSequenceBytes)
{
  // PR #24 sim_arm_drives_node.py order: 82h mode, 8Ch replies, F3h enable, 98h heartbeat.
  const char * const commands[] = {"mode 1", "response 1 1 1", "enable 1", "heartbeat 1 0"};
  const char * const frames[] = {"001#820588", "001#8C01018F", "001#F301F5", "001#980000000099"};
  const uint8_t codes[] = {mks::kSetMode, mks::kSetResponse, mks::kEnable, mks::kSetHeartbeat};
  const char * const replies[] = {"001#820184", "001#8C018E", "001#F301F5", "001#98019A"};
  for (size_t i = 0; i < 4; ++i) {
    run(commands[i]);
    EXPECT_EQ(lastSent(), frames[i]) << commands[i];
    Frame status;
    ASSERT_TRUE(mks::buildFrame(1, codes[i], &mks::kStatusOk, 1, status));
    EXPECT_EQ(candump(status), replies[i]) << commands[i];
    console.handleFrame(status, 1000);
  }
  EXPECT_EQ(sentCount(), 4u);
  EXPECT_TRUE(console.enabledConfirmed(1));
}

TEST_F(ConsoleTest, EveryMoveNeedsANewReading)
{
  arm();
  run("move 1 100 60 2");
  ASSERT_EQ(sentCount(), 1u);
  run("move 1 200 60 2");
  EXPECT_EQ(sentCount(), 1u);
  EXPECT_FALSE(console.encoderFresh(1, 1000));
}

TEST_F(ConsoleTest, StaleReadingIsRefused)
{
  arm(1000);
  EXPECT_TRUE(console.encoderFresh(1, 11000));
  run("move 1 100 60 2", 11001);
  EXPECT_EQ(sentCount(), 0u);
}

TEST_F(ConsoleTest, AxisBoundsMatchMksCan)
{
  arm();
  run("move 1 8388608 60 2");
  run("move 1 -8388608 60 2");
  EXPECT_EQ(sentCount(), 0u);
  EXPECT_TRUE(out.contains("outside mks_can.py limits"));
}

TEST_F(ConsoleTest, SpeedBoundsMatchMksCanAndThePolicy)
{
  arm();
  run("move 1 100 3001 2");
  run("move 1 100 -1 2");
  EXPECT_TRUE(out.contains("outside mks_can.py limits"));
  run("move 1 100 601 2");
  EXPECT_TRUE(out.contains("bench policy: rpm 1-600"));
  run("move 1 100 0 2");
  EXPECT_TRUE(out.contains("use 'stop <id> [acc]'"));
  EXPECT_EQ(sentCount(), 0u);
}

TEST_F(ConsoleTest, AccelerationBoundsMatchMksCanAndThePolicy)
{
  arm();
  run("move 1 100 60 256");
  run("move 1 100 60 -1");
  EXPECT_TRUE(out.contains("outside mks_can.py limits"));
  run("move 1 100 60 0");
  run("move 1 100 60 129");
  EXPECT_TRUE(out.contains("acc 1-128"));
  EXPECT_EQ(sentCount(), 0u);
}

TEST_F(ConsoleTest, StepLimitIsMeasuredFromTheLastReading)
{
  run("enable 1");
  reply("001#F301F5");
  run("read 1");
  reply("001#31FFFFFFFFFFF01D");  // -16 counts
  can.sent.clear();
  run("move 1 16369 60 2");  // 16385 counts away
  EXPECT_EQ(sentCount(), 0u);
  EXPECT_TRUE(out.contains("16385 counts from the last reading -16"));
  run("move 1 16368 60 2");  // exactly one turn
  EXPECT_EQ(sentCount(), 1u);
}

TEST_F(ConsoleTest, DisableLocksMoveImmediately)
{
  arm();
  run("disable 1");
  run("read 1");
  reply("001#3100000000000032");
  can.sent.clear();
  run("move 1 100 60 2");
  EXPECT_EQ(sentCount(), 0u);
}

TEST_F(ConsoleTest, StopIsAlwaysAllowedAndUsesSpeedZero)
{
  run("stop 1");
  EXPECT_EQ(lastSent(), "001#F5000000000000F6");
  run("stop 1 4");
  EXPECT_EQ(lastSent(), "001#F5000004000000FA");  // manual vector absolute_axis(1, 0, 0, 4)
  run("stop 1 256");
  run("stop 0");
  EXPECT_EQ(sentCount(), 2u);
}

TEST_F(ConsoleTest, EstopSendsNothingBecauseF7hIsUndefined)
{
  run("estop 1");
  EXPECT_EQ(sentCount(), 0u);
  EXPECT_TRUE(out.contains("F7h emergency stop is NOT implemented"));
  EXPECT_TRUE(out.contains("PHYSICAL E-stop"));
}

TEST_F(ConsoleTest, RawSendsBytesExactlyButNotMotionCodes)
{
  run("raw 001#3133");  // deliberately bad checksum
  EXPECT_EQ(lastSent(), "001#3133");
  run("raw 001#F5025802004000");
  run("raw 001#F301F5");
  run("raw 000#3131");
  run("raw 001#");
  run("raw 800#31");
  run("raw 001#313");
  run("raw 001#313233343536373839");
  EXPECT_EQ(sentCount(), 1u);
}

TEST_F(ConsoleTest, ChecksumHelperAppendsTheMksChecksumWithoutSending)
{
  run("ck 001#31");
  EXPECT_TRUE(out.contains("CK 001#3132"));
  run("ck 001#F5025802004000");
  EXPECT_TRUE(out.contains("CK 001#F502580200400092"));
  EXPECT_EQ(sentCount(), 0u);
}

TEST_F(ConsoleTest, ReceivedFramesAreValidatedAndDecoded)
{
  run("read 1");
  reply("001#3133", 50);
  EXPECT_TRUE(out.contains("RX 001#3133 t=50 MKS checksum BAD (got 33, expected 32)"));
  reply("001#31", 51);
  EXPECT_TRUE(out.contains("not an MKS frame: bad DLC"));
  reply("001#31FFFFFFFFFFF01D", 52);
  EXPECT_TRUE(out.contains("READ_ENCODER value=-16 counts"));
  reply("001#F502F8", 53);
  EXPECT_TRUE(out.contains("ABSOLUTE_AXIS status=2 (run complete)"));
}

TEST_F(ConsoleTest, AnotherDrivesRepliesDoNotArmThisDrive)
{
  run("enable 1");
  run("read 1");
  reply("002#F301F6");
  reply("002#3100000000000033");
  EXPECT_FALSE(console.enabledConfirmed(1));
  EXPECT_FALSE(console.encoderFresh(1, 1000));
}

TEST_F(ConsoleTest, SendFailureIsReported)
{
  can.accept = false;
  run("read 1");
  EXPECT_TRUE(out.contains("SEND FAILED"));
}

TEST_F(ConsoleTest, UnknownCommandsAreLeftToTheSketch)
{
  std::string line = "status";
  EXPECT_FALSE(console.handleLine(&line[0], 0));
  EXPECT_EQ(sentCount(), 0u);
}

TEST(ConsoleHelpers, ParseIntegerIsStrict)
{
  int32_t value = 0;
  EXPECT_TRUE(BringupConsole::parseInteger("0x4000", value));
  EXPECT_EQ(value, 0x4000);
  EXPECT_TRUE(BringupConsole::parseInteger("-16384", value));
  EXPECT_EQ(value, -16384);
  EXPECT_TRUE(BringupConsole::parseInteger("08", value));  // decimal, never octal
  EXPECT_EQ(value, 8);
  EXPECT_TRUE(BringupConsole::parseInteger("-2147483648", value));
  EXPECT_EQ(value, INT32_MIN);
  EXPECT_FALSE(BringupConsole::parseInteger("2147483648", value));
  EXPECT_FALSE(BringupConsole::parseInteger("", value));
  EXPECT_FALSE(BringupConsole::parseInteger("-", value));
  EXPECT_FALSE(BringupConsole::parseInteger("0x", value));
  EXPECT_FALSE(BringupConsole::parseInteger("12a", value));
  EXPECT_FALSE(BringupConsole::parseInteger("1.5", value));
}

TEST(ConsoleHelpers, FormatInt64CoversTheEncoderRange)
{
  char text[24];
  BringupConsole::formatInt64(0, text, sizeof(text));
  EXPECT_STREQ(text, "0");
  BringupConsole::formatInt64(-16, text, sizeof(text));
  EXPECT_STREQ(text, "-16");
  BringupConsole::formatInt64(0x7FFFFFFFFFFFLL, text, sizeof(text));
  EXPECT_STREQ(text, "140737488355327");
  BringupConsole::formatInt64(-0x800000000000LL, text, sizeof(text));
  EXPECT_STREQ(text, "-140737488355328");
  BringupConsole::formatInt64(INT64_MIN, text, sizeof(text));
  EXPECT_STREQ(text, "-9223372036854775808");
}

TEST(ConsoleHelpers, FirstWordIsMatchesWholeWords)
{
  EXPECT_TRUE(BringupConsole::firstWordIs("  status", "status"));
  EXPECT_TRUE(BringupConsole::firstWordIs("poll 100", "poll"));
  EXPECT_FALSE(BringupConsole::firstWordIs("statusx", "status"));
  EXPECT_FALSE(BringupConsole::firstWordIs("stat", "status"));
}

TEST(Candump, ParsesAndRejects)
{
  Frame frame;
  ASSERT_TRUE(parseCandump("7ff#F502580200400092", frame));
  EXPECT_EQ(frame.id, 0x7FF);
  EXPECT_EQ(frame.dlc, 8u);
  EXPECT_EQ(frame.data[7], 0x92);
  ASSERT_TRUE(parseCandump("1#", frame));
  EXPECT_EQ(frame.dlc, 0u);
  EXPECT_FALSE(parseCandump("800#31", frame));
  EXPECT_FALSE(parseCandump("0001#31", frame));
  EXPECT_FALSE(parseCandump("#31", frame));
  EXPECT_FALSE(parseCandump("001", frame));
  EXPECT_FALSE(parseCandump("001#3", frame));
  EXPECT_FALSE(parseCandump("001#3G", frame));
  EXPECT_FALSE(parseCandump("001#313233343536373839", frame));
}

}  // namespace
}  // namespace waybionic
