// SlcanBridge: the SLCAN subset behind carrier_bridge.ino. The command sequences mirror what
// python-can's can/interfaces/slcan.py writes.

#include <gtest/gtest.h>

#include <deque>
#include <string>
#include <vector>

#include "common/MksFrame.h"
#include "common/SlcanBridge.h"

namespace waybionic
{
namespace
{

class FakeCan : public ICanTransport, public CanStarter
{
public:
  bool start(uint32_t bitrate) override
  {
    starts.push_back(bitrate);
    return start_ok;
  }
  bool send(const Frame & frame) override
  {
    sent.push_back(frame);
    return send_ok;
  }
  bool receive(Frame & frame, uint32_t) override
  {
    if (inbox.empty()) {
      return false;
    }
    frame = inbox.front();
    inbox.pop_front();
    return true;
  }

  std::vector<uint32_t> starts;
  std::vector<Frame> sent;
  std::deque<Frame> inbox;
  bool start_ok = true;
  bool send_ok = true;
};

class StringOutput : public SlcanOutput
{
public:
  void write(const char * text) override {data += text;}
  std::string take()
  {
    std::string out;
    out.swap(data);
    return out;
  }
  std::string data;
};

class SlcanTest : public ::testing::Test
{
protected:
  SlcanTest()
  : bridge(can, can, out) {}

  std::string feed(const std::string & bytes)
  {
    for (const char c : bytes) {
      bridge.handleChar(c);
    }
    return out.take();
  }

  // What slcanBus.__init__(bitrate=500000) sends: close, S6, empty data-bitrate code, open.
  void openLikePythonCan()
  {
    EXPECT_EQ(feed("C\rS6\r\rO\r"), "\r\r\r\r");
  }

  FakeCan can;
  StringOutput out;
  SlcanBridge bridge;
};

TEST_F(SlcanTest, ClosedAtBootAndSendsNothing)
{
  EXPECT_FALSE(bridge.isOpen());
  EXPECT_EQ(bridge.bitrate(), 500000u);
  bridge.pollCan();
  EXPECT_TRUE(can.starts.empty());
  EXPECT_TRUE(can.sent.empty());
  EXPECT_EQ(out.take(), "");
}

TEST_F(SlcanTest, PythonCanStartupSequenceOpensAt500k)
{
  openLikePythonCan();
  EXPECT_TRUE(bridge.isOpen());
  ASSERT_EQ(can.starts.size(), 1u);
  EXPECT_EQ(can.starts[0], 500000u);
}

TEST_F(SlcanTest, BitrateCodesFollowPythonCan)
{
  EXPECT_EQ(feed("S4\r"), "\r");
  EXPECT_EQ(bridge.bitrate(), 125000u);
  EXPECT_EQ(feed("S5\r"), "\r");
  EXPECT_EQ(bridge.bitrate(), 250000u);
  EXPECT_EQ(feed("S8\r"), "\r");
  EXPECT_EQ(bridge.bitrate(), 1000000u);
  // 10k-100k, 750k and 83.3k are not CanBitRate values on the UNO R4.
  for (const char * code : {"S0\r", "S1\r", "S2\r", "S3\r", "S7\r", "S9\r", "SA\r", "S\r", "S66\r"}) {
    EXPECT_EQ(feed(code), "\a") << code;
  }
  EXPECT_EQ(bridge.bitrate(), 1000000u);
}

TEST_F(SlcanTest, BitrateCannotChangeWhileOpen)
{
  openLikePythonCan();
  EXPECT_EQ(feed("S8\r"), "\a");
  EXPECT_EQ(bridge.bitrate(), 500000u);
  EXPECT_EQ(feed("O\r"), "\a");  // already open
}

TEST_F(SlcanTest, OpenFailsWhenTheControllerCannotStart)
{
  can.start_ok = false;
  EXPECT_EQ(feed("O\r"), "\a");
  EXPECT_FALSE(bridge.isOpen());
}

TEST_F(SlcanTest, TransmitSendsTheExactFrame)
{
  openLikePythonCan();
  // Successful sends are silent; only refusals are answered.
  EXPECT_EQ(feed("t00123132\r"), "");
  ASSERT_EQ(can.sent.size(), 1u);
  EXPECT_EQ(can.sent[0].id, 1u);
  EXPECT_EQ(can.sent[0].dlc, 2u);
  EXPECT_EQ(can.sent[0].data[0], 0x31);
  EXPECT_EQ(can.sent[0].data[1], 0x32);
  // python-can formats IDs and data in upper case; lower case is accepted too.
  EXPECT_EQ(feed("t7ff8f502580200400092\r"), "");
  EXPECT_EQ(can.sent[1].id, 0x7FF);
  EXPECT_EQ(can.sent[1].dlc, 8u);
  EXPECT_EQ(can.sent[1].data[7], 0x92);
  EXPECT_EQ(feed("t0010\r"), "");
  EXPECT_EQ(can.sent[2].dlc, 0u);
}

TEST_F(SlcanTest, TransmitIsRefusedWhileClosed)
{
  EXPECT_EQ(feed("t00123132\r"), "\a");
  openLikePythonCan();
  EXPECT_EQ(feed("C\r"), "\r");
  EXPECT_EQ(feed("t00123132\r"), "\a");
  EXPECT_TRUE(can.sent.empty());
}

TEST_F(SlcanTest, MalformedTransmitIsRefused)
{
  openLikePythonCan();
  for (const char * line : {
      "t800131\r",          // ID above 7FF
      "t001231\r",          // fewer bytes than the DLC
      "t00113132\r",        // more bytes than the DLC
      "t0019313233343536373839\r",  // DLC 9
      "t0012313G\r",        // not hex
      "t01\r",              // too short
    })
  {
    EXPECT_EQ(feed(line), "\a") << line;
  }
  EXPECT_TRUE(can.sent.empty());
}

TEST_F(SlcanTest, ControllerWriteFailureIsAnError)
{
  openLikePythonCan();
  can.send_ok = false;
  EXPECT_EQ(feed("t00123132\r"), "\a");
}

TEST_F(SlcanTest, UnsupportedCommandsAnswerBell)
{
  openLikePythonCan();
  for (const char * line : {
      "T0000000123132\r",  // extended ID
      "r0010\r",           // remote frame
      "R000000010\r",      // extended remote frame
      "L\r",               // listen-only
      "s031C\r",           // BTR registers
      "F\r",               // status flags
      "Z1\r",              // timestamps
      "d0012AABB\r",       // CAN FD
      "x\r",
    })
  {
    EXPECT_EQ(feed(line), "\a") << line;
  }
  EXPECT_TRUE(can.sent.empty());
}

TEST_F(SlcanTest, VersionAndSerialNumberMatchWhatPythonCanParses)
{
  // slcanBus.get_version reads int(string[1:3]) and int(string[3:5]).
  EXPECT_EQ(feed("V\r"), "V0100\r");
  EXPECT_EQ(feed("N\r"), "NWB01\r");
}

TEST_F(SlcanTest, ReceivedFramesAreForwardedOnlyWhileOpen)
{
  Frame reply;
  ASSERT_TRUE(mks::encoderReply(1, -16, reply));
  can.inbox.push_back(reply);
  bridge.pollCan();
  EXPECT_EQ(out.take(), "");
  EXPECT_TRUE(can.inbox.empty());  // drained, not queued for later

  openLikePythonCan();
  can.inbox.push_back(reply);
  Frame status;
  ASSERT_TRUE(mks::statusReply(1, mks::kEnable, 1, status));
  can.inbox.push_back(status);
  bridge.pollCan();
  EXPECT_EQ(out.take(), "t001831FFFFFFFFFFF01D\rt0013F301F5\r");
}

TEST_F(SlcanTest, LineFeedsAreIgnoredAndOverlongLinesRejected)
{
  EXPECT_EQ(feed("V\r\n"), "V0100\r");
  EXPECT_EQ(feed(std::string(40, 'A') + "\r"), "\a");
  EXPECT_EQ(feed("V\r"), "V0100\r");  // recovers on the next line
}

TEST_F(SlcanTest, RefusedLinesAreCounted)
{
  EXPECT_EQ(feed("t00123132\r"), "\a");  // still closed
  EXPECT_EQ(feed("x\r"), "\a");
  EXPECT_EQ(feed("V\r"), "V0100\r");
  EXPECT_EQ(bridge.refusedLines(), 2u);
}

TEST_F(SlcanTest, StatusGoesOnlyToTheHostAndOnlyWhileOpen)
{
  CarrierStatus status;
  bridge.reportStatus(status);
  EXPECT_EQ(out.take(), "");

  openLikePythonCan();
  EXPECT_EQ(feed("x\r"), "\a");
  status.estop_wired = true;
  status.supply_wired = true;
  status.supply_millivolts = 24000;
  status.can_errors = 70000;  // saturates at FFFF
  status.failed_writes = 3;
  bridge.reportStatus(status);
  EXPECT_EQ(out.take(), "t7F0800055DC0FFFF0301\r");
  status = CarrierStatus{};
  status.estop_wired = true;
  status.estop_pressed = true;
  bridge.reportStatus(status);
  EXPECT_EQ(out.take(), "t7F080103000000000001\r");
  EXPECT_TRUE(can.sent.empty());
}

TEST_F(SlcanTest, UnknownSafetySignalsCannotLookHealthyOrMeasured)
{
  openLikePythonCan();
  CarrierStatus status;
  status.estop_pressed = true;       // invalid without estop_wired
  status.supply_millivolts = 24000;  // invalid without supply_wired
  bridge.reportStatus(status);
  EXPECT_EQ(out.take(), "t7F080000000000000000\r");
}

TEST(SlcanFormat, RoundTripsEveryLength)
{
  for (uint8_t dlc = 0; dlc <= 8; ++dlc) {
    Frame frame;
    frame.id = 0x7FF;
    frame.dlc = dlc;
    for (uint8_t i = 0; i < dlc; ++i) {
      frame.data[i] = static_cast<uint8_t>(0xA0 + i);
    }
    char text[24];
    SlcanBridge::formatFrame(frame, text, sizeof(text));
    const std::string line(text);
    ASSERT_EQ(line.back(), '\r');
    Frame parsed;
    ASSERT_TRUE(SlcanBridge::parseTransmit(text, line.size() - 1, parsed));
    EXPECT_EQ(parsed.id, frame.id);
    EXPECT_EQ(parsed.dlc, frame.dlc);
    for (uint8_t i = 0; i < dlc; ++i) {
      EXPECT_EQ(parsed.data[i], frame.data[i]);
    }
  }
}

TEST(SlcanFormat, ShortBufferWritesNothing)
{
  Frame frame;
  frame.dlc = 8;
  char text[10] = "x";
  SlcanBridge::formatFrame(frame, text, sizeof(text));
  EXPECT_STREQ(text, "");
}

}  // namespace
}  // namespace waybionic
