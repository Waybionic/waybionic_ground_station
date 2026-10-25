// Mcp2515CanTransport: carrier_bridge_mcp2515's link to the arm's MCP2515 module, run against a
// modeled chip (fake_mcp2515/AA_MCP2515.h) in place of SPI.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "carrier_bridge_mcp2515/Mcp2515CanTransport.h"
#include "common/SlcanBridge.h"

uint32_t millis()
{
  static uint32_t now = 0;
  return ++now;
}

namespace waybionic
{
namespace
{

constexpr uint8_t kSidlRemote = 0x10;
constexpr uint8_t kSidlExtended = 0x08;

Frame standard(uint16_t id, const std::vector<uint8_t> & data)
{
  Frame frame;
  frame.id = id;
  frame.dlc = static_cast<uint8_t>(data.size());
  for (size_t i = 0; i < data.size(); ++i) {
    frame.data[i] = data[i];
  }
  return frame;
}

std::vector<uint8_t> cnf(const FakeMcp2515 & chip)
{
  return {chip.registers[CNF1], chip.registers[CNF2], chip.registers[CNF3]};
}

class Mcp2515TransportTest : public ::testing::Test
{
protected:
  void SetUp() override {fakeChip() = FakeMcp2515{};}

  FakeMcp2515 & chip() {return fakeChip();}

  void open()
  {
    ASSERT_TRUE(can.setup());
    ASSERT_TRUE(can.begin(500000));
  }

  Mcp2515CanTransport can{10, 2};
};

TEST_F(Mcp2515TransportTest, StaysOffTheBusUntilOpened)
{
  ASSERT_TRUE(can.setup());
  EXPECT_EQ(chip().mode(), kModeConfig);
  EXPECT_FALSE(can.isOpen());
  EXPECT_FALSE(can.send(standard(0x001, {0x31, 0x32})));
  EXPECT_TRUE(chip().transmitted.empty());
  EXPECT_FALSE(chip().arrive(0x001, {0x31}));
}

TEST_F(Mcp2515TransportTest, OpensAtTheDrivesBitrateWithRollover)
{
  open();
  EXPECT_EQ(cnf(chip()), (std::vector<uint8_t>{0x00, 0x91, 0x01}));
  EXPECT_NE(chip().registers[RXB0CTRL] & RXB0CTRL_BUKT, 0);
  EXPECT_EQ(chip().mode(), kModeNormal);
  EXPECT_EQ(can.bitrate(), 500000u);
  EXPECT_TRUE(can.begin(500000));
  // The module's 8 MHz crystal cannot run 1 Mbit/s; the bus stays as it was.
  EXPECT_FALSE(can.begin(1000000));
  EXPECT_TRUE(can.isOpen());
  EXPECT_EQ(chip().mode(), kModeNormal);
}

TEST_F(Mcp2515TransportTest, ChangesBitrateOnceTheChipIsInConfigurationMode)
{
  open();
  chip().mode_delay = 3;
  ASSERT_TRUE(can.begin(250000));
  EXPECT_EQ(cnf(chip()), (std::vector<uint8_t>{0x00, 0xac, 0x03}));
  EXPECT_EQ(chip().mode(), kModeNormal);
  EXPECT_EQ(can.bitrate(), 250000u);
}

TEST_F(Mcp2515TransportTest, GivesUpOnAChipThatDoesNotChangeMode)
{
  open();
  chip().mode_delay = 1000;
  EXPECT_FALSE(can.begin(125000));
  EXPECT_FALSE(can.isOpen());
  EXPECT_FALSE(can.send(standard(0x001, {0x31, 0x32})));
  EXPECT_EQ(can.failedWrites(), 1u);
}

TEST_F(Mcp2515TransportTest, ResetsAChipThatDidNotAnswerAtBoot)
{
  chip().present = false;
  EXPECT_FALSE(can.setup());
  EXPECT_FALSE(can.begin(500000));
  chip().present = true;
  ASSERT_TRUE(can.begin(500000));
  EXPECT_EQ(chip().mode(), kModeNormal);
}

TEST_F(Mcp2515TransportTest, SendsOneFrameAtATimeFromTheFirstBuffer)
{
  open();
  ASSERT_TRUE(can.send(standard(0x123, {0xF5, 0x01, 0x02})));
  ASSERT_EQ(chip().transmitted.size(), 1u);
  EXPECT_EQ(chip().transmitted[0], (std::vector<uint8_t>{0x24, 0x60, 0, 0, 3, 0xF5, 0x01, 0x02}));
  // Busy until that frame has left the bus, so the host's frames go out in order.
  EXPECT_FALSE(can.send(standard(0x001, {0x31, 0x32})));
  EXPECT_EQ(chip().transmitted.size(), 1u);
  chip().finishTransmit();
  ASSERT_TRUE(can.send(standard(0x001, {0x31, 0x32})));
  ASSERT_EQ(chip().transmitted.size(), 2u);
  EXPECT_EQ(chip().transmitted[1], (std::vector<uint8_t>{0x00, 0x20, 0, 0, 2, 0x31, 0x32}));
  EXPECT_EQ(can.failedWrites(), 0u);
}

TEST_F(Mcp2515TransportTest, ReceivesOldestFirstAcrossBothBuffers)
{
  open();
  ASSERT_TRUE(chip().arrive(0x001, {0xA1}));
  // The first buffer is full, so this one rolls over into the second.
  ASSERT_TRUE(chip().arrive(0x002, {0xB2}));
  Frame frame;
  ASSERT_TRUE(can.receive(frame, 0));
  EXPECT_EQ(frame.id, 0x001);
  // Lands in the emptied first buffer, behind the one still waiting in the second.
  ASSERT_TRUE(chip().arrive(0x003, {0x31, 0, 0, 0, 0, 0, 0x10, 0x44}));
  ASSERT_TRUE(can.receive(frame, 0));
  EXPECT_EQ(frame.id, 0x002);
  ASSERT_TRUE(can.receive(frame, 0));
  EXPECT_EQ(frame.id, 0x003);
  ASSERT_EQ(frame.dlc, 8);
  EXPECT_EQ(frame.data[0], 0x31);
  EXPECT_EQ(frame.data[7], 0x44);
  EXPECT_FALSE(can.receive(frame, 0));
}

TEST_F(Mcp2515TransportTest, DropsExtendedAndRemoteFrames)
{
  open();
  ASSERT_TRUE(chip().arrive(0x010, {}, kSidlExtended));
  ASSERT_TRUE(chip().arrive(0x011, {}, kSidlRemote));
  Frame frame;
  EXPECT_FALSE(can.receive(frame, 0));
  EXPECT_EQ(can.droppedFrames(), 2u);
  ASSERT_TRUE(chip().arrive(0x005, {0xF3, 0x01, 0xF9}));
  ASSERT_TRUE(can.receive(frame, 0));
  EXPECT_EQ(frame.id, 0x005);
}

TEST_F(Mcp2515TransportTest, CountsEachLostFrameAndErrorStateOnce)
{
  open();
  int code = 0;
  EXPECT_FALSE(can.takeError(code));
  ASSERT_TRUE(chip().arrive(0x001, {0x01}));
  ASSERT_TRUE(chip().arrive(0x002, {0x02}));
  EXPECT_FALSE(chip().arrive(0x003, {0x03}));
  EXPECT_TRUE(can.takeError(code));
  EXPECT_EQ(code & EFLG_RX1OVR, EFLG_RX1OVR);
  EXPECT_EQ(chip().registers[EFLG] & EFLG_RX1OVR, 0);
  EXPECT_FALSE(can.takeError(code));
  chip().registers[EFLG] |= EFLG_TXEP;
  EXPECT_TRUE(can.takeError(code));
  // Still error passive: not a new event.
  EXPECT_FALSE(can.takeError(code));
  EXPECT_EQ(can.errorEvents(), 2u);
}

class StringOutput : public SlcanOutput
{
public:
  void write(const char * text) override {written += text;}
  std::string written;
};

class Starter : public CanStarter
{
public:
  explicit Starter(Mcp2515CanTransport & can)
  : can_(can) {}
  bool start(uint32_t bitrate) override {return can_.begin(bitrate);}

private:
  Mcp2515CanTransport & can_;
};

void feed(SlcanBridge & bridge, const std::string & text)
{
  for (char c : text) {
    bridge.handleChar(c, 0);
  }
}

TEST_F(Mcp2515TransportTest, CarriesTheHostsFramesBothWays)
{
  ASSERT_TRUE(can.setup());
  Starter starter(can);
  StringOutput out;
  SlcanBridge bridge(can, starter, out);
  // What the ground station writes when it opens the carrier at 500 kbit/s.
  feed(bridge, "C\rS6\rO\r");
  EXPECT_EQ(out.written, "\r\r\r");
  EXPECT_EQ(chip().mode(), kModeNormal);
  EXPECT_EQ(cnf(chip()), (std::vector<uint8_t>{0x00, 0x91, 0x01}));

  out.written.clear();
  feed(bridge, "t00123132\r\r");
  bridge.drainTransmit(0);
  ASSERT_EQ(chip().transmitted.size(), 1u);
  EXPECT_EQ(chip().transmitted[0], (std::vector<uint8_t>{0x00, 0x20, 0, 0, 2, 0x31, 0x32}));
  EXPECT_EQ(out.written, "\r");

  out.written.clear();
  ASSERT_TRUE(chip().arrive(0x001, {0x31, 0, 0, 0, 0, 0x40, 0x00, 0x72}));
  bridge.pollCan();
  EXPECT_EQ(out.written, "t00183100000000400072\r");
}

TEST_F(Mcp2515TransportTest, RefusesToOpenAt1Mbit)
{
  ASSERT_TRUE(can.setup());
  Starter starter(can);
  StringOutput out;
  SlcanBridge bridge(can, starter, out);
  feed(bridge, "S8\rO\r");
  EXPECT_EQ(out.written, "\r\a");
  EXPECT_EQ(chip().mode(), kModeConfig);
}

}  // namespace
}  // namespace waybionic
