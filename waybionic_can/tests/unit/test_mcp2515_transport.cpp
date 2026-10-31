// Mcp2515CanTransport against a simulated MCP2515 whose SPI link has the bench fault: a read
// comes back shifted one bit (the next register's MSB shifted in) unless it is a single byte
// right after a two-byte read of zeros. The real link is kinder; this model is the strict form
// of the rule the transport must follow, so every test below fails if any read breaks it.

#include <gtest/gtest.h>

#include <cstring>
#include <functional>
#include <initializer_list>
#include <vector>

#include "common/Mcp2515CanTransport.h"
#include "common/MksFrame.h"

namespace waybionic
{
namespace
{

constexpr uint8_t kCanStat = 0x0E;
constexpr uint8_t kCanCtrl = 0x0F;
constexpr uint8_t kTec = 0x1C;
constexpr uint8_t kRec = 0x1D;
constexpr uint8_t kCnf1 = 0x2A;
constexpr uint8_t kCanIntE = 0x2B;
constexpr uint8_t kCanIntF = 0x2C;
constexpr uint8_t kEflg = 0x2D;
constexpr uint8_t kTxb0Ctrl = 0x30;
constexpr uint8_t kTxb2Data = 0x56;
constexpr uint8_t kRxb0Ctrl = 0x60;
constexpr uint8_t kRxb0Sidh = 0x61;
constexpr uint8_t kRxb1Ctrl = 0x70;
constexpr uint8_t kRxb1Sidh = 0x71;

class FakeMcp2515 : public Mcp2515Spi
{
public:
  enum class Link {kClean, kSlipping, kBroken};

  FakeMcp2515()
  {
    std::memset(regs, 0xA5, sizeof(regs));
    reset();
  }

  void select() override
  {
    index_ = 0;
    instruction_ = 0;
    read_count_ = 0;
    read_all_zero_ = true;
  }

  uint8_t transfer(const uint8_t out) override
  {
    ++transfers;
    if (!present) {
      return 0xFF;
    }
    const size_t index = index_++;
    if (index == 0) {
      instruction_ = out;
      if (out == 0xC0) {
        reset();
      } else if (out == 0x81) {
        requestToSend();
      }
      return 0x00;
    }
    switch (instruction_) {
      case 0x03:  // READ
        if (index == 1) {
          address_ = out;
          return 0x00;
        }
        return readByte();
      case 0x02:  // WRITE
        if (index == 1) {
          address_ = out;
        } else {
          writeRegister(address_++, out, 0xFF);
        }
        return 0x00;
      case 0x05:  // BIT MODIFY
        if (index == 1) {
          address_ = out;
        } else if (index == 2) {
          mask_ = out;
        } else if (index == 3) {
          writeRegister(address_, out, mask_);
        }
        return 0x00;
      default:
        return 0x00;
    }
  }

  void deselect() override
  {
    conditioned_ = instruction_ == 0x03 && read_count_ == 2 && read_all_zero_;
  }

  bool interruptAsserted() override {return (regs[kCanIntF] & regs[kCanIntE]) != 0;}
  uint32_t millis() override {return now_ms++;}
  void delayMs(const uint32_t ms) override {now_ms += ms;}

  uint8_t mode() const {return regs[kCanStat] & 0xE0;}

  // A frame arriving from the bus, into RXB0 or (rollover) RXB1.
  bool deliver(
    const Frame & frame, const uint8_t sidl_flags = 0, const bool valid_can = true)
  {
    if (!valid_can) {
      ++regs[kRec];
      regs[kCanIntF] |= 0x80;  // MERRF
    }
    uint8_t base = 0;
    uint8_t flag = 0;
    if ((regs[kCanIntF] & 0x01) == 0) {
      base = kRxb0Sidh;
      flag = 0x01;
    } else if ((regs[kRxb0Ctrl] & 0x04) != 0 && (regs[kCanIntF] & 0x02) == 0) {
      base = kRxb1Sidh;
      flag = 0x02;
    } else {
      // Measured on the bench chip: with rollover, a frame finding both buffers full sets
      // RX0OVR (and ERRIF), not RX1OVR.
      regs[kEflg] |= 0x40;
      regs[kCanIntF] |= 0x20;
      return false;
    }
    // DS20001801J section 4.2.2: debug RXM=11 also accepts partial, errored messages.
    if (!valid_can && (regs[base - 1] & 0x60) != 0x60) {
      return false;
    }
    regs[base] = static_cast<uint8_t>(frame.id >> 3);
    regs[base + 1] = static_cast<uint8_t>(((frame.id & 0x07) << 5) | sidl_flags);
    regs[base + 2] = 0;
    regs[base + 3] = 0;
    regs[base + 4] = frame.dlc;
    for (uint8_t i = 0; i < kMaxCanDataBytes; ++i) {
      regs[base + 5 + i] = frame.data[i];
    }
    regs[kCanIntF] |= flag;
    return true;
  }

  // The pending TXB0 frame got its ACK.
  void acknowledge()
  {
    if (regs[kTxb0Ctrl] & 0x08) {
      transmitted.push_back(pending_);
      regs[kTxb0Ctrl] &= static_cast<uint8_t>(~0x08);
    }
  }

  uint8_t regs[128];
  // Frames on the wire: each lands the moment a receive buffer is freed.
  std::vector<Frame> arriving;
  Link link = Link::kSlipping;
  bool present = true;
  bool auto_ack = true;
  uint32_t now_ms = 1000;
  uint32_t transfers = 0;
  uint32_t aborted = 0;
  std::vector<Frame> transmitted;
  std::function<void(uint8_t)> on_read;

private:
  void reset()
  {
    // Data registers keep what they held; only the control registers have reset values.
    regs[kCanStat] = 0x80;
    regs[kCanCtrl] = 0x87;
    for (uint8_t a = 0x28; a <= 0x2D; ++a) {
      regs[a] = 0;
    }
    regs[kTec] = 0;
    regs[kTec + 1] = 0;
    regs[kTxb0Ctrl] = 0;
    regs[kRxb0Ctrl] = 0;
    regs[0x70] = 0;
  }

  uint8_t readByte()
  {
    if (on_read) {
      on_read(address_ & 0x7F);
    }
    const uint8_t value = regs[address_ & 0x7F];
    const uint8_t next = regs[(address_ + 1) & 0x7F];
    ++address_;
    ++read_count_;
    read_all_zero_ = read_all_zero_ && value == 0;
    const bool clean = link == Link::kClean ||
      (link == Link::kSlipping && read_count_ == 1 && conditioned_);
    return clean ? value : static_cast<uint8_t>((value << 1) | (next >> 7));
  }

  void writeRegister(const uint8_t address, const uint8_t value, const uint8_t mask)
  {
    const uint8_t a = address & 0x7F;
    const uint8_t merged = static_cast<uint8_t>((regs[a] & ~mask) | (value & mask));
    if (a == kCanStat) {
      return;  // read-only
    }
    if (a == kCanCtrl) {
      regs[a] = merged;
      regs[kCanStat] = static_cast<uint8_t>((regs[kCanStat] & 0x1F) | (merged & 0xE0));
      return;
    }
    if (a == kCanIntF) {
      const bool freed = (regs[a] & 0x03 & ~merged) != 0;
      regs[a] = merged;
      if (freed && !arriving.empty()) {
        deliver(arriving.front());
        arriving.erase(arriving.begin());
      }
      return;
    }
    if (a == kTxb0Ctrl) {
      if ((regs[a] & 0x08) && !(merged & 0x08)) {
        ++aborted;
        regs[a] = static_cast<uint8_t>((merged & ~0x08) | 0x40);  // ABTF
        return;
      }
      regs[a] = merged;
      return;
    }
    regs[a] = merged;
  }

  void requestToSend()
  {
    Frame frame;
    frame.id = static_cast<uint16_t>((regs[0x31] << 3) | (regs[0x32] >> 5));
    frame.dlc = regs[0x35] & 0x0F;
    for (uint8_t i = 0; i < kMaxCanDataBytes; ++i) {
      frame.data[i] = i < frame.dlc ? regs[0x36 + i] : 0;
    }
    regs[kTxb0Ctrl] = 0x08;
    if (mode() == 0x40) {
      regs[kTxb0Ctrl] = 0;
      deliver(frame);
      return;
    }
    pending_ = frame;
    if (auto_ack) {
      acknowledge();
    }
  }

  size_t index_ = 0;
  uint8_t instruction_ = 0;
  uint8_t address_ = 0;
  uint8_t mask_ = 0;
  uint8_t read_count_ = 0;
  bool read_all_zero_ = true;
  bool conditioned_ = false;
  Frame pending_;
};

Frame makeFrame(const uint16_t id, std::initializer_list<uint8_t> bytes)
{
  Frame frame;
  frame.id = id;
  for (const uint8_t b : bytes) {
    frame.data[frame.dlc++] = b;
  }
  return frame;
}

void expectSame(const Frame & expected, const Frame & got)
{
  EXPECT_EQ(expected.id, got.id);
  ASSERT_EQ(expected.dlc, got.dlc);
  for (uint8_t i = 0; i < expected.dlc; ++i) {
    EXPECT_EQ(expected.data[i], got.data[i]) << "byte " << int(i);
  }
}

TEST(Mcp2515Timing, EightQuantaPerBit)
{
  uint8_t c1 = 0;
  uint8_t c2 = 0;
  uint8_t c3 = 0;
  ASSERT_TRUE(Mcp2515CanTransport::timing(8000000, 500000, c1, c2, c3));
  EXPECT_EQ(0x00, c1);  // the register set the bench drives answered at, AA_MCP2515's 8 MHz/500k
  EXPECT_EQ(0x91, c2);
  EXPECT_EQ(0x01, c3);
  ASSERT_TRUE(Mcp2515CanTransport::timing(8000000, 250000, c1, c2, c3));
  EXPECT_EQ(0x01, c1);
  ASSERT_TRUE(Mcp2515CanTransport::timing(8000000, 125000, c1, c2, c3));
  EXPECT_EQ(0x03, c1);
  ASSERT_TRUE(Mcp2515CanTransport::timing(16000000, 1000000, c1, c2, c3));
  EXPECT_EQ(0x00, c1);
  ASSERT_TRUE(Mcp2515CanTransport::timing(16000000, 500000, c1, c2, c3));
  EXPECT_EQ(0x01, c1);
  EXPECT_FALSE(Mcp2515CanTransport::timing(8000000, 1000000, c1, c2, c3));
  EXPECT_FALSE(Mcp2515CanTransport::timing(8000000, 0, c1, c2, c3));
}

TEST(Mcp2515Fake, BurstAndUnconditionedReadsSlip)
{
  // Guards the model itself: without the method, the transport tests would prove nothing.
  FakeMcp2515 chip;
  chip.regs[0x40] = 0x31;
  chip.regs[0x41] = 0xFF;
  chip.regs[0x42] = 0x00;
  chip.select();
  chip.transfer(0x03);
  chip.transfer(0x40);
  const uint8_t first = chip.transfer(0);
  const uint8_t second = chip.transfer(0);
  chip.deselect();
  EXPECT_NE(0x31, first);
  EXPECT_NE(0xFF, second);
}

TEST(Mcp2515Transport, BeginConfiguresSelfTestsAndJoinsTheBus)
{
  FakeMcp2515 chip;
  Mcp2515CanTransport can(chip, 8000000);
  ASSERT_TRUE(can.begin(500000));
  EXPECT_TRUE(can.isOpen());
  EXPECT_EQ(500000u, can.bitrate());
  EXPECT_EQ(0x00, chip.mode());
  EXPECT_EQ(0x00, chip.regs[kCnf1]);
  EXPECT_EQ(0x91, chip.regs[kCnf1 - 1]);
  EXPECT_EQ(0x01, chip.regs[kCnf1 - 2]);
  EXPECT_EQ(0x03, chip.regs[kCanIntE]);
  EXPECT_EQ(0x04, chip.regs[kRxb0Ctrl]);
  EXPECT_EQ(0x00, chip.regs[kRxb1Ctrl]);
  // Reset does not initialize these in the fake; configuration must set every filter and
  // both complete masks, including the standard-frame data-byte mask bits.
  for (const uint8_t base : {0x00, 0x04, 0x08, 0x10, 0x14, 0x18, 0x20, 0x24}) {
    for (uint8_t i = 0; i < 4; ++i) {
      EXPECT_EQ(0, chip.regs[base + i]) << "register " << int(base + i);
    }
  }
  for (uint8_t i = 0; i < 8; ++i) {
    EXPECT_EQ(0x00, chip.regs[kTxb2Data + i]);
  }
  EXPECT_TRUE(can.lastSelfTest().passed());
  EXPECT_EQ(Mcp2515CanTransport::kSelfTestFrames, can.lastSelfTest().frames);
  EXPECT_TRUE(chip.transmitted.empty());  // the self-test never reaches the bus
  EXPECT_EQ(0u, can.linkFaults());
}

TEST(Mcp2515Transport, BeginAgainAtTheSameBitrateIsANoOp)
{
  FakeMcp2515 chip;
  Mcp2515CanTransport can(chip, 8000000);
  ASSERT_TRUE(can.begin(500000));
  const uint32_t transfers = chip.transfers;
  EXPECT_TRUE(can.begin(500000));
  EXPECT_EQ(transfers, chip.transfers);
}

TEST(Mcp2515Transport, RefusesAMissingChipAnUnreachableBitrateAndABrokenLink)
{
  FakeMcp2515 absent;
  absent.present = false;
  Mcp2515CanTransport no_chip(absent, 8000000);
  EXPECT_FALSE(no_chip.begin(500000));
  EXPECT_FALSE(no_chip.isOpen());
  EXPECT_EQ(1u, no_chip.linkFaults());

  FakeMcp2515 chip;
  Mcp2515CanTransport can(chip, 8000000);
  EXPECT_FALSE(can.begin(1000000));
  EXPECT_FALSE(can.isOpen());

  FakeMcp2515 broken;
  broken.link = FakeMcp2515::Link::kBroken;
  Mcp2515CanTransport noisy(broken, 8000000);
  EXPECT_FALSE(noisy.begin(500000));
  EXPECT_FALSE(noisy.isOpen());
  EXPECT_EQ(0x80, broken.mode());  // left off the bus
}

TEST(Mcp2515Transport, ReceivesExactFramesFromBothBuffersInOrder)
{
  FakeMcp2515 chip;
  Mcp2515CanTransport can(chip, 8000000);
  ASSERT_TRUE(can.begin(500000));
  Frame got;
  EXPECT_FALSE(can.receive(got, 0));

  const Frame reply = makeFrame(3, {0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x38});
  const Frame ones = makeFrame(0x7FF, {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF});
  ASSERT_TRUE(chip.deliver(reply));
  ASSERT_TRUE(chip.deliver(ones));  // RXB0 full: rolls over into RXB1
  ASSERT_TRUE(can.receive(got, 0));
  expectSame(reply, got);
  ASSERT_TRUE(can.receive(got, 0));
  expectSame(ones, got);
  EXPECT_FALSE(can.receive(got, 0));
  EXPECT_FALSE(chip.interruptAsserted());

  const Frame short_frame = makeFrame(0x123, {0x80});
  const Frame empty = makeFrame(0x001, {});
  ASSERT_TRUE(chip.deliver(short_frame));
  ASSERT_TRUE(can.receive(got, 0));
  expectSame(short_frame, got);
  ASSERT_TRUE(chip.deliver(empty));
  ASSERT_TRUE(can.receive(got, 0));
  expectSame(empty, got);
  EXPECT_EQ(4u, can.busLoad().totalFrames());
}

TEST(Mcp2515Transport, KeepsArrivalOrderWhenAFrameLandsAfterFlagClear)
{
  FakeMcp2515 chip;
  Mcp2515CanTransport can(chip, 8000000);
  ASSERT_TRUE(can.begin(500000));
  const Frame a = makeFrame(1, {0x31, 0x01});
  const Frame b = makeFrame(3, {0x31, 0x03});
  const Frame c = makeFrame(4, {0x31, 0x04});
  const Frame d = makeFrame(5, {0x31, 0x05});
  ASSERT_TRUE(chip.deliver(a));
  ASSERT_TRUE(chip.deliver(b));  // RXB1
  // C lands in RXB0 as soon as A is read out of it, while B still waits in RXB1; D then rolls
  // over into RXB1 once B is read.
  chip.arriving = {c, d};
  for (const Frame & expected : {a, b, c, d}) {
    Frame got;
    ASSERT_TRUE(can.receive(got, 0));
    expectSame(expected, got);
  }
  Frame got;
  EXPECT_FALSE(can.receive(got, 0));
  EXPECT_TRUE(chip.arriving.empty());
}

TEST(Mcp2515Transport, PendingBuffersStayLockedDuringRegisterReads)
{
  FakeMcp2515 chip;
  Mcp2515CanTransport can(chip, 8000000);
  ASSERT_TRUE(can.begin(500000));
  const Frame a = makeFrame(1, {0x31, 1, 2, 3, 4, 5, 6, 7});
  const Frame b = makeFrame(3, {0xF5, 0x02, 0xFA});
  ASSERT_TRUE(chip.deliver(a));
  ASSERT_TRUE(chip.deliver(b));
  bool injected = false;
  chip.on_read = [&](const uint8_t address) {
      if (!injected && address == kRxb0Sidh + 5) {
        injected = true;
        EXPECT_EQ(0x03, chip.regs[kCanIntF] & 0x03);
        EXPECT_FALSE(chip.deliver(makeFrame(5, {0xF5, 0x02, 0xFC})));
      }
    };
  for (const Frame & expected : {a, b}) {
    Frame got;
    ASSERT_TRUE(can.receive(got, 0));
    expectSame(expected, got);
  }
  EXPECT_TRUE(injected);
  Frame got;
  EXPECT_FALSE(can.receive(got, 0));
  for (int i = 0; i < 30; ++i) {
    can.service();
  }
  EXPECT_GT(can.receiveOverflows(), 0u);
}

TEST(Mcp2515Fake, DebugReceiveModeExposesErroredCanMessages)
{
  FakeMcp2515 chip;
  chip.regs[kRxb0Ctrl] = 0x64;
  chip.regs[kRxb1Ctrl] = 0x60;
  // Recorded at 1793447327.347629 in the bench host log. Its MKS checksum passes;
  // the log does not establish whether the original CAN message had a protocol error.
  const Frame partial = makeFrame(5, {0x31, 0x02, 0xFC, 0xFF, 0xFF, 0xE9, 0x16, 0x31});
  mks::Message message;
  ASSERT_EQ(mks::ParseResult::kOk, mks::parseFrame(partial, message));
  ASSERT_TRUE(chip.deliver(partial, 0, false));
  ASSERT_TRUE(chip.deliver(partial, 0, false));
  EXPECT_EQ(0x03, chip.regs[kCanIntF] & 0x03);
}

TEST(Mcp2515Transport, RejectsErroredCanMessagesEvenWithValidMksChecksums)
{
  for (const bool rollover : {false, true}) {
    FakeMcp2515 chip;
    Mcp2515CanTransport can(chip, 8000000);
    ASSERT_TRUE(can.begin(500000));
    const Frame good = makeFrame(5, {0x31, 0xFF, 0xFF, 0xFF, 0xFF, 0xE9, 0x16, 0x31});
    const Frame partial = makeFrame(5, {0x31, 0x02, 0xFC, 0xFF, 0xFF, 0xE9, 0x16, 0x31});
    if (rollover) {
      ASSERT_TRUE(chip.deliver(good));
    }
    ASSERT_FALSE(chip.deliver(partial, 0, false));
    ASSERT_TRUE(chip.deliver(good));
    for (int i = 0; i < (rollover ? 2 : 1); ++i) {
      Frame got;
      ASSERT_TRUE(can.receive(got, 0));
      expectSame(good, got);
    }
    Frame got;
    EXPECT_FALSE(can.receive(got, 0));
    for (int i = 0; i < 30; ++i) {
      can.service();
    }
    EXPECT_GT(can.errorEvents(), 0u);
    EXPECT_EQ(1, can.receiveErrorCount());
  }
}

TEST(Mcp2515Transport, DetectsAndRepairsUnexpectedDebugReceiveMode)
{
  for (const uint8_t ctrl : {kRxb0Ctrl, kRxb1Ctrl}) {
    FakeMcp2515 chip;
    Mcp2515CanTransport can(chip, 8000000);
    ASSERT_TRUE(can.begin(500000));
    chip.regs[ctrl] |= 0x60;
    const uint32_t until = chip.now_ms + 3 * Mcp2515CanTransport::kLinkCheckPeriodMs;
    while (chip.now_ms < until) {
      can.service();
    }
    EXPECT_GE(can.linkFaults(), 2u);
    EXPECT_EQ(1u, can.restarts());
    EXPECT_EQ(0, chip.regs[kRxb0Ctrl] & 0x60);
    EXPECT_EQ(0, chip.regs[kRxb1Ctrl] & 0x60);
  }
}

TEST(Mcp2515Transport, ReceiveStatusBitsDoNotTriggerLinkFaults)
{
  FakeMcp2515 chip;
  Mcp2515CanTransport can(chip, 8000000);
  ASSERT_TRUE(can.begin(500000));
  chip.regs[kRxb0Ctrl] |= 0x0B;  // RXRTR, BUKT1, FILHIT0
  chip.regs[kRxb1Ctrl] |= 0x0F;  // RXRTR, FILHIT
  const uint32_t until = chip.now_ms + 3 * Mcp2515CanTransport::kLinkCheckPeriodMs;
  while (chip.now_ms < until) {
    can.service();
  }
  EXPECT_EQ(0u, can.linkFaults());
  EXPECT_EQ(0u, can.restarts());
}

TEST(Mcp2515Transport, VerifiesEveryFilterAndMaskByteBeforeJoiningTheBus)
{
  for (const uint8_t base : {0x00, 0x04, 0x08, 0x10, 0x14, 0x18, 0x20, 0x24}) {
    for (uint8_t i = 0; i < 4; ++i) {
      FakeMcp2515 chip;
      chip.on_read = [&](const uint8_t address) {
          if (address == base + i) {
            chip.regs[address] |= 0x01;
          }
        };
      Mcp2515CanTransport can(chip, 8000000);
      EXPECT_FALSE(can.begin(500000)) << "register " << int(base + i);
      EXPECT_EQ(0x80, chip.mode());
      EXPECT_TRUE(chip.transmitted.empty());
      EXPECT_GT(can.linkFaults(), 0u);
    }
  }
}

TEST(Mcp2515Transport, ServiceQueuesFramesBeyondTheChipsTwoBuffers)
{
  FakeMcp2515 chip;
  Mcp2515CanTransport can(chip, 8000000);
  ASSERT_TRUE(can.begin(500000));
  std::vector<Frame> sent;
  for (uint8_t n = 0; n < Mcp2515CanTransport::kReceiveQueueCapacity + 2; n += 2) {
    for (uint8_t k = 0; k < 2; ++k) {
      sent.push_back(makeFrame(static_cast<uint16_t>(0x100 + n + k), {n, k}));
      ASSERT_TRUE(chip.deliver(sent.back()));
    }
    can.service();
  }
  // The queue is full and the chip holds the last two; one more frame is lost and counted.
  EXPECT_TRUE(chip.interruptAsserted());
  EXPECT_FALSE(chip.deliver(makeFrame(0x7FF, {})));
  for (int i = 0; i < 30; ++i) {
    can.service();
  }
  EXPECT_EQ(1u, can.receiveOverflows());
  for (const Frame & expected : sent) {
    Frame got;
    ASSERT_TRUE(can.receive(got, 0));
    expectSame(expected, got);
  }
  Frame got;
  EXPECT_FALSE(can.receive(got, 0));
}

TEST(Mcp2515Transport, DropsExtendedAndRemoteFrames)
{
  FakeMcp2515 chip;
  Mcp2515CanTransport can(chip, 8000000);
  ASSERT_TRUE(can.begin(500000));
  ASSERT_TRUE(chip.deliver(makeFrame(5, {1, 2}), 0x08));  // IDE
  ASSERT_TRUE(chip.deliver(makeFrame(6, {}), 0x10));      // SRR: standard remote frame
  Frame got;
  EXPECT_FALSE(can.receive(got, 0));
  EXPECT_EQ(2u, can.droppedFrames());
  EXPECT_FALSE(chip.interruptAsserted());
}

TEST(Mcp2515Transport, SendsInOrderThroughOneBuffer)
{
  FakeMcp2515 chip;
  chip.auto_ack = false;
  Mcp2515CanTransport can(chip, 8000000);
  ASSERT_TRUE(can.begin(500000));
  const Frame first = makeFrame(1, {0x31, 0x32});
  const Frame second = makeFrame(4, {0xF5, 0x01, 0x2C, 0x02, 0x00, 0x40, 0x00, 0x60});
  ASSERT_TRUE(can.send(first));
  EXPECT_FALSE(can.send(second));  // TXB0 still busy
  chip.acknowledge();
  ASSERT_TRUE(can.send(second));
  chip.acknowledge();
  ASSERT_EQ(2u, chip.transmitted.size());
  expectSame(first, chip.transmitted[0]);
  expectSame(second, chip.transmitted[1]);
  EXPECT_EQ(0u, can.failedWrites());

  Frame invalid;
  invalid.id = 0x800;
  EXPECT_FALSE(can.send(invalid));
}

TEST(Mcp2515Transport, AbortsAFrameNobodyAcknowledges)
{
  FakeMcp2515 chip;
  chip.auto_ack = false;
  Mcp2515CanTransport can(chip, 8000000);
  ASSERT_TRUE(can.begin(500000));
  ASSERT_TRUE(can.send(makeFrame(1, {0xF5, 0x00})));
  const uint32_t sent_at = chip.now_ms;
  while (chip.now_ms - sent_at < Mcp2515CanTransport::kTransmitTimeoutMs + 5) {
    can.service();
  }
  EXPECT_EQ(1u, chip.aborted);
  EXPECT_EQ(1u, can.failedWrites());
  chip.acknowledge();  // the drive powers up: the stale frame must not go out now
  EXPECT_TRUE(chip.transmitted.empty());
  chip.auto_ack = true;
  ASSERT_TRUE(can.send(makeFrame(1, {0x31, 0x32})));
  ASSERT_EQ(1u, chip.transmitted.size());
}

TEST(Mcp2515Transport, CountsErrorStatesAndReceiveOverflows)
{
  FakeMcp2515 chip;
  Mcp2515CanTransport can(chip, 8000000);
  ASSERT_TRUE(can.begin(500000));
  chip.regs[kTec] = 128;
  chip.regs[kEflg] = 0x15 | 0x40;  // error passive and warning, RXB0 overflowed
  for (int i = 0; i < 30; ++i) {
    can.service();
  }
  EXPECT_EQ(1u, can.errorEvents());
  EXPECT_EQ(1u, can.receiveOverflows());
  EXPECT_EQ(128, can.transmitErrorCount());
  EXPECT_EQ(0x15, can.errorFlags());
  EXPECT_EQ(0x15, chip.regs[kEflg]);  // overflow flag cleared, state flags are the chip's
}

TEST(Mcp2515Transport, RestartsAChipThatLostItsConfiguration)
{
  FakeMcp2515 chip;
  Mcp2515CanTransport can(chip, 8000000);
  ASSERT_TRUE(can.begin(500000));
  // A brownout resets the chip: configuration mode, timing registers cleared.
  chip.select();
  chip.transfer(0xC0);
  chip.deselect();
  ASSERT_EQ(0x80, chip.mode());
  const uint32_t until = chip.now_ms + 3 * Mcp2515CanTransport::kLinkCheckPeriodMs;
  while (chip.now_ms < until) {
    can.service();
  }
  EXPECT_GE(can.linkFaults(), 2u);
  EXPECT_EQ(1u, can.restarts());
  EXPECT_TRUE(can.isOpen());
  EXPECT_EQ(0x00, chip.mode());
  EXPECT_EQ(0x00, chip.regs[0x2A]);
  EXPECT_EQ(0x91, chip.regs[0x29]);
}

TEST(Mcp2515Transport, EndLeavesTheBus)
{
  FakeMcp2515 chip;
  Mcp2515CanTransport can(chip, 8000000);
  ASSERT_TRUE(can.begin(500000));
  can.end();
  EXPECT_FALSE(can.isOpen());
  EXPECT_EQ(0x80, chip.mode());
  EXPECT_FALSE(can.send(makeFrame(1, {0x31, 0x32})));
  for (int i = 0; i < 300; ++i) {
    can.service();
  }
  EXPECT_EQ(0x80, chip.mode());
}

}  // namespace
}  // namespace waybionic
