#include "common/Mcp2515CanTransport.h"

namespace waybionic
{
namespace
{

// SPI instructions
constexpr uint8_t kReset = 0xC0;
constexpr uint8_t kRead = 0x03;
constexpr uint8_t kWrite = 0x02;
constexpr uint8_t kBitModify = 0x05;
constexpr uint8_t kRequestToSend0 = 0x81;

// Registers
constexpr uint8_t kCanStat = 0x0E;
constexpr uint8_t kCanCtrl = 0x0F;
constexpr uint8_t kTec = 0x1C;
constexpr uint8_t kRec = 0x1D;
constexpr uint8_t kCnf3 = 0x28;
constexpr uint8_t kCnf2 = 0x29;
constexpr uint8_t kCnf1 = 0x2A;
constexpr uint8_t kCanIntE = 0x2B;
constexpr uint8_t kCanIntF = 0x2C;
constexpr uint8_t kEflg = 0x2D;
constexpr uint8_t kTxb0Ctrl = 0x30;
constexpr uint8_t kTxb0Sidh = 0x31;
constexpr uint8_t kTxb2Data = 0x56;
constexpr uint8_t kRxb0Ctrl = 0x60;
constexpr uint8_t kRxb0Sidh = 0x61;
constexpr uint8_t kRxb1Ctrl = 0x70;
constexpr uint8_t kRxb1Sidh = 0x71;
// Six acceptance filters and two masks, four registers each.
constexpr uint8_t kAcceptanceBases[] = {0x00, 0x04, 0x08, 0x10, 0x14, 0x18, 0x20, 0x24};

// TXB2's data bytes are never transmitted. They hold zeros so that reading two of them leaves
// the chip's SO output low before the read that matters.
constexpr uint8_t kZeroScratch = kTxb2Data + 2;

constexpr uint8_t kModeMask = 0xE0;
constexpr uint8_t kModeNormal = 0x00;
constexpr uint8_t kModeLoopback = 0x40;
constexpr uint8_t kModeConfig = 0x80;

constexpr uint8_t kRx0If = 0x01;
constexpr uint8_t kRx1If = 0x02;
constexpr uint8_t kTxReq = 0x08;
constexpr uint8_t kSidlSrr = 0x10;
constexpr uint8_t kSidlIde = 0x08;
constexpr uint8_t kRx0Ovr = 0x40;
constexpr uint8_t kRx1Ovr = 0x80;
constexpr uint8_t kErrorStateMask = 0x3F;  // EWARN, RXWAR, TXWAR, RXEP, TXEP, TXBO

constexpr uint8_t kRxModeMask = 0x60;
constexpr uint8_t kRollover = 0x04;

constexpr uint32_t kOscillatorStartMs = 5;
constexpr uint32_t kModeChangeMs = 10;
constexpr uint32_t kSelfTestWaitMs = 5;

uint32_t nextRandom(uint32_t & state)
{
  state ^= state << 13;
  state ^= state >> 17;
  state ^= state << 5;
  return state;
}

// The patterns the bench link got wrong most: all ones, alternating bits, a single set MSB, the
// drives' 31h replies around zero, and random bytes.
void selfTestFrame(const uint16_t n, uint32_t & random, Frame & frame)
{
  frame = Frame{};
  frame.id = static_cast<uint16_t>((n * 0x2B5u + 1u) & kMaxStandardCanId);
  frame.dlc = (n % 8 == 7) ? static_cast<uint8_t>(n % (kMaxCanDataBytes + 1)) : kMaxCanDataBytes;
  for (uint8_t i = 0; i < kMaxCanDataBytes; ++i) {
    uint8_t value = 0;
    switch (n % 6) {
      case 0: value = 0xFF; break;
      case 1: value = 0x00; break;
      case 2: value = (i % 2) ? 0x55 : 0xAA; break;
      case 3: value = static_cast<uint8_t>(0x80u >> i); break;
      case 4: value = (i == 0) ? 0x31 : ((n / 6) % 2 ? 0xFF : 0x00); break;
      default: value = static_cast<uint8_t>(nextRandom(random)); break;
    }
    frame.data[i] = i < frame.dlc ? value : 0;
  }
  if (n % 6 == 4 && frame.dlc == kMaxCanDataBytes) {
    uint32_t sum = frame.id;
    for (uint8_t i = 0; i + 1 < kMaxCanDataBytes; ++i) {
      sum += frame.data[i];
    }
    frame.data[kMaxCanDataBytes - 1] = static_cast<uint8_t>(sum);
  }
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

}  // namespace

Mcp2515CanTransport::Mcp2515CanTransport(Mcp2515Spi & spi, const uint32_t crystal_hz)
: spi_(spi), crystal_hz_(crystal_hz)
{
}

bool Mcp2515CanTransport::timing(
  const uint32_t crystal_hz, const uint32_t bitrate, uint8_t & cnf1, uint8_t & cnf2,
  uint8_t & cnf3)
{
  // One time quantum is 2 * (BRP + 1) oscillator periods and a bit is 8 quanta: sync 1,
  // propagation 2, phase 1 3, phase 2 2.
  constexpr uint32_t kQuantaPerBit = 8;
  if (bitrate == 0 || crystal_hz % (2 * kQuantaPerBit * bitrate) != 0) {
    return false;
  }
  const uint32_t prescaler = crystal_hz / (2 * kQuantaPerBit * bitrate);
  if (prescaler < 1 || prescaler > 64) {
    return false;
  }
  cnf1 = static_cast<uint8_t>(prescaler - 1);  // SJW 1 quantum
  cnf2 = 0x91;  // BTLMODE, phase 1 = 3, propagation = 2
  cnf3 = 0x01;  // phase 2 = 2
  return true;
}

uint8_t Mcp2515CanTransport::readRegister(const uint8_t address)
{
  spi_.select();
  spi_.transfer(kRead);
  spi_.transfer(kZeroScratch);
  spi_.transfer(0x00);
  spi_.transfer(0x00);
  spi_.deselect();

  spi_.select();
  spi_.transfer(kRead);
  spi_.transfer(address);
  const uint8_t value = spi_.transfer(0x00);
  spi_.deselect();
  return value;
}

void Mcp2515CanTransport::writeRegisters(
  const uint8_t address, const uint8_t * data, const uint8_t length)
{
  spi_.select();
  spi_.transfer(kWrite);
  spi_.transfer(address);
  for (uint8_t i = 0; i < length; ++i) {
    spi_.transfer(data[i]);
  }
  spi_.deselect();
}

void Mcp2515CanTransport::bitModify(const uint8_t address, const uint8_t mask, const uint8_t value)
{
  spi_.select();
  spi_.transfer(kBitModify);
  spi_.transfer(address);
  spi_.transfer(mask);
  spi_.transfer(value);
  spi_.deselect();
}

void Mcp2515CanTransport::instruction(const uint8_t code)
{
  spi_.select();
  spi_.transfer(code);
  spi_.deselect();
}

bool Mcp2515CanTransport::setMode(const uint8_t mode)
{
  bitModify(kCanCtrl, kModeMask, mode);
  const uint32_t start = spi_.millis();
  do {
    if ((readRegister(kCanStat) & kModeMask) == mode) {
      return true;
    }
  } while (spi_.millis() - start < kModeChangeMs);
  return false;
}

bool Mcp2515CanTransport::configure(const uint8_t mode)
{
  instruction(kReset);
  spi_.delayMs(kOscillatorStartMs);
  // The zero scratch first: every read below depends on it.
  static const uint8_t kZeros[kMaxCanDataBytes] = {};
  writeRegisters(kTxb2Data, kZeros, kMaxCanDataBytes);
  if ((readRegister(kCanStat) & kModeMask) != kModeConfig) {
    return false;  // no chip, or it does not answer
  }
  const uint8_t cnf[3] = {cnf_[2], cnf_[1], cnf_[0]};  // CNF3, CNF2, CNF1 are consecutive
  writeRegisters(kCnf3, cnf, 3);
  // RXM=11 also accepts partial, errored messages (DS20001801J, section 4.2.2).
  // RXM=00, standard-frame filters and zero masks accept every standard ID/data byte,
  // but only valid CAN messages.
  for (const uint8_t base : kAcceptanceBases) {
    writeRegisters(base, kZeros, 4);
  }
  writeRegister(kRxb0Ctrl, kRollover);
  writeRegister(kRxb1Ctrl, 0x00);
  writeRegister(kCanIntE, kRx0If | kRx1If);
  writeRegister(kCanIntF, 0x00);
  writeRegister(kEflg, 0x00);
  return linkHealthy(kModeConfig) && setMode(mode);
}

bool Mcp2515CanTransport::linkHealthy(const uint8_t mode)
{
  if (!(readRegister(kCnf1) == cnf_[0] && readRegister(kCnf2) == cnf_[1] &&
    readRegister(kCnf3) == cnf_[2] && readRegister(kZeroScratch) == 0 &&
    readRegister(kZeroScratch + 1) == 0 && (readRegister(kCanStat) & kModeMask) == mode &&
    (readRegister(kRxb0Ctrl) & (kRxModeMask | kRollover)) == kRollover &&
    (readRegister(kRxb1Ctrl) & kRxModeMask) == 0))
  {
    return false;
  }
  // Masks and filters read as zero outside configuration mode, irrespective of their values.
  if (mode == kModeConfig) {
    for (const uint8_t base : kAcceptanceBases) {
      for (uint8_t i = 0; i < 4; ++i) {
        if (readRegister(static_cast<uint8_t>(base + i)) != 0) {
          return false;
        }
      }
    }
  }
  return true;
}

bool Mcp2515CanTransport::start()
{
  tx_pending_ = false;
  tx_aborting_ = false;
  link_suspect_ = false;
  // Frames read since the last good link check may have been read wrong.
  clearReceived();
  if (!configure(kModeLoopback)) {
    ++link_faults_;
    instruction(kReset);
    return false;
  }
  self_test_ = runSelfTest(kSelfTestFrames);
  writeRegister(kCanIntF, 0x00);
  writeRegister(kEflg, 0x00);
  rxb1_older_ = false;
  if (!self_test_.passed() || !setMode(kModeNormal)) {
    ++link_faults_;
    instruction(kReset);
    return false;
  }
  const uint32_t now = spi_.millis();
  last_error_sample_ms_ = now;
  last_link_check_ms_ = now;
  tec_ = 0;
  rec_ = 0;
  eflg_ = 0;
  return true;
}

bool Mcp2515CanTransport::begin(const uint32_t bitrate)
{
  uint8_t cnf1 = 0;
  uint8_t cnf2 = 0;
  uint8_t cnf3 = 0;
  if (!timing(crystal_hz_, bitrate, cnf1, cnf2, cnf3)) {
    return false;
  }
  if (open_ && bitrate == bitrate_) {
    return true;
  }
  open_ = false;
  bitrate_ = bitrate;
  cnf_[0] = cnf1;
  cnf_[1] = cnf2;
  cnf_[2] = cnf3;
  open_ = start();
  // Only a transport that was running gets restarted by service(); the host retries a failed
  // open itself.
  running_ = open_;
  bus_load_.reset(bitrate_, spi_.millis());
  return open_;
}

void Mcp2515CanTransport::end()
{
  instruction(kReset);
  spi_.delayMs(kOscillatorStartMs);
  open_ = false;
  running_ = false;
  tx_pending_ = false;
  tx_aborting_ = false;
  clearReceived();
}

void Mcp2515CanTransport::loadTransmit(const Frame & frame)
{
  uint8_t buffer[5 + kMaxCanDataBytes] = {};
  buffer[0] = static_cast<uint8_t>(frame.id >> 3);
  buffer[1] = static_cast<uint8_t>((frame.id & 0x07) << 5);
  buffer[4] = frame.dlc;
  for (uint8_t i = 0; i < frame.dlc; ++i) {
    buffer[5 + i] = frame.data[i];
  }
  writeRegisters(kTxb0Sidh, buffer, static_cast<uint8_t>(5 + frame.dlc));
  instruction(kRequestToSend0);
}

bool Mcp2515CanTransport::transmitIdle(const uint32_t now_ms)
{
  if (!tx_pending_) {
    return true;
  }
  if ((readRegister(kTxb0Ctrl) & kTxReq) == 0) {
    tx_pending_ = false;
    tx_aborting_ = false;
    return true;
  }
  if (!tx_aborting_ && now_ms - tx_started_ms_ >= kTransmitTimeoutMs) {
    // A frame nobody acknowledged would otherwise be retried until someone does, and a stale
    // motion command must not reach a drive that comes back later.
    bitModify(kTxb0Ctrl, kTxReq, 0x00);
    tx_aborting_ = true;
    ++failed_writes_;
    if ((readRegister(kTxb0Ctrl) & kTxReq) == 0) {
      tx_pending_ = false;
      tx_aborting_ = false;
      return true;
    }
  }
  return false;
}

bool Mcp2515CanTransport::send(const Frame & frame)
{
  if (!open_ || !isValidFrame(frame)) {
    return false;
  }
  const uint32_t now = spi_.millis();
  if (!transmitIdle(now)) {
    return false;
  }
  loadTransmit(frame);
  tx_pending_ = true;
  tx_started_ms_ = now;
  bus_load_.note(frame, now);
  return true;
}

Mcp2515CanTransport::ReadResult Mcp2515CanTransport::readReceived(Frame & frame)
{
  const uint8_t flags = readRegister(kCanIntF) & (kRx0If | kRx1If);
  if (flags == 0) {
    return ReadResult::kNone;
  }
  // With rollover a frame goes to RXB0, or to RXB1 while RXB0 is full, so RXB0 holds the older
  // frame when both are full. Once RXB0 is emptied ahead of a waiting RXB1, the next frame to
  // reach RXB0 is newer than RXB1's.
  const bool first = (flags & kRx0If) != 0 && !(rxb1_older_ && (flags & kRx1If) != 0);
  rxb1_older_ = first && (flags & kRx1If) != 0;
  const uint8_t base = first ? kRxb0Sidh : kRxb1Sidh;
  const uint8_t sidh = readRegister(base);
  const uint8_t sidl = readRegister(static_cast<uint8_t>(base + 1));
  ReadResult result = ReadResult::kDropped;
  if ((sidl & (kSidlIde | kSidlSrr)) == 0) {
    Frame got;
    got.id = static_cast<uint16_t>((static_cast<uint16_t>(sidh) << 3) | (sidl >> 5));
    const uint8_t dlc = readRegister(static_cast<uint8_t>(base + 4)) & 0x0F;
    got.dlc = dlc > kMaxCanDataBytes ? kMaxCanDataBytes : dlc;
    for (uint8_t i = 0; i < got.dlc; ++i) {
      got.data[i] = readRegister(static_cast<uint8_t>(base + 5 + i));
    }
    frame = got;
    result = ReadResult::kFrame;
  } else {
    ++dropped_frames_;
  }
  // RXnIF locks this buffer until cleared (DS20001801J, section 4.1.3), including between
  // the separate SPI register reads above.
  bitModify(kCanIntF, first ? kRx0If : kRx1If, 0x00);
  return result;
}

void Mcp2515CanTransport::collectReceived()
{
  for (uint8_t reads = 0; reads < kMaxReadsPerCall && rx_count_ < kReceiveQueueCapacity &&
    spi_.interruptAsserted(); ++reads)
  {
    Frame frame;
    const ReadResult result = readReceived(frame);
    if (result == ReadResult::kNone) {
      return;
    }
    if (result == ReadResult::kFrame) {
      rx_queue_[(rx_head_ + rx_count_) % kReceiveQueueCapacity] = frame;
      ++rx_count_;
      bus_load_.note(frame, spi_.millis());
    }
  }
}

void Mcp2515CanTransport::clearReceived()
{
  rx_head_ = 0;
  rx_count_ = 0;
  rxb1_older_ = false;
}

bool Mcp2515CanTransport::receive(Frame & frame, const uint32_t timeout_ms)
{
  if (!open_) {
    return false;
  }
  const uint32_t start = spi_.millis();
  for (;;) {
    collectReceived();
    if (rx_count_ > 0) {
      frame = rx_queue_[rx_head_];
      rx_head_ = static_cast<uint8_t>((rx_head_ + 1) % kReceiveQueueCapacity);
      --rx_count_;
      return true;
    }
    if (spi_.millis() - start >= timeout_ms) {
      return false;
    }
  }
}

void Mcp2515CanTransport::sampleErrors()
{
  const uint8_t eflg = readRegister(kEflg);
  const uint8_t tec = readRegister(kTec);
  const uint8_t rec = readRegister(kRec);
  // With rollover the chip flags RX0OVR, not RX1OVR, when a frame finds both buffers full
  // (measured on the bench chip); either way a frame was lost.
  if ((eflg & (kRx0Ovr | kRx1Ovr)) != 0) {
    ++receive_overflows_;
    bitModify(kEflg, kRx0Ovr | kRx1Ovr, 0x00);
  }
  const uint8_t entered = static_cast<uint8_t>(eflg & kErrorStateMask & ~eflg_);
  if (entered != 0 || tec > tec_ || rec > rec_) {
    ++error_events_;
  }
  eflg_ = eflg;
  tec_ = tec;
  rec_ = rec;
}

void Mcp2515CanTransport::checkLink()
{
  if (linkHealthy(kModeNormal)) {
    link_suspect_ = false;
    return;
  }
  ++link_faults_;
  if (!link_suspect_) {
    link_suspect_ = true;  // confirm on the next check before restarting
    return;
  }
  // Two failed checks in a row: the chip lost its configuration (a reset or brownout) or the
  // link is broken. Start again; service() keeps retrying while that fails.
  ++restarts_;
  open_ = start();
}

void Mcp2515CanTransport::service()
{
  if (!running_) {
    return;
  }
  const uint32_t now = spi_.millis();
  if (!open_) {
    if (now - last_link_check_ms_ >= kLinkCheckPeriodMs) {
      last_link_check_ms_ = now;
      ++restarts_;
      open_ = start();
    }
    return;
  }
  if (tx_pending_) {
    transmitIdle(now);
  }
  collectReceived();
  if (now - last_error_sample_ms_ >= kErrorSamplePeriodMs) {
    last_error_sample_ms_ = now;
    sampleErrors();
  }
  if (now - last_link_check_ms_ >= kLinkCheckPeriodMs) {
    last_link_check_ms_ = now;
    checkLink();
  }
}

Mcp2515CanTransport::SelfTest Mcp2515CanTransport::runSelfTest(const uint16_t frames)
{
  SelfTest result;
  uint32_t random = 0x2545F491u;
  for (uint16_t n = 0; n < frames; n += 2) {
    // Two at a time, so the second rolls over into RXB1 and both buffers are read.
    const uint8_t count = (frames - n >= 2) ? 2 : 1;
    Frame sent[2];
    for (uint8_t k = 0; k < count; ++k) {
      selfTestFrame(static_cast<uint16_t>(n + k), random, sent[k]);
      loadTransmit(sent[k]);
      const uint32_t start = spi_.millis();
      while ((readRegister(kTxb0Ctrl) & kTxReq) != 0 &&
        spi_.millis() - start < kSelfTestWaitMs)
      {
      }
    }
    for (uint8_t k = 0; k < count; ++k) {
      ++result.frames;
      const uint32_t start = spi_.millis();
      while (!spi_.interruptAsserted() && spi_.millis() - start < kSelfTestWaitMs) {
      }
      Frame got;
      if (!spi_.interruptAsserted() || readReceived(got) != ReadResult::kFrame) {
        ++result.missing;
      } else if (!sameFrame(got, sent[k])) {
        ++result.bad;
      }
    }
  }
  return result;
}

}  // namespace waybionic
