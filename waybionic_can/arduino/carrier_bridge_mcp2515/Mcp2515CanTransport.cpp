#include "Mcp2515CanTransport.h"

#include <Arduino.h>
#include <CANRegisters.h>

namespace waybionic
{

namespace
{

// A transmit or receive buffer from SIDH on: SIDH, SIDL, EID8, EID0, DLC and eight data bytes.
constexpr uint8_t kBufferBytes = 5 + kMaxCanDataBytes;
constexpr uint8_t kSidlRemote = 0x10;  // SRR: a standard remote frame
constexpr uint8_t kSidlExtended = 0x08;  // IDE
constexpr uint8_t kOverflows = EFLG_RX0OVR | EFLG_RX1OVR;
constexpr uint8_t kSeriousErrors = EFLG_TXBO | EFLG_TXEP | EFLG_RXEP;
constexpr uint32_t kModeTimeoutMs = 10;
constexpr uint8_t kMaxDroppedPerReceive = 4;

const CANBitrate::Config * rateFor(const uint32_t bitrate)
{
  switch (bitrate) {
    case 125000: return &CANBitrate::Config_8MHz_125kbps;
    case 250000: return &CANBitrate::Config_8MHz_250kbps;
    case 500000: return &CANBitrate::Config_8MHz_500kbps;
    default: return nullptr;
  }
}

}  // namespace

Mcp2515CanTransport::Mcp2515CanTransport(const uint8_t cs_pin, const int8_t int_pin)
: config_(CANBitrate::Config_8MHz_500kbps, cs_pin, int_pin),
  controller_(config_),
  io_(config_)
{
}

bool Mcp2515CanTransport::isSupportedBitrate(const uint32_t bitrate)
{
  return rateFor(bitrate) != nullptr;
}

bool Mcp2515CanTransport::setup()
{
  open_ = false;
  bitrate_ = 0;
  ready_ = controller_.begin(CANController::Mode::Config) == CANController::OK;
  return ready_;
}

bool Mcp2515CanTransport::setMode(const CANController::Mode mode)
{
  // AA_MCP2515 only checks the request. The chip finishes a frame on the bus before it switches.
  controller_.setMode(mode);
  const uint8_t wanted = static_cast<uint8_t>(mode);
  const uint32_t start = millis();
  do {
    if ((io_.read(CANSTAT) & CANSTAT_OPMOD_MASK) == wanted) {
      return true;
    }
  } while (millis() - start < kModeTimeoutMs);
  return false;
}

bool Mcp2515CanTransport::begin(const uint32_t bitrate)
{
  const CANBitrate::Config * const rate = rateFor(bitrate);
  if (rate == nullptr) {
    return false;
  }
  if (open_ && bitrate == bitrate_) {
    return true;
  }
  // A chip that did not answer at boot, say through a loose wire since fixed, is reset again.
  if (!ready_ && !setup()) {
    return false;
  }
  open_ = false;
  bitrate_ = 0;
  if (!setMode(CANController::Mode::Config)) {
    return false;
  }
  io_.write(CNF1, rate->cnf1);
  io_.write(CNF2, rate->cnf2);
  io_.write(CNF3, rate->cnf3);
  // The filters pass every frame to the first receive buffer. Rollover puts a frame that arrives
  // while it is full in the second one instead of losing it.
  io_.writeBits(RXB0CTRL, RXB0CTRL_BUKT, RXB0CTRL_BUKT);
  io_.writeBits(CANINTF, CANINTF_RX0IF | CANINTF_RX1IF, 0);
  io_.writeBits(EFLG, kOverflows, 0);
  second_buffer_older_ = false;
  open_ = setMode(CANController::Mode::Normal);
  bitrate_ = open_ ? bitrate : 0;
  return open_;
}

bool Mcp2515CanTransport::send(const Frame & frame)
{
  if (!open_) {
    ++failed_writes_;
    return false;
  }
  if (!isValidFrame(frame)) {
    return false;
  }
  // Only the first transmit buffer is used, like the UNO R4's single mailbox: the chip sends its
  // highest-numbered pending buffer first, so filling all three could reorder the host's frames,
  // such as a drive's set-up sequence. SlcanBridge retries while it is busy.
  if ((io_.readStatus() & READSTATUS_TX0REQ) != 0) {
    return false;
  }
  uint8_t buffer[kBufferBytes] = {};
  buffer[0] = static_cast<uint8_t>(frame.id >> 3);
  buffer[1] = static_cast<uint8_t>((frame.id & 0x07) << 5);
  buffer[4] = frame.dlc;
  for (uint8_t i = 0; i < frame.dlc; ++i) {
    buffer[5 + i] = frame.data[i];
  }
  io_.write(TXB0SIDH, buffer, static_cast<uint8_t>(5 + frame.dlc));
  io_.write(INSTRUCTION_RTS0);
  return true;
}

bool Mcp2515CanTransport::readBuffer(const uint8_t address, const uint8_t flag, Frame & frame)
{
  uint8_t buffer[kBufferBytes];
  io_.read(address, buffer, kBufferBytes);
  io_.writeBits(CANINTF, flag, 0);
  if ((buffer[1] & (kSidlRemote | kSidlExtended)) != 0) {
    ++dropped_frames_;
    return false;
  }
  frame = Frame{};
  frame.id = static_cast<uint16_t>((buffer[0] << 3) | (buffer[1] >> 5));
  const uint8_t dlc = buffer[4] & 0x0F;
  frame.dlc = dlc > kMaxCanDataBytes ? kMaxCanDataBytes : dlc;
  for (uint8_t i = 0; i < frame.dlc; ++i) {
    frame.data[i] = buffer[5 + i];
  }
  return true;
}

bool Mcp2515CanTransport::receive(Frame & frame, const uint32_t timeout_ms)
{
  if (!open_) {
    return false;
  }
  const uint32_t start = millis();
  uint8_t dropped = 0;
  while (dropped < kMaxDroppedPerReceive) {
    const uint8_t status = io_.readStatus();
    const bool first = (status & READSTATUS_RX0IF) != 0;
    const bool second = (status & READSTATUS_RX1IF) != 0;
    if (!first && !second) {
      if (millis() - start >= timeout_ms) {
        return false;
      }
      continue;
    }
    // Oldest first: a frame rolls over into the second buffer only while the first is full.
    bool received = false;
    if (second && (!first || second_buffer_older_)) {
      received = readBuffer(RXB1SIDH, CANINTF_RX1IF, frame);
      second_buffer_older_ = false;
    } else {
      received = readBuffer(RXB0SIDH, CANINTF_RX0IF, frame);
      second_buffer_older_ = second;
    }
    if (received) {
      return true;
    }
    ++dropped;
  }
  return false;
}

bool Mcp2515CanTransport::takeError(int & code)
{
  const uint32_t now = millis();
  if (!open_ || now == error_checked_ms_) {
    return false;
  }
  error_checked_ms_ = now;
  const uint8_t flags = io_.read(EFLG);
  const uint8_t overflows = flags & kOverflows;
  if (overflows != 0) {
    // The chip holds these until they are cleared, so each lost frame counts once.
    io_.writeBits(EFLG, overflows, 0);
  }
  const uint8_t serious = flags & kSeriousErrors;
  const bool event = overflows != 0 || (serious & ~error_flags_) != 0;
  error_flags_ = serious;
  if (!event) {
    return false;
  }
  ++error_events_;
  code = flags;
  return true;
}

}  // namespace waybionic
