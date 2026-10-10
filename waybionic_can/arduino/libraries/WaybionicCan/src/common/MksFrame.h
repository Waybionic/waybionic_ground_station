#pragma once

#include <stddef.h>
#include <stdint.h>

#include "common/Frame.h"

// C++ port of waybionic_teleop/mks_can.py (PR #24), the MKS SERVO42D/57D CAN manual V1.0.9
// subset. Each frame's CAN ID is the motor ID and its data is a function code, big-endian
// arguments and a checksum byte. Commands and replies use the same layout and the same ID.
namespace waybionic
{
namespace mks
{

constexpr int32_t kCountsPerRev = 0x4000;
constexpr int32_t kMaxSpeedRpm = 3000;
constexpr int32_t kMinAxis = -0x800000;
constexpr int32_t kMaxAxis = 0x7FFFFF;
constexpr uint8_t kMaxArguments = 6;  // 8 data bytes minus code and checksum

constexpr uint8_t kReadEncoder = 0x31;
constexpr uint8_t kSetMode = 0x82;
constexpr uint8_t kSetResponse = 0x8C;
constexpr uint8_t kSetHeartbeat = 0x98;
constexpr uint8_t kEnable = 0xF3;
constexpr uint8_t kAbsoluteAxis = 0xF5;
// F7h emergency stop is intentionally host-owned in feature/real-arm. Add it to this shared
// bench subset only if the console/node simulation needs the same confirmed contract.

constexpr uint8_t kModeSrVfoc = 0x05;

// Status byte of setting replies, and of F5h replies (RUN_STATUS in mks_can.py).
constexpr uint8_t kStatusFailed = 0;
constexpr uint8_t kStatusOk = 1;  // F5h: running
constexpr uint8_t kRunComplete = 2;
constexpr uint8_t kRunEndLimit = 3;

struct Message
{
  uint16_t can_id = 0;
  uint8_t code = 0;
  uint8_t arguments[kMaxArguments] = {};
  uint8_t argument_count = 0;
};

struct AbsoluteAxisArgs
{
  uint16_t speed_rpm = 0;
  uint8_t acc = 0;
  int32_t axis = 0;
};

enum class ParseResult : uint8_t
{
  kOk,
  kBadId,
  kBadDlc,
  kBadChecksum,
};

// The manual's CHECKSUM 8bit: the low byte of the CAN ID plus every data byte.
uint8_t checksum(uint16_t can_id, const uint8_t * body, uint8_t body_length);

// Return false, like mks_can.frame raising ValueError, for IDs over 11 bits or over 6 arguments.
bool buildFrame(
  uint16_t can_id, uint8_t code, const uint8_t * arguments, uint8_t argument_count,
  Frame & out);

ParseResult parseFrame(const Frame & frame, Message & out);
const char * parseResultName(ParseResult result);

bool absoluteAxis(uint16_t can_id, int32_t axis, int32_t speed_rpm, int32_t acc, Frame & out);
bool readEncoder(uint16_t can_id, Frame & out);
bool setMode(uint16_t can_id, uint8_t mode, Frame & out);
bool setResponse(uint16_t can_id, bool respond, bool active, Frame & out);
bool setHeartbeat(uint16_t can_id, uint32_t milliseconds, Frame & out);
bool enable(uint16_t can_id, bool on, Frame & out);

// Reply builders used by receiver nodes.
bool statusReply(uint16_t can_id, uint8_t code, uint8_t status, Frame & out);
bool encoderReply(uint16_t can_id, int64_t value, Frame & out);

// Decoders return false when the argument count does not match the command layout.
bool decodeAbsoluteAxis(const Message & message, AbsoluteAxisArgs & out);
bool decodeEncoderValue(const Message & message, int64_t & out);  // signed 48-bit
bool decodeStatus(const Message & message, uint8_t & out);

const char * codeName(uint8_t code);

// candump text such as "001#F502580200400092"; out needs at least 21 bytes.
void formatFrame(const Frame & frame, char * out, size_t out_size);

}  // namespace mks
}  // namespace waybionic
