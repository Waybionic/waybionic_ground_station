#include "common/MksFrame.h"

#include <stdio.h>

namespace waybionic
{
namespace mks
{

uint8_t checksum(const uint16_t can_id, const uint8_t * body, const uint8_t body_length)
{
  uint32_t sum = can_id;
  for (uint8_t i = 0; i < body_length; ++i) {
    sum += body[i];
  }
  return static_cast<uint8_t>(sum & 0xFF);
}

bool buildFrame(
  const uint16_t can_id, const uint8_t code, const uint8_t * arguments,
  const uint8_t argument_count, Frame & out)
{
  if (can_id > kMaxStandardCanId || argument_count > kMaxArguments) {
    return false;
  }
  out = Frame{};
  out.id = can_id;
  out.data[0] = code;
  for (uint8_t i = 0; i < argument_count; ++i) {
    out.data[1 + i] = arguments[i];
  }
  const uint8_t body_length = static_cast<uint8_t>(1 + argument_count);
  out.data[body_length] = checksum(can_id, out.data, body_length);
  out.dlc = static_cast<uint8_t>(body_length + 1);
  return true;
}

ParseResult parseFrame(const Frame & frame, Message & out)
{
  if (frame.id > kMaxStandardCanId) {
    return ParseResult::kBadId;
  }
  if (frame.dlc < 2 || frame.dlc > kMaxCanDataBytes) {
    return ParseResult::kBadDlc;
  }
  const uint8_t body_length = static_cast<uint8_t>(frame.dlc - 1);
  if (frame.data[body_length] != checksum(frame.id, frame.data, body_length)) {
    return ParseResult::kBadChecksum;
  }
  out = Message{};
  out.can_id = frame.id;
  out.code = frame.data[0];
  out.argument_count = static_cast<uint8_t>(body_length - 1);
  for (uint8_t i = 0; i < out.argument_count; ++i) {
    out.arguments[i] = frame.data[1 + i];
  }
  return ParseResult::kOk;
}

const char * parseResultName(const ParseResult result)
{
  switch (result) {
    case ParseResult::kOk: return "ok";
    case ParseResult::kBadId: return "bad CAN ID";
    case ParseResult::kBadDlc: return "bad DLC";
    case ParseResult::kBadChecksum: return "bad checksum";
  }
  return "unknown";
}

bool absoluteAxis(
  const uint16_t can_id, const int32_t axis, const int32_t speed_rpm, const int32_t acc,
  Frame & out)
{
  if (axis < kMinAxis || axis > kMaxAxis) {
    return false;
  }
  if (speed_rpm < 0 || speed_rpm > kMaxSpeedRpm || acc < 0 || acc > 255) {
    return false;
  }
  const uint32_t raw_axis = static_cast<uint32_t>(axis) & 0xFFFFFF;  // int24 two's complement
  const uint8_t arguments[6] = {
    static_cast<uint8_t>(speed_rpm >> 8), static_cast<uint8_t>(speed_rpm),
    static_cast<uint8_t>(acc),
    static_cast<uint8_t>(raw_axis >> 16), static_cast<uint8_t>(raw_axis >> 8),
    static_cast<uint8_t>(raw_axis),
  };
  return buildFrame(can_id, kAbsoluteAxis, arguments, 6, out);
}

bool readEncoder(const uint16_t can_id, Frame & out)
{
  return buildFrame(can_id, kReadEncoder, nullptr, 0, out);
}

bool setMode(const uint16_t can_id, const uint8_t mode, Frame & out)
{
  return buildFrame(can_id, kSetMode, &mode, 1, out);
}

bool setResponse(const uint16_t can_id, const bool respond, const bool active, Frame & out)
{
  const uint8_t arguments[2] = {static_cast<uint8_t>(respond), static_cast<uint8_t>(active)};
  return buildFrame(can_id, kSetResponse, arguments, 2, out);
}

bool setHeartbeat(const uint16_t can_id, const uint32_t milliseconds, Frame & out)
{
  const uint8_t arguments[4] = {
    static_cast<uint8_t>(milliseconds >> 24), static_cast<uint8_t>(milliseconds >> 16),
    static_cast<uint8_t>(milliseconds >> 8), static_cast<uint8_t>(milliseconds),
  };
  return buildFrame(can_id, kSetHeartbeat, arguments, 4, out);
}

bool enable(const uint16_t can_id, const bool on, Frame & out)
{
  const uint8_t argument = static_cast<uint8_t>(on);
  return buildFrame(can_id, kEnable, &argument, 1, out);
}

bool statusReply(const uint16_t can_id, const uint8_t code, const uint8_t status, Frame & out)
{
  return buildFrame(can_id, code, &status, 1, out);
}

bool encoderReply(const uint16_t can_id, const int64_t value, Frame & out)
{
  const uint64_t raw = static_cast<uint64_t>(value);
  uint8_t arguments[6];
  for (uint8_t i = 0; i < 6; ++i) {
    arguments[i] = static_cast<uint8_t>(raw >> (8 * (5 - i)));
  }
  return buildFrame(can_id, kReadEncoder, arguments, 6, out);
}

bool decodeAbsoluteAxis(const Message & message, AbsoluteAxisArgs & out)
{
  if (message.code != kAbsoluteAxis || message.argument_count != 6) {
    return false;
  }
  const uint8_t * a = message.arguments;
  out.speed_rpm = static_cast<uint16_t>((a[0] << 8) | a[1]);
  out.acc = a[2];
  uint32_t raw = (static_cast<uint32_t>(a[3]) << 16) | (static_cast<uint32_t>(a[4]) << 8) | a[5];
  if (raw & 0x800000) {
    raw |= 0xFF000000;  // sign-extend int24
  }
  out.axis = static_cast<int32_t>(raw);
  return true;
}

bool decodeEncoderValue(const Message & message, int64_t & out)
{
  if (message.code != kReadEncoder || message.argument_count != 6) {
    return false;
  }
  uint64_t raw = 0;
  for (uint8_t i = 0; i < 6; ++i) {
    raw = (raw << 8) | message.arguments[i];
  }
  if (raw & 0x800000000000ULL) {
    raw |= 0xFFFF000000000000ULL;  // sign-extend int48
  }
  out = static_cast<int64_t>(raw);
  return true;
}

bool decodeStatus(const Message & message, uint8_t & out)
{
  if (message.argument_count != 1) {
    return false;
  }
  out = message.arguments[0];
  return true;
}

const char * codeName(const uint8_t code)
{
  switch (code) {
    case kReadEncoder: return "READ_ENCODER";
    case kSetMode: return "SET_MODE";
    case kSetResponse: return "SET_RESPONSE";
    case kSetHeartbeat: return "SET_HEARTBEAT";
    case kEnable: return "ENABLE";
    case kAbsoluteAxis: return "ABSOLUTE_AXIS";
  }
  return "UNSUPPORTED";
}

void formatFrame(const Frame & frame, char * out, const size_t out_size)
{
  static const char kHex[] = "0123456789ABCDEF";
  if (out_size == 0) {
    return;
  }
  const int written = snprintf(out, out_size, "%03X#", static_cast<unsigned>(frame.id));
  size_t position = written < 0 ? 0 : static_cast<size_t>(written);
  const uint8_t dlc = frame.dlc > kMaxCanDataBytes ? kMaxCanDataBytes : frame.dlc;
  for (uint8_t i = 0; i < dlc && position + 2 < out_size; ++i) {
    out[position++] = kHex[frame.data[i] >> 4];
    out[position++] = kHex[frame.data[i] & 0x0F];
  }
  out[position < out_size ? position : out_size - 1] = '\0';
}

}  // namespace mks
}  // namespace waybionic
