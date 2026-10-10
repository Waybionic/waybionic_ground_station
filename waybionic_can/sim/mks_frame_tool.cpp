// Line-oriented front end to MksFrame for cross-checking against waybionic_teleop/mks_can.py
// (see tests/cross_check/cross_check_mks.py). One request per stdin line, one answer per line:
//
//   absolute_axis <id> <axis> <rpm> <acc>   -> data hex, or ERR
//   read_encoder <id> | enable <id> <0|1> | set_mode <id> <mode>
//   set_response <id> <0|1> <0|1> | set_heartbeat <id> <ms>
//   parse <id> <hex>                        -> "<code hex> <arguments hex>", or ERR
//   encoder_value <12 hex digits>           -> signed decimal, or ERR

#include <stdint.h>

#include <iostream>
#include <sstream>
#include <string>

#include "common/MksFrame.h"

namespace
{

using waybionic::Frame;
namespace mks = waybionic::mks;

std::string hex(const uint8_t * data, uint8_t length)
{
  static const char kHex[] = "0123456789ABCDEF";
  std::string out;
  for (uint8_t i = 0; i < length; ++i) {
    out += kHex[data[i] >> 4];
    out += kHex[data[i] & 0x0F];
  }
  return out;
}

bool fromHex(const std::string & text, uint8_t * out, uint8_t & length, uint8_t max_length)
{
  if (text.size() % 2 != 0 || text.size() / 2 > max_length) {
    return false;
  }
  length = static_cast<uint8_t>(text.size() / 2);
  for (uint8_t i = 0; i < length; ++i) {
    out[i] = static_cast<uint8_t>(std::stoul(text.substr(2 * i, 2), nullptr, 16));
  }
  return true;
}

std::string answer(const std::string & line)
{
  std::istringstream in(line);
  std::string verb;
  long long id = 0, a = 0, b = 0, c = 0;
  in >> verb;
  Frame frame;
  bool ok = false;
  if (verb == "encoder_value") {
    std::string text;
    uint8_t bytes[6];
    uint8_t length = 0;
    if (!(in >> text) || !fromHex(text, bytes, length, 6) || length != 6) {
      return "ERR";
    }
    mks::Message message;
    message.code = mks::kReadEncoder;
    message.argument_count = 6;
    for (uint8_t i = 0; i < 6; ++i) {
      message.arguments[i] = bytes[i];
    }
    int64_t value = 0;
    return mks::decodeEncoderValue(message, value) ? std::to_string(value) : "ERR";
  }
  if (!(in >> id) || id < 0 || id > 0xFFFF) {
    return "ERR";
  }
  const uint16_t can_id = static_cast<uint16_t>(id);
  if (verb == "parse") {
    std::string text;
    if (!(in >> text) || !fromHex(text, frame.data, frame.dlc, 8)) {
      return "ERR";
    }
    frame.id = can_id;
    mks::Message message;
    if (mks::parseFrame(frame, message) != mks::ParseResult::kOk) {
      return "ERR";
    }
    return hex(&message.code, 1) + " " + hex(message.arguments, message.argument_count);
  }
  if (verb == "absolute_axis" && in >> a >> b >> c) {
    ok = a >= INT32_MIN && a <= INT32_MAX && b >= INT32_MIN && b <= INT32_MAX &&
      c >= INT32_MIN && c <= INT32_MAX &&
      mks::absoluteAxis(can_id, static_cast<int32_t>(a), static_cast<int32_t>(b),
        static_cast<int32_t>(c), frame);
  } else if (verb == "read_encoder") {
    ok = mks::readEncoder(can_id, frame);
  } else if (verb == "enable" && in >> a) {
    ok = mks::enable(can_id, a != 0, frame);
  } else if (verb == "set_mode" && in >> a && a >= 0 && a <= 255) {
    ok = mks::setMode(can_id, static_cast<uint8_t>(a), frame);
  } else if (verb == "set_response" && in >> a >> b) {
    ok = mks::setResponse(can_id, a != 0, b != 0, frame);
  } else if (verb == "set_heartbeat" && in >> a && a >= 0 && a <= UINT32_MAX) {
    ok = mks::setHeartbeat(can_id, static_cast<uint32_t>(a), frame);
  }
  return ok ? hex(frame.data, frame.dlc) : "ERR";
}

}  // namespace

int main()
{
  std::string line;
  while (std::getline(std::cin, line)) {
    std::cout << answer(line) << '\n';
  }
  return 0;
}
