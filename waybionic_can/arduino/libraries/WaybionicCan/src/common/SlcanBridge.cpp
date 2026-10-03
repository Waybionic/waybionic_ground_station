#include "common/SlcanBridge.h"

namespace waybionic
{
namespace
{

int hexDigit(const char c)
{
  if (c >= '0' && c <= '9') {
    return c - '0';
  }
  if (c >= 'A' && c <= 'F') {
    return c - 'A' + 10;
  }
  if (c >= 'a' && c <= 'f') {
    return c - 'a' + 10;
  }
  return -1;
}

}  // namespace

SlcanBridge::SlcanBridge(ICanTransport & can, CanStarter & starter, SlcanOutput & out)
: can_(can), starter_(starter), out_(out)
{
}

bool SlcanBridge::bitrateForCode(const char code, uint32_t & out)
{
  // Same table as python-can's slcanBus._BITRATES, limited to the Arduino CanBitRate values.
  switch (code) {
    case '4': out = 125000; return true;
    case '5': out = 250000; return true;
    case '6': out = 500000; return true;
    case '8': out = 1000000; return true;
  }
  return false;
}

bool SlcanBridge::parseTransmit(const char * line, const size_t length, Frame & out)
{
  if (length < 5 || line[0] != 't') {
    return false;
  }
  uint32_t id = 0;
  for (size_t i = 1; i <= 3; ++i) {
    const int digit = hexDigit(line[i]);
    if (digit < 0) {
      return false;
    }
    id = (id << 4) | static_cast<uint32_t>(digit);
  }
  const int dlc = line[4] - '0';
  if (id > kMaxStandardCanId || dlc < 0 || dlc > kMaxCanDataBytes ||
    length != 5 + 2 * static_cast<size_t>(dlc))
  {
    return false;
  }
  Frame frame;
  frame.id = static_cast<uint16_t>(id);
  frame.dlc = static_cast<uint8_t>(dlc);
  for (int i = 0; i < dlc; ++i) {
    const int high = hexDigit(line[5 + 2 * i]);
    const int low = hexDigit(line[6 + 2 * i]);
    if (high < 0 || low < 0) {
      return false;
    }
    frame.data[i] = static_cast<uint8_t>((high << 4) | low);
  }
  out = frame;
  return true;
}

void SlcanBridge::formatFrame(const Frame & frame, char * out, const size_t out_size)
{
  static const char kHex[] = "0123456789ABCDEF";
  const uint8_t dlc = frame.dlc > kMaxCanDataBytes ? kMaxCanDataBytes : frame.dlc;
  const size_t needed = 1 + 3 + 1 + 2 * dlc + 1 + 1;
  if (out_size < needed) {
    if (out_size > 0) {
      out[0] = '\0';
    }
    return;
  }
  size_t p = 0;
  out[p++] = 't';
  out[p++] = kHex[(frame.id >> 8) & 0x7];
  out[p++] = kHex[(frame.id >> 4) & 0xF];
  out[p++] = kHex[frame.id & 0xF];
  out[p++] = static_cast<char>('0' + dlc);
  for (uint8_t i = 0; i < dlc; ++i) {
    out[p++] = kHex[frame.data[i] >> 4];
    out[p++] = kHex[frame.data[i] & 0xF];
  }
  out[p++] = '\r';
  out[p] = '\0';
}

void SlcanBridge::handleChar(const char c)
{
  if (c == '\n') {
    return;
  }
  if (c == '\r') {
    if (overflow_) {
      error();
    } else {
      line_[length_] = '\0';
      handleLine(line_, length_);
    }
    length_ = 0;
    overflow_ = false;
    return;
  }
  if (length_ < kMaxLine) {
    line_[length_++] = c;
  } else {
    overflow_ = true;
  }
}

void SlcanBridge::handleLine(const char * line, const size_t length)
{
  if (length == 0) {
    ok();
    return;
  }
  uint32_t bitrate = 0;
  Frame frame;
  switch (line[0]) {
    case 'S':
      if (length == 2 && !open_ && bitrateForCode(line[1], bitrate)) {
        bitrate_ = bitrate;
        ok();
      } else {
        error();
      }
      return;
    case 'O':
      if (length == 1 && !open_ && starter_.start(bitrate_)) {
        open_ = true;
        ok();
      } else {
        error();
      }
      return;
    case 'C':
      if (length == 1) {
        open_ = false;
        ok();
      } else {
        error();
      }
      return;
    case 't':
      if (open_ && parseTransmit(line, length, frame) && can_.send(frame)) {
        out_.write("z\r");
      } else {
        error();
      }
      return;
    case 'V':
      if (length == 1) {
        out_.write("V0100\r");
      } else {
        error();
      }
      return;
    case 'N':
      if (length == 1) {
        out_.write("NWB01\r");
      } else {
        error();
      }
      return;
    default:
      error();
      return;
  }
}

void SlcanBridge::pollCan()
{
  Frame frame;
  char text[24];
  while (can_.receive(frame, 0)) {
    if (open_) {
      formatFrame(frame, text, sizeof(text));
      out_.write(text);
    }
  }
}

}  // namespace waybionic
