#include "common/Candump.h"

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

bool parseCandump(const char * text, Frame & out)
{
  if (text == nullptr) {
    return false;
  }
  uint32_t id = 0;
  uint8_t id_digits = 0;
  const char * p = text;
  for (; *p != '\0' && *p != '#'; ++p) {
    const int digit = hexDigit(*p);
    if (digit < 0 || ++id_digits > 3) {
      return false;
    }
    id = (id << 4) | static_cast<uint32_t>(digit);
  }
  if (*p != '#' || id_digits == 0 || id > kMaxStandardCanId) {
    return false;
  }
  ++p;

  Frame frame;
  frame.id = static_cast<uint16_t>(id);
  while (*p != '\0') {
    const int high = hexDigit(p[0]);
    const int low = high < 0 ? -1 : hexDigit(p[1]);
    if (low < 0 || frame.dlc == kMaxCanDataBytes) {
      return false;
    }
    frame.data[frame.dlc++] = static_cast<uint8_t>((high << 4) | low);
    p += 2;
  }
  out = frame;
  return true;
}

}  // namespace waybionic
