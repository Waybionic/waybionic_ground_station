#pragma once

#include "common/Frame.h"

namespace waybionic
{

// Parse candump text such as "001#3132": 1-3 hex digits of 11-bit ID, '#', then 0-8 bytes as
// pairs of hex digits. Bytes are taken exactly as written; no checksum is added or checked.
bool parseCandump(const char * text, Frame & out);

}  // namespace waybionic
