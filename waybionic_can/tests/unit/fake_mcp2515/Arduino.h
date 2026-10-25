#pragma once

// Host stand-in for the Arduino core: only what the MCP2515 transport calls. Every call to
// millis() advances the clock by a millisecond, so a loop waiting on the chip cannot hang a test.

#include <stdint.h>

uint32_t millis();
