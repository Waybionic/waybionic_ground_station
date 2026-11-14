#pragma once

// The MCP2515 module on the board's hardware SPI pins (UNO R4: D11 MOSI, D12 MISO, D13 SCK),
// with chip select and INT on any digital pins. Mcp2515CanTransport does the rest.

#if defined(ARDUINO)

#include <stdint.h>

#include "common/Mcp2515CanTransport.h"

namespace waybionic
{

class ArduinoMcp2515Spi : public Mcp2515Spi
{
public:
  // 8 MHz is the clock the bench link was validated at; the MCP2515 allows up to 10 MHz.
  ArduinoMcp2515Spi(uint8_t cs_pin, uint8_t int_pin, uint32_t clock_hz = 8000000);

  // Call once from setup(), before the transport's begin().
  void begin();

  void select() override;
  uint8_t transfer(uint8_t out) override;
  void deselect() override;
  bool interruptAsserted() override;
  uint32_t millis() override;
  void delayMs(uint32_t ms) override;

private:
  void csLow();
  void csHigh();

  uint8_t cs_pin_;
  uint8_t int_pin_;
  uint32_t clock_hz_;
  // The port's set/reset register for chip select, where the core has one (UNO R4).
  volatile uint32_t * cs_set_reset_ = nullptr;
  uint32_t cs_mask_ = 0;
};

}  // namespace waybionic

#endif
