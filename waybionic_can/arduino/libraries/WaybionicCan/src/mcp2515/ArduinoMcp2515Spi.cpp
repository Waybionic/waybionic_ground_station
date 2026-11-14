#include "mcp2515/ArduinoMcp2515Spi.h"

#if defined(ARDUINO)

#include <Arduino.h>
#include <SPI.h>

namespace waybionic
{

ArduinoMcp2515Spi::ArduinoMcp2515Spi(
  const uint8_t cs_pin, const uint8_t int_pin, const uint32_t clock_hz)
: cs_pin_(cs_pin), int_pin_(int_pin), clock_hz_(clock_hz)
{
}

void ArduinoMcp2515Spi::begin()
{
  pinMode(cs_pin_, OUTPUT);
  digitalWrite(cs_pin_, HIGH);
  pinMode(int_pin_, INPUT);
#if defined(ARDUINO_ARCH_RENESAS)
  // Every register read is two transactions, so chip select through the port's set/reset
  // register instead of digitalWrite saves a tenth of each read.
  const bsp_io_port_pin_t pin = g_pin_cfg[cs_pin_].pin;
  const uintptr_t port_stride =
    reinterpret_cast<uintptr_t>(R_PORT1) - reinterpret_cast<uintptr_t>(R_PORT0);
  R_PORT0_Type * const port = reinterpret_cast<R_PORT0_Type *>(
    reinterpret_cast<uintptr_t>(R_PORT0) + port_stride * (static_cast<uint32_t>(pin) >> 8));
  cs_set_reset_ = &port->PCNTR3;
  cs_mask_ = 1u << (static_cast<uint32_t>(pin) & 0xFFu);
#endif
  // Not SPI.end() first: on the UNO R4 core it faults if SPI has not been started.
  SPI.begin();
}

void ArduinoMcp2515Spi::csLow()
{
  if (cs_set_reset_ != nullptr) {
    *cs_set_reset_ = cs_mask_ << 16;  // the upper half resets the pin
  } else {
    digitalWrite(cs_pin_, LOW);
  }
}

void ArduinoMcp2515Spi::csHigh()
{
  if (cs_set_reset_ != nullptr) {
    *cs_set_reset_ = cs_mask_;
  } else {
    digitalWrite(cs_pin_, HIGH);
  }
}

void ArduinoMcp2515Spi::select()
{
  SPI.beginTransaction(SPISettings(clock_hz_, MSBFIRST, SPI_MODE0));
  csLow();
}

uint8_t ArduinoMcp2515Spi::transfer(const uint8_t out)
{
  return SPI.transfer(out);
}

void ArduinoMcp2515Spi::deselect()
{
  csHigh();
  SPI.endTransaction();
  // The MCP2515 needs chip select high for 50 ns between transactions; the next select is
  // further off than that, and these cover it even if it were not (21 ns each at 48 MHz).
  __asm__ volatile("nop\n nop\n nop");
}

bool ArduinoMcp2515Spi::interruptAsserted()
{
  return digitalRead(int_pin_) == LOW;
}

uint32_t ArduinoMcp2515Spi::millis()
{
  return ::millis();
}

void ArduinoMcp2515Spi::delayMs(const uint32_t ms)
{
  delay(ms);
}

}  // namespace waybionic

#endif
