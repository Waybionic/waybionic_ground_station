/*
  WayBionic carrier bridge for the arm's CAN module: laptop USB serial <-> UNO R4 WiFi <-> MCP2515
  (8 MHz crystal) <-> CAN bus at the MKS drives' factory 500 kbit/s.

  It speaks the same SLCAN subset as carrier_bridge (see SlcanBridge.h), so the ground station
  drives the arm through either board. Only S4, S5 and S6 (125, 250 and 500 kbit/s) open: the
  module's crystal cannot run 1 Mbit/s.

  The MCP2515 stays in configuration mode, off the bus, until the host sends 'O'. Nothing else is
  printed on the serial port, because any extra text would corrupt the SLCAN stream; flash
  manual_arm_console for a human-readable console. Ten times a second the host gets a carrier
  status frame, whose CAN error count includes receive buffer overflows (each one a lost frame).
*/

#include <WaybionicCan.h>

#include "Mcp2515CanTransport.h"

// This is the USB serial baud rate, not the CAN bus bitrate. Both ends must use this value.
static const unsigned long kSerialBaud = 1000000;
static const uint32_t kStatusPeriodMs = 100;
// The module's SPI chip select and interrupt pins, as wired for manual_arm_console.
static const uint8_t kCanCsPin = 10;
static const int8_t kCanIntPin = 2;

static waybionic::Mcp2515CanTransport can(kCanCsPin, kCanIntPin);

class Mcp2515Starter : public waybionic::CanStarter
{
public:
  bool start(uint32_t bitrate) override {return can.begin(bitrate);}
};

class SerialSlcanOutput : public waybionic::SlcanOutput
{
public:
  void write(const char * text) override {Serial.print(text);}
};

static Mcp2515Starter starter;
static SerialSlcanOutput serialOutput;
static waybionic::SlcanBridge bridge(can, starter, serialOutput);
static uint32_t lastStatusMs = 0;

void setup()
{
  Serial.begin(kSerialBaud);
  // Resetting the chip takes half a second. If it does not answer, 'O' tries again.
  can.setup();
}

void loop()
{
  while (Serial.available() > 0) {
    bridge.handleChar(static_cast<char>(Serial.read()), millis());
  }
  bridge.drainTransmit(millis());
  bridge.pollCan();
  int code = 0;
  can.takeError(code);

  const uint32_t now = millis();
  if (now - lastStatusMs >= kStatusPeriodMs) {
    lastStatusMs = now;
    const waybionic::BenchTelemetry bench = waybionic::readBenchTelemetry();
    waybionic::CarrierStatus status;
    status.estop_wired = bench.estop != waybionic::EstopState::kUnknown;
    status.estop_pressed = bench.estop == waybionic::EstopState::kPressed;
    status.supply_wired = bench.supply_known;
    status.supply_millivolts = static_cast<uint16_t>(
      bench.supply_millivolts > 0xFFFF ? 0xFFFF : bench.supply_millivolts);
    status.can_errors = can.errorEvents();
    status.failed_writes = can.failedWrites();
    bridge.reportStatus(status);
  }
}
