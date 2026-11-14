/*
  WayBionic carrier bridge for an MCP2515 CAN module: laptop USB serial <-> UNO R4 <-> SPI <->
  MCP2515 + TJA1050 module <-> CAN bus.

  Same SLCAN subset, USB link and carrier status frame as carrier_bridge.ino (see
  SlcanBridge.h), so the ground station cannot tell the two apart. Only the CAN controller
  differs: this one is the external MCP2515 the arm's bench uses instead of the R4's own.

  Wiring (UNO R4 WiFi):
    MCP2515 CS  -> D10      MCP2515 SO (MISO) -> D12
    MCP2515 INT -> D2       MCP2515 SI (MOSI) -> D11
    MCP2515 SCK -> D13      VCC 5 V, GND
  kCrystalHz must match the oscillator printed on the module's can (8.000 MHz on the bench).

  Each 'O' from the host makes Mcp2515CanTransport check the chip and run its loopback
  self-test before joining the bus; a link that fails is answered with BELL. Its controller
  errors and failed link checks are both reported as CAN errors in the status frame.

  As in carrier_bridge.ino, nothing else is printed on the serial port, and the CAN controller
  stays in configuration mode (off the bus) until the host sends 'O'.
*/

#include <WaybionicCan.h>

#if defined(ARDUINO_UNOWIFIR4)
// Take the USB-C port off the WiFi board's ESP32-S3 bridge, as carrier_bridge.ino explains.
void __USBStart();
void __maybe_start_usb() {__USBStart();}
#endif

static const uint32_t kStatusPeriodMs = 100;
static const uint8_t kChipSelectPin = 10;
static const uint8_t kInterruptPin = 2;
static const uint32_t kCrystalHz = 8000000;

static waybionic::ArduinoMcp2515Spi spi(kChipSelectPin, kInterruptPin);
static waybionic::Mcp2515CanTransport can(spi, kCrystalHz);

class Mcp2515Starter : public waybionic::CanStarter
{
public:
  bool start(uint32_t bitrate) override {return can.begin(bitrate);}
};

class SerialSlcanOutput : public waybionic::SlcanOutput
{
public:
  void write(const char * text) override {SerialUSB.print(text);}
};

static Mcp2515Starter starter;
static SerialSlcanOutput serialOutput;
static waybionic::SlcanBridge bridge(can, starter, serialOutput);
static uint32_t lastStatusMs = 0;

void setup()
{
  // USB ignores the baud rate.
  SerialUSB.begin(0);
  spi.begin();
  // A board reset leaves the MCP2515 as it was, possibly still on the bus: take it off.
  can.end();
}

void loop()
{
  waybionic::handBackUsbWhenAsked();
  while (SerialUSB.available() > 0) {
    bridge.handleChar(static_cast<char>(SerialUSB.read()), millis());
  }
  can.service();
  bridge.drainTransmit(millis());
  // Each receive() empties the MCP2515's two buffers before taking the next frame.
  bridge.pollCan();

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
    status.can_errors = can.errorEvents() + can.linkFaults();
    status.failed_writes = can.failedWrites();
    bridge.reportStatus(status);
  }
}
