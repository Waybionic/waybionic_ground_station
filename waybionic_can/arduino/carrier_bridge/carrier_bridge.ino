/*
  WayBionic carrier bridge prototype: laptop USB serial <-> UNO R4 <-> TJA1051T <-> CAN bus.

  Speaks a documented SUBSET of the Lawicel SLCAN protocol (see SlcanBridge.h) over the RA4M1's
  own USB port, so the serial baud rate the host opens it at does not matter, except 1200: that
  hands the port back for an upload (UsbHandBack.h). The host selects the CAN bus bitrate with Sn.

  Standard 11-bit data frames only. The CAN controller stays off until the host sends 'O', so
  the board puts nothing on the bus by itself. Host frames wait in a bounded queue while the
  UNO R4's single standard-frame transmit mailbox is busy.

  Nothing else is printed on the serial port, because any extra text would corrupt the SLCAN
  stream. Use single_drive_bringup for a human-readable console. Ten times a second the host
  gets a carrier status frame (SlcanBridge::reportStatus) that never goes on the CAN bus.

  TODO(hardware-TBD): sense the E-stop circuit state and motor supply voltage once Electrical
  specifies how they are wired (BenchTelemetry.h). Until then their status flags remain clear,
  which means unknown/unavailable rather than healthy.
*/

#include <WaybionicCan.h>

#if defined(ARDUINO_UNOWIFIR4)
// The UNO R4 WiFi's USB-C port normally belongs to its ESP32-S3, whose USB bridge firmware stops
// passing bytes on for good, in one direction or the other, under steady two-way traffic.
// Defining this hands the port to the RA4M1's own USB at boot (pin P408), as on the UNO R4 Minima,
// so SerialUSB below reaches the host directly. See README.md, "USB link".
void __USBStart();
void __maybe_start_usb() {__USBStart();}
#endif

static const uint32_t kStatusPeriodMs = 100;

static waybionic::R4CanTransport can;

class R4Starter : public waybionic::CanStarter
{
public:
  bool start(uint32_t bitrate) override {return can.begin(bitrate);}
};

class SerialSlcanOutput : public waybionic::SlcanOutput
{
public:
  void write(const char * text) override {SerialUSB.print(text);}
};

static R4Starter starter;
static SerialSlcanOutput serialOutput;
static waybionic::SlcanBridge bridge(can, starter, serialOutput);
static uint32_t lastStatusMs = 0;

void setup()
{
  // USB ignores the baud rate.
  SerialUSB.begin(0);
}

void loop()
{
  waybionic::handBackUsbWhenAsked();
  while (SerialUSB.available() > 0) {
    bridge.handleChar(static_cast<char>(SerialUSB.read()), millis());
  }
  bridge.drainTransmit(millis());
  bridge.pollCan();
  // Consume each callback-latched controller error event once for the status counter.
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
