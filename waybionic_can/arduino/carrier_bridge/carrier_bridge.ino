/*
  WayBionic carrier bridge prototype: laptop USB serial <-> UNO R4 <-> TJA1051T <-> CAN bus.

  Speaks a documented SUBSET of the Lawicel SLCAN protocol (see SlcanBridge.h). The real-arm
  host opens it at 1,000,000 baud over USB serial and selects the CAN bus bitrate separately.

  Standard 11-bit data frames only. The CAN controller stays off until the host sends 'O', so
  the board puts nothing on the bus by itself. It forwards whatever the host sends without MKS
  checks or motion limits: safety is the host's job, and powered motion needs Yassin or
  Mujtaba present.

  Nothing else is printed on the serial port, because any extra text would corrupt the SLCAN
  stream. Use single_drive_bringup for a human-readable console. Ten times a second the host
  gets a carrier status frame (SlcanBridge::reportStatus) that never goes on the CAN bus.

  TODO(hardware-TBD): sense the E-stop circuit state and motor supply voltage once Electrical
  specifies how they are wired (BenchTelemetry.h). Until then their status flags remain clear,
  which means unknown/unavailable rather than healthy.
*/

#include <WaybionicCan.h>

// This is the USB serial baud rate, not the CAN bus bitrate. Both ends must use this value.
static const unsigned long kSerialBaud = 1000000;
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
  void write(const char * text) override {Serial.print(text);}
};

static R4Starter starter;
static SerialSlcanOutput serialOutput;
static waybionic::SlcanBridge bridge(can, starter, serialOutput);
static uint32_t lastStatusMs = 0;

void setup()
{
  Serial.begin(kSerialBaud);
}

void loop()
{
  while (Serial.available() > 0) {
    bridge.handleChar(static_cast<char>(Serial.read()));
  }
  bridge.pollCan();
  // Clear controller error events; errorEvents() counts them for the status frame.
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
