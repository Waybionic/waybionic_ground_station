/*
  WayBionic carrier bridge: laptop USB serial <-> UNO R4 <-> TJA1051T <-> CAN bus.

  Speaks a documented SUBSET of the Lawicel SLCAN protocol (see SlcanBridge.h). The ground
  station's drive node opens it with interface slcan, at kSerialBaud:

    ros2 launch waybionic_bringup ground_station.launch.py teleop:=true \
      drive_interface:=slcan drive_channel:=/dev/cu.usbmodem1101

  python-can's "slcan" interface works too, with tty_baudrate=1000000.

  Standard 11-bit data frames only. The CAN controller stays off until the host sends 'O', so
  the board puts nothing on the bus by itself. It forwards whatever the host sends without MKS
  checks or motion limits: safety is the host's job, and powered motion needs Yassin or
  Mujtaba present.

  Nothing else is printed on the serial port, because any extra text would corrupt the
  SLCAN stream. Use single_drive_bringup for a human-readable console. Ten times a second the
  host gets a status frame (SlcanBridge::reportStatus) that never goes on the CAN bus.

  TODO(hardware-TBD): sense the E-stop circuit state and motor supply voltage once Electrical
  specifies how they are wired (BenchTelemetry.h). Until then the status says "not wired".
*/

#include <WaybionicCan.h>

// Six drives at 120 Hz need about 240 kbit/s each way through the ESP32-S3 USB bridge, so
// 115200 baud is far too slow. The host sets the same speed (tty_baudrate in arm_drives.yaml).
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
