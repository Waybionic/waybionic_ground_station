/*
  WayBionic carrier bridge prototype: laptop USB serial <-> UNO R4 <-> TJA1051T <-> CAN bus.

  Speaks a documented SUBSET of the Lawicel SLCAN protocol (see SlcanBridge.h) so python-can's
  "slcan" interface can use the board without a custom desktop driver:

    import can
    bus = can.Bus(interface="slcan", channel="COM5", bitrate=500000)   # or /dev/ttyACM0

  Standard 11-bit data frames only. The CAN controller stays off until the host sends 'O', so
  the board puts nothing on the bus by itself. It forwards whatever the host sends without MKS
  checks or motion limits: safety is the host's job, and powered motion needs Yassin or
  Mujtaba present.

  Nothing else is printed on the serial port, because any extra text would corrupt the
  SLCAN stream. Use single_drive_bringup for a human-readable console.

  TODO(hardware-TBD): report the E-stop circuit state and motor supply voltage once Electrical
  specifies how they are wired (BenchTelemetry.h). SLCAN has no standard message for them.
*/

#include <WaybionicCan.h>

// python-can's slcan default tty_baudrate. On the UNO R4 WiFi the USB port goes through the
// ESP32-S3, so the host and this value should match.
static const unsigned long kSerialBaud = 115200;

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
  // Controller error events are cleared but not reported: SLCAN status flags ('F') are not
  // implemented yet.
  int code = 0;
  can.takeError(code);
}
