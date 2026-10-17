/*
  WayBionic single-drive CAN bring-up: UNO R4 (built-in CAN) + TJA1051T + one MKS SERVO42D/57D.

  Starts read-only. It never enables or moves the motor by itself: every frame is sent only
  because a command was typed, and every frame on the bus is logged as
    TX 001#3132 t=...    (sent by this board)
    RX 001#31...  t=...  (received from the bus)
  so the log can be compared byte for byte with waybionic_teleop/mks_can.py and the manual.

  Serial monitor: 115200 baud, newline or carriage return line endings. Type 'help'.

  Powered motion only with Yassin or Mujtaba present. The software stop is not an E-stop.
*/

#include <WaybionicCan.h>

// MKS drives ship at 500 kbit/s. The simulator config (waybionic_teleop/config/arm_drives.yaml)
// assumes 1 Mbit/s; change this only after the leads agree AND the drives have been
// reconfigured, otherwise this board and the drive will not hear each other.
static const uint32_t kCanBitrate = 500000;
static const unsigned long kSerialBaud = 115200;

class SerialOutput : public waybionic::ConsoleOutput
{
public:
  void line(const char * text) override {Serial.println(text);}
};

static waybionic::MotionPolicy benchPolicy()
{
  // Team limits for the first powered tests; see MotionPolicy in BringupConsole.h.
  waybionic::MotionPolicy policy;
  policy.max_rpm = 60;
  policy.min_acc = 1;
  policy.max_acc = 128;
  policy.max_step_counts = 0x4000;  // one motor turn
  policy.encoder_fresh_ms = 10000;
  return policy;
}

static waybionic::R4CanTransport can;
static SerialOutput serialOutput;
static waybionic::BringupConsole console(can, serialOutput, benchPolicy());

static char lineBuffer[96];
static size_t lineLength = 0;
static bool lineOverflow = false;

static void printConfig()
{
  char text[128];
  Serial.println("WayBionic single-drive bring-up (UNO R4 + TJA1051T + MKS SERVO42D/57D)");
  snprintf(text, sizeof(text), "CAN %s, %lu bit/s, standard 11-bit IDs, TX pin D%d, RX pin D%d",
    can.isOpen() ? "started" : "FAILED TO START", static_cast<unsigned long>(kCanBitrate),
    PIN_CAN0_TX, PIN_CAN0_RX);
  Serial.println(text);
  Serial.println("MKS checksum = (CAN ID + data bytes) & 0xFF, checked here; CAN CRC/ACK are "
    "the controller's job");
  const waybionic::BenchTelemetry telemetry = waybionic::readBenchTelemetry();
  snprintf(text, sizeof(text), "E-stop circuit: %s; motor supply: %s",
    waybionic::estopStateName(telemetry.estop),
    telemetry.supply_known ? "measured" : "UNKNOWN (hardware-TBD)");
  Serial.println(text);
  console.printPolicy();
  Serial.println("Nothing is sent until you type a command. Start with 'read <id>'.");
}

static void printBusStats()
{
  char text[160];
  can.busLoad().format(millis(), text, sizeof(text));
  Serial.println(text);
  snprintf(text, sizeof(text), "bus errors: %lu controller error events, %lu failed writes "
    "(last code %d), %lu extended frames dropped",
    static_cast<unsigned long>(can.errorEvents()), static_cast<unsigned long>(can.failedWrites()),
    can.lastWriteError(), static_cast<unsigned long>(can.droppedExtendedFrames()));
  Serial.println(text);
}

static void handleLine(char * line)
{
  if (waybionic::BringupConsole::firstWordIs(line, "config") ||
    waybionic::BringupConsole::firstWordIs(line, "status"))
  {
    printConfig();
    console.printDrives(millis());
    printBusStats();
    return;
  }
  if (!console.handleLine(line, millis())) {
    Serial.print("ERR unknown command: ");
    Serial.println(line);
    Serial.println("type 'help'");
  }
}

static void readSerial()
{
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') {
      if (lineOverflow) {
        Serial.println("ERR line too long, ignored");
      } else if (lineLength > 0) {
        lineBuffer[lineLength] = '\0';
        handleLine(lineBuffer);
      }
      lineLength = 0;
      lineOverflow = false;
    } else if (lineLength < sizeof(lineBuffer) - 1) {
      lineBuffer[lineLength++] = c;
    } else {
      lineOverflow = true;
    }
  }
}

static void reportCanErrors()
{
  int code = 0;
  uint32_t suppressed = 0;
  if (can.takeErrorReport(millis(), code, suppressed)) {
    char text[160];
    snprintf(text, sizeof(text), "CAN controller error event %d (+%lu more): bus/transceiver "
      "fault such as no ACK, bus-off or wrong bitrate; not an MKS checksum error", code,
      static_cast<unsigned long>(suppressed));
    Serial.println(text);
  }
}

void setup()
{
  Serial.begin(kSerialBaud);
  const unsigned long start = millis();
  while (!Serial && millis() - start < 3000) {
  }
  can.begin(kCanBitrate);
  printConfig();
}

void loop()
{
  readSerial();

  waybionic::Frame frame;
  while (can.receive(frame, 0)) {
    console.handleFrame(frame, millis());
  }

  reportCanErrors();
}
