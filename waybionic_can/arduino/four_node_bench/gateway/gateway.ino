/*
  WayBionic four-node CAN bench: gateway (Arduino 1).

    laptop --USB serial--> gateway --CAN--> receiver 1, receiver 2, receiver 3

  Hardware port of waybionic_can/sim/gateway_main.cpp. Typed commands become MKS frames built
  by the shared library (ported from waybionic_teleop/mks_can.py); GatewayLogic tracks which
  receivers actually replied, because a CAN ACK only proves some controller saw the frame.
  Moves go through the same safety checks as single_drive_bringup (enable confirmed, fresh
  31h reading, bench limits).

  Serial monitor: 115200 baud. Type 'help'. Nothing is sent until a command is typed; 'poll'
  is off by default and only ever sends 31h encoder reads.
*/

#include <WaybionicCan.h>

// Must match the receivers. 500 kbit/s is the MKS factory rate; see the README before changing.
static const uint32_t kCanBitrate = 500000;
static const unsigned long kSerialBaud = 115200;
static const uint16_t kNodeIds[] = {1, 2, 3};
static const uint32_t kReplyTimeoutMs = 500;

class SerialOutput : public waybionic::ConsoleOutput
{
public:
  void line(const char * text) override {Serial.println(text);}
};

static waybionic::R4CanTransport can;
static waybionic::GatewayLogic gateway(
  kNodeIds, sizeof(kNodeIds) / sizeof(kNodeIds[0]), kReplyTimeoutMs);

// Records each frame sent so GatewayLogic can match the addressed receiver's reply.
class TrackingTransport : public waybionic::ICanTransport
{
public:
  bool send(const waybionic::Frame & frame) override
  {
    if (!can.send(frame)) {
      return false;
    }
    gateway.noteSent(frame, millis());
    return true;
  }
  bool receive(waybionic::Frame & frame, uint32_t timeout_ms) override
  {
    return can.receive(frame, timeout_ms);
  }
};

static TrackingTransport tracked;
static SerialOutput serialOutput;
static waybionic::BringupConsole console(tracked, serialOutput, waybionic::MotionPolicy());

static waybionic::NodeHealth lastHealth[sizeof(kNodeIds) / sizeof(kNodeIds[0])];
static uint32_t pollMs = 0;
static uint32_t nextPollMs = 0;

static char lineBuffer[96];
static size_t lineLength = 0;
static bool lineOverflow = false;

static void printConfig()
{
  char text[128];
  snprintf(text, sizeof(text), "[gateway] CAN %s, %lu bit/s, TX pin D%d, RX pin D%d",
    can.isOpen() ? "started" : "FAILED TO START", static_cast<unsigned long>(kCanBitrate),
    PIN_CAN0_TX, PIN_CAN0_RX);
  Serial.println(text);
  Serial.print("[gateway] receivers:");
  for (uint8_t i = 0; i < gateway.nodeCount(); ++i) {
    Serial.print(' ');
    Serial.print(gateway.nodeId(i));
  }
  snprintf(text, sizeof(text), "; reply timeout %lu ms; poll %s",
    static_cast<unsigned long>(kReplyTimeoutMs), pollMs == 0 ? "off" : "on");
  Serial.println(text);
  Serial.println("[gateway] extra commands: status | poll <ms> (0 = off; 31h reads only) | config");
}

static void printStatus()
{
  char text[96];
  const uint32_t now = millis();
  for (uint8_t i = 0; i < gateway.nodeCount(); ++i) {
    const uint16_t id = gateway.nodeId(i);
    snprintf(text, sizeof(text), "[gateway] node %u %s unexpected=%lu", static_cast<unsigned>(id),
      waybionic::nodeHealthName(gateway.health(id, now)),
      static_cast<unsigned long>(gateway.unexpectedReplies(id)));
    Serial.println(text);
  }
  console.printDrives(now);
  char bus[160];
  can.busLoad().format(now, bus, sizeof(bus));
  Serial.print("[gateway] ");
  Serial.println(bus);
  snprintf(bus, sizeof(bus), "[gateway] bus errors: %lu controller error events, %lu failed "
    "writes (last code %d), %lu extended frames dropped",
    static_cast<unsigned long>(can.errorEvents()), static_cast<unsigned long>(can.failedWrites()),
    can.lastWriteError(), static_cast<unsigned long>(can.droppedExtendedFrames()));
  Serial.println(bus);
}

static void handleLine(char * line)
{
  if (waybionic::BringupConsole::firstWordIs(line, "status")) {
    printStatus();
    return;
  }
  if (waybionic::BringupConsole::firstWordIs(line, "config")) {
    printConfig();
    return;
  }
  if (waybionic::BringupConsole::firstWordIs(line, "poll")) {
    int32_t ms = -1;
    char * value = line;
    while (*value == ' ' || *value == '\t') {
      ++value;
    }
    value += 4;  // "poll"
    while (*value == ' ' || *value == '\t') {
      ++value;
    }
    char * end = value + strlen(value);
    while (end > value && (end[-1] == ' ' || end[-1] == '\t')) {
      *--end = '\0';
    }
    if (!waybionic::BringupConsole::parseInteger(value, ms) || ms < 0) {
      Serial.println("ERR usage: poll <ms> (0 turns polling off)");
      return;
    }
    pollMs = static_cast<uint32_t>(ms);
    nextPollMs = millis();
    Serial.println(pollMs == 0 ? "[gateway] poll off" : "[gateway] poll on (31h reads only)");
    return;
  }
  if (!console.handleLine(line, millis())) {
    Serial.print("ERR unknown command: ");
    Serial.println(line);
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

static void pollEncoders()
{
  if (pollMs == 0 || static_cast<int32_t>(millis() - nextPollMs) < 0) {
    return;
  }
  nextPollMs = millis() + pollMs;
  for (uint8_t i = 0; i < gateway.nodeCount(); ++i) {
    char command[16];
    snprintf(command, sizeof(command), "read %u", static_cast<unsigned>(gateway.nodeId(i)));
    console.handleLine(command, millis());
  }
}

static void reportHealthChanges()
{
  const uint32_t now = millis();
  for (uint8_t i = 0; i < gateway.nodeCount(); ++i) {
    const waybionic::NodeHealth health = gateway.health(gateway.nodeId(i), now);
    if (health != lastHealth[i]) {
      Serial.print("[gateway] node ");
      Serial.print(gateway.nodeId(i));
      Serial.print(' ');
      Serial.println(waybionic::nodeHealthName(health));
      lastHealth[i] = health;
    }
  }
}

static void reportCanErrors()
{
  int code = 0;
  uint32_t suppressed = 0;
  if (can.takeErrorReport(millis(), code, suppressed)) {
    char text[112];
    snprintf(text, sizeof(text), "[gateway] CAN controller error event %d (+%lu more); not an "
      "MKS checksum error", code, static_cast<unsigned long>(suppressed));
    Serial.println(text);
  }
}

void setup()
{
  Serial.begin(kSerialBaud);
  const unsigned long start = millis();
  while (!Serial && millis() - start < 3000) {
  }
  for (auto & health : lastHealth) {
    health = waybionic::NodeHealth::kUnknown;
  }
  can.begin(kCanBitrate);
  printConfig();
  Serial.println("[gateway] type 'help'");
}

void loop()
{
  readSerial();
  pollEncoders();

  waybionic::Frame frame;
  while (can.receive(frame, 0)) {
    const uint32_t now = millis();
    const waybionic::ReplyEvent event = gateway.handleFrame(frame, now);
    console.handleFrame(frame, now);
    if (event.kind == waybionic::ReplyKind::kNotANode) {
      Serial.println("[gateway]   from an ID that is not a configured receiver");
    } else if (event.kind == waybionic::ReplyKind::kUnexpected) {
      Serial.println("[gateway]   unexpected reply: duplicate node ID or late answer?");
    } else if (event.kind == waybionic::ReplyKind::kReply) {
      Serial.print("[gateway]   reply latency ");
      Serial.print(static_cast<unsigned long>(event.latency_ms));
      Serial.println(" ms");
    }
  }

  reportHealthChanges();
  reportCanErrors();
}
