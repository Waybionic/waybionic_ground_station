/*
  WayBionic four-node CAN bench: receiver (Arduinos 2-4).

  Hardware port of waybionic_can/sim/receiver_main.cpp. The board answers MKS frames like a
  SERVO42D/57D (NodeLogic follows waybionic_teleop/sim_drives.py): it sees all bus traffic,
  acts only on frames carrying its own CAN ID, drops frames with a bad MKS checksum or DLC
  without replying, and replies with status / encoder frames. The actuator is software only
  (SoftwareActuator): nothing physical moves.

  Set kNodeId before uploading to each receiver (1, 2 and 3 for the bench).

  Do NOT put a receiver on a bus with a real drive that uses the same CAN ID: both would answer.

  Serial monitor: 115200 baud (log only).
*/

#include <WaybionicCan.h>

static const uint16_t kNodeId = 1;          // 1-2047, unique on the bus
static const uint32_t kCanBitrate = 500000;  // must match the gateway
static const uint32_t kMoveMs = 1000;        // how long a simulated move takes
static const unsigned long kSerialBaud = 115200;

static waybionic::R4CanTransport can;
static waybionic::SoftwareActuator actuator(kMoveMs);
static waybionic::NodeLogic node(kNodeId, actuator, 0);
static waybionic::ActuatorState lastState;

static void logFrame(const char * direction, const waybionic::Frame & frame, const char * note)
{
  char candump[24];
  waybionic::mks::formatFrame(frame, candump, sizeof(candump));
  char text[112];
  snprintf(text, sizeof(text), "[node %u] %s %s t=%lu %s", static_cast<unsigned>(kNodeId),
    direction, candump, static_cast<unsigned long>(millis()), note);
  Serial.println(text);
}

static void transmit(const waybionic::Frame & frame, const char * why)
{
  logFrame("TX", frame, can.send(frame) ? why : "SEND FAILED");
}

static void handleFrame(const waybionic::Frame & frame)
{
  const waybionic::NodeEvent event = node.handleFrame(frame, millis());
  char note[64];
  if (event.action == waybionic::NodeAction::kIgnored) {
    snprintf(note, sizeof(note), "ignored (addressed to %u)", static_cast<unsigned>(frame.id));
  } else if (event.action == waybionic::NodeAction::kRejected) {
    snprintf(note, sizeof(note), "rejected: %s",
      waybionic::mks::parseResultName(event.parse_result));
  } else {
    snprintf(note, sizeof(note), "%s %s", waybionic::mks::codeName(event.code),
      waybionic::nodeActionName(event.action));
  }
  logFrame("RX", frame, note);
  if (event.has_reply) {
    transmit(event.reply, "reply");
  }
}

static void reportState()
{
  const waybionic::ActuatorState state = actuator.state();
  if (state.enabled == lastState.enabled && state.moving == lastState.moving &&
    state.target == lastState.target && state.position == lastState.position)
  {
    return;
  }
  char text[112];
  snprintf(text, sizeof(text), "[node %u] actuator enabled=%d moving=%d target=%ld position=%ld",
    static_cast<unsigned>(kNodeId), state.enabled, state.moving,
    static_cast<long>(state.target), static_cast<long>(state.position));
  Serial.println(text);
  lastState = state;
}

static void reportCanErrors()
{
  int code = 0;
  uint32_t suppressed = 0;
  if (can.takeErrorReport(millis(), code, suppressed)) {
    char text[112];
    snprintf(text, sizeof(text), "[node %u] CAN controller error event %d (+%lu more)",
      static_cast<unsigned>(kNodeId), code, static_cast<unsigned long>(suppressed));
    Serial.println(text);
  }
}

void setup()
{
  Serial.begin(kSerialBaud);
  const unsigned long start = millis();
  while (!Serial && millis() - start < 3000) {
  }
  const bool started = can.begin(kCanBitrate);
  char text[112];
  snprintf(text, sizeof(text), "[node %u] receiver, CAN %s at %lu bit/s (TX D%d, RX D%d)",
    static_cast<unsigned>(kNodeId), started ? "started" : "FAILED TO START",
    static_cast<unsigned long>(kCanBitrate), PIN_CAN0_TX, PIN_CAN0_RX);
  Serial.println(text);
  lastState = actuator.state();
}

void loop()
{
  waybionic::Frame frame;
  while (can.receive(frame, 0)) {
    handleFrame(frame);
  }

  const waybionic::PollEvent poll = node.poll(millis());
  if (poll.heartbeat_stop) {
    Serial.print("[node ");
    Serial.print(kNodeId);
    Serial.println("] heartbeat timeout: stopped");
  }
  if (poll.has_frame) {
    transmit(poll.frame, "run complete");
  }

  reportState();
  reportCanErrors();
}
