// Tool board: four NEMA 8 steppers on A4988 drivers, commanded over CAN with the MKS SERVO
// frames the arm's joint drives use. Each motor answers on its own CAN ID like an MKS drive,
// so the ground station drives the tool with the same host code. See README.md.

#include <AccelStepper.h>
#include <Arduino_CAN.h>

#include "mks_frames.h"

// ---- Check every value in this block against the tool board before flashing ----

constexpr uint8_t MOTORS = 4;
// The motors answer on CAN IDs 7 to 10, after the arm's six drives.
constexpr uint16_t FIRST_CAN_ID = 7;
#define CAN_BITRATE CanBitRate::BR_1000k

// CNC Shield V3 pins for X, Y and Z. The fourth motor's direction pin moves from D13 to D11
// because D13 is the UNO R4 WiFi's CAN receive pin (D10 transmits).
#if defined(ARDUINO_MINIMA)
#error "The UNO R4 Minima's CAN uses D4 and D5, which drive Z step and X direction here"
#endif
constexpr uint8_t STEP_PINS[MOTORS] = {2, 3, 4, 12};
constexpr uint8_t DIR_PINS[MOTORS] = {5, 6, 7, 11};
constexpr bool INVERT_DIR[MOTORS] = {false, false, false, false};
// Active low and shared by the four A4988s, so enabling one motor powers them all.
constexpr uint8_t ENABLE_PIN = 8;

// 8HS11-0204S: 200 full steps per turn, at 1/16 microstepping (all three shield jumpers).
constexpr long STEPS_PER_REV = 200L * 16;
// Placeholder travel from the zeroed position. The cables limit how far each motor may turn:
// take the real limits from the Biomedical team's sketch before driving the tool.
constexpr long MIN_STEPS[MOTORS] = {-STEPS_PER_REV, -STEPS_PER_REV, -STEPS_PER_REV,
  -STEPS_PER_REV};
constexpr long MAX_STEPS[MOTORS] = {STEPS_PER_REV, STEPS_PER_REV, STEPS_PER_REV,
  STEPS_PER_REV};
// Faster requests are slowed to what the motors can follow without losing steps.
constexpr float MAX_STEPS_PER_S = 3200.0f;  // 60 rpm
constexpr float MAX_STEPS_PER_S2 = 20000.0f;

// ---- End of board settings ----

static_assert(MOTORS == 4, "steppers[] below lists four motors");

AccelStepper steppers[MOTORS] = {
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[0], DIR_PINS[0]),
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[1], DIR_PINS[1]),
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[2], DIR_PINS[2]),
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[3], DIR_PINS[3]),
};

uint8_t mode[MOTORS] = {};
bool enabled[MOTORS] = {};
bool respond[MOTORS] = {true, true, true, true};
bool active[MOTORS] = {true, true, true, true};
uint32_t heartbeat_ms[MOTORS] = {};
uint32_t last_frame_ms[MOTORS] = {};
// A move is under way, and whether its target was cut short at a travel limit.
bool moving[MOTORS] = {};
bool limited[MOTORS] = {};

long steps_from_counts(int32_t counts)
{
  return lround(static_cast<double>(counts) * STEPS_PER_REV / mks::COUNTS_PER_REV);
}

int64_t counts_from_steps(long steps)
{
  return llround(static_cast<double>(steps) * mks::COUNTS_PER_REV / STEPS_PER_REV);
}

// The manual's ramp: 1 rpm every (256 - acc) * 50 us, and no ramp at all when acc is 0.
float steps_per_s2(uint8_t acc)
{
  if (acc == 0) {
    return MAX_STEPS_PER_S2;
  }
  const float rpm_per_s = 1.0f / ((256 - acc) * 50e-6f);
  return min(rpm_per_s * STEPS_PER_REV / 60.0f, MAX_STEPS_PER_S2);
}

void send(uint16_t id, const uint8_t * data, uint8_t length)
{
  CanMsg const message(CanStandardId(id), length, data);
  CAN.write(message);
}

void send_status(uint8_t motor, uint8_t code, uint8_t status)
{
  const uint16_t id = FIRST_CAN_ID + motor;
  uint8_t out[8];
  send(id, out, mks::status_reply(id, code, status, out));
}

void send_encoder(uint8_t motor)
{
  const uint16_t id = FIRST_CAN_ID + motor;
  uint8_t out[8];
  send(id, out, mks::encoder_reply(id, counts_from_steps(steppers[motor].currentPosition()),
    out));
}

void hold(uint8_t motor)
{
  // Stop at once and make the current position the target.
  steppers[motor].setCurrentPosition(steppers[motor].currentPosition());
  moving[motor] = false;
}

void enable(uint8_t motor, bool on)
{
  enabled[motor] = on;
  if (!on) {
    hold(motor);
  }
  bool any = false;
  for (uint8_t i = 0; i < MOTORS; ++i) {
    any = any || enabled[i];
  }
  digitalWrite(ENABLE_PIN, any ? LOW : HIGH);
}

uint8_t move(uint8_t motor, const mks::Move & request)
{
  if (!enabled[motor] || mode[motor] != mks::MODE_SR_VFOC) {
    return mks::MOVE_FAILED;
  }
  AccelStepper & stepper = steppers[motor];
  if (request.rpm == 0) {
    // Speed 0 is the manual's stop command: slow down with the current ramp.
    stepper.stop();
    moving[motor] = false;
    return mks::MOVE_RUNNING;
  }
  const long wanted = steps_from_counts(request.axis);
  const long target = constrain(wanted, MIN_STEPS[motor], MAX_STEPS[motor]);
  limited[motor] = target != wanted;
  stepper.setMaxSpeed(min(request.rpm * static_cast<float>(STEPS_PER_REV) / 60.0f,
    MAX_STEPS_PER_S));
  stepper.setAcceleration(steps_per_s2(request.acc));
  stepper.moveTo(target);
  moving[motor] = true;
  return mks::MOVE_RUNNING;
}

void handle(uint8_t motor, const uint8_t * data, uint8_t length)
{
  const uint16_t id = FIRST_CAN_ID + motor;
  if (!mks::valid(id, data, length)) {
    return;
  }
  last_frame_ms[motor] = millis();
  const uint8_t code = data[0];
  const uint8_t * arguments = data + 1;
  const uint8_t count = length - 2;
  uint8_t status = 1;
  if (code == mks::READ_ENCODER && count == 0) {
    send_encoder(motor);
    return;
  } else if (code == mks::SET_MODE && count == 1) {
    mode[motor] = arguments[0];
  } else if (code == mks::SET_RESPONSE && count == 2) {
    respond[motor] = arguments[0];
    active[motor] = arguments[1];
  } else if (code == mks::SET_HEARTBEAT && count == 4) {
    heartbeat_ms[motor] = mks::u32(arguments);
  } else if (code == mks::SET_ZERO && count == 0) {
    steppers[motor].setCurrentPosition(0);
    moving[motor] = false;
  } else if (code == mks::ENABLE && count == 1) {
    enable(motor, arguments[0]);
  } else if (code == mks::ABSOLUTE_AXIS && count == 6) {
    status = move(motor, mks::absolute_axis(arguments));
  } else {
    return;
  }
  if (respond[motor]) {
    send_status(motor, code, status);
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);  // drivers off until the host enables a motor
  for (uint8_t i = 0; i < MOTORS; ++i) {
    steppers[i].setPinsInverted(INVERT_DIR[i], false, false);
    steppers[i].setMinPulseWidth(2);
    steppers[i].setMaxSpeed(MAX_STEPS_PER_S);
    steppers[i].setAcceleration(MAX_STEPS_PER_S2);
  }
  if (!CAN.begin(CAN_BITRATE)) {
    for (;;) {
      Serial.println("Tool board: CAN.begin failed; check the transceiver wiring");
      delay(1000);
    }
  }
  Serial.print("Tool board ready on CAN IDs ");
  Serial.print(FIRST_CAN_ID);
  Serial.print(" to ");
  Serial.println(FIRST_CAN_ID + MOTORS - 1);
}

void loop()
{
  while (CAN.available()) {
    CanMsg const message = CAN.read();
    if (message.isStandardId()) {
      const uint32_t id = message.getStandardId();
      if (id >= FIRST_CAN_ID && id < FIRST_CAN_ID + MOTORS) {
        handle(id - FIRST_CAN_ID, message.data, message.data_length);
      }
    }
  }
  const uint32_t now = millis();
  for (uint8_t i = 0; i < MOTORS; ++i) {
    AccelStepper & stepper = steppers[i];
    stepper.run();
    if (moving[i] && heartbeat_ms[i] && now - last_frame_ms[i] > heartbeat_ms[i]) {
      // The host went quiet: stop like an MKS drive's heartbeat.
      stepper.stop();
      moving[i] = false;
    } else if (moving[i] && stepper.distanceToGo() == 0) {
      moving[i] = false;
      if (respond[i] && active[i]) {
        send_status(i, mks::ABSOLUTE_AXIS, limited[i] ? mks::MOVE_AT_LIMIT : mks::MOVE_COMPLETE);
      }
    }
  }
}
