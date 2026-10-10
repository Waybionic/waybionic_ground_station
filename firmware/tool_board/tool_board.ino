// Tool board: four NEMA 8 steppers on A4988 drivers, commanded over CAN with the MKS SERVO
// frames the arm's joint drives use. Each motor answers on its own CAN ID like an MKS drive,
// so the ground station drives the tool with the same host code. See README.md.

#include <AccelStepper.h>
#include <Arduino_CAN.h>

#include "tool_motor.h"

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

static_assert(MOTORS == 4, "the arrays below list four motors");

AccelStepper steppers[MOTORS] = {
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[0], DIR_PINS[0]),
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[1], DIR_PINS[1]),
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[2], DIR_PINS[2]),
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[3], DIR_PINS[3]),
};

constexpr MotorConfig config(uint8_t motor)
{
  return {STEPS_PER_REV, MIN_STEPS[motor], MAX_STEPS[motor], MAX_STEPS_PER_S, MAX_STEPS_PER_S2};
}

ToolMotor<AccelStepper> motors[MOTORS] = {
  ToolMotor<AccelStepper>(FIRST_CAN_ID + 0, steppers[0], config(0)),
  ToolMotor<AccelStepper>(FIRST_CAN_ID + 1, steppers[1], config(1)),
  ToolMotor<AccelStepper>(FIRST_CAN_ID + 2, steppers[2], config(2)),
  ToolMotor<AccelStepper>(FIRST_CAN_ID + 3, steppers[3], config(3)),
};

void send(uint16_t id, const uint8_t * data, uint8_t length)
{
  CanMsg const message(CanStandardId(id), length, data);
  CAN.write(message);
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
  uint8_t reply[8];
  while (CAN.available()) {
    CanMsg const message = CAN.read();
    if (!message.isStandardId()) {
      continue;
    }
    const uint32_t id = message.getStandardId();
    if (id >= FIRST_CAN_ID && id < FIRST_CAN_ID + MOTORS) {
      ToolMotor<AccelStepper> & motor = motors[id - FIRST_CAN_ID];
      if (const uint8_t length = motor.handle(message.data, message.data_length, millis(),
          reply))
      {
        send(motor.id(), reply, length);
      }
    }
  }
  bool any = false;
  for (ToolMotor<AccelStepper> & motor : motors) {
    if (const uint8_t length = motor.update(millis(), reply)) {
      send(motor.id(), reply, length);
    }
    any = any || motor.enabled();
  }
  digitalWrite(ENABLE_PIN, any ? LOW : HIGH);
}
