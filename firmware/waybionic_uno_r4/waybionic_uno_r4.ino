/*
  Uno R4 WiFi - Four-servo IK calibration controller

  Wiring:
    Servo 1 signal -> D1   (base yaw)
    Servo 2 signal -> D2   (shoulder)
    Servo 3 signal -> D3   (elbow)
    Servo 4 signal -> D4   (wrist roll)
    Hold switch    -> D7 and GND (uses INPUT_PULLUP)

  Serial at 115200 baud:
    ID
    MOVE,s1,s2,s3,s4,durationMs
    JOG,servoNumber,deltaDegrees,durationMs
    HOLD

  Angles are physical 270-degree servo angles.
*/

#include <Servo.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

const byte SERVO_COUNT = 4;
const byte SERVO_PINS[SERVO_COUNT] = {1, 2, 3, 4};
const byte HOLD_SWITCH_PIN = 7;

const float PHYSICAL_RANGE = 270.0;
const int SERVO_PULSE_MIN_US = 544;
const int SERVO_PULSE_MAX_US = 2400;
const float SERVO_MIN[SERVO_COUNT] = {0.0, 0.0, 90.0, 0.0};
const float SERVO_MAX[SERVO_COUNT] = {270.0, 112.5, 270.0, 270.0};

float currentAngle[SERVO_COUNT] = {90.0, 35.0, 151.5, 27.5};
float startAngle[SERVO_COUNT];
float targetAngle[SERVO_COUNT];

Servo servos[SERVO_COUNT];
bool moving = false;
bool switchWasPressed = false;
unsigned long moveStartedMs = 0;
unsigned long moveDurationMs = 3000;
unsigned long lastServoUpdateMs = 0;
const unsigned long SERVO_UPDATE_INTERVAL_MS = 20;

char serialLine[128];
byte serialLength = 0;

void readSerialCommands();
void processCommand(char *line);
void updateMotion();
void updateHoldSwitch();
void startMove(const float requested[], unsigned long durationMs);
void writeAllOutputs();
void writePhysicalServo(byte servoIndex, float physicalAngle);
bool anglesAreSafe(const float values[]);

void setup()
{
  Serial.begin(115200);
  pinMode(HOLD_SWITCH_PIN, INPUT_PULLUP);

  for (byte servo = 0; servo < SERVO_COUNT; servo++)
  {
    servos[servo].attach(
        SERVO_PINS[servo], SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
  }

  writeAllOutputs();
  Serial.println("READY,IK4,1");
}

void loop()
{
  readSerialCommands();
  updateHoldSwitch();
  updateMotion();
}

void readSerialCommands()
{
  while (Serial.available() > 0)
  {
    char incoming = Serial.read();

    if (incoming == '\r')
    {
      continue;
    }

    if (incoming == '\n')
    {
      serialLine[serialLength] = '\0';
      if (serialLength > 0)
      {
        processCommand(serialLine);
      }
      serialLength = 0;
      continue;
    }

    if (serialLength < sizeof(serialLine) - 1)
    {
      serialLine[serialLength++] = incoming;
    }
    else
    {
      serialLength = 0;
      Serial.println("ERROR,line too long");
    }
  }
}

void processCommand(char *line)
{
  if (strcmp(line, "ID") == 0)
  {
    Serial.println("READY,IK4,1");
    return;
  }

  if (strcmp(line, "HOLD") == 0)
  {
    moving = false;
    Serial.println("OK,HOLD");
    return;
  }

  char *savePointer;
  char *token = strtok_r(line, ",", &savePointer);

  if (token != nullptr && strcmp(token, "JOG") == 0)
  {
    char *servoToken = strtok_r(nullptr, ",", &savePointer);
    char *deltaToken = strtok_r(nullptr, ",", &savePointer);
    char *durationToken = strtok_r(nullptr, ",", &savePointer);
    if (servoToken == nullptr || deltaToken == nullptr || durationToken == nullptr)
    {
      Serial.println("ERROR,JOG needs servo,delta,duration");
      return;
    }

    int servoNumber = atoi(servoToken);
    if (servoNumber < 1 || servoNumber > SERVO_COUNT)
    {
      Serial.println("ERROR,invalid servo number");
      return;
    }

    float requested[SERVO_COUNT];
    for (byte servo = 0; servo < SERVO_COUNT; servo++)
    {
      requested[servo] = currentAngle[servo];
    }
    requested[servoNumber - 1] += atof(deltaToken);

    if (!anglesAreSafe(requested))
    {
      Serial.println("ERROR,jog outside software limits");
      return;
    }

    unsigned long duration = strtoul(durationToken, nullptr, 10);
    startMove(requested, constrain(duration, 300UL, 15000UL));
    Serial.println("OK,JOG");
    return;
  }

  if (token == nullptr || strcmp(token, "MOVE") != 0)
  {
    Serial.println("ERROR,unknown command");
    return;
  }

  float requested[SERVO_COUNT];
  for (byte servo = 0; servo < SERVO_COUNT; servo++)
  {
    token = strtok_r(nullptr, ",", &savePointer);
    if (token == nullptr)
    {
      Serial.println("ERROR,missing servo angle");
      return;
    }
    requested[servo] = atof(token);
  }

  token = strtok_r(nullptr, ",", &savePointer);
  if (token == nullptr)
  {
    Serial.println("ERROR,missing duration");
    return;
  }

  if (!anglesAreSafe(requested))
  {
    Serial.println("ERROR,target outside software limits");
    return;
  }

  unsigned long duration = strtoul(token, nullptr, 10);
  startMove(requested, constrain(duration, 300UL, 15000UL));
  Serial.println("OK,MOVE");
}

void startMove(const float requested[], unsigned long durationMs)
{
  for (byte servo = 0; servo < SERVO_COUNT; servo++)
  {
    startAngle[servo] = currentAngle[servo];
    targetAngle[servo] = requested[servo];
  }
  moveDurationMs = durationMs;
  moveStartedMs = millis();
  lastServoUpdateMs = moveStartedMs - SERVO_UPDATE_INTERVAL_MS;
  moving = true;
}

bool anglesAreSafe(const float values[])
{
  for (byte servo = 0; servo < SERVO_COUNT; servo++)
  {
    if (values[servo] < SERVO_MIN[servo] || values[servo] > SERVO_MAX[servo])
    {
      return false;
    }
  }
  return true;
}

void updateHoldSwitch()
{
  bool pressed = digitalRead(HOLD_SWITCH_PIN) == LOW;
  if (pressed)
  {
    moving = false;
  }
  if (pressed && !switchWasPressed)
  {
    Serial.println("OK,SWITCH HOLD");
  }
  switchWasPressed = pressed;
}

void updateMotion()
{
  if (!moving)
  {
    return;
  }

  unsigned long elapsed = millis() - moveStartedMs;
  float progress = constrain(
      (float)elapsed / (float)moveDurationMs, 0.0, 1.0);
  if (progress < 1.0 && millis() - lastServoUpdateMs < SERVO_UPDATE_INTERVAL_MS)
  {
    return;
  }
  lastServoUpdateMs = millis();

  float progress2 = progress * progress;
  float progress3 = progress2 * progress;
  float eased = progress3 * (progress * (progress * 6.0 - 15.0) + 10.0);

  for (byte servo = 0; servo < SERVO_COUNT; servo++)
  {
    currentAngle[servo] = startAngle[servo] + (targetAngle[servo] - startAngle[servo]) * eased;
  }
  writeAllOutputs();

  if (progress >= 1.0)
  {
    moving = false;
    Serial.println("OK,ARRIVED");
  }
}

void writeAllOutputs()
{
  for (byte servo = 0; servo < SERVO_COUNT; servo++)
  {
    writePhysicalServo(servo, currentAngle[servo]);
  }
}

void writePhysicalServo(byte servoIndex, float physicalAngle)
{
  physicalAngle = constrain(physicalAngle, 0.0, PHYSICAL_RANGE);
  int pulseUs = round(
      SERVO_PULSE_MIN_US + physicalAngle / PHYSICAL_RANGE * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US));
  servos[servoIndex].writeMicroseconds(
      constrain(pulseUs, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US));
}
