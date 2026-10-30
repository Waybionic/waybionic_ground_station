// =====================================================
// CHANGELOG (newest first) - update this on every edit
// =====================================================
// 2026-10-30 18:10 MDT
//   - Finish hand-track restore polish: coastCurrentMa=250 docs/comments
//   - ASCII-safe approx/degree comments; stale Ma=0 comment fixed
// 2026-10-30 18:00 MDT
//   - Restore OG hand-track firmware (pre CAN-debug / J1+J3-only smoke)
//   - Source: ground_station bd19abb waybionic_ascii_teleop
//   - J1+J3+J4+J5, Config_8MHz_500kbps, DISABLE=coast for hand ENC
//   - Keep digit printI64 (R4 %lld prints "ld ld"); coast Ma=250 (Ma=0 freezes ENC)
// =====================================================

#include "AA_MCP2515.h"

// =====================================================
// CAN SETUP
// =====================================================

const CANBitrate::Config CAN_BITRATE =
    CANBitrate::Config_8MHz_500kbps;

const uint8_t CAN_CS  = 10;
const int8_t  CAN_INT = 2;

CANConfig config(
    CAN_BITRATE,
    CAN_CS,
    CAN_INT
);

CANController CAN(config);


// =====================================================
// MOTOR IDS
// =====================================================
//
// J2 IS INTENTIONALLY UNUSED.
//
// =====================================================

const uint16_t J1 = 1;
const uint16_t J3 = 3;
const uint16_t J4 = 4;
const uint16_t J5 = 5;


// =====================================================
// MOTION SETTINGS (gear-aware joint-space targets)
// =====================================================
//
// J1/J3: SERVO42D + 26:1 gearbox
// J4/J5: SERVO42D + 3:1 reduction
//
// MKS 0xF6 speed is motor shaft RPM. Joint rate ~= motor_rpm / gear.
// Reflected inertia scales ~gear^2, so arm needs much higher accel
// than wrist or it stalls / never reaches the commanded RPM.
//
// Targets (approx joint-space):
//   arm   ~28 deg/s  -> motor_rpm = 28/6 * 26 ~= 121
//   wrist ~180 deg/s -> motor_rpm = 180/6 * 3 ~= 90
//
// =====================================================

uint16_t armSpeed = 120;
uint8_t armAccel  = 240;

uint16_t wristSpeed = 90;
uint8_t wristAccel  = 45;


// =====================================================
// J4/J5 SOFTWARE LIMITS
// =====================================================
//
// Current mechanical middle = 0 degrees.
//
// Allowed estimated range:
//
//      -270 deg -------- 0 deg -------- +270 deg
//
// IMPORTANT:
// This is estimated from commanded runtime.
// There is no real encoder feedback.
//
// =====================================================

float j4Angle = 0.0;
float j5Angle = 0.0;

const float J4_MIN = -270.0;
const float J4_MAX =  270.0;

const float J5_MIN = -270.0;
const float J5_MAX =  270.0;


// =====================================================
// ESTIMATED WRIST ANGULAR SPEED
// =====================================================

float DEG_PER_SEC = 30.0;


// =====================================================
// J4/J5 MOTION STATE
// =====================================================

int j4Direction = 0;
int j5Direction = 0;

unsigned long lastAngleUpdate = 0;

// =====================================================
// MKS MAGNETIC ENCODER STREAM (31h) FOR RVIZ CLOSED-LOOP
// =====================================================
// Active joints J1/J3/J4/J5 = SERVO42D. J2 = SERVO57D (unused).
// Host parses: ENC <c1> <c3> <c4> <c5>
// Counts from 35h raw magnetic multi-turn (0x4000 counts/rev).
// DISABLE coasts (Ma=coastCurrentMa, F3 on) so ENC still tracks hand motion.
// =====================================================

int64_t encJ1 = 0;
int64_t encJ3 = 0;
int64_t encJ4 = 0;
int64_t encJ5 = 0;

bool haveEncJ1 = false;
bool haveEncJ3 = false;
bool haveEncJ4 = false;
bool haveEncJ5 = false;

bool encStream = true;
unsigned long lastEncRequest = 0;
unsigned long lastEncPrint = 0;
// 25 Hz: fast enough that hand-moving a disabled joint still tracks in RViz.
const unsigned long ENC_PERIOD_MS = 40;


// =====================================================
// CHECKSUM
// =====================================================

uint8_t checksum(
    uint16_t id,
    uint8_t *data,
    uint8_t len
)
{
    uint16_t sum = id;

    for (uint8_t i = 0; i < len; i++)
    {
        sum += data[i];
    }

    return sum & 0xFF;
}


// =====================================================
// SEND CAN
// =====================================================

void sendMKS(
    uint16_t id,
    uint8_t *data,
    uint8_t len
)
{
    CANFrame frame(
        id,
        data,
        len
    );

    CANController::IOResult result =
        CAN.write(frame);

    if (result != CANController::IOResult::OK)
    {
        Serial.print("CAN_TX_FAIL ID ");
        Serial.println(id);
    }
}


// =====================================================
// ENCODER REQUEST / PARSE (MKS 31h)
// =====================================================

// 35h = raw accumulated multi-turn (magnetic). Prefer over 31h: some
// firmware freezes 31h after F3 disable; 35h tracks shaft motion either way.
const uint8_t CMD_READ_ENCODER_RAW = 0x35;

void requestEncoder(uint16_t id)
{
    uint8_t data[2];

    data[0] = CMD_READ_ENCODER_RAW;
    data[1] = checksum(id, data, 1);

    sendMKS(id, data, 2);
}


int64_t decodeEncoder48(const uint8_t *args)
{
    uint64_t raw = 0;

    for (uint8_t i = 0; i < 6; i++)
    {
        raw = (raw << 8) | args[i];
    }

    // Sign-extend 48-bit to 64-bit.
    if (raw & 0x800000000000ULL)
    {
        raw |= 0xFFFF000000000000ULL;
    }

    return (int64_t)raw;
}


void storeEncoder(uint32_t id, int64_t value)
{
    if (id == J1)
    {
        encJ1 = value;
        haveEncJ1 = true;
    }
    else if (id == J3)
    {
        encJ3 = value;
        haveEncJ3 = true;
    }
    else if (id == J4)
    {
        encJ4 = value;
        haveEncJ4 = true;
    }
    else if (id == J5)
    {
        encJ5 = value;
        haveEncJ5 = true;
    }
}


void printI64(int64_t value)
{
    // UNO R4 / newlib often breaks "%lld" (Serial shows "ld ld").
    if (value < 0)
    {
        Serial.print('-');
        value = -value;
    }
    char buf[24];
    uint8_t i = 0;
    if (value == 0)
    {
        buf[i++] = '0';
    }
    else
    {
        while (value > 0 && i < sizeof(buf) - 1)
        {
            buf[i++] = '0' + (char)(value % 10);
            value /= 10;
        }
    }
    while (i > 0)
    {
        Serial.print(buf[--i]);
    }
}


void printEncoderLine()
{
    // Wait until every active SERVO42D has answered 35h at least once.
    if (!(haveEncJ1 && haveEncJ3 && haveEncJ4 && haveEncJ5))
    {
        return;
    }

    Serial.print("ENC ");
    printI64(encJ1);
    Serial.print(' ');
    printI64(encJ3);
    Serial.print(' ');
    printI64(encJ4);
    Serial.print(' ');
    printI64(encJ5);
    Serial.println();
}


void serviceEncoders()
{
    unsigned long now = millis();

    if (!encStream)
    {
        return;
    }

    if (now - lastEncRequest >= ENC_PERIOD_MS)
    {
        lastEncRequest = now;

        requestEncoder(J1);
        delay(2);
        requestEncoder(J3);
        delay(2);
        requestEncoder(J4);
        delay(2);
        requestEncoder(J5);
    }

    if (now - lastEncPrint >= ENC_PERIOD_MS)
    {
        lastEncPrint = now;
        printEncoderLine();
    }
}


// =====================================================
// READ CAN
// =====================================================

void readCAN()
{
    CANFrame rx;

    while (
        CAN.read(rx)
        == CANController::IOResult::OK
    )
    {
        const uint8_t dlc = rx.getDlc();
        const uint8_t *data = rx.getData();

        // 35h (raw) or 31h (addition) reply: code + 6-byte int48 (+ optional CRC)
        if (
            dlc >= 7 &&
            (data[0] == CMD_READ_ENCODER_RAW || data[0] == 0x31)
        )
        {
            storeEncoder(
                rx.getId(),
                decodeEncoder48(&data[1])
            );
        }
    }
}


// =====================================================
// ENABLE / DISABLE
// =====================================================

// Working current for SERVO42D (mA). Restored on ENABLE after coast.
uint16_t workCurrentMa = 1600;
// Soft freewheel current. Ma=0 often FREEZES 42D encoder updates so hand-turn
// never reaches RViz. Small Ma keeps magnetic ENC alive while shaft is free.
uint16_t coastCurrentMa = 250;

void enableMotor(uint16_t id)
{
    uint8_t data[3];

    data[0] = 0xF3;
    data[1] = 0x01;
    data[2] = checksum(id, data, 2);

    sendMKS(id, data, 3);
}


void disableMotor(uint16_t id)
{
    // True coil power-down (F3=0). Encoder often freezes - prefer coastMotor.
    uint8_t data[3];

    data[0] = 0xF3;
    data[1] = 0x00;
    data[2] = checksum(id, data, 2);

    sendMKS(id, data, 3);
}


void setMotorCurrentMa(uint16_t id, uint16_t ma, bool save)
{
    // 83h: set Ma. Trailing 00h = apply without EEPROM save (runtime only).
    if (ma > 3000)
    {
        ma = 3000;
    }

    if (save)
    {
        uint8_t data[4];
        data[0] = 0x83;
        data[1] = (uint8_t)(ma >> 8);
        data[2] = (uint8_t)(ma & 0xFF);
        data[3] = checksum(id, data, 3);
        sendMKS(id, data, 4);
    }
    else
    {
        uint8_t data[5];
        data[0] = 0x83;
        data[1] = (uint8_t)(ma >> 8);
        data[2] = (uint8_t)(ma & 0xFF);
        data[3] = 0x00; // do not save
        data[4] = checksum(id, data, 4);
        sendMKS(id, data, 5);
    }
}


void coastMotor(uint16_t id)
{
    // Soft freewheel: stay F3-enabled so magnetic encoder keeps updating,
    // drop runtime current so the shaft can be turned by hand.
    stopMotor(id);
    delay(2);
    enableMotor(id);
    delay(2);
    setMotorCurrentMa(id, coastCurrentMa, false);
}


// =====================================================
// SPEED MODE
// =====================================================

void moveMotor(
    uint16_t id,
    uint16_t speed,
    uint8_t accel,
    bool ccw
)
{
    if (speed > 3000)
    {
        speed = 3000;
    }

    uint8_t data[5];

    data[0] = 0xF6;

    data[1] =
        (ccw ? 0x80 : 0x00) |
        ((speed >> 8) & 0x0F);

    data[2] = speed & 0xFF;
    data[3] = accel;

    data[4] = checksum(id, data, 4);

    sendMKS(id, data, 5);
}


// =====================================================
// QUICK STOP
// =====================================================

void stopMotor(uint16_t id)
{
    uint8_t data[5];

    data[0] = 0xF6;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;

    data[4] = checksum(id, data, 4);

    sendMKS(id, data, 5);

    if (id == J4)
    {
        j4Direction = 0;
    }

    if (id == J5)
    {
        j5Direction = 0;
    }
}


// =====================================================
// EMERGENCY STOP
// =====================================================

void emergencyStopMotor(uint16_t id)
{
    uint8_t data[2];

    data[0] = 0xF7;
    data[1] = checksum(id, data, 1);

    sendMKS(id, data, 2);
}


// =====================================================
// ALL MOTOR HELPERS
// =====================================================

void restoreCurrentAndEnable(uint16_t id)
{
    enableMotor(id);
    delay(2);
    setMotorCurrentMa(id, workCurrentMa, false);
}


void enableAll()
{
    // Re-arm coils + restore Ma after coast (DISABLE).
    restoreCurrentAndEnable(J1);
    delay(5);

    restoreCurrentAndEnable(J3);
    delay(5);

    restoreCurrentAndEnable(J4);
    delay(5);

    restoreCurrentAndEnable(J5);

    Serial.println("ENABLED");
}


void disableAll()
{
    // Coast (current=0, stay enabled) so hand-moved joints still update ENC.
    // Use POWEROFF for true F3 coil shutdown (encoder may freeze).
    coastMotor(J1);
    delay(5);

    coastMotor(J3);
    delay(5);

    coastMotor(J4);
    delay(5);

    coastMotor(J5);

    j4Direction = 0;
    j5Direction = 0;

    Serial.println("DISABLED");
}


void powerOffAll()
{
    disableMotor(J1);
    delay(5);
    disableMotor(J3);
    delay(5);
    disableMotor(J4);
    delay(5);
    disableMotor(J5);

    j4Direction = 0;
    j5Direction = 0;

    Serial.println("POWEROFF");
}


void stopAll()
{
    stopMotor(J1);
    delay(3);

    stopMotor(J3);
    delay(3);

    stopMotor(J4);
    delay(3);

    stopMotor(J5);

    Serial.println("STOPPED");
}


void emergencyStopAll()
{
    emergencyStopMotor(J1);
    emergencyStopMotor(J3);
    emergencyStopMotor(J4);
    emergencyStopMotor(J5);

    j4Direction = 0;
    j5Direction = 0;

    Serial.println("ESTOP");
}


// =====================================================
// WRIST ANGLE ESTIMATION
// =====================================================

void updateAngles()
{
    unsigned long now = millis();

    float dt =
        (now - lastAngleUpdate)
        / 1000.0;

    lastAngleUpdate = now;

    if (dt > 0.25)
    {
        dt = 0.25;
    }

    j4Angle +=
        j4Direction *
        DEG_PER_SEC *
        dt;

    j5Angle +=
        j5Direction *
        DEG_PER_SEC *
        dt;

    // Software travel limits DISABLED for bring-up (no auto-stop / clamp).
}


// =====================================================
// SAFE J4/J5 CONTROL
// =====================================================

void moveJ4CW()
{
    j4Direction = 1;

    moveMotor(
        J4,
        wristSpeed,
        wristAccel,
        false
    );
}


void moveJ4CCW()
{
    j4Direction = -1;

    moveMotor(
        J4,
        wristSpeed,
        wristAccel,
        true
    );
}


void moveJ5CW()
{
    j5Direction = 1;

    moveMotor(
        J5,
        wristSpeed,
        wristAccel,
        false
    );
}


void moveJ5CCW()
{
    j5Direction = -1;

    moveMotor(
        J5,
        wristSpeed,
        wristAccel,
        true
    );
}


// =====================================================
// DIFFERENTIAL MODES
// =====================================================

void differentialW()
{
    moveJ4CW();
    delay(3);
    moveJ5CW();
    // Quiet on repeat pulses - USB CDC hangs if we println every 150-400ms.
}


void differentialS()
{
    moveJ4CCW();
    delay(3);
    moveJ5CCW();
}


void differentialA()
{
    moveJ4CCW();
    delay(3);
    moveJ5CW();
}


void differentialD()
{
    moveJ4CW();
    delay(3);
    moveJ5CCW();
}


void stopDifferential()
{
    stopMotor(J4);
    delay(3);
    stopMotor(J5);
}


// =====================================================
// WRIST ZERO / POSITION
// =====================================================

void zeroDifferential()
{
    stopDifferential();

    j4Angle = 0.0;
    j5Angle = 0.0;

    Serial.println("WRIST CENTER SET TO 0 DEG");
}


void printAngles()
{
    Serial.print("J4 = ");
    Serial.print(j4Angle, 1);
    Serial.print(" deg");

    Serial.print("    J5 = ");
    Serial.print(j5Angle, 1);
    Serial.println(" deg");
}


// =====================================================
// AUTO DEMO
// =====================================================
//
// J1, J3, J4, J5 move at the same time.
//
// =====================================================

void autoDemo()
{
    Serial.println("AUTO_START");


    // FORWARD
    moveMotor(
        J1,
        armSpeed,
        armAccel,
        false
    );

    moveMotor(
        J3,
        armSpeed,
        armAccel,
        false
    );

    moveJ4CW();
    moveJ5CW();


    unsigned long start = millis();

    while (millis() - start < 1000)
    {
        readCAN();
        updateAngles();

        if (Serial.available())
        {
            String cmd =
                Serial.readStringUntil('\n');

            cmd.trim();
            cmd.toUpperCase();

            if (
                cmd == "STOP" ||
                cmd == "ESTOP"
            )
            {
                emergencyStopAll();

                Serial.println("AUTO_ABORTED");
                return;
            }
        }
    }

    stopAll();

    delay(500);


    // REVERSE
    moveMotor(
        J1,
        armSpeed,
        armAccel,
        true
    );

    moveMotor(
        J3,
        armSpeed,
        armAccel,
        true
    );

    moveJ4CCW();
    moveJ5CCW();


    start = millis();

    while (millis() - start < 1000)
    {
        readCAN();
        updateAngles();

        if (Serial.available())
        {
            String cmd =
                Serial.readStringUntil('\n');

            cmd.trim();
            cmd.toUpperCase();

            if (
                cmd == "STOP" ||
                cmd == "ESTOP"
            )
            {
                emergencyStopAll();

                Serial.println("AUTO_ABORTED");
                return;
            }
        }
    }

    stopAll();

    Serial.println("AUTO_DONE");
}


// =====================================================
// COMMAND PROCESSOR
// =====================================================

void processCommand(String cmd)
{
    cmd.trim();
    cmd.toUpperCase();


    if (cmd == "ENABLE")
    {
        enableAll();
    }

    else if (cmd == "DISABLE")
    {
        disableAll();
    }

    else if (cmd == "STOP")
    {
        stopAll();
    }

    else if (cmd == "ESTOP")
    {
        emergencyStopAll();
    }

    else if (cmd == "POWEROFF")
    {
        powerOffAll();
    }

    else if (cmd == "AUTO")
    {
        autoDemo();
    }


    // J1
    else if (cmd == "J1_CW")
    {
        moveMotor(
            J1,
            armSpeed,
            armAccel,
            false
        );
    }

    else if (cmd == "J1_CCW")
    {
        moveMotor(
            J1,
            armSpeed,
            armAccel,
            true
        );
    }

    else if (cmd == "J1_STOP")
    {
        stopMotor(J1);
    }


    // J3
    else if (cmd == "J3_CW")
    {
        moveMotor(
            J3,
            armSpeed,
            armAccel,
            false
        );
    }

    else if (cmd == "J3_CCW")
    {
        moveMotor(
            J3,
            armSpeed,
            armAccel,
            true
        );
    }

    else if (cmd == "J3_STOP")
    {
        stopMotor(J3);
    }


    // Differential wrist
    else if (cmd == "DIFF_W")
    {
        differentialW();
    }

    else if (cmd == "DIFF_S")
    {
        differentialS();
    }

    else if (cmd == "DIFF_A")
    {
        differentialA();
    }

    else if (cmd == "DIFF_D")
    {
        differentialD();
    }

    else if (cmd == "DIFF_STOP")
    {
        stopDifferential();
    }


    // Wrist zero / position
    else if (cmd == "ZERO")
    {
        zeroDifferential();
    }

    else if (cmd == "POS")
    {
        printAngles();
    }

    else if (cmd == "ENC")
    {
        requestEncoder(J1);
        delay(2);
        requestEncoder(J3);
        delay(2);
        requestEncoder(J4);
        delay(2);
        requestEncoder(J5);
        delay(5);
        readCAN();
        printEncoderLine();
    }

    else if (cmd == "ENCON")
    {
        encStream = true;
        Serial.println("ENC_STREAM=1");
    }

    else if (cmd == "ENCOFF")
    {
        encStream = false;
        Serial.println("ENC_STREAM=0");
    }


    // Arm speed
    else if (cmd.startsWith("ARMSPEED "))
    {
        int value =
            cmd.substring(9).toInt();

        if (
            value >= 0 &&
            value <= 3000
        )
        {
            armSpeed = value;

            Serial.print("ARM_SPEED=");
            Serial.println(armSpeed);
        }
    }


    // Arm acceleration
    else if (cmd.startsWith("ARMACCEL "))
    {
        int value =
            cmd.substring(9).toInt();

        if (
            value >= 0 &&
            value <= 255
        )
        {
            armAccel = value;

            Serial.print("ARM_ACCEL=");
            Serial.println(armAccel);
        }
    }


    // Wrist speed
    else if (cmd.startsWith("WRISTSPEED "))
    {
        int value =
            cmd.substring(11).toInt();

        if (
            value >= 0 &&
            value <= 3000
        )
        {
            wristSpeed = value;

            Serial.print("WRIST_SPEED=");
            Serial.println(wristSpeed);
        }
    }


    // Wrist acceleration
    else if (cmd.startsWith("WRISTACCEL "))
    {
        int value =
            cmd.substring(11).toInt();

        if (
            value >= 0 &&
            value <= 255
        )
        {
            wristAccel = value;

            Serial.print("WRIST_ACCEL=");
            Serial.println(wristAccel);
        }
    }


    // Estimated wrist degree rate
    else if (cmd.startsWith("DEGRATE "))
    {
        float value =
            cmd.substring(8).toFloat();

        if (value > 0)
        {
            DEG_PER_SEC = value;

            Serial.print("DEG_PER_SEC=");
            Serial.println(DEG_PER_SEC);
        }
    }


    else
    {
        Serial.print("UNKNOWN: ");
        Serial.println(cmd);
    }
}


// =====================================================
// SETUP
// =====================================================

void setup()
{
    Serial.begin(115200);
    // UNO R4 native USB: wait briefly so host sees boot lines.
    unsigned long serialWait = millis();
    while (!Serial && (millis() - serialWait) < 3000)
    {
        delay(10);
    }
    delay(200);

    Serial.println();
    Serial.println("WAYBIONIC CONTROLLER");
    Serial.println("HAND-TRACK RESTORE J1+J3+J4+J5");
    Serial.println("CAN INIT...");

    uint8_t canTries = 0;
    while (
        CAN.begin(
            CANController::Mode::Normal
        )
        != CANController::OK
    )
    {
        canTries++;
        Serial.print("CAN INIT FAILED try=");
        Serial.println(canTries);
        delay(500);
        if (canTries >= 10)
        {
            Serial.println("CAN INIT GIVING UP - serial alive");
            break;
        }
    }

    if (canTries < 10)
    {
        Serial.println("CAN INIT OK");
    }

    lastAngleUpdate = millis();

    j4Angle = 0.0;
    j5Angle = 0.0;
    Serial.println("SETUP DONE");
}


// =====================================================
// MAIN LOOP
// =====================================================

void loop()
{
    readCAN();

    updateAngles();

    serviceEncoders();


    if (Serial.available())
    {
        String command =
            Serial.readStringUntil('\n');

        processCommand(command);
    }
}
