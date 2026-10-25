/*
  The serial console that drove the arm's J1, J3, J4 and J5 drives before the ground station did,
  kept as the known-good fallback. UNO R4 WiFi with an MCP2515 module (8 MHz crystal, CS D10,
  INT D2) on the drives' 500 kbit/s bus. Open the serial monitor at 115200 baud and type HELP.

  It runs the drives in speed mode without reading their encoders, so the J4/J5 limits are
  estimates from running time. To drive the arm from the ground station, flash
  carrier_bridge_mcp2515 instead.
*/

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
// MOTION SETTINGS
// =====================================================
//
// J1/J3 = faster arm joints
// J4/J5 = slower differential wrist
//
// =====================================================

uint16_t armSpeed = 600;
uint8_t armAccel  = 60;

uint16_t wristSpeed = 150;
uint8_t wristAccel  = 10;


// =====================================================
// J4/J5 SOFTWARE LIMITS
// =====================================================
//
// Current mechanical middle = 0 degrees.
//
// Allowed estimated motor position:
//
//      -270° -------- 0° -------- +270°
//
// IMPORTANT:
//
// These are software-estimated positions.
// There is currently NO encoder feedback.
//
// The estimate is based on commanded running time.
// If a motor skips or the wrist is moved manually,
// the estimate can become incorrect.
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
//
// Tune this experimentally.
//
// Example:
//
// If the wrist motor takes 6 seconds to move 270°:
//
// 270 / 6 = 45 deg/sec
//
// Then:
//
// DEGRATE 45
//
// =====================================================

float DEG_PER_SEC = 30.0;


// =====================================================
// J4/J5 MOTION STATE
// =====================================================

int j4Direction = 0;
int j5Direction = 0;

// -1 = CCW
//  0 = stopped
// +1 = CW

unsigned long lastAngleUpdate = 0;


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
        rx.print("RX");
    }
}


// =====================================================
// ENABLE MOTOR
// =====================================================

void enableMotor(uint16_t id)
{
    uint8_t data[3];

    data[0] = 0xF3;
    data[1] = 0x01;

    data[2] =
        checksum(
            id,
            data,
            2
        );

    sendMKS(
        id,
        data,
        3
    );
}


// =====================================================
// DISABLE MOTOR
// =====================================================

void disableMotor(uint16_t id)
{
    uint8_t data[3];

    data[0] = 0xF3;
    data[1] = 0x00;

    data[2] =
        checksum(
            id,
            data,
            2
        );

    sendMKS(
        id,
        data,
        3
    );
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

    data[2] =
        speed & 0xFF;

    data[3] =
        accel;

    data[4] =
        checksum(
            id,
            data,
            4
        );

    sendMKS(
        id,
        data,
        5
    );
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

    // Zero acceleration parameter gives our quick stop.
    data[3] = 0x00;

    data[4] =
        checksum(
            id,
            data,
            4
        );

    sendMKS(
        id,
        data,
        5
    );


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

    data[1] =
        checksum(
            id,
            data,
            1
        );

    sendMKS(
        id,
        data,
        2
    );
}


// =====================================================
// ENABLE ALL
// =====================================================

void enableAll()
{
    enableMotor(J1);

    delay(5);

    enableMotor(J3);

    delay(5);

    enableMotor(J4);

    delay(5);

    enableMotor(J5);

    Serial.println("ENABLED");
}


// =====================================================
// DISABLE ALL
// =====================================================

void disableAll()
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

    Serial.println("DISABLED");
}


// =====================================================
// STOP ALL
// =====================================================

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


// =====================================================
// ESTOP ALL
// =====================================================

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
// UPDATE ESTIMATED J4/J5 ANGLES
// =====================================================

void updateAngles()
{
    unsigned long now = millis();

    float dt =
        (now - lastAngleUpdate)
        / 1000.0;

    lastAngleUpdate = now;


    // Prevent a long blocking operation from
    // creating a giant angle jump.
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


    // =================================================
    // J4 +270 LIMIT
    // =================================================

    if (j4Angle >= J4_MAX)
    {
        j4Angle = J4_MAX;

        if (j4Direction > 0)
        {
            stopMotor(J4);

            Serial.println(
                "J4 +270 LIMIT"
            );
        }
    }


    // =================================================
    // J4 -270 LIMIT
    // =================================================

    if (j4Angle <= J4_MIN)
    {
        j4Angle = J4_MIN;

        if (j4Direction < 0)
        {
            stopMotor(J4);

            Serial.println(
                "J4 -270 LIMIT"
            );
        }
    }


    // =================================================
    // J5 +270 LIMIT
    // =================================================

    if (j5Angle >= J5_MAX)
    {
        j5Angle = J5_MAX;

        if (j5Direction > 0)
        {
            stopMotor(J5);

            Serial.println(
                "J5 +270 LIMIT"
            );
        }
    }


    // =================================================
    // J5 -270 LIMIT
    // =================================================

    if (j5Angle <= J5_MIN)
    {
        j5Angle = J5_MIN;

        if (j5Direction < 0)
        {
            stopMotor(J5);

            Serial.println(
                "J5 -270 LIMIT"
            );
        }
    }
}


// =====================================================
// SAFE J4 CW
// =====================================================

void moveJ4CW()
{
    if (j4Angle >= J4_MAX)
    {
        Serial.println(
            "J4 +270 BLOCKED"
        );

        return;
    }


    j4Direction = 1;


    moveMotor(
        J4,
        wristSpeed,
        wristAccel,
        false
    );
}


// =====================================================
// SAFE J4 CCW
// =====================================================

void moveJ4CCW()
{
    if (j4Angle <= J4_MIN)
    {
        Serial.println(
            "J4 -270 BLOCKED"
        );

        return;
    }


    j4Direction = -1;


    moveMotor(
        J4,
        wristSpeed,
        wristAccel,
        true
    );
}


// =====================================================
// SAFE J5 CW
// =====================================================

void moveJ5CW()
{
    if (j5Angle >= J5_MAX)
    {
        Serial.println(
            "J5 +270 BLOCKED"
        );

        return;
    }


    j5Direction = 1;


    moveMotor(
        J5,
        wristSpeed,
        wristAccel,
        false
    );
}


// =====================================================
// SAFE J5 CCW
// =====================================================

void moveJ5CCW()
{
    if (j5Angle <= J5_MIN)
    {
        Serial.println(
            "J5 -270 BLOCKED"
        );

        return;
    }


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
//
// W:
// J4 CW
// J5 CW
//
// S:
// J4 CCW
// J5 CCW
//
// A:
// J4 CCW
// J5 CW
//
// D:
// J4 CW
// J5 CCW
//
// =====================================================

void differentialW()
{
    moveJ4CW();

    delay(3);

    moveJ5CW();

    Serial.println("DIFF_W");
}


void differentialS()
{
    moveJ4CCW();

    delay(3);

    moveJ5CCW();

    Serial.println("DIFF_S");
}


void differentialA()
{
    moveJ4CCW();

    delay(3);

    moveJ5CW();

    Serial.println("DIFF_A");
}


void differentialD()
{
    moveJ4CW();

    delay(3);

    moveJ5CCW();

    Serial.println("DIFF_D");
}


void stopDifferential()
{
    stopMotor(J4);

    delay(3);

    stopMotor(J5);

    Serial.println("DIFF_STOP");
}


// =====================================================
// SET CURRENT WRIST POSITION AS ZERO
// =====================================================

void zeroDifferential()
{
    stopDifferential();

    j4Angle = 0.0;
    j5Angle = 0.0;

    Serial.println(
        "WRIST CENTER SET TO 0 DEG"
    );
}


// =====================================================
// PRINT WRIST POSITION
// =====================================================

void printAngles()
{
    Serial.print("J4 = ");

    Serial.print(
        j4Angle,
        1
    );

    Serial.print(" deg");


    Serial.print("    J5 = ");

    Serial.print(
        j5Angle,
        1
    );

    Serial.println(" deg");
}


// =====================================================
// AUTO DEMO
// =====================================================
//
// All four active joints move at the same time.
//
// Phase 1:
//
// J1 CW
// J3 CW
// J4 CW
// J5 CW
//
// Phase 2:
//
// J1 CCW
// J3 CCW
// J4 CCW
// J5 CCW
//
// =====================================================

void autoDemo()
{
    Serial.println("AUTO_START");


    // =================================================
    // FORWARD PHASE
    // =================================================

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


    unsigned long start =
        millis();


    while (
        millis() - start
        < 1000
    )
    {
        readCAN();

        updateAngles();


        // Allow STOP / ESTOP during auto demo
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

                Serial.println(
                    "AUTO_ABORTED"
                );

                return;
            }
        }
    }


    stopAll();


    delay(500);


    // =================================================
    // REVERSE PHASE
    // =================================================

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


    start =
        millis();


    while (
        millis() - start
        < 1000
    )
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

                Serial.println(
                    "AUTO_ABORTED"
                );

                return;
            }
        }
    }


    stopAll();


    Serial.println(
        "AUTO_DONE"
    );
}


// =====================================================
// HELP
// =====================================================

void printHelp()
{
    Serial.println();

    Serial.println(
        "========== WAYBIONIC =========="
    );

    Serial.println();

    Serial.println("ENABLE");
    Serial.println("DISABLE");
    Serial.println("STOP");
    Serial.println("ESTOP");
    Serial.println("AUTO");

    Serial.println();

    Serial.println(
        "J1_CW / J1_CCW / J1_STOP"
    );

    Serial.println(
        "J3_CW / J3_CCW / J3_STOP"
    );

    Serial.println();

    Serial.println("DIFF_W");
    Serial.println("DIFF_A");
    Serial.println("DIFF_S");
    Serial.println("DIFF_D");
    Serial.println("DIFF_STOP");

    Serial.println();

    Serial.println("ZERO");
    Serial.println("POS");

    Serial.println();

    Serial.println("ARMSPEED 600");
    Serial.println("ARMACCEL 60");

    Serial.println("WRISTSPEED 150");
    Serial.println("WRISTACCEL 10");

    Serial.println("DEGRATE 30");

    Serial.println();

    Serial.println(
        "J2 DOES NOT EXIST"
    );

    Serial.println(
        "==============================="
    );
}


// =====================================================
// COMMAND PROCESSOR
// =====================================================

void processCommand(String cmd)
{
    cmd.trim();

    cmd.toUpperCase();


    // =================================================
    // GLOBAL
    // =================================================

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

    else if (cmd == "AUTO")
    {
        autoDemo();
    }


    // =================================================
    // J1
    // =================================================

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


    // =================================================
    // J3
    // =================================================

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


    // =================================================
    // DIFFERENTIAL
    // =================================================

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


    // =================================================
    // WRIST ZERO / POSITION
    // =================================================

    else if (cmd == "ZERO")
    {
        zeroDifferential();
    }

    else if (cmd == "POS")
    {
        printAngles();
    }


    // =================================================
    // ARM SPEED
    // =================================================

    else if (
        cmd.startsWith("ARMSPEED ")
    )
    {
        int value =
            cmd.substring(9).toInt();


        if (
            value >= 0 &&
            value <= 3000
        )
        {
            armSpeed =
                value;

            Serial.print(
                "ARM_SPEED="
            );

            Serial.println(
                armSpeed
            );
        }
    }


    // =================================================
    // ARM ACCELERATION
    // =================================================

    else if (
        cmd.startsWith("ARMACCEL ")
    )
    {
        int value =
            cmd.substring(9).toInt();


        if (
            value >= 0 &&
            value <= 255
        )
        {
            armAccel =
                value;

            Serial.print(
                "ARM_ACCEL="
            );

            Serial.println(
                armAccel
            );
        }
    }


    // =================================================
    // WRIST SPEED
    // =================================================

    else if (
        cmd.startsWith("WRISTSPEED ")
    )
    {
        int value =
            cmd.substring(11).toInt();


        if (
            value >= 0 &&
            value <= 3000
        )
        {
            wristSpeed =
                value;

            Serial.print(
                "WRIST_SPEED="
            );

            Serial.println(
                wristSpeed
            );
        }
    }


    // =================================================
    // WRIST ACCELERATION
    // =================================================

    else if (
        cmd.startsWith("WRISTACCEL ")
    )
    {
        int value =
            cmd.substring(11).toInt();


        if (
            value >= 0 &&
            value <= 255
        )
        {
            wristAccel =
                value;

            Serial.print(
                "WRIST_ACCEL="
            );

            Serial.println(
                wristAccel
            );
        }
    }


    // =================================================
    // WRIST DEGREE RATE
    // =================================================

    else if (
        cmd.startsWith("DEGRATE ")
    )
    {
        float value =
            cmd.substring(8).toFloat();


        if (value > 0)
        {
            DEG_PER_SEC =
                value;

            Serial.print(
                "DEG_PER_SEC="
            );

            Serial.println(
                DEG_PER_SEC
            );
        }
    }


    else if (cmd == "HELP")
    {
        printHelp();
    }


    else
    {
        Serial.print(
            "UNKNOWN: "
        );

        Serial.println(cmd);
    }
}


// =====================================================
// SETUP
// =====================================================

void setup()
{
    Serial.begin(115200);

    delay(1500);


    Serial.println();

    Serial.println(
        "WAYBIONIC CONTROLLER"
    );


    while (
        CAN.begin(
            CANController::Mode::Normal
        )
        != CANController::OK
    )
    {
        Serial.println(
            "CAN INIT FAILED"
        );

        delay(1000);
    }


    Serial.println(
        "CAN INIT OK"
    );


    lastAngleUpdate =
        millis();


    // Current J4/J5 pose is assumed to be center.
    j4Angle = 0.0;
    j5Angle = 0.0;


    printHelp();
}


// =====================================================
// MAIN LOOP
// =====================================================

void loop()
{
    readCAN();

    updateAngles();


    if (Serial.available())
    {
        String command =
            Serial.readStringUntil('\n');

        processCommand(command);
    }
}