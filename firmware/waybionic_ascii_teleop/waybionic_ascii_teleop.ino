// =====================================================
// CHANGELOG (newest first) - update this on every edit
// =====================================================
// 2026-10-31 05:10 MDT
//   - Pitch ROM gate ALL DIFF_* via motor pitch estimate (host maps pitch to
//     DIFF_A/D so old DIFF_W/S-only limits let motors run past +/-75)
//   - HOME / SETHOME: F5 absolute return to captured startup encoder pose
// 2026-10-31 04:45 MDT
//   - 16MHz tables also TXBO/rx=0 on this bench -> revert Config_8MHz_500kbps
//   - Pause ENC TX while bus dead (stop digging TEC hole); CANRECOVER cmd
//   - Set SR_vFOC (82h mode=05) on ENABLE per MKS bus-control path
// 2026-10-31 04:30 MDT
//   - Probe: TEC=128 TXBO rx=0 with 8MHz tables -> try Config_16MHz_500kbps
//     (8M timing on 16M xtal ~= 1Mbit; motors @500k never ACK; ENC never arrives)
//   - SPI 8 MHz; bus-off recover via CAN.begin+setFilters; XTAL printed at boot
// 2026-10-31 04:15 MDT
//   - CANDUMP: print raw RX hex so we can see 31h/35h/30h uplink shapes
//   - Accept DLC>=7 encoder replies; CRC required only when DLC==8
//   - Also decode 30h carry|value (manual 5.1.1) into cumulative counts
//   - Longer settle after 31h TX (3ms) before RX drain
// 2026-10-31 04:00 MDT
//   - MKS V1.0.9 align: ENC uses 31h (cumulative, DLC8+CRC), not dual 35h/31h
//   - MCP setFilters(false) after begin (default masks drop motor replies)
//   - 8Ch respond=1/active=1 on setup + ENABLE so drives answer reads
//   - Parse requires DLC==8 and CHECKSUM 8bit (id + body) & 0xFF
// 2026-10-31 03:35 MDT
//   - Wrist pitch soft ROM +/-75 deg (DIFF_W/S stop at limits)
// 2026-10-31 03:30 MDT
//   - coastCurrentMa back to 250 (user request)
//   - Rate-limit CAN_TX_FAIL (was flooding CDC so host never saw ENC)
//   - Also request 31h; accept std ID mask in storeEncoder
// 2026-10-31 03:25 MDT
//   - ENC never locked: burst 4x 35h without drain (MCP 2 RX bufs) + required
//     all four haveEnc before print. Round-robin + ENC_WAIT + print partial.
// 2026-10-31 03:20 MDT
//   - DISABLE gravity-hold: coastCurrentMa default 800 (hand-move + anti-droop)
//   - COASTMA <mA> runtime tune; ENC stays on so host can hand-follow in Q mode
// 2026-10-30 18:12 MDT
//   - Uploaded restored hand-track to UNO; Serial answers ENABLE/DISABLE/POS
//   - Re-added R4 Serial wait + CAN init give-up (pure OG setup hung silent)
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
// MCP2515 module crystal MUST match Config_* below.
// 8M tables on a 16M crystal => ~1 Mbit on the wire => TX_FAIL / TXBO
// while MKS drives look fine on the OLED (they stay at 500 kbit/s).
// Common blue MCP2515 boards are 8M OR 16M — check the can on the module.
// =====================================================

const CANBitrate::Config CAN_BITRATE =
    CANBitrate::Config_8MHz_500kbps;
const uint8_t CAN_XTAL_MHZ = 8;

const uint8_t CAN_CS  = 10;
const int8_t  CAN_INT = 2;
const uint32_t CAN_SPI_HZ = 8000000;

CANConfig config(
    CAN_BITRATE,
    CAN_CS,
    CAN_INT,
    SPI,
    CAN_SPI_HZ
);

CANController CAN(config);

uint32_t canTxFailCount = 0;
unsigned long lastCanRecoverMs = 0;

bool recoverCanBus(const char *reason)
{
    Serial.print("CAN_RECOVER ");
    Serial.println(reason ? reason : "");
    if (CAN.begin(CANController::Mode::Normal) != CANController::OK)
    {
        Serial.println("CAN_RECOVER FAIL");
        return false;
    }
    CAN.setFilters(false);
    canTxFailCount = 0;
    Serial.println("CAN_RECOVER OK");
    return true;
}


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

// Soft wrist estimates (deg). Host maps pitch keys to DIFF_A/D and roll to DIFF_W/S.
float j4Angle = 0.0;
float j5Angle = 0.0;
float pitchAngle = 0.0;
float rollAngle = 0.0;
int pitchDir = 0; // +1 DIFF_A, -1 DIFF_D, 0 none
int rollDir = 0;  // +1 DIFF_W, -1 DIFF_S, 0 none

// Pitch soft ROM: +/-75 deg each side of ZERO (enforced on DIFF_A/D).
const float PITCH_MIN = -75.0;
const float PITCH_MAX =  75.0;

const float J5_MIN = -270.0;
const float J5_MAX =  270.0;

// Startup / SETHOME encoder pose for HOME (F5 absolute).
bool haveHomePose = false;
int64_t homeEncJ1 = 0;
int64_t homeEncJ3 = 0;
int64_t homeEncJ4 = 0;
int64_t homeEncJ5 = 0;


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
// Manual V1.0.9 5.1.2: 31h cumulative multi-turn (0x4000 counts/rev).
// Reply DLC=8: 31h + int48_t + CHECKSUM8. Coordinates for F4/F5 use 31h.
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
    const uint8_t *data,
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
        canTxFailCount++;
        // Do not flood USB CDC - host must still see ENC lines.
        static unsigned long lastFailPrint = 0;
        unsigned long now = millis();
        if (now - lastFailPrint >= 1000)
        {
            lastFailPrint = now;
            Serial.print("CAN_TX_FAIL ID ");
            Serial.print(id);
            Serial.print(" fails=");
            Serial.print(canTxFailCount);
            Serial.print(" xtal=");
            Serial.print(CAN_XTAL_MHZ);
            Serial.println("M");
        }
        if (canTxFailCount > 30 && (now - lastCanRecoverMs) > 2000)
        {
            lastCanRecoverMs = now;
            recoverCanBus("tx_fail");
        }
    }
    else
    {
        canTxFailCount = 0;
    }
}


// =====================================================
// ENCODER REQUEST / PARSE (MKS V1.0.9 31h / 8Ch)
// =====================================================

// 31h = cumulative multi-turn (manual 5.1.2) - coordinate source for F4/F5.
// 35h = raw accumulated (5.1.5) - accepted if a drive answers it instead.
const uint8_t CMD_READ_ENCODER_CARRY = 0x30; // carry(int32) + value(uint16)
const uint8_t CMD_READ_ENCODER_ADD = 0x31;
const uint8_t CMD_READ_ENCODER_RAW = 0x35;
const uint8_t CMD_SET_RESPONSE = 0x8C;

bool canDump = false;
uint32_t canRxCount = 0;
uint32_t canEncOk = 0;

void requestEncoderCmd(uint16_t id, uint8_t code)
{
    uint8_t data[2];

    data[0] = code;
    data[1] = checksum(id, data, 1);

    sendMKS(id, data, 2);
}

void requestEncoder(uint16_t id)
{
    // Manual 5.1.2: 31h cumulative is the coordinate encoder. One request per tick.
    requestEncoderCmd(id, CMD_READ_ENCODER_ADD);
}

int64_t decodeEncoderCarry(const uint8_t *args)
{
    // Manual 5.1.1: carry int32 big-endian + value uint16 (0..0x3FFF).
    int32_t carry = (int32_t)(
        ((uint32_t)args[0] << 24) |
        ((uint32_t)args[1] << 16) |
        ((uint32_t)args[2] << 8) |
        (uint32_t)args[3]
    );
    uint16_t value = (uint16_t)(((uint16_t)args[4] << 8) | args[5]);
    return ((int64_t)carry * 0x4000LL) + (int64_t)value;
}

void setMotorResponse(uint16_t id, bool respond, bool active)
{
    // Manual 5.2.12 / menu CanRSP: XX=respond, YY=active. Force on so 31h replies.
    uint8_t data[4];
    data[0] = CMD_SET_RESPONSE;
    data[1] = respond ? 0x01 : 0x00;
    data[2] = active ? 0x01 : 0x00;
    data[3] = checksum(id, data, 3);
    sendMKS(id, data, 4);
}

void setMotorWorkMode(uint16_t id, uint8_t mode)
{
    // Manual 5.2.2: 05h = SR_vFOC (bus FOC). Needed for reliable bus control.
    uint8_t data[3];
    data[0] = 0x82;
    data[1] = mode;
    data[2] = checksum(id, data, 2);
    sendMKS(id, data, 3);
}

void enableMotorResponses()
{
    const uint16_t ids[4] = { J1, J3, J4, J5 };
    for (uint8_t i = 0; i < 4; i++)
    {
        setMotorResponse(ids[i], true, true);
        delay(2);
        setMotorWorkMode(ids[i], 0x05);
        delay(2);
        readCAN();
    }
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
    uint16_t sid = (uint16_t)(id & 0x7FF);
    if (sid == J1)
    {
        encJ1 = value;
        haveEncJ1 = true;
    }
    else if (sid == J3)
    {
        encJ3 = value;
        haveEncJ3 = true;
    }
    else if (sid == J4)
    {
        encJ4 = value;
        haveEncJ4 = true;
    }
    else if (sid == J5)
    {
        encJ5 = value;
        haveEncJ5 = true;
    }

    // First full ENC sample becomes startup home if none set yet.
    if (
        !haveHomePose &&
        haveEncJ1 && haveEncJ3 && haveEncJ4 && haveEncJ5
    )
    {
        homeEncJ1 = encJ1;
        homeEncJ3 = encJ3;
        homeEncJ4 = encJ4;
        homeEncJ5 = encJ5;
        haveHomePose = true;
        Serial.println("SETHOME AUTO");
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


uint8_t encRoundRobin = 0;

void printEncoderLine()
{
    static unsigned long encStartMs = 0;
    if (encStartMs == 0)
    {
        encStartMs = millis();
    }

    bool any = haveEncJ1 || haveEncJ3 || haveEncJ4 || haveEncJ5;
    bool all = haveEncJ1 && haveEncJ3 && haveEncJ4 && haveEncJ5;
    if (!any)
    {
        static unsigned long lastWait;
        unsigned long now = millis();
        if (now - lastWait > 2000)
        {
            lastWait = now;
            Serial.print("ENC_WAIT ");
            Serial.print(haveEncJ1 ? "J1" : "-");
            Serial.print(haveEncJ3 ? "J3" : "-");
            Serial.print(haveEncJ4 ? "J4" : "-");
            Serial.print(haveEncJ5 ? "J5" : "-");
            Serial.print(" rx=");
            Serial.print(canRxCount);
            Serial.print(" encok=");
            Serial.println(canEncOk);
        }
        // Do not publish ENC 0 0 0 0 - host would false-lock and RViz freezes.
        return;
    }

    if (!all)
    {
        static unsigned long lastPartial;
        unsigned long now = millis();
        if (now - encStartMs < 1500 && (now - lastPartial) > 500)
        {
            lastPartial = now;
            Serial.print("ENC_PARTIAL ");
            Serial.print(haveEncJ1 ? "J1" : "-");
            Serial.print(haveEncJ3 ? "J3" : "-");
            Serial.print(haveEncJ4 ? "J4" : "-");
            Serial.println(haveEncJ5 ? "J5" : "-");
        }
    }

    Serial.print("ENC ");
    printI64(haveEncJ1 ? encJ1 : 0);
    Serial.print(' ');
    printI64(haveEncJ3 ? encJ3 : 0);
    Serial.print(' ');
    printI64(haveEncJ4 ? encJ4 : 0);
    Serial.print(' ');
    printI64(haveEncJ5 ? encJ5 : 0);
    Serial.println();
}


void serviceEncoders()
{
    unsigned long now = millis();

    if (!encStream)
    {
        return;
    }

    // Do not keep hammering TX while bus-off - TEC never recovers.
    bool busDead = canTxFailCount > 20;

    // MCP2515 only has 2 RX buffers. One drive per tick + drain after TX.
    if (!busDead && now - lastEncRequest >= (ENC_PERIOD_MS / 4))
    {
        lastEncRequest = now;

        const uint16_t ids[4] = { J1, J3, J4, J5 };
        uint16_t id = ids[encRoundRobin & 3];
        encRoundRobin++;

        requestEncoder(id);
        delay(3);
        readCAN();
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
        const uint16_t sid = (uint16_t)(rx.getId() & 0x7FF);
        canRxCount++;

        if (canDump)
        {
            Serial.print("CANRX id=");
            Serial.print(sid);
            Serial.print(" dlc=");
            Serial.print(dlc);
            Serial.print(" ");
            for (uint8_t i = 0; i < dlc && i < 8; i++)
            {
                if (data[i] < 16)
                {
                    Serial.print('0');
                }
                Serial.print(data[i], HEX);
                Serial.print(' ');
            }
            Serial.println();
        }

        // Manual uplink: code + payload + CHECKSUM8. Some FW omit CRC (DLC 7).
        bool crcOk = true;
        if (dlc == 8)
        {
            crcOk = (data[7] == checksum(sid, data, 7));
        }

        if (
            dlc >= 7 &&
            crcOk &&
            (data[0] == CMD_READ_ENCODER_ADD || data[0] == CMD_READ_ENCODER_RAW)
        )
        {
            storeEncoder(sid, decodeEncoder48(&data[1]));
            canEncOk++;
        }
        else if (dlc >= 7 && crcOk && data[0] == CMD_READ_ENCODER_CARRY)
        {
            storeEncoder(sid, decodeEncoderCarry(&data[1]));
            canEncOk++;
        }
    }
}


// =====================================================
// ENABLE / DISABLE
// =====================================================

// Working current for SERVO42D (mA). Restored on ENABLE after coast.
uint16_t workCurrentMa = 1600;
// Soft coast / hand-follow current while DISABLE (F3 on so ENC lives).
// Ma=0 freezes ENC on these 42Ds. Default 250 per user (hand-easy).
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
    // Ensure drives answer 31h reads (8Ch may have been cleared on some boards).
    enableMotorResponses();

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
    // Soft hold: F3 on + coastCurrentMa so ENC tracks hand moves and
    // joints resist gravity. POWEROFF = true F3=0 (ENC may freeze).
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

    j4Angle += j4Direction * DEG_PER_SEC * dt;
    j5Angle += j5Direction * DEG_PER_SEC * dt;
    pitchAngle += pitchDir * DEG_PER_SEC * dt;
    rollAngle += rollDir * DEG_PER_SEC * dt;

    if (pitchAngle > PITCH_MAX)
    {
        pitchAngle = PITCH_MAX;
        if (pitchDir > 0)
        {
            stopDifferential();
            Serial.println("PITCH_LIMIT +75");
        }
    }
    else if (pitchAngle < PITCH_MIN)
    {
        pitchAngle = PITCH_MIN;
        if (pitchDir < 0)
        {
            stopDifferential();
            Serial.println("PITCH_LIMIT -75");
        }
    }
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
    // Host maps roll keys here (a/d). No pitch ROM gate.
    pitchDir = 0;
    rollDir = +1;
    moveJ4CW();
    delay(3);
    moveJ5CW();
}


void differentialS()
{
    pitchDir = 0;
    rollDir = -1;
    moveJ4CCW();
    delay(3);
    moveJ5CCW();
}


void differentialA()
{
    // Host maps pitch+ (W) -> DIFF_A. Enforce +/-75 here so motors stop.
    if (pitchAngle >= PITCH_MAX)
    {
        stopDifferential();
        Serial.println("PITCH_LIMIT +75");
        return;
    }
    pitchDir = +1;
    rollDir = 0;
    moveJ4CCW();
    delay(3);
    moveJ5CW();
}


void differentialD()
{
    // Host maps pitch- (S) -> DIFF_D.
    if (pitchAngle <= PITCH_MIN)
    {
        stopDifferential();
        Serial.println("PITCH_LIMIT -75");
        return;
    }
    pitchDir = -1;
    rollDir = 0;
    moveJ4CW();
    delay(3);
    moveJ5CCW();
}


void stopDifferential()
{
    pitchDir = 0;
    rollDir = 0;
    stopMotor(J4);
    delay(3);
    stopMotor(J5);
}


// =====================================================
// ABSOLUTE MOVE (F5) + HOME
// =====================================================

int32_t clampInt24(int64_t value)
{
    if (value > 8388607LL)
    {
        return 8388607L;
    }
    if (value < -8388607LL)
    {
        return -8388607L;
    }
    return (int32_t)value;
}


void moveAbsoluteAxis(uint16_t id, int64_t axisCounts, uint16_t speed, uint8_t acc)
{
    // Manual 11.x F5h: speed uint16, acc uint8, axis int24, CRC.
    int32_t axis = clampInt24(axisCounts);
    if (speed > 3000)
    {
        speed = 3000;
    }

    uint8_t data[8];
    data[0] = 0xF5;
    data[1] = (uint8_t)(speed >> 8);
    data[2] = (uint8_t)(speed & 0xFF);
    data[3] = acc;
    data[4] = (uint8_t)((axis >> 16) & 0xFF);
    data[5] = (uint8_t)((axis >> 8) & 0xFF);
    data[6] = (uint8_t)(axis & 0xFF);
    data[7] = checksum(id, data, 7);
    sendMKS(id, data, 8);
}


void captureHomePose()
{
    if (!(haveEncJ1 && haveEncJ3 && haveEncJ4 && haveEncJ5))
    {
        Serial.println("SETHOME FAIL need ENC");
        return;
    }
    homeEncJ1 = encJ1;
    homeEncJ3 = encJ3;
    homeEncJ4 = encJ4;
    homeEncJ5 = encJ5;
    haveHomePose = true;
    j4Angle = 0.0;
    j5Angle = 0.0;
    pitchAngle = 0.0;
    rollAngle = 0.0;
    Serial.println("SETHOME OK");
}


void goHomePose()
{
    if (!haveHomePose)
    {
        // Fall back to whatever ENC we first locked, else fail.
        if (haveEncJ1 && haveEncJ3 && haveEncJ4 && haveEncJ5)
        {
            captureHomePose();
        }
        else
        {
            Serial.println("HOME FAIL no pose");
            return;
        }
    }

    stopAll();
    delay(20);
    // Moderate return speed.
    moveAbsoluteAxis(J1, homeEncJ1, armSpeed, armAccel);
    delay(5);
    moveAbsoluteAxis(J3, homeEncJ3, armSpeed, armAccel);
    delay(5);
    moveAbsoluteAxis(J4, homeEncJ4, wristSpeed, wristAccel);
    delay(5);
    moveAbsoluteAxis(J5, homeEncJ5, wristSpeed, wristAccel);

    j4Angle = 0.0;
    j5Angle = 0.0;
    pitchAngle = 0.0;
    rollAngle = 0.0;
    Serial.println("HOME");
}


// =====================================================
// WRIST ZERO / POSITION
// =====================================================

void zeroDifferential()
{
    stopDifferential();

    j4Angle = 0.0;
    j5Angle = 0.0;
    pitchAngle = 0.0;
    rollAngle = 0.0;

    Serial.println("WRIST CENTER SET TO 0 DEG");
}


void printAngles()
{
    Serial.print("J4 = ");
    Serial.print(j4Angle, 1);
    Serial.print(" deg");

    Serial.print("    J5 = ");
    Serial.print(j5Angle, 1);
    Serial.print(" deg");

    Serial.print("    pitch = ");
    Serial.print(pitchAngle, 1);
    Serial.print(" deg");

    Serial.print("    roll = ");
    Serial.print(rollAngle, 1);
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

    else if (cmd == "SETHOME")
    {
        captureHomePose();
    }

    else if (cmd == "HOME")
    {
        goHomePose();
    }

    else if (cmd == "POS")
    {
        printAngles();
    }

    else if (cmd == "ENC")
    {
        // One ID at a time + drain (MCP only has 2 RX buffers).
        const uint16_t ids[4] = { J1, J3, J4, J5 };
        for (uint8_t i = 0; i < 4; i++)
        {
            requestEncoder(ids[i]);
            delay(3);
            readCAN();
        }
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

    else if (cmd == "CANDUMP")
    {
        canDump = true;
        Serial.println("CANDUMP=1");
    }

    else if (cmd == "CANDUMPOFF")
    {
        canDump = false;
        Serial.println("CANDUMP=0");
    }

    else if (cmd == "CANSTAT")
    {
        Serial.print("CANSTAT rx=");
        Serial.print(canRxCount);
        Serial.print(" encok=");
        Serial.print(canEncOk);
        Serial.print(" txfail=");
        Serial.print(canTxFailCount);
        Serial.print(" xtal=");
        Serial.print(CAN_XTAL_MHZ);
        Serial.print("M have=");
        Serial.print(haveEncJ1 ? "J1" : "-");
        Serial.print(haveEncJ3 ? "J3" : "-");
        Serial.print(haveEncJ4 ? "J4" : "-");
        Serial.println(haveEncJ5 ? "J5" : "-");
        CAN.getErrors().print(Serial);
    }

    else if (cmd == "CANRECOVER")
    {
        recoverCanBus("cmd");
        enableMotorResponses();
    }

    else if (cmd.startsWith("COASTMA "))
    {
        int value = cmd.substring(8).toInt();
        if (value >= 0 && value <= 3000)
        {
            coastCurrentMa = (uint16_t)value;
            Serial.print("COAST_MA=");
            Serial.println(coastCurrentMa);
        }
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
    Serial.println("HAND-TRACK J1+J3+J4+J5");
    Serial.print("MCP XTAL CONFIG=");
    Serial.print(CAN_XTAL_MHZ);
    Serial.println("MHz @ 500kbps");
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
        // MCP2515 reset leaves RX masks/filters on; default IDs drop motor replies.
        // Accept-all so 31h uplink frames (IDs 1/3/4/5) reach the host.
        if (!CAN.setFilters(false))
        {
            Serial.println("CAN FILTERS FAIL");
        }
        else
        {
            Serial.println("CAN FILTERS OFF (accept all)");
        }
        Serial.println("CAN INIT OK");

        enableMotorResponses();
        Serial.println("MKS 8Ch RESPOND ON");
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
