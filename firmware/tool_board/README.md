# Tool Board Firmware

Firmware for the biomedical tool board. The board drives four NEMA 8 steppers
(8HS11-0204S) through A4988 drivers and takes its commands over the arm's CAN bus.
Each motor answers on its own CAN ID the way an MKS SERVO drive does. The ground
station can therefore drive the tool with the same host code and frames it uses
for the arm's joints, defined in `waybionic_teleop/waybionic_teleop/mks_can.py`.

## Hardware

The hardware below is assumed, so check it before flashing:

- An Arduino UNO R4 WiFi.
- A CAN transceiver (TJA1051T or similar) on D10 (TX) and D13 (RX).
- A4988 drivers wired like a CNC Shield V3. The fourth motor's direction pin is on
  D11 instead of D13, because D13 is the CAN receive pin.

Every setting is at the top of `tool_board.ino`:

| Motor | CAN ID | Step | Direction |
| --- | --- | --- | --- |
| 1 (X) | 7 | D2 | D5 |
| 2 (Y) | 8 | D3 | D6 |
| 3 (Z) | 9 | D4 | D7 |
| 4 (A) | 10 | D12 | D11 |

- **Enable:** D8, active low, shared by all four drivers.
- **Microstepping:** 1/16 (3200 steps per turn).
- **CAN bitrate:** 1 Mbit/s.
- **UNO R4 Minima:** its CAN pins are D4 and D5, so the sketch refuses to compile
  for it until the pins are moved.

**Travel limits.** The limits are placeholders: one turn either side of zero. The
cables limit how far each motor may turn, so take the real limits from the
Biomedical team's `nema8_2pair_LimitedRotationV3.ino` before driving the tool. A
target past a limit stops at the limit, and the move reports status 3 (stopped at
end limit).

## Commands

The board answers the MKS commands that the host sends. Positions are in MKS encoder
counts, 16384 per motor turn.

| Code | Command | On this board |
| --- | --- | --- |
| 31h | Read encoder | Reports the step count, because the motors have no encoders |
| 82h | Set work mode | Moves need mode 5 (SR_vFOC), as on the drives |
| 8Ch | Replies and completion reports | Turns each one on or off |
| 92h | Set zero | Makes the current position zero |
| 98h | Heartbeat | Stops a moving motor if no frame arrives in time |
| F3h | Enable | Enabling any motor powers all four drivers |
| F5h | Absolute move | Speed 0 stops the motor. Speed is capped at 60 rpm and acceleration at 20000 steps/s² |

The motors have no encoders, so the reported position is the commanded one. A
stalled motor goes unnoticed until it is zeroed again.

## Build and Flash

**Arduino IDE:**

1. Install **Arduino UNO R4 Boards** from the Boards Manager.
2. Install **AccelStepper** from the Library Manager.
3. Open `tool_board.ino`, select **Arduino UNO R4 WiFi**, and upload.

**arduino-cli:**

```bash
arduino-cli core install arduino:renesas_uno
arduino-cli lib install AccelStepper
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi --upload --port /dev/ttyACM0 firmware/tool_board
```

At 115200 baud, the serial monitor prints `Tool board ready on CAN IDs 7 to 10`.

## Bench Check

Use a SocketCAN adapter at 1 Mbit/s and run `candump can0` in a second terminal.
Each command below should print the reply shown:

```bash
cansend can0 007#3138              # read motor 1: 007#3100000000000038
cansend can0 007#82058E            # bus mode: 007#82018A
cansend can0 007#F301FB            # enable: 007#F301FB
cansend can0 007#F5001EC800200002  # half a turn at 30 rpm: 007#F501FD, then 007#F502FE
cansend can0 007#9299              # zero here: 007#92019A
```

The `waybionic_teleop` tests check this firmware's frame code against `mks_can.py`. They
also run the motor logic in `tool_motor.h` with a fake stepper driver: moves, travel limits,
the heartbeat stop, stops, enable and zero.

## Using It from the Ground Station

The clamp model defines the tool's joints. Once it exists, add the four motors to
`waybionic_teleop/config/arm_drives.yaml` as drives with CAN IDs 7 to 10. Polling
all ten drives at 120 Hz uses about half of the 1 Mbit/s bus.
