# CAN bench firmware (UNO R4 + TJA1051T + MKS SERVO42D/57D)

This folder holds the Arduino firmware for bringing up the arm's MKS drives on a real classic
CAN bus. The shared MKS subset is ported from `waybionic_teleop/mks_can.py` (merged through
PR #28) and checked against it byte for byte. The host build in `../CMakeLists.txt` compiles
and unit-tests the same library files.

For the supervised session with all five arm drives, use the
[five-drive hardware checklist](../docs/HARDWARE_BRINGUP_CHECKLIST.md).

| Sketch | Board role | Moves anything? |
| --- | --- | --- |
| `single_drive_bringup/` | Serial console for the first SERVO42D test: read, enable, small move, stop. Logs every frame. | Only when you type `enable` and then `move` |
| `four_node_bench/gateway/` | Arduino 1 of the four-node bench: the laptop types commands, the board puts them on the bus and tracks which receiver answered | Only when typed |
| `four_node_bench/receiver/` | Arduinos 2-4: answer MKS frames like a drive, with a software-only actuator | No (software only) |
| `carrier_bridge/` | Laptop USB serial ↔ CAN bridge speaking a subset of SLCAN, so python-can can use it | Forwards whatever the host sends |
| `libraries/WaybionicCan/` | Shared code: MKS framing, receiver/gateway logic, console, SLCAN, UNO R4 CAN transport | - |

## Safety rules (read before powering a drive)

- **Powered motion only with Yassin or Mujtaba present.**
- The software `stop` and `disable` commands **do not replace the physical emergency stop**. Keep the E-stop within reach and the 24 V supply current-limited.
- No sketch enables or moves a motor at boot, and none of them runs a motion loop. Every motion frame is sent because someone typed a command, and the console prints it as a `MOTION ...` line before the `TX` line.
- `single_drive_bringup` and the bench gateway refuse a `move` unless:
  - the drive confirmed `enable` (an F3h reply with status 1) during this session;
  - a 31h encoder reading has arrived since the previous move and within the last 10 s;
  - the target is within one motor turn (0x4000 counts) of that reading;
  - speed is 1-60 rpm and acc is 1-128.
  These are bench limits on top of the `mks_can.py` protocol limits (signed 24-bit axis
  -0x800000..0x7FFFFF, 0-3000 rpm, acc 0-255). They are constants in the sketch
  (`benchPolicy()` / `MotionPolicy`). Change them only with the leads, then re-upload.
- The human-readable bench sketches stay at the drive's factory **500 kbit/s**. The
  `feature/real-arm` carrier/host selects **1 Mbit/s CAN** after each drive is configured.
- `F7h` emergency stop, drive-ID (8Bh), and bitrate (8Ah) commands are owned by the real-arm
  Python host. The bench console's `estop` deliberately still sends nothing.

## Hardware topology

```text
laptop (USB serial: 1000000 baud for carrier_bridge, 115200 for the consoles)
   |
UNO R4 WiFi -- CAN TX = D10 --> TJA1051T TXD
            <- CAN RX = D13 --- TJA1051T RXD
                                    |
              120 Ω    CAN_H ===========================  120 Ω
            (bus end)  CAN_L ===========================  (bus end)
                                    |
                           MKS SERVO42D (one drive first)
                           24 V supply, current limited
```

- **The UNO R4 has a CAN controller built in.** The TJA1051T is only a **transceiver**: it turns the controller's TX/RX logic signals into the differential CAN_H/CAN_L pair. No MCP2515 is used.
- **Pins:** on the **UNO R4 WiFi**, CAN TX = **D10** and CAN RX = **D13**, as in the official core (`variants/UNOWIFIR4/pins_arduino.h`: `PIN_CAN0_TX 10`, `PIN_CAN0_RX 13`). The **UNO R4 Minima** uses different pins, D4 (TX) and D5 (RX), per `variants/MINIMA/pins_arduino.h`. The sketches use the core's `CAN` object, so they pick the right pins for whichever board you compile for, and they print those pins at boot.
- **Termination:** one 120 Ω resistor across CAN_H/CAN_L at **each of the two physical ends** of the bus, and nowhere else. With power off, CAN_H to CAN_L should measure about 60 Ω.
- **Bus wiring:** CAN_H to CAN_H and CAN_L to CAN_L on every node.
- **Not yet specified by Electrical (TBD, ask Yassin):** how the TJA1051T module is powered, how its mode pin is strapped (it must be in normal mode, not silent), the ground reference between the Arduino, the transceiver and the drive, and the connector pinout on the drive side. This README doesn't guess at them.

## Bitrate

| Where | Bitrate | Status |
| --- | --- | --- |
| MKS drives as shipped | 500 kbit/s | Factory default |
| Bench sketches (`kCanBitrate`) | 500 kbit/s | Match drives as shipped |
| `feature/real-arm` host through `carrier_bridge` | 1 Mbit/s | Target for the five arm drives |
| Carrier USB serial | 1,000,000 baud | Separate serial-link setting, not CAN bitrate |

Configure each drive alone before using the real-arm host. The carrier keeps the CAN rate at
which it first opened until reset, so replug it before checking the shared 1 Mbit/s bus:

```bash
./scripts/macos.sh drives set-bitrate 3 1000000
# after every drive is switched, replug the carrier
./scripts/macos.sh drives --bitrate 1000000 scan --ids 1 2 3 4 5
```

These commands require `feature/real-arm` (or a later integration containing its macOS helper
and `mks_setup`). If the carrier and a drive disagree on CAN bitrate, no frames are ACKed.

## Checksum vs CRC

There are two separate checks:

- **MKS application checksum:** the last data byte, `(CAN_ID + sum(data bytes before it)) & 0xFF`. The sketches compute it for every frame they build and check it on every frame they receive (`RX ... MKS checksum BAD (got .., expected ..)`). A drive silently ignores frames with a bad checksum.
- **CAN CRC, ACK and error handling:** done by the CAN controller hardware. They're reported as `CAN controller error event <n>` (for example no ACK, bus-off or a wrong bitrate) and have nothing to do with the MKS checksum.

## Install, compile and upload

The Arduino CLI (or IDE 2) needs the official UNO R4 core, which includes the `Arduino_CAN` library:

```powershell
arduino-cli core update-index
arduino-cli core install arduino:renesas_uno
cd waybionic_can/arduino
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi --libraries libraries single_drive_bringup
arduino-cli upload  --fqbn arduino:renesas_uno:unor4wifi -p COM5 single_drive_bringup
arduino-cli monitor -p COM5 -c baudrate=115200
```

Use `arduino:renesas_uno:minima` for a Minima. Swap `single_drive_bringup` for `carrier_bridge`, `four_node_bench/gateway` or `four_node_bench/receiver` as needed. On Linux the port is usually `/dev/ttyACM0`.

**Arduino IDE 2:** set *File > Preferences > Sketchbook location* to this `waybionic_can/arduino` folder, so the IDE finds `libraries/WaybionicCan`. Then open a sketch, choose the board and upload. Alternatively, copy `libraries/WaybionicCan` into your own sketchbook's `libraries/` folder.

All four sketches have been compiled successfully in Arduino IDE for **Arduino UNO R4 WiFi**. Nothing has been uploaded to a board yet, and no powered motor test has been run. Powered testing still needs Yassin or Mujtaba present.

## First drive: step-by-step

Do this with one SERVO42D on the bus and Yassin or Mujtaba present.

1. **Unpowered checks:** termination measures about 60 Ω across CAN_H/CAN_L, and wiring is as above. Upload `single_drive_bringup` and open the monitor. The banner must say `CAN started, 500000 bit/s ... TX pin D10, RX pin D13` (D4/D5 on a Minima). Nothing is sent yet.
2. **Power the drive** from the current-limited 24 V supply. Find its CAN ID from the drive's own configuration. The factory ID is not documented in this repo, and the placeholders 1-6 in `arm_drives.yaml` aren't confirmed hardware values.
3. **Read the encoder first (31h):**
   ```text
   read 1
   TX 001#3132 t=... READ_ENCODER request
   RX 001#31xxxxxxxxxxxxCC t=... READ_ENCODER value=<counts> counts (0x4000 per turn)
   ```
   No reply means a wrong ID, wrong bitrate, wiring, termination or drive power problem. Look for `CAN controller error event` lines. Don't continue until `read` works.
4. **Start-up sequence, in the merged simulator's order.** `sim_arm_drives_node.py` sends 82h,
   then 8Ch, then F3h, then 98h to each drive. Type them in the same order. 82h, 8Ch and 98h
   write drive settings, so agree them with the leads first.
   | Step | Command | Frame for ID 1 | Expected reply |
   | --- | --- | --- | --- |
   | 82h set mode SR_vFOC | `mode 1` | `001#820588` | `RX 001#820184 ... SET_MODE status=1 (ok)` |
   | 8Ch replies on (respond, active) | `response 1 1 1` | `001#8C01018F` | `RX 001#8C018E ... SET_RESPONSE status=1 (ok)` |
   | F3h enable | `enable 1` | `001#F301F5` | `RX 001#F301F5 ... ENABLE status=1 (ok)`; the shaft is now held |
   | 98h heartbeat | `heartbeat 1 0` (see below) | `001#980000000099` | `RX 001#98019A ... SET_HEARTBEAT status=1 (ok)` |

   The unit test `ReadmeStartupSequenceBytes` checks these bytes. `move` stays locked until the F3h reply arrives, and the drive only sends that reply once 8Ch has turned replies on.

   **Heartbeat:** `arm_drives.yaml` sets `heartbeat_ms: 500` because the host talks to every
   drive at 120 Hz. In `sim_drives.py`, a moving drive stops once nothing has been addressed to
   it for longer than the heartbeat. This console sends nothing on its own, so with a 500 ms
   heartbeat a typed move would stop about 500 ms after F5h. For first console moves, send
   `heartbeat 1 0` (off). Test the real 500 ms behavior separately under the supervised
   checklist; it has not been verified on a real drive.
5. Check `drives`: drive 1 should show `enabled=confirmed`.
6. **Read again**, then make a **small, slow absolute move (F5h)** relative to the reading. For example, if `read` gave `value=0`:
   ```text
   move 1 4096 30 2          (a quarter motor turn at 30 rpm, gentle ramp)
   MOTION F5h ABSOLUTE_AXIS id=1 axis=4096 rpm=30 acc=2 (from 0, delta 4096 counts)
   TX 001#F5001E0200100026 t=... ABSOLUTE_AXIS
   RX 001#F501F7 ... ABSOLUTE_AXIS status=1 (running)
   RX 001#F502F8 ... ABSOLUTE_AXIS status=2 (run complete)
   ```
   Then `read 1` again and compare: the simulator assumes the 31h value and F5h axis share one
   coordinate (0x4000 per turn). The hardware session must confirm that assumption.
7. **Stop:** `stop 1` sends F5h with speed 0 (`001#F5000000000000F6`), which the manual and `sim_drives.py` define as a stop (acc 0 = at once; `stop 1 4` gives the manual vector `001#F5000004000000FA`). `disable 1` (`001#F300F4`) releases the shaft. **`estop` does nothing yet** because F7h is undefined. Once the F7h bytes are confirmed from the manual, test them with `raw`. You can use `ck 001#F7...` to append the checksum before sending.
8. **Save the log** (next section) and compare every byte.

Command reference (`help` prints it):

| Command | Frame | Notes |
| --- | --- | --- |
| `read <id>` | 31h | Do this first, and before every move |
| `enable <id>` / `disable <id>` | F3h 01 / 00 | `move` needs the enable reply |
| `move <id> <axis> <rpm> <acc>` | F5h | All four values are required; bench limits apply |
| `stop <id> [acc]` | F5h, speed 0 | Always allowed |
| `estop <id>` | - | TODO: F7h is undefined, so nothing is sent |
| `mode <id>` | 82h 05 | SR_vFOC only; writes a setting |
| `response <id> <0/1> <0/1>` | 8Ch | Writes a setting |
| `heartbeat <id> <ms>` | 98h | 0 = off; writes a setting |
| `raw <ID#HEX>` | as typed | No checksum added; F3h/F5h are refused; for byte checks |
| `ck <ID#HEX>` | - | Prints the frame with its MKS checksum; nothing is sent |
| `status`, `drives`, `policy`, `help` | - | Status also shows the E-stop and supply placeholders, the bus load and the error counters |

IDs must be 1-2047; 0 (broadcast) is refused. Numbers can be decimal or `0x` hex.

## Frame logs

Every line that starts with `TX` or `RX` is a frame in candump notation (`ID#DATA`) with a millisecond timestamp. To save a session:

```powershell
arduino-cli monitor -p COM5 -c baudrate=115200 | Tee-Object -FilePath frames_servo42d_$(Get-Date -Format yyyyMMdd_HHmm).log
```

(PuTTY's session logging or the IDE's serial monitor output work too.) For the byte-by-byte comparison:

- Each command's expected bytes are the merged vectors in
  `waybionic_teleop/test/test_mks_can.py`. `read 1` = `001#3132`, `enable 1` =
  `001#F301F5`, `mode 1` = `001#820588`, and `move 1 16384 600 2` =
  `001#F502580200400092` (600 rpm is above the bench limit).
- `ck` recomputes the checksum of any frame.
- Replies follow the same layout: code, arguments, checksum. Their meaning (status 1 = ok; F5h 0/1/2/3 = failed/running/complete/end limit) comes from `mks_can.py`/`sim_drives.py`. Anything in a log that doesn't match is worth raising with the leads.

## Four-node bench

```text
laptop --USB--> gateway (Arduino 1) --CAN--> receiver 1, receiver 2, receiver 3
```

This is the hardware version of last week's vcan simulation (`../sim/`), using the same `NodeLogic`/`GatewayLogic` code. Each board needs its own transceiver on the shared CAN_H/CAN_L pair, with termination at the two physical ends. The repo doesn't document which boards were used last week. Any UNO R4 WiFi or Minima with a transceiver will run these sketches.

1. Set `kNodeId` in `receiver.ino` to 1, 2 and 3, and upload one to each receiver.
2. Upload `gateway.ino` to the board on the laptop and open its monitor.
3. Try `read 2`. Every receiver logs the frame, but only node 2 replies, and receivers 1 and 3 log `ignored (addressed to 2)`. `status` shows ONLINE/OFFLINE per node, and `poll 1000` polls the encoders (31h only).

The receivers act like drives at IDs 1-3. **Never put a receiver on a bus with a real drive that uses the same ID**: both would answer.

**Receivers don't drive a servo yet.** Last week's task asks each receiver to move a hobby servo. These sketches use `SoftwareActuator`, which only logs `actuator enabled=.. moving=.. target=.. position=..`. A servo `IActuator` needs two things nobody has decided: the servo signal pin on each receiver, and how MKS axis counts map to servo angles (and what 31h then reports). Agree both with Yassin before adding one.

### Bench tests to log

Save the serial output of all four boards for each test, one `Tee-Object` monitor per COM port (see [Frame logs](#frame-logs)). The unit tests in `../tests/unit/test_gateway_logic.cpp` cover the same cases on an in-memory bus. None of them has been run on hardware yet.

| Test | Gateway commands | Expected result | Unit test |
| --- | --- | --- | --- |
| Addressed move | `mode 2`, `response 2 1 1`, `enable 2`, `read 2`, `move 2 4096 30 2` | Node 2 logs `running`, then `run complete`. Nodes 1 and 3 log `ignored (addressed to 2)` for every frame. | `OnlyTheAddressedReceiverActsAndReplies`, `MoveIsConfirmedByTheReplyAndTheCompletionReport` |
| Bad checksum | `raw 002#3134` (the valid frame is `002#3133`) | Node 2 logs `rejected: bad checksum` and sends no reply. The gateway reports node 2 `OFFLINE` once the 500 ms reply timeout passes. | `BadChecksumGetsNoReplyAndNodeStaysUnknown` |
| Unplugged receiver | Unplug node 3's CAN_H/CAN_L (or its power), then `read 3` | The gateway logs `node 3 OFFLINE` after 500 ms. The other nodes keep answering. After replugging and a fresh `read 3`, node 3 comes back `ONLINE` but must be configured again. | `SilentReceiverGoesOfflineExactlyAfterTheTimeout`, `RestartedReceiverMustBeConfiguredAgain` |
| Stop during a move | Set `kMoveMs` to a few seconds on node 2, start `move 2 4096 30 2`, then type `stop 2` before it finishes | Node 2 replies `status=1` and logs `moving=0`, with no `run complete`. Another node's move carries on. | `SpeedZeroStopHaltsOnlyTheAddressedMove` |

The stop here is F5h with speed 0. F7h is not implemented (see [Protocol status](#protocol-status)).

## Carrier bridge (SLCAN subset)

`carrier_bridge` turns the UNO R4 into the ground station's USB-to-CAN adapter. The current
real-arm host opens it at 1,000,000 baud and selects the 1 Mbit/s CAN rate separately.
python-can's standard `slcan` interface can also open it:

```python
import can
bus = can.Bus(interface="slcan", channel="/dev/cu.usbmodem1101",
              bitrate=1000000, tty_baudrate=1000000)
bus.send(can.Message(arbitration_id=0x001, data=bytes.fromhex("3132"), is_extended_id=False))
print(bus.recv(timeout=1.0))   # encoder reply, or host-only carrier status ID 0x7F0
bus.shutdown()
```

Only part of SLCAN is supported:

| Command | Behaviour |
| --- | --- |
| `S4` `S5` `S6` `S8` | 125k / 250k / 500k / 1M, only while closed. Other `Sn` codes answer BELL (the UNO R4 API only offers these four). |
| `O` | Start CAN (500 kbit/s unless `Sn` was sent) and forward frames both ways |
| `C` | Stop forwarding. The controller itself stays started, so changing bitrate after the first `O` needs a board reset (`Arduino_CAN`'s `begin()` re-registers interrupts and a restart isn't known to be safe). |
| `tIIILDD..` | Send a standard data frame; no reply on success, BELL if refused |
| `V`, `N` | `V0100`, `NWB01` |
| empty line | OK (python-can sends one after `Sn`) |
| received frames | sent to the host as `tIIILDD..\r` while open |
| `L`, `s`, `T`, `r`, `R`, `F`, `Z`, `d`/`D`/`b`/`B`, anything else | BELL (not supported: no listen-only mode or remote frames in `Arduino_CAN`, no extended IDs or CAN FD, no status flags or timestamps) |

While open, the bridge sends a host-only status frame from standard ID `0x7F0` ten times per
second. It never transmits that frame on CAN. Its eight bytes are sequence, flags, supply mV,
CAN error-event count, failed writes, and refused serial lines; the exact layout is documented
in `SlcanBridge.h`. The flags distinguish an unwired/unknown E-stop or supply input from a
released E-stop or a measured voltage.

The bridge does **not** check MKS checksums or apply motion limits; the host software is
responsible for that. Successful transmitted lines are silent because the real-arm host batches
a control tick and treats non-frame lines as an end-of-read marker. It has been tested through
`slcan_sim` and python-can; it has **not** been tested on the UNO R4/TJA1051T or real drives.

**E-stop and supply telemetry:** `BenchTelemetry.h` has the interface, but currently returns
unknown for both. Electrical has not specified the input pins, active polarity, isolation,
voltage-divider values, ADC reference/scaling, or valid voltage limits. The physical E-stop
must remain effective with the carrier absent or failed.

## Adding drives one at a time

1. Bring up each drive **alone** because all five ship as ID 1.
2. Using the `feature/real-arm` `mks_setup` command, assign base 1, shoulder 2, elbow 3,
   wrist-left/pitch 4, and wrist-right/roll 5; then set that isolated drive to 1 Mbit/s.
3. Add it to the shared bus with power off. Move the termination so the 120 Ω resistors stay
   at the two physical ends.
4. Replug the carrier and scan IDs 1-5 at 1 Mbit/s. Every ID must produce exactly one `31h`
   encoder reply. Missing or duplicate replies block motion testing.
5. **Record bus load and errors.** The bench console's `status` command demonstrates the
   package-local estimate and counters:
   ```text
   bus load last 1000 ms: 12 frames, 0.2-0.3 % of 500000 bit/s; peak 0.4 %; 345 frames total
   bus errors: 0 controller error events, 0 failed writes (last code 0), 0 extended frames dropped
   ```
   The load is an estimate from the frames this board sent and received over the last 1 s window. The range runs from no bit stuffing to worst-case bit stuffing, for classic frames with 11-bit IDs. The board can't see error frames or retransmissions, so treat the figure as a lower bound. With only typed commands the load stays near zero; the gateway's `poll <ms>` gives a steady 31h load to measure. *Controller error events* counts every `Arduino_CAN` error event since boot, including the ones the once-per-second log line folds together. Paste both lines into the session log.
6. Only then follow the [supervised checklist](../docs/HARDWARE_BRINGUP_CHECKLIST.md) for
   zeroing, one-joint motion, E-stop, heartbeat, and evidence capture.

## Protocol status

| Command | Status | Source |
| --- | --- | --- |
| 31h READ_ENCODER + 6-byte signed reply | Implemented and verified against merged vectors and an ~81k-case cross-check with `mks_can.py` | `mks_can.py`, `test_mks_can.py` |
| 82h SET_MODE (05h SR_vFOC) | Implemented and verified | same |
| 8Ch SET_RESPONSE | Implemented and verified | same |
| 98h SET_HEARTBEAT | Implemented and verified | same |
| F3h ENABLE | Implemented and verified | same |
| F5h ABSOLUTE_AXIS (speed 0 = stop) | Implemented and verified | same, plus `sim_drives.py` for the speed-0 stop |
| **F7h emergency stop** | Intentionally absent from the bench console; implemented by `feature/real-arm` host | MKS CAN manual V1.0.9 via PR #31 stack |
| **Set CAN ID (8Bh)** | Host-owned; implemented by `feature/real-arm` `mks_setup` | same |
| **Set CAN bitrate (8Ah)** | Host-owned; implemented by `feature/real-arm` `mks_setup` | same |
| Broadcast ID 0, other MKS commands | Not implemented | - |

"Verified" means checked against the repo's reference implementation and its manual vectors. The real-hardware check is the frame log from the bench session.

## Verification status

| What | Status |
| --- | --- |
| Shared library logic (framing, receiver, gateway, console, SLCAN, bus load) | Unit-tested on the host with gtest |
| Frame bytes vs merged `mks_can.py` | Cross-check command documented; rerun results belong in the PR/test report |
| SLCAN bridge vs python-can's `slcan` driver | Tested against `../sim/slcan_sim` on a pseudo-terminal |
| Sketches compiled for UNO R4 WiFi (`arduino:renesas_uno:unor4wifi`) | **Pass.** All four sketches compile in Arduino IDE: `single_drive_bringup`, `four_node_bench/gateway`, `four_node_bench/receiver` and `carrier_bridge`. |
| Upload to a board in this assignment | **Not performed.** No hardware was flashed in this software session. |
| Any acceptance test on the UNO R4, TJA1051T or MKS drives | **Not performed.** No new bench logs, measurements, photos or video were produced. |
| Frame bytes vs the MKS manual | **Not directly in this package.** The manual is not committed; merged host vectors are checked. |

## TODO

- Confirm the physical labels match IDs 1-5 and the wrist-left/right mapping with Yassin and
  Mujtaba.
- Run and record the 1 Mbit/s five-drive session; factory drives still begin at 500 kbit/s.
- E-stop sensing: Electrical must provide a dry/isolated sense source, UNO pin, polarity,
  pull-up/down behavior, and fail/disconnect semantics.
- Motor-supply sensing: Electrical must provide pin, divider/isolation, ADC reference,
  scaling/calibration, safe maximum input, and agreed warning/fault limits.
- Malik must confirm that the existing `0x7F0` status bytes and unknown/stale handling remain
  the ROS-side contract before hardware sensing is added.
- Transceiver power, mode pin, ground reference and connector pinout (Yassin).
- Upload the sketches and run the first powered bench with Yassin or Mujtaba present; save the frame logs, wiring photos and a short demo video.
- A hobby-servo actuator for the bench receivers: signal pin and axis-to-angle mapping (Yassin).
- Photos of the wiring for this README, and the bench video.
