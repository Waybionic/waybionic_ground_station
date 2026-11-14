# CAN bench firmware (UNO R4 + MKS SERVO42D/57D)

This folder holds the Arduino firmware for bringing up the arm's MKS drives on a real classic
CAN bus. The shared MKS subset is ported from `waybionic_teleop/mks_can.py` (merged through
PR #28) and checked against it byte for byte. The host build in `../CMakeLists.txt` compiles
and unit-tests the same library files.

There are **two different carrier topologies**: the R4's built-in CAN controller with a
TJA1051T transceiver, and the arm's external [MCP2515 SPI module](#mcp2515-carrier).
Select the matching sketch before flashing; identical host protocols do not make the
hardware interchangeable.

For a supervised built-in-CAN session with all five arm drives, use the
[five-drive hardware checklist](../docs/HARDWARE_BRINGUP_CHECKLIST.md).

| Sketch | Board role | Moves anything? |
| --- | --- | --- |
| `single_drive_bringup/` | Serial console for the first SERVO42D test: read, enable, small move, stop. Logs every frame. | Only when you type `enable` and then `move` |
| `four_node_bench/gateway/` | Arduino 1 of the four-node bench: the laptop types commands, the board puts them on the bus and tracks which receiver answered | Only when typed |
| `four_node_bench/receiver/` | Arduinos 2-4: answer MKS frames like a drive, with a software-only actuator | No (software only) |
| `carrier_bridge/` | Laptop USB serial ↔ CAN bridge speaking a subset of SLCAN, so python-can can use it | Forwards whatever the host sends |
| `carrier_bridge_mcp2515/` | Same host protocol, using the external MCP2515 SPI module rather than the R4's built-in CAN controller | Forwards whatever the host sends |
| `libraries/WaybionicCan/` | Shared code: MKS framing, receiver/gateway logic, console, SLCAN, built-in and MCP2515 CAN transports | - |

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
- The human-readable bench sketches stay at the drive's factory **500 kbit/s**. The real-arm
  built-in-CAN configuration targets **1 Mbit/s CAN** after each drive is configured.
  The MCP2515 arm configuration stays at **500 kbit/s**.
- `estop <id>` sends the manual's F7h immediate emergency-stop command. It is a software
  command and does not replace the physical E-stop. Drive-ID (8Bh) and bitrate (8Ah) are not
  implemented in the merged `mks_can.py` or this shared library.

## Built-in CAN hardware topology

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

- **The UNO R4 has a CAN controller built in.** The TJA1051T is only a **transceiver**: it turns the controller's TX/RX logic signals into the differential CAN_H/CAN_L pair. No MCP2515 is used in this variant.
- **Pins:** on the **UNO R4 WiFi**, CAN TX = **D10** and CAN RX = **D13**, as in the official core (`variants/UNOWIFIR4/pins_arduino.h`: `PIN_CAN0_TX 10`, `PIN_CAN0_RX 13`). The **UNO R4 Minima** uses different pins, D4 (TX) and D5 (RX), per `variants/MINIMA/pins_arduino.h`. The sketches use the core's `CAN` object, so they pick the right pins for whichever board you compile for, and they print those pins at boot.
- **Termination:** one 120 Ω resistor across CAN_H/CAN_L at **each of the two physical ends** of the bus, and nowhere else. With power off, CAN_H to CAN_L should measure about 60 Ω.
- **Bus wiring:** CAN_H to CAN_H and CAN_L to CAN_L on every node.
- **Not yet specified by Electrical (TBD, ask Yassin):** how the TJA1051T module is powered, how its mode pin is strapped (it must be in normal mode, not silent), the ground reference between the Arduino, the transceiver and the drive, and the connector pinout on the drive side. This README doesn't guess at them.

## MCP2515 carrier

The arm's carrier uses `carrier_bridge_mcp2515`: UNO R4 WiFi hardware SPI on D11
(MOSI), D12 (MISO), D13 (SCK), with CS on D10 and INT on D2. The configured module
oscillator is 8 MHz and CAN runs at 500 kbit/s. D13 is **SPI clock in this topology**,
not a transceiver RXD pin. A pin-level diagnosis for the built-in-CAN topology does
not apply to this carrier.

`scripts/macos.sh flash` selects this sketch by default. The built-in-CAN sketch
requires `scripts/macos.sh flash carrier_bridge`. Neither operation should be
used as a diagnostic probe without confirming the connected hardware and a safe bench state.

The MCP2515 receive configuration uses **RXM=00 and explicitly zeroed acceptance
masks/filters**, retaining all standard IDs/data bytes while requiring valid CAN messages.
Do not use RXM=11 as a shortcut for accepting all IDs: according to the
[MCP2515 datasheet, sections 4.1.3 and 4.2.2, page 23](https://ww1.microchip.com/downloads/en/DeviceDoc/MCP2515-Stand-Alone-CAN-Controller-with-SPI-20001801J.pdf#page=23),
that debug mode also delivers partially assembled messages when CAN errors occur.
The MKS eight-bit checksum can accidentally accept corrupted data; it does not
replace the controller's CAN validation.
All filters use EXIDE=0, so reception is limited to standard frames. Software also
defensively rejects extended and remote frames if they reach the receive buffers.

RXnIF locks each receive buffer until the firmware clears it **after** copying the
frame. Slow, separate register reads can contribute to overflow of the other
buffer, but do not by themselves cause a pending frame to be overwritten mid-read.
The existing bench-tested SPI read conditioning is retained. Controller errors
remain counted; no encoder-jump or motion-stop checks are relaxed.

The previous RXM=11 setting is a confirmed software defect. It is not proof that
every historical corrupt host-log entry originated there: those logs contain
neither the CAN wire traffic nor the SPI transactions. A host fault message alone
does not establish a wiring or transceiver fault.

### Stationary validation of valid-only reception

The corrected sketch was uploaded to the UNO R4 WiFi and checked on 2026-10-31,
with the arm supported and the operator present. The five-minute check sent only
31h encoder requests to drives 1, 3, 4 and 5 at 500 kbit/s; it sent no enable, zero,
stop or position commands.

- **143,985 requests and matching replies**, approximately 119.99 Hz per drive.
- Per-drive p95 host-queue-to-next-matching-reply time: **3.96-4.37 ms**.
- No bad MKS checksums; the largest successive count change was **one count**.
- No new host errors, refusals, reopen events, or carrier-counter increases during
  the measurement. A separate audit of the raw serial records confirmed the
  encoder-only command whitelist and reply totals.

Carrier counters are cumulative across serial sessions. The refused-line count
increased between sessions from 0 to 1 to 2, before any encoder requests in those
new sessions, and stayed at 2 throughout the five-minute measurement. The initial
absolute-counter check stopped before polling, and the later measurement explicitly
recorded its baseline rather than resetting the counter. The subsequent
[serial-close check](#serial-close-acknowledgement) isolated a host lifecycle issue,
not evidence of a CAN wiring fault.

E-stop and supply-voltage telemetry reported **unknown**, not healthy or faulty.
The operator's physical power cutoff, not those unavailable telemetry inputs, was
the safeguard for this stationary check.

This validates stationary request/reply traffic only. It does **not** establish
calibration accuracy, motion safety, or the source of every historical corrupt
frame. Comparable motion-traffic testing and, if corruption recurs, independent
CAN/SPI capture remain necessary before declaring that failure resolved.

### Serial-close acknowledgement

A control-only bench comparison reproduced a refused-line increment when USB was
closed immediately after writing SLCAN `C`. The tested in-band close/open,
flush-before-close and acknowledged-close cycles did not increment it. The host
now waits up to 250 ms for the carrier's close acknowledgement before releasing
USB, rather than treating `write()` as proof that the final line was received.
Timeouts, refusals and disconnects are reported; the port is still released.

With the corrected host, **30 USB close/reopen cycles and 120 encoder requests**
completed with matching replies and no counter increase. Close acknowledgements
took at most **2.90 ms** in that run. This change does not alter the drive node's
stop/hold sequence and does not send additional motor commands.

## Bitrate

| Where | Bitrate | Status |
| --- | --- | --- |
| MKS drives as shipped | 500 kbit/s | Factory default |
| Bench sketches (`kCanBitrate`) | 500 kbit/s | Match drives as shipped |
| Real-arm host through `carrier_bridge` | 1 Mbit/s | Target for the five arm drives |
| Real-arm host through `carrier_bridge_mcp2515` | 500 kbit/s | Current 8 MHz MCP2515 arm configuration |
| Carrier USB serial | 1,000,000 baud | Separate serial-link setting, not CAN bitrate |

The following migration is **only for the built-in-CAN 1 Mbit/s topology**, not the
8 MHz MCP2515 carrier (which cannot use 1 Mbit/s with this driver). Configure each
drive alone. The carrier keeps the CAN rate at which it first opened until reset,
so replug it before checking the shared 1 Mbit/s bus:

```bash
./scripts/macos.sh drives set-bitrate 3 1000000
# after every drive is switched, replug the carrier
./scripts/macos.sh drives --bitrate 1000000 scan --ids 1 2 3 4 5
```

If the carrier and a drive disagree on CAN bitrate, no frames are ACKed.

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

All four sketches compile for **Arduino UNO R4 WiFi and Minima**. Nothing has been uploaded to
a board yet, and no powered motor test has been run. Powered testing still needs Yassin or
Mujtaba present.

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
7. **Stop:** `stop 1` sends F5h with speed 0 (`001#F5000000000000F6`), which the manual and `sim_drives.py` define as a stop (acc 0 = at once; `stop 1 4` gives the manual vector `001#F5000004000000FA`). `disable 1` (`001#F300F4`) releases the shaft. `estop 1` sends the manual section 9.2.3 immediate-stop frame `001#F7F8`. None of these software commands replaces the physical E-stop.
8. **Save the log** (next section) and compare every byte.

Command reference (`help` prints it):

| Command | Frame | Notes |
| --- | --- | --- |
| `read <id>` | 31h | Do this first, and before every move |
| `enable <id>` / `disable <id>` | F3h 01 / 00 | `move` needs the enable reply |
| `move <id> <axis> <rpm> <acc>` | F5h | All four values are required; bench limits apply |
| `stop <id> [acc]` | F5h, speed 0 | Always allowed |
| `estop <id>` | F7h | Immediate software emergency stop; always allowed |
| `mode <id>` | 82h 05 | SR_vFOC only; writes a setting |
| `response <id> <0/1> <0/1>` | 8Ch | Writes a setting |
| `heartbeat <id> <ms>` | 98h | 0 = off; writes a setting |
| `raw <ID#HEX>` | as typed | No checksum added; only read-only 31h is allowed |
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

The stop here is F5h with speed 0. F7h is also covered by the unit and vcan tests.

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
| `tIIILDD..` | Queue a standard data frame; BELL if malformed, reserved ID `0x7F0`, or the 16-frame queue is full |
| `V`, `N` | `V0100`, `NWB01` |
| empty line | CR. After a queued frame, held until that frame is sent; BELL then CR if it expires or `C` drops it. |
| received frames | sent to the host as `tIIILDD..\r` while open |
| `L`, `s`, `T`, `r`, `R`, `F`, `Z`, `d`/`D`/`b`/`B`, anything else | BELL (not supported: no listen-only mode or remote frames in `Arduino_CAN`, no extended IDs or CAN FD, no status flags or timestamps) |

While open, the bridge sends a host-only status frame from standard ID `0x7F0` ten times per
second. It never transmits that frame on CAN, and drops bus frames using that reserved ID. Its
eight bytes are sequence, flags, supply mV, CAN error-event count, failed writes, and refused
serial lines; counter fields wrap so the host can take modulo differences. The exact layout is
documented in `SlcanBridge.h`.

The bridge does **not** check MKS checksums or apply motion limits; the host software is
responsible for that. A 16-frame queue absorbs a complete control tick while the UNO R4's
single standard-frame mailbox is busy; entries older than 10 ms are dropped and `C` clears the
queue. The real-arm host (`waybionic_teleop/slcan_bus.py`) writes an empty line after each
frame, keeps no more than 16 frames unanswered so the queue never fills, and waits for the
answers before closing; a frame that expires or is dropped produces BELL immediately before its
CR. It has been tested through
`slcan_sim` and python-can; it has **not** been tested on the UNO R4/TJA1051T or real drives.

**E-stop and supply telemetry:** `BenchTelemetry.h` has the interface, but currently returns
unknown for both. Electrical has not specified the input pins, active polarity, isolation,
voltage-divider values, ADC reference/scaling, or valid voltage limits. The physical E-stop
must remain effective with the carrier absent or failed.

## Adding drives one at a time

1. Bring up each drive **alone** because all five ship as ID 1.
2. Using the drive setup tooling, assign base 1, shoulder 2, elbow 3, left wrist motor 4,
   and right wrist motor 5; then set that isolated drive to 1 Mbit/s.
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
| 39h READ_POSITION_ERROR + signed int32 | Python decoder and read-only `mks_setup read-error` query; 1424 matching replies in an elbow pressure recording | `mks_can.py`, `mks_setup.py`, their unit tests |
| 82h SET_MODE (05h SR_vFOC) | Implemented and verified | same |
| 8Ch SET_RESPONSE | Implemented and verified | same |
| 98h SET_HEARTBEAT | Implemented and verified | same |
| F3h ENABLE | Implemented and verified | same |
| F5h ABSOLUTE_AXIS (speed 0 = stop) | Implemented and verified | same, plus `sim_drives.py` for the speed-0 stop |
| **F7h emergency stop** | Implemented and verified | MKS CAN manual V1.0.9 section 9.2.3 |
| **Set CAN ID (8Bh)** | Not implemented in this subset or merged `mks_can.py` | MKS CAN manual |
| **Set CAN bitrate (8Ah)** | Not implemented in this subset or merged `mks_can.py` | MKS CAN manual |
| Broadcast ID 0, other MKS commands | Not implemented | - |

"Verified" means checked against the repo's reference implementation and its manual vectors. The real-hardware check is the frame log from the bench session.

The observed `39h` reply has DLC 6: command byte, four signed error bytes, and checksum.
Its scale is 51200 units per motor turn, unlike the `31h` encoder's 16384 counts per
turn. It is motor-shaft position error, not a fault code or a measured force. See
[powered hand-guiding](../../docs/operator_guide.md#powered-hand-guiding) for the limits
of using it to infer a push.

### F5h on firmware V1.0.9

The bench drives (SERVO42D_CAN, firmware V1.0.9, hardware 2) take the F5h frames above as the
manual says, with one exception found with `waybionic_teleop`'s 120 Hz target stream. A target
sent while a running move is reversing or slowing to turn back can be kept, and so can an
occasional target sent mid-stroke in the same direction. Once the moves stop (F5h speed 0,
F7h, or the last target reached), the drive runs to that kept target at the old speed, up to
12° of base travel and 0.07-0.9 s after the frame. Frame logs show the run starting 3-5 ms
after the stop. Each runaway ended 1-8 counts from a target sent earlier. The exact trigger
isn't known.

The drive node avoids this in two ways.

- **No target behind a running move.** When the commanded target passes the last encoder
  reading against the drive's direction, the node sends an F5h stop (speed 0, acc 0). It
  sends no new target until the drive has settled: 12 ms after the stop, with no encoder
  step in the old direction; or a run-complete reply after the stop, with a step under 5
  counts. The bench pause before the drive turns back is 17 ms median, 42 ms worst case.
  Over the same random stick profiles on the base, streaming straight through each reversal
  ran away 7 times in 80 cycles; the stop and settle ran away 0 times in 80 (p≈0.007).
- **A hold right behind every stop.** Every stop of a running move is followed at once by an
  F5h to where the drive is to stay, at 60 rpm. That is the stopped move's last target, at
  most two ticks of travel past the last encoder reading. The hold goes out again every tick
  for 0.25 s. A drive that may have lost its zero (lost, faulted, being zeroed) or whose
  E-stop is pressed gets no hold. Bench, base, 80 ramped-down stops per variant:

  | After the stop | Runs to a kept target | Worst excursion |
  | --- | --- | --- |
  | One re-target once a run showed | 7 | 2515 counts (2.1°) |
  | Hold every tick from 8 ms | 8 | 1282 counts (1.1°) |
  | Hold 1 ms after the stop, then every tick | 5, each caught | 437 counts (0.37°) |
  | **Hold right behind the stop, then every tick** (the node) | **0** | **107 counts (0.09°)** |

`sim_drives.py` models the kept target so the tests catch any regression.

The speed-0, acc-0 stop itself overshoots by about 100 counts on the base, then creeps back to
the last F5h target.

## Verification status

| What | Status |
| --- | --- |
| Shared library logic (framing, receiver, gateway, console, SLCAN, bus load) | Unit-tested on the host with gtest |
| Frame bytes vs merged `mks_can.py` | Cross-check command documented; rerun results belong in the PR/test report |
| SLCAN bridge vs python-can's `slcan` driver | Tested against `../sim/slcan_sim` on a pseudo-terminal |
| Sketches compiled for UNO R4 WiFi and Minima | **Pass.** All four sketches compile for `unor4wifi` and `minima`: `single_drive_bringup`, `four_node_bench/gateway`, `four_node_bench/receiver` and `carrier_bridge`. CI repeats this matrix. |
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
