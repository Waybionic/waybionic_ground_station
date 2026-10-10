# Five-drive carrier bring-up checklist

Use this checklist with Yassin or Mujtaba present. It prepares the five arm drives (IDs 1-5)
for the `feature/real-arm` ground station. No powered step in this document has been completed
or validated by the `waybionic_can` software-only tests.

## Safety and prerequisites

- [ ] Yassin or Mujtaba is present before motor power is applied.
- [ ] The arm is supported, motors are initially unloaded where practical, speeds are at their
  minimum, and the physical emergency stop is within reach.
- [ ] The physical E-stop removes motor energy independently of this firmware. A software stop,
  the MKS heartbeat, USB monitoring, and the carrier status frame are not safety substitutes.
- [ ] Electrical has confirmed transceiver power/mode, common reference, connector pinout, two
  120-ohm end terminations, and the powered-off CAN_H-to-CAN_L resistance.
- [ ] Record the repository commit, operator/lead names, date, wiring photos, drive labels, and
  supply/current-limit settings in the results template below.
- [ ] Use the `feature/real-arm` integration (or a later branch containing its `flash`, `drives`,
  and `arm` macOS commands). Those host commands are not part of this focused package PR.

Stop immediately on unexpected motion, duplicate replies, smoke/heat, wiring uncertainty,
repeated CAN errors, or loss of the physical E-stop. De-energize the motor supply before
changing bus wiring.

## 1. Flash and verify the carrier

With only the UNO R4 WiFi connected over USB:

```bash
brew install arduino-cli
./scripts/macos.sh flash
```

Expected: the helper finds one `/dev/cu.usbmodem*`, compiles `carrier_bridge`, uploads it, and
prints `Flashed carrier_bridge to ...`. If several ports exist, set
`WAYBIONIC_CARRIER=/dev/cu.usbmodem...`. The firmware uses **1,000,000 baud on USB serial**.
That is independent of the selected **1,000,000 bit/s CAN bus** rate.

Before changing drive settings, connect one factory drive at 500 kbit/s and run:

```bash
./scripts/macos.sh drives scan --ids 1
```

Expected: no `No status from the carrier` warning and one `CAN ID 1: encoder ... turns` line.
The scan sends MKS command `31h`. If it fails, do not continue: check the selected serial port,
carrier flash, drive power, common reference, CAN_H/CAN_L polarity, termination, and 500 kbit/s
factory setting. Log the exact output and carrier/CAN error counts.

## 2. Assign IDs one drive at a time

All drives ship as ID 1. Connect exactly one drive at a time at 500 kbit/s. Label the physical
drive before proceeding.

| Joint | Final ID | Command |
| --- | ---: | --- |
| Base | 1 | Leave at ID 1; verify with `./scripts/macos.sh drives scan --ids 1` |
| Shoulder | 2 | `./scripts/macos.sh drives set-id 1 2` |
| Elbow | 3 | `./scripts/macos.sh drives set-id 1 3` |
| Wrist pitch / left wrist motor | 4 | `./scripts/macos.sh drives set-id 1 4` |
| Wrist roll / right wrist motor | 5 | `./scripts/macos.sh drives set-id 1 5` |

Expected after each `set-id`: `The drive on CAN ID 1 now answers on N`. The CLI refuses an ID
that already answers. If verification fails, power down and leave that drive isolated; do not
connect it to the shared bus until its displayed/configured ID is known.

## 3. Change each drive to 1 Mbit/s

While each drive is still alone, at the carrier's current 500 kbit/s session:

```bash
./scripts/macos.sh drives set-bitrate 1 1000000
./scripts/macos.sh drives set-bitrate 2 1000000
./scripts/macos.sh drives set-bitrate 3 1000000
./scripts/macos.sh drives set-bitrate 4 1000000
./scripts/macos.sh drives set-bitrate 5 1000000
```

Run only the line for the isolated drive. Expected: `CAN ID N now runs at 1000000 bit/s`.
The drive switches after replying, while the carrier retains the rate at which it first opened.
Do not interpret the expected loss of communication after the reply as a failed change.

After all five are configured, unplug/replug the carrier, connect all five drives with power
off, verify termination, power them, and run:

```bash
./scripts/macos.sh drives --bitrate 1000000 scan --ids 1 2 3 4 5
```

Expected: exactly one encoder line for every ID 1-5. This is also the required `31h` encoder
read. Save all five values. Missing IDs suggest power, wiring, bitrate, or ID problems; unstable
or duplicate responses suggest duplicate IDs or bus integrity problems. Do not command motion.

## 4. Start, zero, and move one joint at a time

Place the arm in the designated upright zero pose before starting:

```bash
./scripts/macos.sh arm
```

Keep teleoperation disabled. In a second terminal:

```bash
./scripts/macos.sh run ros2 service call /sim_arm_drives/zero std_srvs/srv/Trigger
```

Expected: the Trigger service reports success and joint states become available. Zeroing uses
MKS `92h` in the `feature/real-arm` host and must be performed only in the agreed physical pose.
If any drive was lost or power-cycled, stop and zero the complete arm again.

At the controller's lowest speed, enable only when controls are neutral. Exercise base,
shoulder, elbow, wrist pitch, and wrist roll one at a time through a very small range. For each
joint record commanded direction, encoder direction/change, unexpected coupling, noise, and
stop response. If direction is wrong, stop; Yassin/Mujtaba must review the corresponding
`factors` in `waybionic_teleop/config/arm_drives.yaml` before rebuilding. Do not increase speed
or attempt coordinated motion during this acceptance pass.

## 5. E-stop and 500 ms heartbeat tests

### Physical E-stop

With a single joint moving slowly and a lead ready to support the arm, press the physical
E-stop. Expected: motor energy is removed by hardware regardless of USB/carrier software.
Record whether all axes stop, whether power is removed, carrier status before/after, drive
recovery behavior, and whether re-zeroing is required.

The current firmware reports E-stop as **unknown** until Electrical supplies the sense contact,
logic polarity, isolation, and UNO pin. Unknown must not be logged as released. Do not add a
jumper or infer E-stop state from CAN traffic.

### USB loss / drive heartbeat

The real-arm configuration programs each drive's MKS heartbeat to 500 ms. During one slow,
small motion, disconnect USB from the carrier without touching CAN power.

Expected acceptance criterion: the drive stops due to its own heartbeat approximately 500 ms
after the last addressed frame. Measure from a timestamped recording; do not claim exact timing
from observation alone. Reconnect USB, confirm the carrier recovers, and confirm motion remains
disabled until the documented operator re-enable/zero sequence is complete. A host command
timeout and physical E-stop are separate mechanisms.

If motion continues past the agreed tolerance, press the physical E-stop, de-energize, preserve
logs, and do not resume until Yassin reviews the programmed `98h` heartbeat and host traffic.

## 6. Collect load, errors, and evidence

With all five drives connected at 1 Mbit/s, capture:

- the five-drive scan output and encoder values;
- `can.carrier` monitor values: status sequence/freshness, CAN controller errors, failed writes,
  refused serial lines, E-stop state, and supply state;
- `can.bus` frame rate/load from the `feature/real-arm` engineering monitor;
- a simultaneous `candump`/SocketCAN trace only if using a SocketCAN adapter;
- wiring and termination photos and a demonstration video approved by the lead.

The package's `BusLoad` class estimates load only from observed data frames and excludes error
frames, arbitration delays, and retransmissions. Treat it as a lower-bound estimate. The
carrier's status frame is host-only ID `0x7F0`; it is not transmitted on CAN.

## Results template

```text
Date / repository commit:
Operators / lead present:
Carrier port / firmware result:
USB serial baud: 1000000
CAN bitrate: 1000000 bit/s
Termination / powered-off resistance:
Supply setting / current limit:

Joint          ID  Label/serial  31h encoder  Small-motion result  Direction
Base           1
Shoulder       2
Elbow          3
Wrist pitch    4
Wrist roll     5

Five-drive scan: PASS / FAIL
Zero pose confirmed by:
~/zero result:
Physical E-stop result:
Carrier E-stop report: UNKNOWN / RELEASED / PRESSED
Motor supply report: UNKNOWN / ______ V
USB disconnect heartbeat stop: PASS / FAIL / NOT RUN
Measured stop time / method:
CAN load (method, average, peak):
CAN controller errors before / after:
Failed writes before / after:
Refused SLCAN lines before / after:
Unexpected frames, replies, or motion:
Corrective actions:
Logs:
Wiring photos:
Video:
Open questions / owner:
```
