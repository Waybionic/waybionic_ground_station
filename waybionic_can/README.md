# waybionic_can

This package provides hardware-independent MKS CAN framing, gateway/node simulations, test
tools, and UNO R4 firmware for the WayBionic arm. The real carrier is an UNO R4 WiFi connected
through a TJA1051T transceiver to five MKS drives (IDs 1-5). The laptop reaches that bus over a
1,000,000-baud USB SLCAN link; the CAN bus itself runs at 1,000,000 bit/s. Those two rates are
configured independently.

The four-node Arduino/SocketCAN bench and software actuators remain simulation tools. They do
not demonstrate that a real drive, E-stop input, or voltage input has been validated.

```text
Laptop --stdin / USB serial--> gateway
                                  |
============== vcan0 (simulation) or CAN_H/CAN_L via TJA1051T (hardware) ==============
          |                        |                        |
   receiver --id 1          receiver --id 2          receiver --id 3
   SoftwareActuator         SoftwareActuator         SoftwareActuator
```

For firmware details, see **[arduino/README.md](arduino/README.md)**. For the supervised
five-drive session, use the **[hardware bring-up checklist](docs/HARDWARE_BRINGUP_CHECKLIST.md)**.

## Layout

The portable code lives in the Arduino library `arduino/libraries/WaybionicCan/src/` (written `lib/` below). The CMake build compiles the same files.

| Path | Role | Runs on Arduino |
| --- | --- | --- |
| `lib/common/Frame.h` | Neutral CAN frame: 11-bit ID, DLC, 8 data bytes | yes |
| `lib/common/MksFrame.*` | MKS encode/parse/checksum, ported from `waybionic_teleop/mks_can.py` | yes |
| `lib/common/ICanTransport.h` | `send(frame)` / `receive(frame, timeout_ms)` | yes |
| `lib/common/IActuator.h`, `SoftwareActuator.*` | Actuator interface; the software one records commands and finishes moves after a delay | yes |
| `lib/common/NodeLogic.*` | Receiver: ID filter, validation, commands, replies, heartbeat stop | yes |
| `lib/common/GatewayLogic.*` | Gateway: matches replies to requests, node health | yes |
| `lib/common/BringupConsole.*` | Serial commands to validated MKS frames, with motion gating and TX/RX logs | yes |
| `lib/common/SlcanBridge.*`, `Candump.*` | SLCAN subset for python-can; candump text parsing | yes |
| `lib/common/BusLoad.*` | Bus-load estimate from the frames a node sends and receives | yes |
| `lib/common/BenchTelemetry.h` | E-stop / supply-voltage placeholders (hardware-TBD) | yes |
| `lib/r4/R4CanTransport.*` | UNO R4 `Arduino_CAN` transport; the only board-specific code | R4 only |
| `arduino/*/*.ino` | Sketches: single-drive bring-up, bench gateway/receiver, carrier bridge | yes |
| `sim/SocketCanTransport.*` | Linux SocketCAN; the only code that sees `struct can_frame` | no |
| `sim/gateway_main.cpp`, `sim/receiver_main.cpp` | The vcan simulation executables | no |
| `sim/slcan_sim_main.cpp` | `slcan_sim`: the SLCAN bridge on a pty in front of simulated drives, for python-can without hardware | no |
| `sim/mks_frame_tool.cpp` | `mks_frame_tool`: stdin front end to MksFrame, used by the cross-check | no |
| `tests/unit/` | gtest, no vcan needed (includes an in-memory four-node bus) | |
| `tests/integration/` | Real processes on vcan0; skipped if the interface is missing | |
| `tests/cross_check/` | Scripts comparing against the merged teleop `mks_can.py`, and python-can against `slcan_sim` | |
| `docs/HARDWARE_BRINGUP_CHECKLIST.md` | Supervised five-drive setup, test, evidence, and results template | |

`lib/common/` uses only C headers (`<stdint.h>`, `<stddef.h>`, `<stdio.h>`, `<string.h>`, `<stdarg.h>`). It has no STL, exceptions, RTTI or heap use. The `waybionic_can_embedded_check` target compiles it with `-fno-exceptions -fno-rtti -nostdinc++ -Werror` to keep it that way.

## Protocol source

The merged Xbox teleop implementation (`waybionic_teleop/mks_can.py`, from PR #28) defines the
shared command subset, and `sim_drives.py` defines simulated replies. The C++ tests repeat its
manual vectors byte for byte. PR #31 / `feature/real-arm` extends the host with drive-ID,
bitrate, zeroing, and emergency-stop frames; this package does not duplicate those host-owned
commands.

- The frame's CAN ID is the motor ID, and data is `code, big-endian arguments, checksum`. The checksum is `(CAN ID + sum(body)) & 0xFF`. Replies use the same layout and the same ID.
- The implemented commands are 31h READ_ENCODER, 82h SET_MODE, 8Ch SET_RESPONSE, 98h SET_HEARTBEAT, F3h ENABLE and F5h ABSOLUTE_AXIS.
- Setting commands reply `status=1`. F5h replies `0` (refused) unless the drive is enabled and in SR_vFOC mode (05h). Otherwise it replies `1` (running), and later sends an unprompted `2` (run complete) if `active` is set. Speed 0 means stop. Speed is clamped to 3000 rpm.
- Bad checksums, bad DLCs, unknown codes and wrong argument counts get no reply.
- SET_RESPONSE `respond=0` silences status replies but not encoder data.
- With a heartbeat set, a moving drive stops once it has received nothing addressed to it for more than that many milliseconds. Sending the next F5h is how it recovers.

A CAN ACK only shows that some controller received a valid frame. The gateway and tests count a command as successful only when the addressed node replies.

## Set up vcan0 (Linux)

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

You can also run the existing `scripts/setup_vcan.sh` at the repository root. `sudo apt install can-utils` gives you `candump vcan0` for watching the bus.

The stock WSL2 kernel does not ship the `vcan` module. There, the vcan integration tests are skipped and the demo cannot run.

## Build and test

From a colcon workspace that contains this repository under `src/`:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select waybionic_can
colcon test --packages-select waybionic_can --event-handlers console_direct+
colcon test-result --verbose
```

To run a single binary directly, use `build/waybionic_can/test_mks_frame`,
`test_node_logic`, `test_gateway_logic`, `test_bringup_console`, `test_slcan_bridge`,
`test_bus_load` or, on Linux, `test_vcan_four_node`. The integration test uses
`$WAYBIONIC_CAN_IFACE` (default `vcan0`) and skips if that interface is missing. SocketCAN
executables and the vcan test are Linux-only; common logic and unit tests also build on macOS.

Two extra checks need files or packages outside this package, so they aren't colcon tests:

```bash
# C++ framing vs the merged Python implementation on ~80k edge and random cases
python3 src/waybionic_can/tests/cross_check/cross_check_mks.py \
  --mks-can src/waybionic_teleop/waybionic_teleop/mks_can.py \
  --tool build/waybionic_can/mks_frame_tool

# python-can's real slcan driver against the SLCAN bridge code (needs python-can and pyserial)
python3 src/waybionic_can/tests/cross_check/slcan_python_can_check.py --sim build/waybionic_can/slcan_sim
```

## Four-node demo

```bash
source install/setup.bash
ros2 run waybionic_can run_four_node_demo.sh        # three receivers + interactive gateway
```

Or run each process in its own terminal:

```bash
ros2 run waybionic_can receiver --id 1
ros2 run waybionic_can receiver --id 2
ros2 run waybionic_can receiver --id 3
ros2 run waybionic_can gateway --nodes 1,2,3 --poll-ms 1000 --timeout-ms 500
```

Receiver options are `--iface`, `--move-ms` (how long a simulated move takes) and `--reply-delay-ms` (for testing a slow node). Stop any receiver to watch the gateway mark it `OFFLINE`. Start it again to see it come back `ONLINE`, but with its settings lost.

Gateway commands:

```text
mode 2                  enable 2 [0|1]          move 2 16384 600 2
read 2                  response 2 1 0          heartbeat 2 500
raw 002#3134            wait 500                status        help        quit
```

Here is what an exchange looks like. Receivers 1 and 3 log `ignored (addressed to 2)` for every frame:

```text
[gateway] TX 002#F301F6
[node 2] RX 002#F301F6 ENABLE accepted
[node 2] TX 002#F301F6 reply
[node 2] actuator enabled=1 moving=0 target=0 position=0
[gateway] RX 002#F301F6 node 2 ENABLE reply status=1 (ok) latency=1ms
```

## Test coverage

- `test_mks_frame`: checks request/reply bytes, checksum rejection, signed 24/48-bit limits,
  argument counts, and formatting.
- `test_node_logic`: checks address filtering, command validation, response policy, actuator
  state, completion reports, and heartbeat stops.
- `test_gateway_logic`: checks request/reply matching, timeouts, node recovery, and the
  in-memory multi-node bus.
- `test_bringup_console`: checks command parsing, motion gates, status/error output, and the
  documented startup vectors.
- `test_slcan_bridge`: checks SLCAN parsing, bitrate/open state, rejected-line accounting,
  forwarding, and carrier status serialization without putting status on CAN.
- `test_bus_load`: checks classic-CAN lower/worst-case bit estimates and reporting windows.
- `test_vcan_four_node`: launches real gateway/receiver processes over Linux `vcan`.
- `cross_check_mks.py`: compares C++ frames with Python over edge and randomized cases.
- `slcan_python_can_check.py`: drives the shared bridge through a pseudo-terminal using
  python-can and ignores the host-only carrier status frames.

## Hardware status and TODOs

- **F7h emergency stop, set CAN ID, set bitrate:** intentionally absent from this package's
  bench console and shared C++ subset. The Arduino `estop` command sends nothing. PR #31 /
  `feature/real-arm` implements these in the host from MKS CAN manual V1.0.9.
- **Broadcast ID 0 and other MKS commands:** not implemented in the C++ bench subset.
- **Physical receiver actuator:** the bench receivers use `SoftwareActuator`. A hobby-servo `IActuator` would need a defined mapping from MKS axis counts to angles and of what READ_ENCODER reports.
- **Carrier telemetry hardware:** status ID `0x7F0` reports unknown E-stop/supply plus CAN error
  counters today. Electrical must provide sensing pins, polarity, divider/isolation, scaling,
  and valid voltage limits before `readBenchTelemetry()` can sample hardware. Unknown is kept
  distinct from released/healthy.
- **Hardware:** flash, five-drive ID/bitrate setup, encoder checks, zeroing, motion, physical
  E-stop, 500 ms heartbeat timing, load/error capture, photos, and video remain untested. Use
  the [checklist](docs/HARDWARE_BRINGUP_CHECKLIST.md) with Yassin or Mujtaba present.
