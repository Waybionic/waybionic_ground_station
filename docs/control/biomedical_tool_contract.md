# Biomedical clamp integration contract (draft)

This records the information needed to replace the provisional simulated clamp.
It is an integration worksheet, not a specification for the physical tool board.

## Confirmed from the task brief and Biomedical's demo files

- The tool is a clamp on a rod with two claws.
- Four NEMA 8 stepper motors pull lines through a rerouter: one pair per claw.
- The demo's X/Y motor pair drives one claw; its Z/A pair drives the other. In the
  controlled sketch, one joystick axis drives each pair. Both can move together.
- Opposite physical motor rotation is intended, but actual direction depends on
  wiring, line tension and assembly. Biomedical prefers swapping motor wires on
  the CNC Shield to changing the prototype firmware's direction logic.
- The prototype uses an Arduino Uno R3, CNC Shield and A4988 drivers. Its demo
  code gives motor settings, not a measured claw-angle calibration.
- The ground station now has two independent *simulated* claw coordinates and
  four simulated motor counts. The visuals use supplied STL meshes as RViz markers,
  not calibrated CAD joints.

## Simulation-only assumptions

The current `tool_drives.yaml` uses normalized claw closure from 0 (open) to 1
(closed), a 0.25-unit/s limit and 320 microsteps per normalized unit. That step
count is an arbitrary simulation stand-in: Biomedical clarified that its 200
steps correspond to the full joystick-axis sweep, not a measured claw angle.
It is **not** steps per degree of claw motion. Motor signs
are configurable and their present values are unverified. No CAN or A4988
commands are sent. Do not use these values as hardware limits.

Julie's `Julie - Assembly.stl` is now the source for three RViz meshes: its six
fixed components and two independently moving claw components. The export gives
the parts' relative assembled positions. Its base is 80 STL units wide, so RViz
currently interprets one unit as one millimetre. The base is centred on the
arm's `tool_link` with its bottom on the flange; Biomedical still needs to
confirm that mounting transform and the file units. The claws rotate by a
provisional 0.35 radians about the assembled bar axis from Julie's pose. This
is a visual demonstration, not a measured open-to-closed range.

For a simulation-only stalled-motor demonstration, set
`simulated_stalled_motor` to one of `left_1`, `left_2`, `right_1`, or `right_2`
in `tool_drives.yaml` and restart the simulator. The affected motor count then
stays fixed while its paired motor moves; both the motor and claw diagnostics
report a warning, including the pair error. Leave the setting empty for normal
use. This does not emulate actual board feedback or authorize powered tests.

## Measurements and decisions to obtain from Biomedical

| Item | Left claw | Right claw |
| --- | --- | --- |
| Claw angle or travel at fully open | Pending | Pending |
| Claw angle or travel at fully closed | Pending | Pending |
| Safe motion limits and home position | Pending | Pending |
| Motor identifiers and physical direction for opening | Pending | Pending |
| Steps per degree (or steps per mm), including microstepping | Pending | Pending |
| Maximum command speed and acceleration | Pending | Pending |
| Position sensing or homing method | Pending | Pending |

Biomedical confirmed that the demo can move both claws together. Obtain the full
assembly/CAD, tool-flange mounting transform and file units, plus any obstruction
or fault behavior. If line routing makes motor steps non-linear in claw angle,
obtain a measured calibration curve rather than one steps-per-degree value.

## Proposed software boundary

The controller commands two claw positions. A separate tool mapping converts
each claw position into two motor step targets with configurable signs. Each motor
has its own target, estimated feedback, limit state and diagnostic entry. The arm
drive map is square (one independent joint coordinate per drive), so it cannot simply
be extended with four tool motors for two claw joints.

This four-motor path is simulation only. It encodes the same F5h target frames
as tool board PR #32 for CAN IDs 7 to 10, but does not transmit them to a
physical bus. PR #32 uses 3200 microsteps and 16384 MKS counts per motor turn.
The arm's five-joint `joint_commands` topic stays separate from `tool_commands`.
Do not transmit provisional targets before physical calibration is confirmed.

## Tool-board interface for the CAN bring-up group

The provisional motor IDs and position-frame encoding follow PR #32. Physical
claw-to-motor calibration, command rate, acceleration, feedback ownership and
fault behavior are **pending**. The logical interface should cover:

| Host command | Tool-board feedback |
| --- | --- |
| Four target motor positions in steps | Four measured/estimated motor positions in steps |
| Per-motor speed/acceleration limits or common move duration | Motion state and actual speed for each motor |
| Enable, hold/stop and homing requests | Enabled/homed state and limit status |
| Sequence number or command timestamp | Heartbeat, last accepted command and fault flags |

The board and host must agree on what happens after a command timeout, a disconnected
controller, a limit event, loss of position reference, or disagreement within a motor
pair. These behaviors need Biomedical and CAN bring-up confirmation before hardware
control is enabled.
