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
  four simulated motor counts. The visuals are temporary cubes, not CAD joints.

## Simulation-only assumptions

The current `tool_drives.yaml` uses normalized claw closure from 0 (open) to 1
(closed), a 0.25-unit/s limit and 66 motor steps per normalized unit. That step
count is only a stand-in based on the demo's 120-degree motor rotation at 200
full steps/revolution; it is **not** steps per degree of claw motion. Motor signs
are configurable and their present values are unverified. No CAN or A4988
commands are sent. Do not use these values as hardware limits.

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

This four-motor path is simulation only. Do not send its targets to the Arduino
or assume its A4988 wiring is a CAN interface.

## Tool-board interface for the CAN bring-up group

The final message IDs, encoding, units, update rate and ownership of step generation
are **pending**. The logical interface should cover:

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
