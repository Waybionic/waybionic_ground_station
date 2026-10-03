# Biomedical clamp integration contract (draft)

This records the information needed to replace the simulated `tool_grip` placeholder.
It is an integration worksheet, not a specification for the physical tool board.

## Confirmed from the October 3 task brief

- The tool is a clamp on a rod with two claws.
- Four NEMA 8 stepper motors pull lines through a rerouter: one pair per claw.
- The two motors in each pair turn in opposite directions.
- The current Biomedical prototype uses an Arduino Uno R3, CNC Shield and A4988 drivers,
  with `nema8_2pair_LimitedRotationV3.ino` as its joystick-driven firmware.
- The ground station currently has one simulated `tool_grip` joint and one simulated
  `tool` drive. Controller triggers close and open that placeholder.

## Measurements and decisions to obtain from Biomedical

| Item | Left claw | Right claw |
| --- | --- | --- |
| Claw angle or travel at fully open | Pending | Pending |
| Claw angle or travel at fully closed | Pending | Pending |
| Safe motion limits and home position | Pending | Pending |
| Motor identifiers and direction for opening | Pending | Pending |
| Steps per degree (or steps per mm), including microstepping | Pending | Pending |
| Maximum command speed and acceleration | Pending | Pending |
| Position sensing or homing method | Pending | Pending |

Also confirm whether both claws may move together, the tool-flange mounting transform,
the latest CAD/STL files and their units, and any obstruction or fault behavior. If the
line routing makes motor steps non-linear in claw angle, obtain the measured calibration
curve rather than using one steps-per-degree value.

## Proposed software boundary

The controller will command two claw positions. A separate tool mapping will convert
each claw position into two motor step targets with opposite directions. Each motor
will have its own target, feedback, limit state and diagnostic entry. The present arm
drive map is square (one independent joint coordinate per drive), so it cannot simply
be extended with four tool motors for two claw joints.

Until the physical interface is defined, the four-motor path is simulation only. Do
not send these targets to the Arduino or assume its A4988 wiring is a CAN interface.

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
