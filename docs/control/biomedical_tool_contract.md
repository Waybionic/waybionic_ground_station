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

## Provisional-value replacement checklist (PR #43)

All host-side unmeasured values below are in `waybionic_teleop/config/tool_drives.yaml`.
The `xbox_teleop` section is loaded by teleop and the `sim_tool_motors` section by
the simulator. ROS does not support YAML aliases here, so update the shared
normalized limits and command speed in both sections; a test checks they agree.
Record the measurement, units, fixture, direction, and reviewer before
changing a value. Rerun the named tests and record an RViz check. Do not treat
the current numbers as hardware authorization.

| YAML value | Unit | How Biomedical should confirm it | Test or check using it |
| --- | --- | --- | --- |
| `xbox_teleop.tool_limits` / each `*_claw.lower`, `upper` | normalized closure (0 open, 1 closed) | Keep normalization; establish physical open/closed stops and map them with the motor steps below. Change these bounds only if the command convention changes. | `test_teleop.py`, `test_tool_sim.py`; RViz endpoints |
| `xbox_teleop.tool_speed` / each `*_claw.max_speed` | normalized closure/s | Time repeatable safe open/close motions for each claw under load; use the lower safe common rate for teleop. | `test_teleop.py`, `test_tool_sim.py`; stop demonstration |
| each `*_claw.reference` | normalized closure | Identify and mark the reproducible home pose, separately for left and right. | `test_tool_motor_map.py`, `test_tool_sim.py` |
| each `left_1`, `left_2`, `right_1`, `right_2` `.steps_per_unit` | microsteps/normalized closure | Count microsteps from measured fully open to fully closed for each cable/motor, at the board's actual microstep setting; use a calibration curve if non-linear. | `test_tool_motor_map.py`, `test_tool_sim.py`, `test_tool_board_frames.py` |
| each motor `.direction` | sign (+1 or -1) | Move each motor individually at a safe low speed and confirm which winding opens its claw and which paired motor turns oppositely. | `test_tool_motor_map.py`, `test_tool_sim.py`; RViz motion |
| each motor `.reference_steps` | microsteps | Home each motor safely and read its step count at the reference pose. | `test_tool_motor_map.py`, `test_tool_sim.py` |
| `xbox_teleop.tool_mesh_scale_m_per_unit` | m/STL unit | Compare a known dimension of the final exported assembly with a physical or CAD dimension. | `test_tool_visual.py`; RViz size |
| `xbox_teleop.tool_assembly_origin_m` | x, y, z metres in `tool_link` | Measure the tool flange to assembly datum and confirm clearance in the final CAD. | `test_tool_visual.py`; RViz mounting |
| `xbox_teleop.tool_assembly_quaternion_xyzw` | unit quaternion in `tool_link` | Measure the tool flange to assembled mesh orientation and verify the axis convention. | `test_tool_visual.py`; RViz mounting |
| `xbox_teleop.tool_claw_pivot_m` | x, y, z metres in assembly mesh coordinates | Locate the actual claw rotation axis in final CAD and check both moving meshes against it. | `test_tool_visual.py`; RViz open/close |
| `xbox_teleop.tool_preview_open_angles_rad` | signed left and right radians | Measure each claw's open-to-closed angle and direction from the final assembly. | `test_tool_visual.py`; RViz endpoints |
| `sim_tool_motors.board_safe_rpm` | motor rpm | Establish the safe motor rate under load; it must stay at or below PR #32 firmware's 60 rpm ceiling. | `test_tool_board_frames.py`; no powered test without lead and E-stop |
| `sim_tool_motors.board_acceleration_code` | F5 protocol byte, 0–255 | Agree on a safe acceleration ramp with the board team; this is *not* steps/s². | `test_tool_board_frames.py`; firmware parser |

The PR #32 firmware separately hard-codes CAN IDs 7–10, 3200 microsteps/rev,
placeholder ±3200-microstep travel and a 20000 steps/s² cap. Check wiring,
microstep jumpers and board limits with the board team before flashing or powered
tests. Changing this host YAML does **not** update those firmware constants.

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
