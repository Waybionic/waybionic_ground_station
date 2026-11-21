# Software section copy (conference poster)

Audience: general engineering. Each block is 3–4 sentences. Claims match `main` at `cf3e887`. Nothing here asserts clinical validation, surgical safety, or powered-arm demonstration.

Suggested poster heading: **Ground-station software: from operator input to arm motion**

---

## A. Teleoperation and incision group

An Xbox or DualSense controller drives the ground station. Stick and button samples become a 120 Hz stream of joint setpoints: two joint-space groups for the base and upper arm, a Cartesian group that moves the tool tip in straight lines and tilts about the tip, and an incision group for work through a port. The incision group stores the tip pose at the moment it is selected and then only inserts, withdraws, or tilts so that the tool axis keeps passing through that point; homing and group changes that would drag the tool sideways are blocked while the tip is inserted. These behaviours are implemented in software and have been exercised on simulated drives, not as a clinical procedure.

---

## B. Collision checks and joint limits

Before a command is sent, the next pose is checked against the URDF joint ranges and against collision boxes on the base, shoulder, upper arm, forearm, wrist, and tool. A step that would drive any joint past its limit, take the tip out of reach, or deepen contact with the table or the arm’s own base is discarded; the controller rumbles once and the on-screen label names the cause. An arm that is already in contact may only move in directions that do not make any contact worse. The joint ranges on the current model are provisional, and the boxes come from the committed meshes rather than a live CAD measurement.

---

## C. CAN communication

The same drive node used in simulation can send MKS SERVO frames through a UNO R4 carrier that bridges USB serial to a 1 Mbit/s CAN bus. Each tick writes absolute-position commands and encoder polls; replies are converted back into joint states so the RViz twin tracks measured (or simulated) encoder pose, and a drive that stops answering, jumps, or lags its target stops the whole arm. Real drives are not moved until they are zeroed, and they stop on command silence, a host pause past the 500 ms heartbeat, or a carrier e-stop once that input is wired. Carrier firmware, framing, and host retry logic are covered by software tests and a virtual-CAN job; a supervised powered five-drive run is not in the merged record.

---

## D. Doctor view and diagnostics

A camera computer captures video, stamps each frame, and sends JPEG images over TCP to the ground station, which shows them as the Doctor View and reports frame rate plus capture-to-arrival delay. A USB webcam or a video file stands in until a procedure camera is chosen; the delay figure does not include time inside the sensor or on the display. The engineering monitor lists live rows for controller input, teleop group and speed, the drive command gate, CAN bus load, per-drive encoder health, and camera delay, and raises an alert when a row leaves OK. Mock controls remain in the panel for UI checks; with teleoperation running the table is fed from the real `/diagnostics` topic.

---

## Caption one-liners (optional callouts)

- Controller mapping: Start enables, B holds, Y cycles groups, D-pad sets 10–100% speed.
- Incision point: pink marker in RViz; 2 mm axis tolerance.
- Collision margin: 10 mm above the table and around the base.
- Bus: USB 1 Mbaud to the carrier, CAN 1 Mbit/s to drives 1–5.
- Not claimed: clinical use, Gemini 2 integration, completed powered-arm trial.
