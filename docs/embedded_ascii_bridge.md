# Embedded ASCII bridge (Windows bring-up)

Interim path for driving the physical arm from a Windows laptop while the
ground station runs in Docker (COM ports are not available inside Docker Desktop).

| Piece | Location |
| --- | --- |
| Host Python teleop + UDP twin | [`waybionic_embedded_bridge/`](../waybionic_embedded_bridge/) |
| UNO R4 ASCII + CAN sketch | [`firmware/waybionic_ascii_teleop/`](../firmware/waybionic_ascii_teleop/) |

This does **not** replace `drive_interface:=slcan` / carrier bridge once that stack
is validated on the same hardware.

## Open issues (as of 2026-10-28)

- WASD pitch/roll labeling and J1 CW/CCW sense still need hardware confirmation
  against the URDF twin.
- Coast-`DISABLE` with live magnetic encoder feedback is coded but **not yet
  fully tested** on the arm after the latest flash.
