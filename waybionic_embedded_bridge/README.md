# waybionic_embedded_bridge

Host-side bridge for the **current physical arm bring-up stack** on Windows:

UNO R4 WiFi ASCII serial (115200) → MCP2515 CAN @ 500 kbps → MKS SERVO42D drives
(J1/J3/J4/J5; J2 = SERVO57D unused).

This is **not** the upstream `drive_interface:=slcan` / carrier-board / 1 Mbit path.
Use it when Docker on Windows cannot open COM ports and the arm still speaks the
ASCII teleop firmware in `firmware/waybionic_ascii_teleop/`.

```text
keyboard / ROS
      |
      | USB serial 115200, newline ASCII
      v
Arduino UNO R4  (owns CAN via MCP2515 @ 500 kbps)
      |
      +-- J1 ID1  (42D, 26:1)
      +-- J3 ID3  (42D, 26:1)
      +-- J4/J5 ID4/5 differential wrist (42D, 3:1)
```

## Quick start (two windows)

Docker Desktop running. UNO on `COM3` (typical). Flash
`firmware/waybionic_ascii_teleop/waybionic_ascii_teleop.ino` first.

**Window 1 — RViz twin**
```powershell
powershell -ExecutionPolicy Bypass -File .\waybionic_embedded_bridge\scripts\start_rviz.ps1
```

**Window 2 — teleop**
```powershell
powershell -ExecutionPolicy Bypass -File .\waybionic_embedded_bridge\scripts\start_teleop.ps1
```

Requires host `pip install pyserial`.

Keys / Xbox: `E`/`A` enable, hold arrows+WASD or sticks, release to stop, `Q`/`B` coast,
`M`/`Start` SIM-ALIGN (twin only), `H`/`Y` HOME, `Z`/`X` ZERO, `Esc`/`Back` ESTOP.
Pitch soft ROM ±75° stops **motors** (firmware gates `DIFF_A`/`DIFF_D`), not just the sim.

## What works today

- Hold-to-move teleop with Xbox XInput, release debounce, paced serial TX
- SIM-ALIGN mode to jog the RViz twin until it matches IRL (then bias is kept)
- HOME / SETHOME return to the captured startup encoder pose (F5 absolute)
- Gear-aware default speed/accel (26:1 arm vs 3:1 wrist)
- Closed-loop RViz twin from MKS encoder counts over `ENC` serial lines
- UDP joint states into the Docker ground-station container (`udp_joint_state_receiver`)

## Known issues / open testing

1. **WASD pitch/roll and CW/CCW sense are still being sorted.**
   Current map intends `W/S` = pitch (`DIFF_W`/`DIFF_S`, motors same way) and
   `A/D` = roll (`DIFF_A`/`DIFF_D`, opposite). J1 up/down CW/CCW was flipped once
   for local wiring. Physical motor mounting / sign conventions may still disagree
   with the URDF twin — expect further KEY_DOWN / `differential*` sign fixes.

2. **Coast-DISABLE + live magnetic encoder is implemented but not fully tested yet.**
   `DISABLE` coasts (`F3` on, runtime `83h` Ma=`coastCurrentMa` default **250**;
   `Ma=0` freezes 42D ENC) and encoder poll uses `35h` raw, because hard `F3=0`
   was observed to freeze encoder updates. Hand-moving a joint after `Q` and
   watching RViz follow still needs end-to-end hardware confirmation after flash.
   Use `POWEROFF` for true coil shutdown (encoder may freeze again).

## Related upstream path

When the carrier SLCAN stack is ready on this laptop, prefer
`drive_interface:=slcan` in `waybionic_teleop` instead of this ASCII bridge.
