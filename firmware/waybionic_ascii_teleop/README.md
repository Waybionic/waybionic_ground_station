# waybionic_ascii_teleop

Arduino UNO R4 WiFi sketch for the current physical-arm bring-up:

- MCP2515 / TJA1050 CAN @ **500 kbps** (8 MHz crystal)
- MKS SERVO42D on IDs **1, 3, 4, 5** (J2 / SERVO57D unused)
- USB CDC ASCII @ **115200** (`ENABLE`, `J1_CW`, `DIFF_W`, `ENC`, …)

Pairs with the host package `waybionic_embedded_bridge/` (Windows teleop + RViz twin).

## Notable firmware behavior

- Streams `ENC c1 c3 c4 c5` from MKS **35h** raw magnetic counts (~25 Hz)
- `DISABLE` = **coast** (keep enabled, runtime `coastCurrentMa=250`) so encoders keep updating (`Ma=0` freezes 42D ENC)
- `POWEROFF` = hard `F3=0` (coils off; encoder may freeze)
- Gear-aware defaults: arm 120 RPM / accel 240 (26:1); wrist 90 RPM / accel 45 (3:1)

See `waybionic_embedded_bridge/README.md` for known direction / testing caveats.
