"""ASCII serial command set spoken by the current UNO R4 firmware.

This is NOT the Waybionic ground-station SLCAN / carrier_bridge path.
The UNO owns CAN (MCP2515 @ 500 kbps). The PC only sends newline-terminated
ASCII at 115200 baud.

Active joints: J1, J3, J4/J5 differential. J2 is intentionally absent.
"""

from __future__ import annotations

BAUD = 115200

ENABLE = "ENABLE"
DISABLE = "DISABLE"
STOP = "STOP"
ESTOP = "ESTOP"
AUTO = "AUTO"
ZERO = "ZERO"
POS = "POS"
HOME = "HOME"
SETHOME = "SETHOME"

JOINT_CMDS = {
    "j1": {"cw": "J1_CW", "ccw": "J1_CCW", "stop": "J1_STOP"},
    "j3": {"cw": "J3_CW", "ccw": "J3_CCW", "stop": "J3_STOP"},
}

# Differential wrist (matches arm_drives.yaml):
#   pitch (joint_4) = both motors same direction  -> DIFF_W / DIFF_S
#   roll  (joint_5) = motors opposite             -> DIFF_A / DIFF_D
DIFF = {
    "w": "DIFF_W",  # pitch+: J4 CW,  J5 CW
    "s": "DIFF_S",  # pitch-: J4 CCW, J5 CCW
    "a": "DIFF_A",  # roll-:  J4 CCW, J5 CW
    "d": "DIFF_D",  # roll+:  J4 CW,  J5 CCW
    "stop": "DIFF_STOP",
}


def armspeed(speed: int) -> str:
    return f"ARMSPEED {int(speed)}"


def armaccel(accel: int) -> str:
    return f"ARMACCEL {int(accel)}"


def wristspeed(speed: int) -> str:
    return f"WRISTSPEED {int(speed)}"


def wristaccel(accel: int) -> str:
    return f"WRISTACCEL {int(accel)}"


def degrate(deg_per_sec: float) -> str:
    return f"DEGRATE {deg_per_sec:g}"


# Arrows = arm (J1/J3). WASD = wrist pitch/roll via DIFF_* (J4/J5).
# 2026-10-31: J1 CW/CCW flipped so sim and IRL match.
# WASD pitch/roll DIFF swapped (firmware DIFF_W/S were roll on this build).
KEY_DOWN = {
    "up": JOINT_CMDS["j1"]["cw"],
    "down": JOINT_CMDS["j1"]["ccw"],
    "right": JOINT_CMDS["j3"]["cw"],
    "left": JOINT_CMDS["j3"]["ccw"],
    "w": DIFF["a"],  # want pitch+; DIFF_A was pitching IRL
    "s": DIFF["d"],  # want pitch-
    "a": DIFF["w"],  # want roll-; DIFF_W was rolling IRL
    "d": DIFF["s"],  # want roll+
    "escape": ESTOP,
}

KEY_UP = {
    "up": JOINT_CMDS["j1"]["stop"],
    "down": JOINT_CMDS["j1"]["stop"],
    "right": JOINT_CMDS["j3"]["stop"],
    "left": JOINT_CMDS["j3"]["stop"],
    "w": DIFF["stop"],
    "s": DIFF["stop"],
    "a": DIFF["stop"],
    "d": DIFF["stop"],
}


def velocity_to_joint_cmd(joint: str, velocity: float) -> str | None:
    joint = joint.lower()
    if joint not in JOINT_CMDS:
        return None
    if abs(velocity) < 1e-6:
        return JOINT_CMDS[joint]["stop"]
    return JOINT_CMDS[joint]["cw" if velocity > 0 else "ccw"]


def wrist_axis_to_diff(axis: str, value: float, deadband: float = 0.2) -> str:
    aliases = {
        "pitch_pos": "w",
        "pitch_neg": "s",
        "yaw_pos": "d",
        "yaw_neg": "a",
    }
    key = aliases.get(axis.lower(), axis.lower())
    if abs(value) < deadband or key not in ("w", "s", "a", "d"):
        return DIFF["stop"]
    return DIFF[key]
