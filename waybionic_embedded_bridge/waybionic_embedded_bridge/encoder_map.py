"""Convert MKS SERVO42D encoder counts to URDF joint angles.

Matches waybionic_teleop arm_drives.yaml for the active joints:
  J1 (42D) base_yaw   gear 26:1 -> joint_1
  J3 (42D) elbow      gear 26:1 -> joint_3
  J4 (42D) wrist_left gear 3:1  -> joint_4, joint_5 factors [1, -1]
  J5 (42D) wrist_right gear 3:1 -> joint_4, joint_5 factors [1,  1]
  J2 (57D) unused

ENC line: ENC <c1> <c3> <c4> <c5>
Works with motors disabled — magnetic encoders still report via CAN 31h.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

COUNTS_PER_REV = 0x4000  # MKS multi-turn encoder
ARM_GEAR = 26.0
WRIST_GEAR = 3.0


@dataclass
class EncoderZero:
    c1: int = 0
    c3: int = 0
    c4: int = 0
    c5: int = 0
    set: bool = False


@dataclass
class EncoderState:
    c1: int = 0
    c3: int = 0
    c4: int = 0
    c5: int = 0
    fresh: bool = False
    zero: EncoderZero = field(default_factory=EncoderZero)
    positions: dict[str, float] = field(default_factory=dict)
    velocities: dict[str, float] = field(default_factory=dict)
    _last_t: float = 0.0
    _last_pos: dict[str, float] = field(default_factory=dict)

    def capture_zero(self) -> None:
        self.zero = EncoderZero(self.c1, self.c3, self.c4, self.c5, set=True)
        joints = counts_to_joints(self.c1, self.c3, self.c4, self.c5, self.zero)
        self.positions = dict(joints)
        self.velocities = {k: 0.0 for k in joints}
        self._last_pos = dict(joints)
        self._last_t = time.monotonic()

    def update(self, c1: int, c3: int, c4: int, c5: int) -> None:
        self.c1, self.c3, self.c4, self.c5 = c1, c3, c4, c5
        if not self.zero.set:
            self.capture_zero()
            self.fresh = True
            return

        now = time.monotonic()
        joints = counts_to_joints(c1, c3, c4, c5, self.zero)
        dt = now - self._last_t if self._last_t > 0 else 0.0
        if dt > 1e-4 and self._last_pos:
            # Finite-difference joint velocity for smoother RViz motion.
            # Clamp dt so a USB hitch doesn't spike velocity.
            dt_c = min(dt, 0.2)
            self.velocities = {
                name: (joints[name] - self._last_pos.get(name, joints[name])) / dt_c
                for name in joints
            }
        else:
            self.velocities = {name: 0.0 for name in joints}
        self.positions = joints
        self._last_pos = dict(joints)
        self._last_t = now
        self.fresh = True


def _shaft_rad(counts: int, gear_ratio: float) -> float:
    return (counts / COUNTS_PER_REV) * (2.0 * math.pi) / gear_ratio


def counts_to_joints(
    c1: int,
    c3: int,
    c4: int,
    c5: int,
    zero: EncoderZero | None = None,
) -> dict[str, float]:
    """Return joint_1..joint_5 radians from absolute encoder counts."""
    if zero and zero.set:
        c1 -= zero.c1
        c3 -= zero.c3
        c4 -= zero.c4
        c5 -= zero.c5

    # Output shaft angles after gearbox (rad).
    out1 = _shaft_rad(c1, ARM_GEAR)
    out3 = _shaft_rad(c3, ARM_GEAR)
    out_left = _shaft_rad(c4, WRIST_GEAR)   # wrist_left:  j4 - j5
    out_right = _shaft_rad(c5, WRIST_GEAR)  # wrist_right: j4 + j5

    # Invert [[1,-1],[1,1]] / 2
    j4 = 0.5 * (out_left + out_right)
    j5 = 0.5 * (-out_left + out_right)

    return {
        "joint_1": out1,
        "joint_2": 0.0,
        "joint_3": out3,
        "joint_4": j4,
        "joint_5": j5,
    }


def parse_enc_line(line: str) -> tuple[int, int, int, int] | None:
    parts = line.strip().split()
    if len(parts) != 5 or parts[0].upper() != "ENC":
        return None
    try:
        return int(parts[1]), int(parts[2]), int(parts[3]), int(parts[4])
    except ValueError:
        return None
