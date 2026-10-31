"""Integrate commanded teleop into URDF joint angles for the RViz twin.

Empirical calibration (host teleop vs physical arm, 2026-10-28):
  - J1/J3: when RViz showed ~180 deg, real arm moved ~90 deg  -> arm scale 0.5
  - Wrist/diff: when RViz showed ~180 deg, differential moved ~3 turns (1080 deg)
    -> wrist scale 6.0

These rates are for the ASCII velocity teleop twin only (not ros2_control /
arm_drives.yaml encoder path). Tune ARM_VIZ_DEG_S / WRIST_VIZ_DEG_S if needed.
"""

from __future__ import annotations

import math
import time

# Soft clamps for the twin (wider than URDF so long holds stay visible).
LIMITS = {
    "joint_1": (-2.0 * math.pi, 2.0 * math.pi),
    "joint_2": (-math.pi / 2, math.pi / 2),  # unused on hardware
    "joint_3": (-2.0 * math.pi, 2.0 * math.pi),
    # Wrist pitch soft ROM +/-75 deg each side.
    "joint_4": (-math.radians(75.0), math.radians(75.0)),
    "joint_5": (-6.0 * math.pi, 6.0 * math.pi),
}

# Fallback open-loop rates matched to gear-aware firmware targets:
#   arm motor 120 RPM / 26:1 ≈ 28 deg/s joint
# Pitch: 90 sim ~= 115 IRL -> open-loop rate * 115/90.
ARM_VIZ_DEG_S = 28.0
WRIST_VIZ_DEG_S = 90.0 * (115.0 / 90.0)

DEFAULT_ARM_RATE = math.radians(ARM_VIZ_DEG_S)
DEFAULT_WRIST_RATE = math.radians(WRIST_VIZ_DEG_S)

# Arrows = arm; WASD = wrist pitch (j4) / roll (j5).
# Roll signs flipped so open-loop twin matches IRL (2026-10-31).
KEY_RATES = {
    "up": ("joint_1", -1.0),
    "down": ("joint_1", +1.0),
    "right": ("joint_3", +1.0),
    "left": ("joint_3", -1.0),
    "w": ("joint_4", +1.0),
    "s": ("joint_4", -1.0),
    "a": ("joint_5", +1.0),
    "d": ("joint_5", -1.0),
}


class JointEstimator:
    def __init__(
        self,
        arm_rate: float = DEFAULT_ARM_RATE,
        wrist_rate: float = DEFAULT_WRIST_RATE,
    ) -> None:
        self.arm_rate = arm_rate
        self.wrist_rate = wrist_rate
        self.positions = {
            "joint_1": 0.0,
            "joint_2": 0.0,
            "joint_3": 0.0,
            "joint_4": 0.0,
            "joint_5": 0.0,
        }
        self.velocities = {k: 0.0 for k in self.positions}
        self._held: set[str] = set()
        self._last = time.monotonic()

    def set_held(self, held_keys: set[str] | dict) -> None:
        self._held = set(held_keys)

    def zero(self) -> None:
        for k in self.positions:
            self.positions[k] = 0.0
            self.velocities[k] = 0.0

    def pitch_blocked(self, key: str) -> bool:
        """True when a pitch key would drive joint_4 past +/-75 soft ROM."""
        if key not in ("w", "s"):
            return False
        joint, sign = KEY_RATES[key]
        lo, hi = LIMITS[joint]
        pos = self.positions.get(joint, 0.0)
        return (sign > 0 and pos >= hi - 1e-4) or (sign < 0 and pos <= lo + 1e-4)

    def step(self) -> tuple[list[str], list[float], list[float]]:
        now = time.monotonic()
        dt = max(0.0, min(now - self._last, 0.05))
        self._last = now

        for k in self.velocities:
            self.velocities[k] = 0.0

        for key in self._held:
            if key not in KEY_RATES:
                continue
            if self.pitch_blocked(key):
                continue
            joint, sign = KEY_RATES[key]
            rate = self.wrist_rate if joint in ("joint_4", "joint_5") else self.arm_rate
            self.velocities[joint] += sign * rate

        for joint, vel in self.velocities.items():
            if abs(vel) < 1e-9:
                continue
            lo, hi = LIMITS[joint]
            self.positions[joint] = min(hi, max(lo, self.positions[joint] + vel * dt))

        names = list(self.positions.keys())
        return (
            names,
            [self.positions[n] for n in names],
            [self.velocities[n] for n in names],
        )
