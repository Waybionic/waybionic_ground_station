"""Minimal Windows XInput reader for embedded-bridge teleop (no ROS)."""

from __future__ import annotations

import ctypes
import time
from dataclasses import dataclass, field

ERROR_SUCCESS = 0
XINPUT_BUTTONS = {
    0x0001: "dpad_up",
    0x0002: "dpad_down",
    0x0004: "dpad_left",
    0x0008: "dpad_right",
    0x0010: "start",
    0x0020: "back",
    0x0100: "lb",
    0x0200: "rb",
    0x1000: "a",
    0x2000: "b",
    0x4000: "x",
    0x8000: "y",
}


class _XInputGamepad(ctypes.Structure):
    _fields_ = [
        ("buttons", ctypes.c_ushort),
        ("left_trigger", ctypes.c_ubyte),
        ("right_trigger", ctypes.c_ubyte),
        ("thumb_lx", ctypes.c_short),
        ("thumb_ly", ctypes.c_short),
        ("thumb_rx", ctypes.c_short),
        ("thumb_ry", ctypes.c_short),
    ]


class _XInputState(ctypes.Structure):
    _fields_ = [("packet_number", ctypes.c_uint32), ("gamepad", _XInputGamepad)]


def _load_xinput():
    for name in ("xinput1_4", "xinput1_3", "xinput9_1_0"):
        try:
            lib = ctypes.WinDLL(name)
        except OSError:
            continue
        get_state = lib.XInputGetState
        get_state.argtypes = [ctypes.c_uint, ctypes.POINTER(_XInputState)]
        get_state.restype = ctypes.c_uint
        return get_state
    return None


@dataclass
class PadState:
    connected: bool = False
    lx: float = 0.0
    ly: float = 0.0
    rx: float = 0.0
    ry: float = 0.0
    buttons: dict[str, bool] = field(default_factory=dict)
    edges: dict[str, bool] = field(default_factory=dict)


class XboxPad:
    """Poll one XInput slot. Stick range -1..1, XInput native signs (right/up +)."""

    def __init__(self, index: int = 0, deadzone: float = 0.22) -> None:
        self.index = index
        self.deadzone = deadzone
        self._get_state = _load_xinput()
        self._prev_buttons: dict[str, bool] = {n: False for n in XINPUT_BUTTONS.values()}
        self._last_scan = 0.0
        self._active: int | None = None

    @staticmethod
    def _stick(value: int, deadzone: float) -> float:
        norm = max(-1.0, min(1.0, value / 32767.0))
        if abs(norm) < deadzone:
            return 0.0
        # Rescale so deadzone edge maps to 0.
        sign = 1.0 if norm > 0 else -1.0
        return sign * (abs(norm) - deadzone) / (1.0 - deadzone)

    def poll(self) -> PadState:
        state = PadState(buttons={n: False for n in XINPUT_BUTTONS.values()})
        if self._get_state is None:
            return state

        now = time.monotonic()
        slots = [self.index]
        if self._active is None and (now - self._last_scan) > 1.0:
            slots = list(range(4))
            self._last_scan = now

        xstate = _XInputState()
        found = None
        for slot in slots:
            if self._get_state(slot, ctypes.byref(xstate)) == ERROR_SUCCESS:
                found = slot
                break

        if found is None:
            self._active = None
            self._prev_buttons = {n: False for n in self._prev_buttons}
            return state

        self._active = found
        pad = xstate.gamepad
        state.connected = True
        state.lx = self._stick(pad.thumb_lx, self.deadzone)
        state.ly = self._stick(pad.thumb_ly, self.deadzone)
        state.rx = self._stick(pad.thumb_rx, self.deadzone)
        state.ry = self._stick(pad.thumb_ry, self.deadzone)
        for bit, name in XINPUT_BUTTONS.items():
            state.buttons[name] = bool(pad.buttons & bit)
            state.edges[name] = state.buttons[name] and not self._prev_buttons.get(name, False)
            self._prev_buttons[name] = state.buttons[name]
        return state


def pad_to_motion_keys(pad: PadState, threshold: float = 0.35) -> set[str]:
    """Map sticks to the same motion key names as the keyboard teleop."""
    wanted: set[str] = set()
    if pad.ly > threshold:
        wanted.add("up")
    elif pad.ly < -threshold:
        wanted.add("down")
    if pad.lx > threshold:
        wanted.add("right")
    elif pad.lx < -threshold:
        wanted.add("left")
    if pad.ry > threshold:
        wanted.add("w")
    elif pad.ry < -threshold:
        wanted.add("s")
    if pad.rx > threshold:
        wanted.add("d")
    elif pad.rx < -threshold:
        wanted.add("a")
    return wanted
