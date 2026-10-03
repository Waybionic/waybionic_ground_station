"""
Forward an Xbox controller on a Windows host to the ground station over UDP.

Docker Desktop and WSL2 cannot see a controller plugged into Windows, so this reads it
through XInput (built into Windows, no ROS or extra packages) and sends the same axes and
buttons that game_controller_node would publish. Run from the waybionic_teleop directory:

    python -m waybionic_teleop.xinput_bridge --host 127.0.0.1 --port 47300
"""

import argparse
import ctypes
import socket
import time

from waybionic_teleop.gamepad import AXES, BUTTON, BUTTONS, pack, unpack_rumble

ERROR_SUCCESS = 0
# game_controller_node rumbles for a second per request unless told to stop; so does this.
RUMBLE_S = 1.0
XINPUT_BUTTONS = {
    0x0001: 'dpad_up', 0x0002: 'dpad_down', 0x0004: 'dpad_left', 0x0008: 'dpad_right',
    0x0010: 'start', 0x0020: 'back', 0x0040: 'left_stick', 0x0080: 'right_stick',
    0x0100: 'left_bumper', 0x0200: 'right_bumper', 0x1000: 'a', 0x2000: 'b', 0x4000: 'x',
    0x8000: 'y',
}


class XInputGamepad(ctypes.Structure):
    """XINPUT_GAMEPAD from the Windows SDK."""

    _fields_ = [('buttons', ctypes.c_ushort), ('left_trigger', ctypes.c_ubyte),
                ('right_trigger', ctypes.c_ubyte), ('thumb_lx', ctypes.c_short),
                ('thumb_ly', ctypes.c_short), ('thumb_rx', ctypes.c_short),
                ('thumb_ry', ctypes.c_short)]


class XInputState(ctypes.Structure):
    """XINPUT_STATE from the Windows SDK."""

    _fields_ = [('packet_number', ctypes.c_uint32), ('gamepad', XInputGamepad)]


class XInputVibration(ctypes.Structure):
    """XINPUT_VIBRATION from the Windows SDK."""

    _fields_ = [('left_motor', ctypes.c_ushort), ('right_motor', ctypes.c_ushort)]


def vibration(intensity):
    """Return both motors at the same speed for an intensity between 0 and 1."""
    speed = round(min(max(intensity, 0.0), 1.0) * 65535)
    return XInputVibration(left_motor=speed, right_motor=speed)


def to_joy(gamepad):
    """Return axes and buttons in game_controller_node order and sign convention."""
    def stick(value):
        return max(-1.0, value / 32767.0)

    # XInput reports right and up as positive; ROS sticks are positive left and up.
    axes = [-stick(gamepad.thumb_lx), stick(gamepad.thumb_ly), -stick(gamepad.thumb_rx),
            stick(gamepad.thumb_ry), -gamepad.left_trigger / 255.0,
            -gamepad.right_trigger / 255.0]
    buttons = [0] * len(BUTTONS)
    for bit, name in XINPUT_BUTTONS.items():
        if gamepad.buttons & bit:
            buttons[BUTTON[name]] = 1
    return axes, buttons


def load_xinput():
    """Return XInputGetState and XInputSetState from the newest XInput DLL on this machine."""
    loader = getattr(ctypes, 'WinDLL', None)
    if loader is None:
        raise SystemExit('XInput is Windows-only; on Linux use joy_source:=device instead.')
    for name in ('xinput1_4', 'xinput1_3', 'xinput9_1_0'):
        try:
            library = loader(name)
        except OSError:
            continue
        get_state, set_state = library.XInputGetState, library.XInputSetState
        get_state.argtypes = [ctypes.c_uint, ctypes.POINTER(XInputState)]
        set_state.argtypes = [ctypes.c_uint, ctypes.POINTER(XInputVibration)]
        get_state.restype = set_state.restype = ctypes.c_uint
        return get_state, set_state
    raise SystemExit('No XInput DLL found.')


def main():
    parser = argparse.ArgumentParser(description='Send an Xbox controller to the arm.')
    parser.add_argument('--host', default='127.0.0.1', help='ground station address')
    parser.add_argument('--port', type=int, default=47300, help='ground station joy_udp_port')
    parser.add_argument('--rate', type=float, default=120.0, help='packets per second')
    parser.add_argument('--slot', type=int, choices=range(4),
                        help='XInput controller slot (default: first connected)')
    args = parser.parse_args()
    get_state, set_state = load_xinput()
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sender.setblocking(False)
    state = XInputState()
    slots = [args.slot] if args.slot is not None else list(range(4))
    active, sequence, next_scan, announced, shown = None, 0, 0.0, None, ''
    rumble_until = None
    period = 1.0 / args.rate
    neutral = [0.0] * len(AXES), [0] * len(BUTTONS)
    print(f'Sending controller state to {args.host}:{args.port} (Ctrl+C to stop)')
    try:
        while True:
            started = time.monotonic()
            if active is not None and get_state(active, ctypes.byref(state)) != ERROR_SUCCESS:
                active = None
            if active is None and started >= next_scan:
                # Polling empty XInput slots is slow, so only rescan once a second.
                next_scan = started + 1.0
                active = next((slot for slot in slots
                               if get_state(slot, ctypes.byref(state)) == ERROR_SUCCESS), None)
            if active != announced:
                print(f'\nController connected in slot {active}' if active is not None else
                      '\nNo controller detected; connect an Xbox controller')
                announced = active
            axes, buttons = to_joy(state.gamepad) if active is not None else neutral
            sequence += 1
            try:
                sender.sendto(pack(sequence, axes, buttons, active is not None),
                              (args.host, args.port))
            except BlockingIOError:
                pass
            # The ground station answers on the same socket when the arm stops at a limit.
            while True:
                try:
                    intensity = unpack_rumble(sender.recvfrom(64)[0])
                except ValueError:
                    continue
                except OSError:
                    break
                if active is not None:
                    set_state(active, ctypes.byref(vibration(intensity)))
                    rumble_until = started + RUMBLE_S if intensity > 0 else None
            if rumble_until is not None and started >= rumble_until:
                if active is not None:
                    set_state(active, ctypes.byref(vibration(0.0)))
                rumble_until = None
            if active is not None and sequence % 30 == 0:
                # The packet count only rises while Windows delivers fresh controller input.
                held = ' '.join(name for name, pressed in zip(BUTTONS, buttons) if pressed)
                line = (f'packet {state.packet_number:<8} axes '
                        + ' '.join(f'{value:+.2f}' for value in axes) + f'  {held:<40}')
                if line != shown:
                    print('\r' + line, end='', flush=True)
                    shown = line
            time.sleep(max(0.0, period - (time.monotonic() - started)))
    except KeyboardInterrupt:
        pass
    finally:
        # Never leave the controller vibrating after the bridge stops.
        if rumble_until is not None and active is not None:
            set_state(active, ctypes.byref(vibration(0.0)))
        sender.close()


if __name__ == '__main__':
    main()
