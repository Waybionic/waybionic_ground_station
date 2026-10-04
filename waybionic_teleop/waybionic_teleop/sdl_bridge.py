"""
Send a PS5 DualSense controller from Windows to the ROS teleop UDP receiver.

SDL maps USB and Bluetooth controllers to one layout. It maps DualSense
Cross to A, Circle to B, and Triangle to Y, as teleop expects.
Pygame is needed only on the Windows host running this bridge.
"""

import argparse
import os
import socket
import time

from waybionic_teleop.gamepad import AXES, BUTTON, BUTTONS, pack, pong


BUTTON_NAMES = {
    'a': 'A', 'b': 'B', 'x': 'X', 'y': 'Y',
    'back': 'BACK', 'guide': 'GUIDE', 'start': 'START',
    'left_stick': 'LEFTSTICK', 'right_stick': 'RIGHTSTICK',
    'left_bumper': 'LEFTSHOULDER', 'right_bumper': 'RIGHTSHOULDER',
    'dpad_up': 'DPAD_UP', 'dpad_down': 'DPAD_DOWN',
    'dpad_left': 'DPAD_LEFT', 'dpad_right': 'DPAD_RIGHT',
}
AXIS_NAMES = ('LEFTX', 'LEFTY', 'RIGHTX', 'RIGHTY',
              'TRIGGERLEFT', 'TRIGGERRIGHT')
NEUTRAL = ([0.0] * len(AXES), [0] * len(BUTTONS))


def to_joy(controller, sdl):
    """Translate SDL's DualSense layout to the existing /joy packet layout."""
    raw = [controller.get_axis(getattr(sdl, 'CONTROLLER_AXIS_' + name))
           for name in AXIS_NAMES]
    # SDL's stick Y axes grow downwards. The ROS layout is positive left/up.
    axes = [-raw[0] / 32768.0, -raw[1] / 32768.0,
            -raw[2] / 32768.0, -raw[3] / 32768.0,
            -max(0, raw[4]) / 32768.0, -max(0, raw[5]) / 32768.0]
    axes = [max(-1.0, min(1.0, value)) for value in axes]
    buttons = [0] * len(BUTTONS)
    for name, suffix in BUTTON_NAMES.items():
        buttons[BUTTON[name]] = int(bool(controller.get_button(
            getattr(sdl, 'CONTROLLER_BUTTON_' + suffix))))
    return axes, buttons


def neutral(axes, buttons):
    """Require the operator to release the controls after a new connection."""
    return (all(abs(value) <= 0.1 for value in axes[:4])
            and all(abs(value) <= 0.05 for value in axes[4:])
            and not any(buttons))


class SDLBridge:
    """Poll one SDL controller and enforce a neutral reset on reconnection."""

    def __init__(self, pygame, sdl, index=None):
        """Keep the SDL API and optional controller index for polling."""
        self.pygame = pygame
        self.sdl = sdl
        self.index = index
        self.controller = None
        self.next_scan = 0.0
        self.ready = False

    def close(self):
        """Release the current SDL controller, if one is open."""
        if self.controller is not None:
            self.controller.quit()
            self.controller = None
        self.ready = False

    def poll(self, now):
        """Return axes, buttons, connection state, and a status message."""
        self.pygame.event.pump()
        if self.controller is not None and not self.controller.attached():
            self.close()
            self.next_scan = now + 1.0
            return *NEUTRAL, False, 'Controller disconnected; teleop disabled'

        message = None
        if self.controller is None and now >= self.next_scan:
            self.next_scan = now + 1.0
            indices = ([self.index] if self.index is not None
                       else range(self.sdl.get_count()))
            for index in indices:
                if (index < self.sdl.get_count()
                        and self.sdl.is_controller(index)):
                    try:
                        self.controller = self.sdl.Controller(index)
                    except (OSError, RuntimeError):
                        continue
                    name = self.sdl.name_forindex(index)
                    message = f'Controller connected: {name}'
                    break
            if self.controller is None:
                message = ('No controller detected; connect a PS5 DualSense '
                           'controller')
        if self.controller is None:
            return *NEUTRAL, False, message

        try:
            axes, buttons = to_joy(self.controller, self.pygame)
        except (OSError, RuntimeError):
            self.close()
            self.next_scan = now + 1.0
            return *NEUTRAL, False, 'Controller disconnected; teleop disabled'
        if not self.ready:
            if neutral(axes, buttons):
                self.ready = True
                message = 'Controller ready; press Options to enable teleop'
            else:
                message = 'Release all controls before enabling teleop'
            return *NEUTRAL, False, message
        return axes, buttons, True, message


def answer_pings(sender):
    """Send the ground station's pings straight back so it can time the link."""
    while True:
        try:
            data, station = sender.recvfrom(64)
        except OSError:
            return
        answer = pong(data)
        if answer is not None:
            try:
                sender.sendto(answer, station)
            except OSError:
                pass


def main():
    """Run the Windows SDL controller bridge until Ctrl+C."""
    parser = argparse.ArgumentParser(
        description='Send a PS5 controller to the arm.')
    parser.add_argument('--host', default='127.0.0.1',
                        help='ground station address')
    parser.add_argument('--port', type=int, default=47300,
                        help='ground station joy_udp_port')
    parser.add_argument('--rate', type=float, default=250.0,
                        help='packets per second')
    parser.add_argument('--index', type=int,
                        help='SDL controller index (default: first)')
    args = parser.parse_args()
    if (args.rate <= 0 or not 1 <= args.port <= 65535
            or (args.index is not None and args.index < 0)):
        parser.error(
            'rate must be positive, port 1..65535, and index nonnegative')

    # SDL must keep updating input while the operator is looking at RViz.
    os.environ.setdefault('SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS', '1')
    try:
        import pygame
        from pygame._sdl2 import controller as sdl
    except ImportError as error:
        raise SystemExit(
            'Install Pygame on Windows: python -m pip install pygame==2.6.1'
        ) from error

    pygame.display.init()
    pygame.display.set_mode((1, 1), pygame.HIDDEN)
    sdl.init()
    bridge = SDLBridge(pygame, sdl, args.index)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sender.setblocking(False)
    sequence, period, last_message, shown = 0, 1.0 / args.rate, None, ''
    print(f'Sending controller state to {args.host}:{args.port} '
          '(Ctrl+C to stop)')
    try:
        while True:
            started = time.monotonic()
            axes, buttons, connected, message = bridge.poll(started)
            if message and message != last_message:
                print('\n' + message)
                last_message = message
            sequence += 1
            try:
                sender.sendto(pack(sequence, axes, buttons, connected),
                              (args.host, args.port))
            except BlockingIOError:
                pass
            answer_pings(sender)
            if connected and sequence % 30 == 0:
                held = ' '.join(name for name, pressed in zip(BUTTONS, buttons)
                                if pressed)
                line = ('axes ' + ' '.join(f'{value:+.2f}' for value in axes)
                        + f'  {held:<40}')
                if line != shown:
                    print('\r' + line, end='', flush=True)
                    shown = line
            time.sleep(max(0.0, period - (time.monotonic() - started)))
    except KeyboardInterrupt:
        pass
    finally:
        bridge.close()
        sender.close()
        sdl.quit()
        pygame.display.quit()


if __name__ == '__main__':
    main()
