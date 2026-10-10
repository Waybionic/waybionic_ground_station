"""
The self-running controller demo: what to press, and when, to show the arm off on its own.

The player sends controller samples to teleop the way an operator would, and reads teleop's
state back from its diagnostics, so each run starts the same way whatever group, speed or pose
the last operator left behind.
"""

from dataclasses import dataclass
import math

from waybionic_teleop.gamepad import AXES, BUTTONS

# Longer than teleop's input timeout, so teleop disables itself however it was left.
SILENCE_S = 1.0
# Edges of the square at 25 mm/s (50% speed): about 41 mm each.
EDGE_S = 1.7
LINES = 'STRAIGHT\nLINES'
PIVOT = 'TIP\nFIXED'

# Segments: seconds, caption, held buttons, stick axes. The upper group lifts the arm into a
# working pose; the Cartesian group then draws a square and a vertical line and tilts the tool
# about its tip.
POSE = [
    (2.0, None, (), {'left_x': 1.0, 'left_y': 1.0, 'right_y': 1.0}),
    (0.7, None, (), {}),
]
CARTESIAN = [
    (1.2, LINES, (), {}),
    (EDGE_S, LINES, ('left_bumper',), {'left_x': 1.0}),
    (0.6, LINES, (), {}),
    (EDGE_S, LINES, ('left_bumper',), {'left_y': 1.0}),
    (0.6, LINES, (), {}),
    (EDGE_S, LINES, ('left_bumper',), {'left_x': -1.0}),
    (0.6, LINES, (), {}),
    (EDGE_S, LINES, ('left_bumper',), {'left_y': -1.0}),
    (0.6, LINES, (), {}),
    (1.6, LINES, ('left_bumper',), {'right_y': 1.0}),
    (0.6, LINES, (), {}),
    (1.6, LINES, ('left_bumper',), {'right_y': -1.0}),
    (1.2, LINES, (), {}),
    (1.0, PIVOT, (), {}),
    (2.0, PIVOT, ('dpad_right',), {'right_x': 0.8}),
    (3.0, PIVOT, ('dpad_left',), {'right_x': -0.8}),
    (1.0, PIVOT, ('dpad_right',), {'right_x': 0.8}),
    (1.2, PIVOT, (), {}),
]


@dataclass
class TeleopState:
    """What the player knows about teleop: from its diagnostics, and the joint states."""

    enabled: bool = None
    group: str = None
    at_home: bool = False


def operator_active(axes, buttons, deadzone):
    """Return True when someone is using the controller: a button held, a stick or trigger out."""
    return any(buttons) or any(math.isfinite(value) and abs(value) > deadzone for value in axes)


class DemoPlayer:
    """Produce one controller sample per tick for the demo, over and over."""

    def __init__(self, rate_hz):
        self.rate = rate_hz
        self.state = TeleopState()
        self.caption = None
        self.steps = None

    def restart(self):
        self.steps, self.caption = self.script(), None

    def step(self, state):
        """Return the next (axes, buttons) sample, or None to send nothing this tick."""
        self.state = state
        if self.steps is None:
            self.restart()
        for _ in range(2):
            try:
                return next(self.steps)
            except StopIteration:
                self.restart()
        return None

    def script(self):
        yield from self.silence(SILENCE_S)
        yield from self.hold(0.5)
        yield from self.press('start')
        if not (yield from self.wait(lambda: self.state.enabled, 2.0)):
            # Teleop is not ready (for example no joint states yet): try again shortly.
            yield from self.hold(2.0)
            return
        # Three presses reach the slowest speed from any level, then two give 50%.
        for button in ('dpad_down',) * 3 + ('dpad_up',) * 2:
            yield from self.press(button)
        yield from self.home()
        if (yield from self.select('upper')):
            yield from self.play(POSE)
            if (yield from self.select('cartesian')):
                yield from self.play(CARTESIAN)
        yield from self.press('b')
        yield from self.hold(2.0)

    def sample(self, buttons=(), axes=None):
        axes = axes or {}
        return [axes.get(name, 0.0) for name in AXES], [int(name in buttons) for name in BUTTONS]

    def ticks(self, seconds):
        return max(1, round(seconds * self.rate))

    def silence(self, seconds):
        for _ in range(self.ticks(seconds)):
            yield None

    def hold(self, seconds, buttons=(), axes=None):
        sample = self.sample(buttons, axes)
        for _ in range(self.ticks(seconds)):
            yield sample

    def press(self, button):
        yield from self.hold(0.2, (button,))
        yield from self.hold(0.3)

    def wait(self, done, seconds):
        for _ in range(self.ticks(seconds)):
            if done():
                return True
            yield self.sample()
        return bool(done())

    def home(self):
        """Hold the home button until the arm is home, then a second longer to settle."""
        sample = self.sample(('a',))
        for _ in range(self.ticks(8.0)):
            if self.state.at_home:
                break
            yield sample
        yield from self.hold(1.0, ('a',))
        yield from self.hold(0.3)

    def select(self, group):
        """Press the group button until teleop reports the group; False if it never does."""
        for _ in range(4):
            if self.state.group == group:
                return True
            yield from self.press('y')
            # Teleop reports its group twice a second.
            yield from self.hold(0.7)
        return self.state.group == group

    def play(self, segments):
        for seconds, caption, buttons, axes in segments:
            self.caption = caption
            yield from self.hold(seconds, buttons, axes)
        self.caption = None
