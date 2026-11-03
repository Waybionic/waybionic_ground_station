"""
Slow, smooth wandering of the arm's joints, for demo mode (pure logic, no ROS).

Each joint swings sinusoidally about the pose the arm was in when the demo began, at its own
rate, so the arm moves around without repeating itself for a whole loop. The swing grows from
nothing when the demo starts and shrinks back to nothing when it stops, so the arm leaves its
pose gently and returns to exactly that pose. Everything runs on a demo clock that the speed
level can slow down, so slowing the demo slows the start and the return as well.
"""

import copy
import math

# How fast the demo clock may speed up or slow down, per second, so that changing the speed
# level never jerks the arm.
TEMPO_RATE = 0.25
# The most a ramp changes per ramp duration: 1.875 from rest, and up to 2.3 when it is reversed
# part-way. A joint's speed is at most its swing's own plus this share of its size.
RAMP_PEAK = 2.5
# The least share of the way a ramp plans for, so that a goal given before it has moved at all
# is reached in a moment rather than never.
MIN_TRIP = 1e-4


def clamp(value, low, high):
    return min(max(value, low), high)


class Ramp:
    """
    A level that glides between 0 and 1 in ``duration`` seconds.

    It follows a quintic, minimum-jerk path, so its value, rate and acceleration change
    smoothly. A new goal given part-way takes over from the current level, rate and
    acceleration, so reversing never makes a jump in any of them. A trip of only part of the
    way takes the square root of that part of the duration, which is as gentle as the whole
    trip, whatever its length.
    """

    def __init__(self, duration, level=0.0):
        if not 0 < duration < math.inf:
            raise ValueError('a ramp needs a positive, finite duration')
        self.duration = float(duration)
        self.level, self.rate, self.accel = float(level), 0.0, 0.0
        self.goal = self.level
        # How long the current trip takes, and how far along it is.
        self.span = self.elapsed = self.duration
        self.coefficients = (self.level, 0.0, 0.0, 0.0, 0.0, 0.0)

    @property
    def settled(self):
        return self.elapsed >= self.span

    def aim(self, goal):
        """Head for ``goal`` from wherever the level, its rate and its acceleration are now."""
        if goal == self.goal:
            return
        self.goal = float(goal)
        x0, v0, a0 = self.level, self.rate, self.accel
        change = self.goal - x0
        span = self.span = self.duration * math.sqrt(max(abs(change), MIN_TRIP))
        self.coefficients = (
            x0, v0, a0 / 2.0,
            (20.0 * change - 12.0 * v0 * span - 3.0 * a0 * span ** 2) / (2.0 * span ** 3),
            (-30.0 * change + 16.0 * v0 * span + 3.0 * a0 * span ** 2) / (2.0 * span ** 4),
            (12.0 * change - 6.0 * v0 * span - a0 * span ** 2) / (2.0 * span ** 5))
        self.elapsed = 0.0

    def step(self, dt):
        """Move ``dt`` seconds along the path and return the level."""
        if self.settled:
            return self.level
        self.elapsed = min(self.elapsed + dt, self.span)
        if self.settled:
            self.level, self.rate, self.accel = self.goal, 0.0, 0.0
            return self.level
        t = self.elapsed
        c0, c1, c2, c3, c4, c5 = self.coefficients
        level = c0 + t * (c1 + t * (c2 + t * (c3 + t * (c4 + t * c5))))
        self.rate = c1 + t * (2.0 * c2 + t * (3.0 * c3 + t * (4.0 * c4 + t * 5.0 * c5)))
        self.accel = 2.0 * c2 + t * (6.0 * c3 + t * (12.0 * c4 + t * 20.0 * c5))
        # A reversal can overshoot a hair; the swing never goes beyond its size or below nothing.
        self.level = clamp(level, 0.0, 1.0)
        return self.level


class Wander:
    """
    The joints' swing about a centre pose, on a demo clock.

    ``swings`` is {joint: (amplitude rad, cycles, phase rad)}: the joint makes ``cycles`` whole
    swings per ``loop`` seconds, so the whole motion repeats exactly every ``loop`` seconds of
    demo time once it has grown to full size.
    """

    def __init__(self, centre, swings, loop, ramp, tempo=1.0):
        if not 0 < loop < math.inf:
            raise ValueError('the loop must take a positive, finite time')
        self.centre = dict(centre)
        self.swings = {joint: (amplitude, 2.0 * math.pi * cycles / loop, phase)
                       for joint, (amplitude, cycles, phase) in swings.items()}
        self.ramp = Ramp(ramp)
        self.ramp.aim(1.0)
        self.clock = 0.0
        self.tempo = tempo
        self.before = None

    @property
    def finished(self):
        """Whether the swing has shrunk to nothing, so the arm is back at the centre pose."""
        return self.ramp.goal == 0.0 and self.ramp.settled

    def advance(self, dt, tempo):
        """Move ``dt`` seconds on, with the demo clock heading for ``tempo`` times real time."""
        self.before = (self.clock, self.tempo, copy.copy(self.ramp))
        self.tempo += clamp(tempo - self.tempo, -TEMPO_RATE * dt, TEMPO_RATE * dt)
        gone = self.tempo * dt
        self.clock += gone
        self.ramp.step(gone)

    def rollback(self):
        """Undo the last ``advance``, for a step the arm could not take."""
        if self.before is not None:
            self.clock, self.tempo, self.ramp = self.before
            self.before = None

    def positions(self):
        """Return {joint: angle} at the demo clock."""
        size = self.ramp.level
        return {joint: self.centre[joint]
                + size * amplitude * math.sin(omega * self.clock + phase)
                for joint, (amplitude, omega, phase) in self.swings.items()}
