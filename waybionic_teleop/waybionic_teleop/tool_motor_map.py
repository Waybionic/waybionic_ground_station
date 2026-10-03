"""Map two calibrated claw positions to four opposing motor step targets."""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class MotorCalibration:
    """Motor sign, steps per claw unit, and count at the reference position."""

    name: str
    direction: int
    steps_per_unit: float
    reference_steps: int


@dataclass(frozen=True)
class ClawCalibration:
    """One claw's limits, reference and two opposing motors."""

    name: str
    lower: float
    upper: float
    reference: float
    max_speed: float
    motors: tuple


class ToolMotorMap:
    """Convert claw motion without assuming a tool-board protocol."""

    def __init__(self, claws):
        """Validate two claw calibrations and their motor definitions."""
        self.claws = tuple(claws)
        claw_names = {claw.name for claw in self.claws}
        if len(self.claws) != 2 or len(claw_names) != 2:
            raise ValueError('two differently named claws are required')
        names = []
        for claw in self.claws:
            values = (claw.lower, claw.upper, claw.reference, claw.max_speed)
            if (not all(math.isfinite(value) for value in values)
                    or not claw.lower < claw.upper
                    or not claw.lower <= claw.reference <= claw.upper
                    or claw.max_speed <= 0):
                raise ValueError(f'invalid limits for {claw.name}')
            directions = {motor.direction for motor in claw.motors}
            if len(claw.motors) != 2 or directions != {-1, 1}:
                raise ValueError(f'{claw.name} needs an opposing motor pair')
            for motor in claw.motors:
                if (not motor.name or not math.isfinite(motor.steps_per_unit)
                        or motor.steps_per_unit <= 0
                        or not isinstance(motor.reference_steps, int)):
                    raise ValueError(f'invalid calibration for {motor.name}')
                names.append(motor.name)
        if len(set(names)) != 4:
            raise ValueError('all four motor names must be distinct')

    def target_steps(self, positions):
        """Return four targets after checking both claw limits."""
        self._check_names(positions)
        targets = {}
        for claw in self.claws:
            position = positions[claw.name]
            if (not math.isfinite(position)
                    or not claw.lower <= position <= claw.upper):
                raise ValueError(f'{claw.name} target is outside its limits')
            for motor in claw.motors:
                delta = motor.direction * (position - claw.reference)
                targets[motor.name] = (motor.reference_steps
                                       + round(delta * motor.steps_per_unit))
        return targets

    def speed_steps_per_second(self, speeds):
        """Return four motor speeds after enforcing each claw's limit."""
        self._check_names(speeds)
        result = {}
        for claw in self.claws:
            speed = speeds[claw.name]
            if not math.isfinite(speed) or abs(speed) > claw.max_speed:
                raise ValueError(f'{claw.name} speed is outside its limits')
            for motor in claw.motors:
                result[motor.name] = abs(speed) * motor.steps_per_unit
        return result

    def feedback(self, motor_steps):
        """Return estimated claw positions and paired-motor disagreement."""
        names = {motor.name for claw in self.claws for motor in claw.motors}
        if set(motor_steps) != names:
            raise ValueError('feedback must contain all four motors')
        positions, disagreement = {}, {}
        for claw in self.claws:
            estimates = []
            for motor in claw.motors:
                count = motor_steps[motor.name]
                if not isinstance(count, int):
                    raise ValueError(f'{motor.name} feedback must be steps')
                offset = count - motor.reference_steps
                estimates.append(claw.reference + motor.direction * offset
                                 / motor.steps_per_unit)
            positions[claw.name] = sum(estimates) / 2.0
            disagreement[claw.name] = abs(estimates[0] - estimates[1])
        return positions, disagreement

    def _check_names(self, values):
        if set(values) != {claw.name for claw in self.claws}:
            raise ValueError('both named claw values are required')
