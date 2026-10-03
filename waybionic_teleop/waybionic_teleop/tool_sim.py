"""Simulate two claw positions and four calibrated stepper motors."""

from waybionic_teleop.tool_motor_map import (
    ClawCalibration, MotorCalibration, ToolMotorMap)


def tool_map_from_parameters(params):
    """Build a tool map from flattened ROS parameters."""
    try:
        claws = []
        for name in params['claws']:
            motors = tuple(MotorCalibration(
                motor, int(params[f'{motor}.direction']),
                float(params[f'{motor}.steps_per_unit']),
                int(params[f'{motor}.reference_steps']))
                for motor in params[f'{name}.motors'])
            claws.append(ClawCalibration(
                name, float(params[f'{name}.lower']),
                float(params[f'{name}.upper']),
                float(params[f'{name}.reference']),
                float(params[f'{name}.max_speed']), motors))
    except KeyError as missing:
        raise ValueError(f'missing tool parameter {missing}') from None
    return ToolMotorMap(claws)


class SimulatedTool:
    """Follow two claw targets with four speed-limited motor step counts."""

    def __init__(self, mapping):
        """Start all motors at their configured reference count."""
        self.map = mapping
        self.target_positions = {claw.name: claw.reference
                                 for claw in mapping.claws}
        self.targets = mapping.target_steps(self.target_positions)
        self.counts = {motor.name: float(motor.reference_steps)
                       for claw in mapping.claws for motor in claw.motors}
        self.speeds = mapping.speed_steps_per_second(
            {claw.name: claw.max_speed for claw in mapping.claws})

    def command(self, positions):
        """Set both claw targets within calibrated limits."""
        targets = self.map.target_steps(positions)
        self.target_positions = dict(positions)
        self.targets = targets

    def tick(self, dt):
        """Advance each motor by no more than its calibrated step rate."""
        if dt <= 0:
            return
        for name, target in self.targets.items():
            current = self.counts[name]
            distance = target - current
            travel = min(abs(distance), self.speeds[name] * dt)
            self.counts[name] = current + (travel if distance > 0 else -travel)

    def feedback(self):
        """Return claw positions, pair disagreement and motor counts."""
        counts = {name: round(value) for name, value in self.counts.items()}
        positions, disagreement = self.map.feedback(counts)
        return positions, disagreement, counts
