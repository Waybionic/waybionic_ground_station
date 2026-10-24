"""Simulate two claw positions and four calibrated stepper motors."""

import math

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
        self.stalled_motors = set()

    def set_stalled_motors(self, names):
        """Inject named, simulation-only motor stalls for diagnostic tests."""
        names = set(names)
        unknown = names - self.counts.keys()
        if unknown:
            raise ValueError(f'unknown simulated motor(s): {sorted(unknown)}')
        self.stalled_motors = names

    def command(self, positions):
        """Set both claw targets within calibrated limits."""
        targets = self.map.target_steps(positions)
        self.target_positions = dict(positions)
        self.targets = targets

    def hold(self):
        """Stop at present motor counts instead of finishing an old target."""
        self.targets = {name: round(count)
                        for name, count in self.counts.items()}
        self.counts = {name: float(count)
                       for name, count in self.targets.items()}
        self.target_positions, _ = self.map.feedback(self.targets)

    def tick(self, dt):
        """Advance each motor by no more than its calibrated step rate."""
        if dt <= 0:
            return
        for name, target in self.targets.items():
            current = self.counts[name]
            if name in self.stalled_motors:
                continue
            distance = target - current
            travel = min(abs(distance), self.speeds[name] * dt)
            self.counts[name] = current + (travel if distance > 0 else -travel)

    def feedback(self):
        """Return claw positions, pair disagreement and motor counts."""
        counts = {name: round(value) for name, value in self.counts.items()}
        positions, disagreement = self.map.feedback(counts)
        return positions, disagreement, counts


class ToolSafety:
    """Require live teleop enable and commands before simulated motors move."""

    def __init__(self, simulator, timeout, now):
        """Start disarmed and validate the command/heartbeat timeout."""
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError(
                'tool command timeout must be positive and finite')
        self.sim = simulator
        self.timeout = timeout
        self.enabled = False
        self.requires_release = False
        self.stop_reason = 'teleop disabled'
        self.last_tick = now
        self.last_enable = None
        self.last_command = None

    def stop(self, reason):
        """Hold all motors and require an explicit disable before re-arming."""
        self.sim.hold()
        self.enabled = False
        self.requires_release = True
        self.stop_reason = reason

    def update_enable(self, enabled, now):
        """Track teleop heartbeat; a fresh false-then-true clears a stop."""
        if not enabled:
            was_enabled = self.enabled
            if was_enabled:
                self.stop('teleop disabled')
            self.requires_release = False
            self.last_enable = None
            return was_enabled
        if self.requires_release:
            return False
        if not self.enabled:
            self.enabled = True
            self.last_command = now
            self.stop_reason = ''
        self.last_enable = now
        return False

    def command(self, positions, now):
        """Ignore commands while disabled and timestamp accepted targets."""
        if not self.enabled:
            return False
        self.sim.command(positions)
        self.last_command = now
        return True

    def tick(self, now):
        """Stop if the host, enable heartbeat or tool commands go stale."""
        elapsed = now - self.last_tick
        self.last_tick = now
        if self.enabled:
            if elapsed >= self.timeout:
                self.stop('simulation host paused')
            elif now - self.last_enable >= self.timeout:
                self.stop('teleop enable heartbeat lost')
            elif now - self.last_command >= self.timeout:
                self.stop('tool commands timed out')
            else:
                self.sim.tick(min(max(elapsed, 0.0), 0.1))
        return not self.enabled and self.requires_release
