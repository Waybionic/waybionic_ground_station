"""A straight sideways cut through teleop, synchronized streaming and simulated MKS drives."""

import math
from pathlib import Path

import pytest

from waybionic_teleop import drive_map, mks_can
from waybionic_teleop.gamepad import AXES, BUTTONS
from waybionic_teleop.kinematics import ArmKinematics
from waybionic_teleop.sim_drives import SimulatedServo
from waybionic_teleop.teleop import ArmTeleop, config_from_parameters

URDF = (Path(__file__).resolve().parents[2] / 'waybionic_description' / 'urdf'
        / 'waybionic_arm.urdf').read_text(encoding='utf-8')
LIMITS = {'joint_1': (-math.pi, math.pi), 'joint_2': (-1.5708, 1.5708),
          'joint_3': (-1.5708, 1.5708), 'joint_4': (-1.5708, 1.5708),
          'joint_5': (-1.5708, 1.5708)}
DOWN = {'joint_1': 0.1, 'joint_2': 0.5, 'joint_3': 1.4, 'joint_4': math.pi - 1.9,
        'joint_5': 0.0, 'tool_grip': 0.0}
SUBSTEPS = 10


def sample(*pressed, **axes):
    return [axes.get(name, 0.0) for name in AXES], [int(name in pressed) for name in BUTTONS]


@pytest.fixture
def drives(parameters):
    return parameters('arm_drives.yaml', 'sim_arm_drives')


def arm_map(drives, ratio=None):
    if ratio is not None:
        drives = {**drives, **{f'{name}.gear_ratio': ratio for name in drives['drives']}}
    return drive_map.drive_map_from_parameters(drives, mks_can.COUNTS_PER_REV)


def test_every_drive_gets_the_speed_that_reaches_its_setpoint_in_one_period(drives):
    mapping = arm_map(drives, 30.0)
    counts = mapping.to_counts(DOWN)
    velocities = {'joint_1': 0.2, 'joint_2': -0.05, 'joint_4': 0.1, 'joint_5': 0.3}
    period = 1.0 / drives['rate_hz']
    moves = mapping.synchronized(DOWN, velocities, counts, period, drives['max_rpm'])
    for (axis, rpm), count in zip(moves, counts):
        distance = abs(axis - count) / mks_can.COUNTS_PER_REV
        # Rounding to whole rpm is the only difference between the drives' arrival times.
        assert rpm == max(1, round(distance / period * 60.0))
        assert distance == 0.0 or abs(distance / (rpm / 60.0) - period) <= 0.5 / rpm * period


def test_a_drive_that_fell_behind_is_sped_up(drives):
    mapping = arm_map(drives, 30.0)
    counts = mapping.to_counts(DOWN)
    velocities = {'joint_1': 0.2}
    period = 1.0 / drives['rate_hz']
    on_time = mapping.synchronized(DOWN, velocities, counts, period, drives['max_rpm'])
    behind = mapping.synchronized(DOWN, velocities, [counts[0] - 500, *counts[1:]], period,
                                  drives['max_rpm'])
    assert behind[0][0] == on_time[0][0] and behind[0][1] > on_time[0][1]
    assert behind[1:] == on_time[1:]


def cut(drives, teleop_params, ratio, level, seconds):
    """Hold LB and push left for a sideways cut; return the tip's worst distance from the line."""
    mapping = arm_map(drives, ratio)
    servos = [SimulatedServo(drive.can_id) for drive in mapping.drives]
    for servo, count in zip(servos, mapping.to_counts(DOWN)):
        servo.receive(mks_can.set_mode(servo.can_id))
        servo.receive(mks_can.enable(servo.can_id))
        servo.axis = float(count)
    kinematics = ArmKinematics.from_urdf(URDF)
    teleop = ArmTeleop(config_from_parameters({**teleop_params, 'initial_speed_level': level}),
                       LIMITS, kinematics)
    period = 1.0 / drives['rate_hz']

    def tip():
        return kinematics.forward(mapping.to_positions([servo.axis for servo in servos]))[0]

    def tick(*pressed, **axes):
        measured = mapping.to_positions([servo.axis for servo in servos])
        teleop.update(*sample(*pressed, **axes), measured, period)
        if teleop.targets:
            moves = mapping.synchronized(teleop.targets, teleop.velocities,
                                         [servo.axis for servo in servos], period,
                                         drives['max_rpm'])
            for servo, (axis, rpm) in zip(servos, moves):
                servo.receive(mks_can.absolute_axis(servo.can_id, axis, rpm, drives['acc']))
        for _ in range(SUBSTEPS):
            for servo in servos:
                servo.step(period / SUBSTEPS)
        return tip()

    for pressed in ('start', None, 'y', None, 'y', None):
        tick(*[pressed] if pressed else [])
    assert teleop.active_group.name == 'cartesian'
    start = tip()
    path = [tick('left_bumper', left_x=1.0) for _ in range(round(seconds / period))]
    path += [tick() for _ in range(round(1.0 / period))]
    assert path[-1][1] - start[1] > 0.9 * teleop.linear_speed * seconds
    return max(math.hypot(x - start[0], z - start[2]) for x, y, z in path)


@pytest.mark.parametrize('ratio, level, seconds, bound', [
    (30.0, 0, 3.0, 10e-6), (30.0, 3, 1.5, 25e-6), (1.0, 2, 3.0, 200e-6)])
def test_a_sideways_cut_through_the_drives_stays_on_the_line(
        drives, parameters, ratio, level, seconds, bound):
    teleop_params = parameters('xbox_teleop.yaml', 'xbox_teleop')
    assert cut(drives, teleop_params, ratio, level, seconds) < bound
