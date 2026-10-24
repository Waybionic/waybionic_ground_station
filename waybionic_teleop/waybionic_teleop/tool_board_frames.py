"""Encode simulated tool targets in PR #32's MKS-compatible board frames."""

import math

from waybionic_teleop import mks_can


BOARD_MICROSTEPS_PER_REV = 3200
BOARD_MAX_RPM = 60
BOARD_CAN_IDS = {'left_1': 7, 'left_2': 8, 'right_1': 9, 'right_2': 10}


def move_frames(target_steps, speed_steps_per_second, acceleration=200, max_rpm=BOARD_MAX_RPM):
    """Return motor-name to (CAN ID, F5h bytes) using the tool board units."""
    names = set(BOARD_CAN_IDS)
    if set(target_steps) != names or set(speed_steps_per_second) != names:
        raise ValueError('four named motor targets and speeds are required')
    if not isinstance(max_rpm, int) or not 1 <= max_rpm <= BOARD_MAX_RPM:
        raise ValueError('tool board rpm ceiling must be 1-60')
    if not isinstance(acceleration, int) or not 0 <= acceleration <= 255:
        raise ValueError('acceleration must be an MKS byte')
    frames = {}
    for name, can_id in BOARD_CAN_IDS.items():
        steps = target_steps[name]
        speed = speed_steps_per_second[name]
        if not isinstance(steps, int) or not math.isfinite(speed) or speed <= 0:
            raise ValueError(f'invalid target or speed for {name}')
        # The board converts 16384 MKS counts per turn to 3200 microsteps.
        axis = round(steps * mks_can.COUNTS_PER_REV / BOARD_MICROSTEPS_PER_REV)
        rpm = math.floor(speed * 60 / BOARD_MICROSTEPS_PER_REV)
        if not 1 <= rpm <= max_rpm:
            raise ValueError(f'{name} speed cannot be encoded within the board limit')
        frames[name] = (can_id, mks_can.absolute_axis(can_id, axis, rpm, acceleration))
    return frames


def stop_frames():
    """Return four immediate-stop F5h frames for the configured tool IDs."""
    return {name: (can_id, mks_can.absolute_axis(can_id, 0, 0, 0))
            for name, can_id in BOARD_CAN_IDS.items()}
