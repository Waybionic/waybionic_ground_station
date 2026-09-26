"""Map joint positions to drive encoder counts, including coupled joints such as the wrist."""

from dataclasses import dataclass
import math


@dataclass
class Drive:
    """One motor on the bus: CAN ID, gearbox ratio and how much each joint turns it."""

    name: str
    can_id: int
    gear_ratio: float
    factors: dict


def invert(matrix):
    """Invert a square matrix by Gauss-Jordan elimination; raise ValueError if singular."""
    size = len(matrix)
    rows = [[float(value) for value in row] + [float(i == j) for j in range(size)]
            for i, row in enumerate(matrix)]
    for column in range(size):
        pivot = column
        for row in range(column + 1, size):
            if abs(rows[row][column]) > abs(rows[pivot][column]):
                pivot = row
        if abs(rows[pivot][column]) < 1e-12:
            raise ValueError('the drive mixing matrix is singular')
        rows[column], rows[pivot] = rows[pivot], rows[column]
        rows[column] = [value / rows[column][column] for value in rows[column]]
        for row in range(size):
            if row != column and rows[row][column]:
                factor = rows[row][column]
                rows[row] = [a - factor * b for a, b in zip(rows[row], rows[column])]
    return [row[size:] for row in rows]


class DriveMap:
    """Convert between joint positions (rad) and one encoder coordinate per drive."""

    def __init__(self, drives, counts_per_rev):
        self.drives = list(drives)
        self.counts_per_rev = counts_per_rev
        self.joints = list(dict.fromkeys(
            joint for drive in self.drives for joint in drive.factors))
        ids = [drive.can_id for drive in self.drives]
        if len(set(ids)) != len(ids) or not all(1 <= can_id <= 0x7FF for can_id in ids):
            raise ValueError('drive CAN IDs must be unique and between 1 and 2047')
        if any(drive.gear_ratio <= 0 for drive in self.drives):
            raise ValueError('gear ratios must be positive')
        if len(self.joints) != len(self.drives):
            raise ValueError('the drives must move exactly as many joints as there are drives')
        # Row i gives drive i's output angle as a mix of joint angles.
        self.matrix = [[drive.factors.get(joint, 0.0) for joint in self.joints]
                       for drive in self.drives]
        self.inverse = invert(self.matrix)

    def _shaft(self, values):
        return [drive.gear_ratio * sum(factor * values[joint]
                                       for joint, factor in zip(self.joints, row))
                for drive, row in zip(self.drives, self.matrix)]

    def to_counts(self, positions):
        """Return each drive's encoder target for joint positions in radians."""
        return [round(angle / (2 * math.pi) * self.counts_per_rev)
                for angle in self._shaft(positions)]

    def to_rpm(self, velocities):
        """Return each drive's shaft speed in rpm for joint velocities in rad/s."""
        rates = self._shaft({joint: velocities.get(joint, 0.0) for joint in self.joints})
        return [abs(rate) * 60.0 / (2 * math.pi) for rate in rates]

    def to_positions(self, counts):
        """Return joint positions in radians from one encoder reading per drive."""
        outputs = [count / self.counts_per_rev * 2 * math.pi / drive.gear_ratio
                   for count, drive in zip(counts, self.drives)]
        return {joint: sum(weight * output for weight, output in zip(row, outputs))
                for joint, row in zip(self.joints, self.inverse)}


def drive_map_from_parameters(params, counts_per_rev):
    """Build a DriveMap from flat ROS parameter names such as 'wrist_left.joints'."""
    drives = []
    try:
        for name in params['drives']:
            joints = list(params[f'{name}.joints'])
            factors = [float(factor) for factor in params[f'{name}.factors']]
            if len(joints) != len(factors):
                raise ValueError(f'drive {name} needs one factor per joint')
            drives.append(Drive(name, int(params[f'{name}.can_id']),
                                float(params[f'{name}.gear_ratio']), dict(zip(joints, factors))))
    except KeyError as missing:
        raise ValueError(f'missing drive parameter {missing}') from None
    return DriveMap(drives, counts_per_rev)
