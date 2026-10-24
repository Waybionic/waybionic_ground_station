"""Provisional RViz transforms for Julie's assembled Biomedical clamp."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ToolVisual:
    """Unverified mesh scale, flange pose and claw preview from tool_drives.yaml."""

    mesh_scale: float
    assembly_origin: tuple
    assembly_quaternion: tuple
    claw_pivot: tuple
    open_angles: tuple

    @classmethod
    def from_parameters(cls, params):
        scale = float(params['tool_mesh_scale_m_per_unit'])
        origin = tuple(float(value) for value in params['tool_assembly_origin_m'])
        pivot = tuple(float(value) for value in params['tool_claw_pivot_m'])
        quaternion = tuple(float(value) for value in params['tool_assembly_quaternion_xyzw'])
        angles = tuple(float(value) for value in params['tool_preview_open_angles_rad'])
        if (not math.isfinite(scale) or scale <= 0 or
                len(origin) != 3 or len(pivot) != 3 or len(quaternion) != 4 or
                len(angles) != 2 or
                not all(math.isfinite(value) for value in origin + pivot + quaternion) or
                not all(math.isfinite(value) and value != 0 for value in angles)):
            raise ValueError('invalid provisional tool visual calibration')
        norm = math.sqrt(sum(value * value for value in quaternion))
        if not math.isfinite(norm) or norm == 0:
            raise ValueError('invalid tool mounting quaternion')
        return cls(scale, origin, tuple(value / norm for value in quaternion), pivot, angles)


def quaternion_multiply(a, b):
    """Compose xyzw rotations; the first rotation is in tool_link."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def rotate_vector(quaternion, vector):
    """Rotate a local mesh offset into tool_link."""
    x, y, z, w = quaternion
    rotated = quaternion_multiply(
        quaternion_multiply(quaternion, (*vector, 0.0)), (-x, -y, -z, w))
    return rotated[:3]


def claw_pose(side, closure, visual):
    """
    Return the assembled claw's provisional position and x-axis quaternion.

    ``closure=1`` preserves Julie's assembled pose. ``closure=0`` previews each
    claw swung away from that pose. Biomedical must confirm the range and sign.
    """
    if side not in (-1, 1) or not 0.0 <= closure <= 1.0:
        raise ValueError('invalid claw side or closure')
    angle = visual.open_angles[0 if side == -1 else 1] * (1.0 - closure)
    cosine, sine = math.cos(angle), math.sin(angle)
    _, pivot_y, pivot_z = visual.claw_pivot
    rotated_y = cosine * pivot_y - sine * pivot_z
    rotated_z = sine * pivot_y + cosine * pivot_z
    offset = rotate_vector(visual.assembly_quaternion,
                           (0.0, pivot_y - rotated_y, pivot_z - rotated_z))
    position = tuple(origin + delta for origin, delta in zip(visual.assembly_origin, offset))
    quaternion = quaternion_multiply(
        visual.assembly_quaternion,
        (math.sin(angle / 2.0), 0.0, 0.0, math.cos(angle / 2.0)))
    return position, quaternion
