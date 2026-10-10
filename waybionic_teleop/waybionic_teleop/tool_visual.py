"""Provisional RViz transforms for Julie's assembled Biomedical clamp."""

import math


STL_TO_METRES = 0.001
# Julie's assembly has an 80 mm diameter base at x=0, y=-53.9 mm. Its bottom is
# z=-10 mm. Centre that base on tool_link and rest its bottom on the flange.
ASSEMBLY_ORIGIN = (0.0, 0.0539, 0.01)
# The bar spans x=-15..16 mm; its circular section is centred here. This is a
# visual pivot estimate from the assembled geometry, not a measured claw axis.
CLAW_PIVOT = (0.0, -0.0539, 0.121306)
PREVIEW_OPEN_ANGLE = 0.35


def claw_pose(side, closure):
    """
    Return the assembled claw's provisional position and x-axis quaternion.

    ``closure=1`` preserves Julie's assembled pose. ``closure=0`` previews each
    claw swung away from that pose. Biomedical must confirm the range and sign.
    """
    if side not in (-1, 1) or not 0.0 <= closure <= 1.0:
        raise ValueError('invalid claw side or closure')
    angle = side * PREVIEW_OPEN_ANGLE * (1.0 - closure)
    cosine, sine = math.cos(angle), math.sin(angle)
    _, pivot_y, pivot_z = CLAW_PIVOT
    rotated_y = cosine * pivot_y - sine * pivot_z
    rotated_z = sine * pivot_y + cosine * pivot_z
    position = (ASSEMBLY_ORIGIN[0],
                ASSEMBLY_ORIGIN[1] + pivot_y - rotated_y,
                ASSEMBLY_ORIGIN[2] + pivot_z - rotated_z)
    quaternion = (math.sin(angle / 2.0), 0.0, 0.0, math.cos(angle / 2.0))
    return position, quaternion
