"""Check the assembled clamp preview stays aligned while the claws move."""

import math

import pytest

from waybionic_teleop.tool_visual import (
    ASSEMBLY_ORIGIN, CLAW_PIVOT, claw_pose, PREVIEW_OPEN_ANGLE)


def test_closed_claws_recover_julies_assembled_pose():
    for side in (-1, 1):
        position, orientation = claw_pose(side, 1.0)
        assert position == pytest.approx(ASSEMBLY_ORIGIN)
        assert orientation == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_open_claws_turn_opposite_ways_around_the_same_bar():
    pivot_y, pivot_z = CLAW_PIVOT[1:]
    for side in (-1, 1):
        position, orientation = claw_pose(side, 0.0)
        angle = side * PREVIEW_OPEN_ANGLE
        assert orientation[0] == pytest.approx(math.sin(angle / 2.0))
        cosine, sine = math.cos(angle), math.sin(angle)
        moved_pivot_y = position[1] + cosine * pivot_y - sine * pivot_z
        moved_pivot_z = position[2] + sine * pivot_y + cosine * pivot_z
        assert moved_pivot_y == pytest.approx(ASSEMBLY_ORIGIN[1] + pivot_y)
        assert moved_pivot_z == pytest.approx(ASSEMBLY_ORIGIN[2] + pivot_z)
