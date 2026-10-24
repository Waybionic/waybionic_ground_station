"""Check the assembled clamp preview stays aligned while the claws move."""

import math

import pytest

from waybionic_teleop.tool_visual import claw_pose, ToolVisual


@pytest.fixture
def visual(parameters):
    return ToolVisual.from_parameters(parameters('tool_drives.yaml', 'xbox_teleop'))


def test_closed_claws_recover_julies_assembled_pose(visual):
    for side in (-1, 1):
        position, orientation = claw_pose(side, 1.0, visual)
        assert position == pytest.approx(visual.assembly_origin)
        assert orientation == pytest.approx(visual.assembly_quaternion)


def test_open_claws_turn_opposite_ways_around_the_same_bar(visual):
    pivot_y, pivot_z = visual.claw_pivot[1:]
    for side in (-1, 1):
        position, orientation = claw_pose(side, 0.0, visual)
        angle = visual.open_angles[0 if side == -1 else 1]
        assert orientation[0] == pytest.approx(math.sin(angle / 2.0))
        cosine, sine = math.cos(angle), math.sin(angle)
        moved_pivot_y = position[1] + cosine * pivot_y - sine * pivot_z
        moved_pivot_z = position[2] + sine * pivot_y + cosine * pivot_z
        assert moved_pivot_y == pytest.approx(visual.assembly_origin[1] + pivot_y)
        assert moved_pivot_z == pytest.approx(visual.assembly_origin[2] + pivot_z)


def test_measured_mount_rotation_and_claw_direction_need_only_yaml(parameters):
    values = parameters('tool_drives.yaml', 'xbox_teleop')
    root = math.sqrt(0.5)
    values['tool_assembly_quaternion_xyzw'] = [0.0, 0.0, root, root]
    values['tool_preview_open_angles_rad'] = [0.2, -0.4]
    visual = ToolVisual.from_parameters(values)
    position, orientation = claw_pose(-1, 0.0, visual)
    assert position != pytest.approx(visual.assembly_origin)
    assert orientation != pytest.approx(visual.assembly_quaternion)
    position, orientation = claw_pose(-1, 1.0, visual)
    assert position == pytest.approx(visual.assembly_origin)
    assert orientation == pytest.approx(visual.assembly_quaternion)
