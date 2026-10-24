"""Check tool commands against PR #32's four MKS-compatible motor IDs."""

import pytest

from waybionic_teleop import mks_can
from waybionic_teleop.tool_board_frames import move_frames, stop_frames


def test_each_motor_has_the_board_id_and_mks_position_encoding():
    """One tenth turn in microsteps is one tenth turn in MKS counts."""
    targets = {'left_1': 320, 'left_2': -320, 'right_1': 0, 'right_2': 0}
    speeds = dict.fromkeys(targets, 80.0)
    frames = move_frames(targets, speeds)
    assert {name: can_id for name, (can_id, _) in frames.items()} == {
        'left_1': 7, 'left_2': 8, 'right_1': 9, 'right_2': 10}
    assert frames['left_1'][1] == mks_can.absolute_axis(7, 1638, 1, 200)
    assert frames['left_2'][1] == mks_can.absolute_axis(8, -1638, 1, 200)
    assert frames['right_1'][1] == mks_can.absolute_axis(9, 0, 1, 200)


def test_speed_below_one_rpm_is_rejected_instead_of_rounded_up():
    """A board frame must not exceed the requested step speed."""
    targets = dict.fromkeys(('left_1', 'left_2', 'right_1', 'right_2'), 0)
    speeds = dict.fromkeys(targets, 16.5)
    with pytest.raises(ValueError, match='cannot be encoded'):
        move_frames(targets, speeds)


def test_stop_frames_use_zero_speed_on_all_four_ids():
    """The tool board interprets zero speed as a stop command."""
    for name, (can_id, data) in stop_frames().items():
        assert name in ('left_1', 'left_2', 'right_1', 'right_2')
        assert data == mks_can.absolute_axis(can_id, 0, 0, 0)
