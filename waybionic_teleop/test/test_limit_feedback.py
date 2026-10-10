"""The RViz label says what stopped a move, in words RViz can show."""

import pytest

from waybionic_teleop.xbox_teleop_node import drive_stop_reason, stop_reason


@pytest.mark.parametrize('blocked, label', [
    (['joint_3'], 'STOPPED-joint_3'),
    (['reach'], 'OUT-OF-REACH'),
    (['forearm_link: table'], 'STOPPED-forearm_link-table'),
    (['joint_2', 'joint_3', 'joint_4'], 'STOPPED-joint_2\nSTOPPED-joint_3'),
])
def test_stop_reasons_are_single_words_per_line(blocked, label):
    assert stop_reason(blocked) == label
    assert ' ' not in stop_reason(blocked)


@pytest.mark.parametrize('reason, label', [
    ('Drive encoder feedback stale', 'DRIVES-STOPPED\nDrive-encoder-feedback-stale'),
    ('Cannot synchronize drives: elbow: axis 9000000 is outside int24',
     'DRIVES-STOPPED\nCannot-synchronize-drives'),
    ('', 'DRIVES-STOPPED'),
])
def test_drive_stops_keep_the_reason_without_details_or_spaces(reason, label):
    assert drive_stop_reason(reason) == label
