"""The RViz label says what stopped a move, in words RViz can show."""

import pytest

from waybionic_teleop.xbox_teleop_node import stop_reason


@pytest.mark.parametrize('blocked, label', [
    (['joint_3'], 'STOPPED-joint_3'),
    (['reach'], 'OUT-OF-REACH'),
    (['forearm_link: table'], 'STOPPED-forearm_link-table'),
    (['joint_2', 'joint_3', 'joint_4'], 'STOPPED-joint_2\nSTOPPED-joint_3'),
])
def test_stop_reasons_are_single_words_per_line(blocked, label):
    assert stop_reason(blocked) == label
    assert ' ' not in stop_reason(blocked)
