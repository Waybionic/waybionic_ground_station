"""The RViz motion modes panel and the teleop node agree on every mode's topic and states."""

from pathlib import Path
import re

import pytest

from waybionic_teleop import chicken_head, demo_mode
from waybionic_teleop.teleop import DEMO_STATES, HEAD_STATES

SOURCE = (Path(__file__).resolve().parents[2] / 'waybionic_rviz_plugins' / 'src'
          / 'mode_model.cpp').read_text(encoding='utf-8')
# The panel's table of each mode, the node's module for it, the message it publishes, and the
# states the node can be in.
MODES = [('chickenHeadSpec', chicken_head, chicken_head.head_message, HEAD_STATES),
         ('demoSpec', demo_mode, demo_mode.demo_message, DEMO_STATES)]


def spec_text(function):
    """Return the part of mode_model.cpp that defines one mode's table."""
    start = SOURCE.index(f'const ModeSpec & {function}()')
    return SOURCE[start:SOURCE.index('return spec;', start)]


@pytest.mark.parametrize('function, contract, message, states', MODES)
def test_the_panel_reads_the_row_and_topic_and_calls_the_service_the_node_offers(
        function, contract, message, states):
    table = spec_text(function)
    strings = re.findall(r'"([^"]*)"', table.split('{\n      {')[0])
    assert contract.ROW == strings[0]
    assert [f'/{contract.TOPIC}', f'/{contract.SERVICE}'] == strings[-2:]


@pytest.mark.parametrize('function, contract, message, states', MODES)
def test_the_panel_knows_every_state_the_node_publishes_in_the_same_order(
        function, contract, message, states):
    names = re.findall(r'\{"(\w+)", "[^"]*", (?:true|false), Tone::\w+\}', spec_text(function))
    assert names == list(states)


@pytest.mark.parametrize('function, contract, message, states', MODES)
def test_every_state_is_published_as_one_row_with_that_state(function, contract, message, states):
    for state in states:
        row = message(state, 'Something to say').status[0]
        assert row.name == contract.ROW and row.message == 'Something to say'
        assert [(item.key, item.value) for item in row.values] == [('state', state)]


def test_the_panel_lists_both_modes_and_only_those():
    assert re.findall(r'static const std::vector<const ModeSpec \*> specs\{([^}]*)\}', SOURCE) == [
        '&chickenHeadSpec(), &demoSpec()']
