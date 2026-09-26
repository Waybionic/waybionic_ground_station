"""Load a node's parameters from the package config files, flattened the way ROS names them."""

from pathlib import Path

import pytest
import yaml

CONFIG = Path(__file__).resolve().parent.parent / 'config'


def flatten(values, prefix=''):
    flat = {}
    for key, value in values.items():
        if isinstance(value, dict):
            flat.update(flatten(value, f'{prefix}{key}.'))
        else:
            flat[f'{prefix}{key}'] = value
    return flat


@pytest.fixture
def parameters():
    def load(file_name, node_name):
        with open(CONFIG / file_name, encoding='utf-8') as stream:
            return flatten(yaml.safe_load(stream)[node_name]['ros__parameters'])
    return load
