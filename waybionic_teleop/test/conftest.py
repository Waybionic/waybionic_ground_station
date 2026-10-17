"""Shared test helpers: node parameters from the package config files, and a fake carrier."""

from pathlib import Path

import pytest
import yaml

CONFIG = Path(__file__).resolve().parent.parent / 'config'


class CarrierPort:
    """The carrier's USB port: it answers every line, and BELL to each frame it refuses."""

    def __init__(self, refuse=(), answering=True):
        self.refuse, self.answering = list(refuse), answering
        self.writes, self.inbox, self.closed = [], bytearray(), False

    @property
    def in_waiting(self):
        return len(self.inbox)

    def read(self, size):
        chunk = bytes(self.inbox[:size])
        del self.inbox[:size]
        return chunk

    def write(self, data):
        self.writes.append(bytes(data))
        for line in bytes(data).split(b'\r')[:-1] if self.answering else ():
            if line[:1] != b't':
                # A command, or the empty line after a frame.
                self.inbox += b'\r'
            elif line in self.refuse:
                # Refused once each time it is listed.
                self.refuse.remove(line)
                self.inbox += b'\a'
        return len(data)

    def close(self):
        self.closed = True


@pytest.fixture
def carrier_port():
    return CarrierPort


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
