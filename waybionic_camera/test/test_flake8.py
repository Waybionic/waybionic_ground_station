"""Style check for the package sources."""

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, '\n'.join(['Found %d code style errors / warnings:' % len(errors)] + errors)
