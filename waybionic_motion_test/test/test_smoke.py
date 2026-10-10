"""Run static quality checks for the motion-test package."""

from ament_flake8.main import main_with_errors
from ament_pep257.main import main
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """Require the package source to pass flake8."""
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, '\n'.join(
        ['Found %d code style errors / warnings:' % len(errors)] + errors)


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    """Require the package source and tests to pass pep257."""
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found code style errors / warnings'
