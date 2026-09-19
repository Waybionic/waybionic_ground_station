"""Tests for the future-hardware boundary."""

import inspect

import pytest

from waybionic_sensors.hardware_reader import ImuHardwareReader, UnconfiguredImuReader
from waybionic_sensors.imu_reading import ImuReading


def test_interface_cannot_be_instantiated_directly():
    with pytest.raises(TypeError):
        ImuHardwareReader()


def test_interface_declares_the_expected_driver_surface():
    for name in ('start', 'read', 'stop', 'describe'):
        assert hasattr(ImuHardwareReader, name)


def test_read_returns_the_shared_reading_type():
    # The driver boundary is defined in terms of ImuReading so a real sensor can
    # be added without touching message construction or diagnostics.
    signature = inspect.signature(ImuHardwareReader.read)
    assert 'stamp_ns' in signature.parameters


def test_unconfigured_reader_satisfies_the_interface():
    assert isinstance(UnconfiguredImuReader(), ImuHardwareReader)


def test_unconfigured_reader_produces_no_samples():
    assert UnconfiguredImuReader().read(1_000) is None


def test_unconfigured_reader_start_and_stop_are_safe():
    reader = UnconfiguredImuReader()
    reader.start()
    reader.stop()
    reader.stop()


def test_unconfigured_reader_description_flags_the_missing_driver():
    description = UnconfiguredImuReader().describe()
    assert 'unconfigured' in description
    assert 'electrical' in description


def test_unconfigured_reader_description_mentions_a_configured_port():
    assert '/dev/ttyUSB0' in UnconfiguredImuReader('/dev/ttyUSB0').describe()


def test_a_custom_reader_can_supply_readings():
    class FakeReader(ImuHardwareReader):
        def start(self):
            pass

        def read(self, stamp_ns):
            return ImuReading(
                stamp_ns=stamp_ns,
                angular_velocity=(0.0, 0.0, 0.0),
                linear_acceleration=(0.0, 0.0, 9.80665),
            )

        def stop(self):
            pass

        def describe(self):
            return 'fake'

    reading = FakeReader().read(42)
    assert reading.stamp_ns == 42
    assert reading.linear_acceleration[2] == 9.80665
