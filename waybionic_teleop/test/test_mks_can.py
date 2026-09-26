"""Frames checked against the layout and examples in the MKS SERVO42D/57D CAN manual V1.0.9."""

import pytest

from waybionic_teleop import mks_can


@pytest.mark.parametrize('data, expected', [
    (mks_can.absolute_axis(1, 0x4000, 600, 2), 'F502580200400092'),
    (mks_can.absolute_axis(1, -0x4000, 600, 2), 'F5025802FFC00011'),
    (mks_can.absolute_axis(1, 0x28000, 300, 2), 'F5012C02028000A7'),
    (mks_can.absolute_axis(1, 0x7F8000, 300, 2), 'F5012C027F800024'),
    (mks_can.absolute_axis(1, 0, 0, 4), 'F5000004000000FA'),
    (mks_can.read_encoder(1), '3132'),
    (mks_can.set_mode(1), '820588'),
    (mks_can.enable(1), 'F301F5'),
    (mks_can.set_response(1, respond=True, active=False), '8C01008E'),
    (mks_can.set_heartbeat(1, 500), '98000001F48E'),
])
def test_frames_match_the_manual(data, expected):
    assert data.hex().upper() == expected


def test_encoder_reply_decodes_negative_values():
    code, arguments = mks_can.parse(1, mks_can.frame(1, mks_can.READ_ENCODER,
                                                     bytes.fromhex('FFFFFFFFFFF0')))
    assert code == mks_can.READ_ENCODER
    assert mks_can.encoder_value(arguments) == -16


@pytest.mark.parametrize('can_id, data', [
    (1, bytes.fromhex('3133')),
    (1, b'\x31'),
    (2, mks_can.read_encoder(1)),
])
def test_parse_rejects_bad_checksums_and_lengths(can_id, data):
    with pytest.raises(ValueError):
        mks_can.parse(can_id, data)


@pytest.mark.parametrize('axis, speed, acc', [(0x800000, 100, 2), (0, 3001, 2), (0, 100, 256)])
def test_absolute_axis_rejects_out_of_range_arguments(axis, speed, acc):
    with pytest.raises(ValueError):
        mks_can.absolute_axis(1, axis, speed, acc)


def test_hex_frame_matches_candump():
    assert mks_can.hex_frame(1, mks_can.read_encoder(1)) == '001#3132'
