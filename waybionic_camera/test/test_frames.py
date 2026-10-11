"""The camera bridge's frame stream survives the trip through TCP intact."""

import socket

import pytest

from waybionic_camera import frames


def test_frames_round_trip_through_a_socket():
    sender, receiver = socket.socketpair()
    with sender, receiver:
        sender.sendall(frames.pack(1, 1_700_000_000_123_456_789, 1920, 1080, b'\xff\xd8one'))
        sender.sendall(frames.pack(2, 1_700_000_000_156_789_012, 1920, 1080, b'\xff\xd8two!'))
        sender.close()
        assert frames.read_frame(receiver) == (1, 1_700_000_000_123_456_789, 1920, 1080,
                                               b'\xff\xd8one')
        assert frames.read_frame(receiver) == (2, 1_700_000_000_156_789_012, 1920, 1080,
                                               b'\xff\xd8two!')
        assert frames.read_frame(receiver) is None


def test_a_frame_cut_short_ends_the_stream():
    sender, receiver = socket.socketpair()
    with sender, receiver:
        sender.sendall(frames.pack(1, 0, 640, 480, b'0123456789')[:-3])
        sender.close()
        assert frames.read_frame(receiver) is None


def test_bad_headers_are_rejected():
    header = frames.pack(1, 0, 640, 480, b'')
    with pytest.raises(ValueError):
        frames.unpack_header(b'XXXX' + header[4:])
    with pytest.raises(ValueError):
        frames.unpack_header(frames.HEADER.pack(frames.MAGIC, frames.VERSION, 1, 0, 640, 480,
                                                frames.MAX_JPEG_BYTES + 1))
    with pytest.raises(ValueError):
        frames.pack(1, 0, 640, 480, bytes(frames.MAX_JPEG_BYTES + 1))
