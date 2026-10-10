"""The camera bridge's TCP stream: a fixed header, then one JPEG image, for every frame."""

import struct

MAGIC = b'WBCF'
VERSION = 1
# Magic, version, frame number, capture time (ns since the epoch, on the camera computer's
# clock), width, height and the JPEG length.
HEADER = struct.Struct('<4sB3xQqHHI')
MAX_JPEG_BYTES = 16 * 1024 * 1024


def pack(number, stamp_ns, width, height, jpeg):
    """Encode one frame for the stream."""
    if len(jpeg) > MAX_JPEG_BYTES:
        raise ValueError(f'a {len(jpeg)} byte frame is larger than {MAX_JPEG_BYTES} bytes')
    return HEADER.pack(MAGIC, VERSION, number, stamp_ns, width, height, len(jpeg)) + jpeg


def unpack_header(data):
    """Return (number, stamp_ns, width, height, length); raise ValueError if malformed."""
    magic, version, number, stamp_ns, width, height, length = HEADER.unpack(data)
    if magic != MAGIC or version != VERSION:
        raise ValueError('not a version 1 WayBionic camera frame')
    if length > MAX_JPEG_BYTES:
        raise ValueError(f'frame of {length} bytes is larger than {MAX_JPEG_BYTES} bytes')
    return number, stamp_ns, width, height, length


def read_exactly(stream, size):
    """Read size bytes from a socket, or return None if it closes first."""
    chunks, remaining = [], size
    while remaining:
        chunk = stream.recv(min(remaining, 1 << 20))
        if not chunk:
            return None
        chunks.append(chunk)
        remaining -= len(chunk)
    return b''.join(chunks)


def read_frame(stream):
    """Return (number, stamp_ns, width, height, jpeg) for the next frame, or None at the end."""
    header = read_exactly(stream, HEADER.size)
    if header is None:
        return None
    number, stamp_ns, width, height, length = unpack_header(header)
    jpeg = read_exactly(stream, length)
    return None if jpeg is None else (number, stamp_ns, width, height, jpeg)
