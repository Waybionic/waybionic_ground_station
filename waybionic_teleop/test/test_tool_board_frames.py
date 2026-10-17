"""The tool board firmware reads and writes MKS frames exactly like mks_can."""

from pathlib import Path
import shutil
import subprocess

import pytest

from waybionic_teleop import mks_can

FIRMWARE = Path(__file__).resolve().parents[2] / 'firmware' / 'tool_board'

# Reads lines such as "parse 7 F5...", "encoder 7 -16" or "status 7 146 1" and prints what the
# firmware's frame code makes of them.
CHECK = r"""
#include <cstdio>
#include <cstring>
#include "mks_frames.h"

static void print(const uint8_t * data, uint8_t length)
{
  for (uint8_t i = 0; i < length; ++i) {
    std::printf("%02X", data[i]);
  }
  std::printf("\n");
}

int main()
{
  char kind[16];
  unsigned id;
  while (std::scanf("%15s %u", kind, &id) == 2) {
    uint8_t out[8];
    if (std::strcmp(kind, "encoder") == 0) {
      long long value;
      if (std::scanf("%lld", &value) != 1) {
        return 1;
      }
      print(out, mks::encoder_reply(id, value, out));
    } else if (std::strcmp(kind, "status") == 0) {
      unsigned code, status;
      if (std::scanf("%u %u", &code, &status) != 2) {
        return 1;
      }
      print(out, mks::status_reply(id, code, status, out));
    } else {
      char hex[32];
      if (std::scanf("%31s", hex) != 1) {
        return 1;
      }
      uint8_t data[8];
      const size_t length = std::strlen(hex) / 2;
      for (size_t i = 0; i < length && i < 8; ++i) {
        if (std::sscanf(hex + 2 * i, "%2hhx", &data[i]) != 1) {
          return 1;
        }
      }
      if (length > 8 || !mks::valid(id, data, length)) {
        std::printf("invalid\n");
      } else if (data[0] == mks::ABSOLUTE_AXIS && length == 8) {
        const mks::Move move = mks::absolute_axis(data + 1);
        std::printf("move %u %u %ld\n", move.rpm, move.acc, static_cast<long>(move.axis));
      } else if (data[0] == mks::SET_HEARTBEAT && length == 6) {
        std::printf("heartbeat %lu\n", static_cast<unsigned long>(mks::u32(data + 1)));
      } else {
        std::printf("code %02X\n", data[0]);
      }
    }
  }
  return 0;
}
"""


@pytest.fixture(scope='module')
def firmware(tmp_path_factory):
    compiler = shutil.which('g++') or shutil.which('c++')
    if compiler is None:
        pytest.skip('needs a C++ compiler')
    directory = tmp_path_factory.mktemp('tool_board')
    (directory / 'check.cpp').write_text(CHECK, encoding='utf-8')
    program = directory / 'check'
    subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror', '-I', str(FIRMWARE),
                    str(directory / 'check.cpp'), '-o', str(program)], check=True)

    def run(*lines):
        result = subprocess.run([str(program)], input='\n'.join(lines) + '\n',
                                capture_output=True, text=True, check=True)
        return result.stdout.splitlines()
    return run


@pytest.mark.parametrize('axis, speed, acc', [
    (0x4000, 600, 2), (-0x4000, 600, 2), (0x7F8000, 300, 2), (-mks_can.MAX_AXIS, 3000, 255),
    (0, 0, 4)])
def test_moves_decode_as_mks_can_encodes_them(firmware, axis, speed, acc):
    data = mks_can.absolute_axis(7, axis, speed, acc)
    assert firmware(f'parse 7 {data.hex()}') == [f'move {speed} {acc} {axis}']


def test_commands_are_recognised_and_bad_frames_rejected(firmware):
    assert firmware(
        f'parse 9 {mks_can.set_heartbeat(9, 500).hex()}',
        f'parse 9 {mks_can.set_zero(9).hex()}',
        f'parse 9 {mks_can.read_encoder(9).hex()}',
        f'parse 8 {mks_can.read_encoder(9).hex()}',
        'parse 9 31',
    ) == ['heartbeat 500', 'code 92', 'code 31', 'invalid', 'invalid']


@pytest.mark.parametrize('value', [0, -16, 123456789, -(2 ** 47)])
def test_encoder_replies_decode_with_mks_can(firmware, value):
    code, arguments = mks_can.parse(10, bytes.fromhex(firmware(f'encoder 10 {value}')[0]))
    assert code == mks_can.READ_ENCODER
    assert mks_can.encoder_value(arguments) == value


@pytest.mark.parametrize('code, status', [
    (mks_can.SET_ZERO, 1), (mks_can.ABSOLUTE_AXIS, 2), (mks_can.ABSOLUTE_AXIS, 3),
    (mks_can.ENABLE, 0)])
def test_status_replies_parse_with_mks_can(firmware, code, status):
    reply = bytes.fromhex(firmware(f'status 7 {code} {status}')[0])
    assert mks_can.parse(7, reply) == (code, bytes([status]))
