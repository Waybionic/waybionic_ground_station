"""The tool board keeps its CAN replies until its one transmit mailbox takes them."""

from pathlib import Path
import shutil
import subprocess

import pytest

from waybionic_teleop import mks_can

FIRMWARE = Path(__file__).resolve().parents[2] / 'firmware' / 'tool_board'
IDS = range(7, 11)

# Runs the sketch's reply queue against one transmit mailbox that, like the UNO R4's CAN
# library, refuses a frame until the last one has gone out. Commands on stdin:
#   reply ID HEX   queue a reply                pass      offer the queue to the mailbox
#   sent           the mailbox's frame is out   dropped   print how many were dropped
#   capacity       print the queue's size
# pass prints the frame the mailbox took, like candump (007#82018A), or - for none.
HARNESS = r"""
#include <cstdio>
#include <cstring>
#include "reply_queue.h"

int main()
{
  ReplyQueue queue;
  bool busy = false;
  char command[16];
  while (std::scanf("%15s", command) == 1) {
    if (std::strcmp(command, "reply") == 0) {
      unsigned id;
      char hex[32];
      if (std::scanf("%u %31s", &id, hex) != 2 || std::strlen(hex) > 16) {
        return 1;
      }
      uint8_t data[8];
      const size_t length = std::strlen(hex) / 2;
      for (size_t i = 0; i < length; ++i) {
        if (std::sscanf(hex + 2 * i, "%2hhx", &data[i]) != 1) {
          return 1;
        }
      }
      queue.push(static_cast<uint16_t>(id), data, static_cast<uint8_t>(length));
    } else if (std::strcmp(command, "pass") == 0) {
      bool taken = false;
      queue.send([&](const Reply & reply) {
        if (busy) {
          return false;
        }
        std::printf("%03X#", reply.id);
        for (uint8_t i = 0; i < reply.length; ++i) {
          std::printf("%02X", reply.data[i]);
        }
        std::printf("\n");
        busy = taken = true;
        return true;
      });
      if (!taken) {
        std::printf("-\n");
      }
    } else if (std::strcmp(command, "sent") == 0) {
      busy = false;
    } else if (std::strcmp(command, "dropped") == 0) {
      std::printf("%lu\n", static_cast<unsigned long>(queue.dropped()));
    } else {
      std::printf("%u\n", static_cast<unsigned>(ReplyQueue::CAPACITY));
    }
  }
  return 0;
}
"""


@pytest.fixture(scope='module')
def queue(tmp_path_factory):
    compiler = shutil.which('g++') or shutil.which('c++')
    if compiler is None:
        pytest.skip('needs a C++ compiler')
    directory = tmp_path_factory.mktemp('reply_queue')
    (directory / 'harness.cpp').write_text(HARNESS, encoding='utf-8')
    program = directory / 'harness'
    subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror', '-I', str(FIRMWARE),
                    str(directory / 'harness.cpp'), '-o', str(program)], check=True)

    def run(*commands):
        result = subprocess.run([str(program)], input='\n'.join(commands) + '\n',
                                capture_output=True, text=True, check=True)
        return result.stdout.splitlines()
    return run


def queued(can_id, data):
    """Return the command that queues a reply and the line the mailbox prints for it."""
    return f'reply {can_id} {data.hex()}', mks_can.hex_frame(can_id, data)


def test_replies_to_back_to_back_frames_all_go_out_in_order(queue):
    # The host sets up every motor and then zeroes them, frame after frame. Its frames keep the
    # bus busy, so the mailbox takes the first reply and refuses the others until it is free.
    setup = [queued(can_id, mks_can.frame(can_id, code, [1])) for can_id in IDS
             for code in (mks_can.SET_MODE, mks_can.SET_RESPONSE, mks_can.ENABLE,
                          mks_can.SET_HEARTBEAT)]
    zero = [queued(can_id, mks_can.frame(can_id, code, [1]))
            for code in (mks_can.ABSOLUTE_AXIS, mks_can.SET_ZERO) for can_id in IDS]
    replies = setup + zero
    out = queue(*[command for push, _ in replies for command in (push, 'pass')],
                *['sent', 'pass'] * len(replies), 'dropped')
    burst, drained = out[:len(replies)], out[len(replies):-1]
    assert burst == [replies[0][1]] + ['-'] * (len(replies) - 1)
    assert drained == [line for _, line in replies[1:]] + ['-']
    assert out[-1] == '0'


def test_a_full_queue_drops_its_oldest_reply_and_counts_it(queue):
    capacity = int(queue('capacity')[0])
    replies = [queued(7, mks_can.frame(7, mks_can.READ_ENCODER, value.to_bytes(6, 'big')))
               for value in range(capacity + 3)]
    # The mailbox takes the first reply and nothing more, so the others fill the queue.
    out = queue(replies[0][0], 'pass', *[push for push, _ in replies[1:]],
                *['sent', 'pass'] * (capacity + 1), 'dropped')
    assert out[0] == replies[0][1]
    assert out[1:-1] == [line for _, line in replies[3:]] + ['-']
    assert out[-1] == '2'
