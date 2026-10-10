// CAN replies waiting for the transmit mailbox: the UNO R4's CAN library sends standard frames
// from one mailbox and refuses a frame while the last is still going out. Plain C++, so
// waybionic_teleop/test/test_tool_board_replies.py tests it on a computer.

#pragma once

#include <stdint.h>

struct Reply
{
  uint16_t id;
  uint8_t length;
  uint8_t data[8];
};

class ReplyQueue
{
public:
  // Room for the replies to the host's setup, zeroing and first poll of all four motors at once.
  static constexpr uint8_t CAPACITY = 32;

  // Queue a reply; when the queue is full, drop the oldest one and count it.
  void push(uint16_t id, const uint8_t * data, uint8_t length)
  {
    if (count_ == CAPACITY) {
      head_ = (head_ + 1) % CAPACITY;
      --count_;
      ++dropped_;
    }
    Reply & reply = replies_[(head_ + count_) % CAPACITY];
    reply.id = id;
    reply.length = length;
    for (uint8_t i = 0; i < length; ++i) {
      reply.data[i] = data[i];
    }
    ++count_;
  }

  // Offer the replies to write, oldest first, until it refuses one or none are left.
  template<typename Write>
  void send(Write write)
  {
    while (count_ && write(replies_[head_])) {
      head_ = (head_ + 1) % CAPACITY;
      --count_;
    }
  }

  uint32_t dropped() const {return dropped_;}

private:
  Reply replies_[CAPACITY];
  uint8_t head_ = 0;
  uint8_t count_ = 0;
  uint32_t dropped_ = 0;
};
