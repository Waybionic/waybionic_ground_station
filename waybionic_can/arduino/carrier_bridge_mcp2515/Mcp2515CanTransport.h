#pragma once

// The arm's carrier talks to the bus through an MCP2515 module on SPI, wired as in
// manual_arm_console: CS on D10, INT on D2 (unused here, the chip is polled). The module's
// 8 MHz crystal cannot run 1 Mbit/s, so the bus stays at the drives' factory 500 kbit/s.
//
// AA_MCP2515 resets and configures the chip. Its write() waits up to 20 ms for each frame to
// leave the bus, which would let the two receive buffers overflow while the drives answer, so
// transmit and receive here go straight to the chip's registers without waiting.

#include <stdint.h>

#include <AA_MCP2515.h>
#include <WaybionicCan.h>

namespace waybionic
{

class Mcp2515CanTransport : public ICanTransport
{
public:
  Mcp2515CanTransport(uint8_t cs_pin, int8_t int_pin);

  // Reset the chip and leave it in configuration mode, which never touches the bus. False if
  // the chip does not answer on SPI.
  bool setup();
  // Join the bus at 125000, 250000 or 500000 bit/s; a later call may change the bitrate.
  bool begin(uint32_t bitrate);
  bool isOpen() const {return open_;}
  uint32_t bitrate() const {return bitrate_;}

  bool send(const Frame & frame) override;
  // Extended and remote frames are dropped and counted: every protocol here uses 11-bit IDs.
  bool receive(Frame & frame, uint32_t timeout_ms) override;

  // Consume new controller errors: a receive buffer overflow (a frame was lost), or the chip
  // going error passive or bus-off. Reads the chip at most once a millisecond.
  bool takeError(int & code);
  uint32_t errorEvents() const {return error_events_;}
  // Frames refused because the chip was not on the bus.
  uint32_t failedWrites() const {return failed_writes_;}
  uint32_t droppedFrames() const {return dropped_frames_;}

  static bool isSupportedBitrate(uint32_t bitrate);

private:
  bool setMode(CANController::Mode mode);
  bool readBuffer(uint8_t address, uint8_t flag, Frame & frame);

  CANConfig config_;
  CANController controller_;
  CANIO io_;
  bool ready_ = false;
  bool open_ = false;
  uint32_t bitrate_ = 0;
  // After the first buffer is emptied, a frame waiting in the second one is the oldest.
  bool second_buffer_older_ = false;
  uint8_t error_flags_ = 0;
  uint32_t error_checked_ms_ = 0;
  uint32_t error_events_ = 0;
  uint32_t failed_writes_ = 0;
  uint32_t dropped_frames_ = 0;
};

}  // namespace waybionic
