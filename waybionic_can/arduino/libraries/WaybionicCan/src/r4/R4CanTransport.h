#pragma once

// The only file that talks to the UNO R4's CAN controller. It uses the Arduino_CAN library
// bundled with the arduino:renesas_uno core (libraries/Arduino_CAN, R7FA4M1_CAN.h), which
// exposes the global `CAN` object on PIN_CAN0_TX / PIN_CAN0_RX. An external transceiver such
// as the TJA1051T turns those logic pins into CAN_H / CAN_L. No MCP2515 is involved.

#if defined(ARDUINO_MINIMA) || defined(ARDUINO_UNOWIFIR4)

#include <stdint.h>

#include "common/BusLoad.h"
#include "common/ICanTransport.h"

namespace waybionic
{

class R4CanTransport : public ICanTransport
{
public:
  // Accepts only the CanBitRate values of the Arduino HardwareCAN API: 125000, 250000,
  // 500000 and 1000000. Returns false if the bitrate is not one of them or CAN.begin fails.
  // The controller is started once: Arduino_CAN's begin() registers its interrupts with
  // IRQManager on every call and a restart is not known to be safe, so a later call with a
  // different bitrate returns false (reset the board instead) and the same bitrate is a no-op.
  bool begin(uint32_t bitrate);
  bool isOpen() const {return open_;}
  uint32_t bitrate() const {return bitrate_;}

  bool send(const Frame & frame) override;
  // Extended (29-bit) frames are dropped and counted: every protocol here uses 11-bit IDs.
  bool receive(Frame & frame, uint32_t timeout_ms) override;

  // Latest controller error event (bus-off, error passive, ...) reported by Arduino_CAN.
  // These are CAN-controller faults, unrelated to the MKS application checksum.
  bool takeError(int & code);
  // Same, but at most once per second; suppressed counts the events folded into this report.
  // A bus nobody ACKs (drive unpowered, wrong bitrate, open bus) raises events continuously.
  bool takeErrorReport(uint32_t now_ms, int & code, uint32_t & suppressed);
  uint32_t droppedExtendedFrames() const {return dropped_extended_;}
  uint32_t failedWrites() const {return failed_writes_;}
  int lastWriteError() const {return last_write_error_;}
  uint32_t errorEvents() const {return error_events_;}
  // Frames queued by send() and frames returned by receive(), since begin().
  BusLoad & busLoad() {return bus_load_;}

  static bool isSupportedBitrate(uint32_t bitrate);

private:
  bool open_ = false;
  uint32_t bitrate_ = 0;
  uint32_t dropped_extended_ = 0;
  uint32_t failed_writes_ = 0;
  int last_write_error_ = 0;
  uint32_t error_events_ = 0;
  BusLoad bus_load_;
  bool reported_error_ = false;
  uint32_t last_report_ms_ = 0;
  uint32_t suppressed_errors_ = 0;
};

}  // namespace waybionic

#endif
