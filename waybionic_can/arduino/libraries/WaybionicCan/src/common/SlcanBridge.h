#pragma once

#include <stddef.h>
#include <stdint.h>

#include "common/Frame.h"
#include "common/ICanTransport.h"

namespace waybionic
{

// Starts the CAN controller at a bitrate; false if it cannot (unsupported rate, or already
// running at a different rate).
class CanStarter
{
public:
  virtual ~CanStarter() = default;
  virtual bool start(uint32_t bitrate) = 0;
};

class SlcanOutput
{
public:
  virtual ~SlcanOutput() = default;
  virtual void write(const char * text) = 0;  // written as-is, no line ending added
};

// What the carrier knows about itself and the bus, for the host's monitor.
struct CarrierStatus
{
  bool estop_wired = false;
  bool estop_pressed = false;
  bool supply_wired = false;
  uint16_t supply_millivolts = 0;
  uint32_t can_errors = 0;     // CAN controller error events
  uint32_t failed_writes = 0;  // frames the controller would not queue
};

// A subset of the Lawicel SLCAN ASCII protocol, enough for python-can's "slcan" interface
// (can/interfaces/slcan.py) with standard 11-bit data frames. Commands end in '\r'; '\n' is
// ignored. Replies are '\r' for OK and '\a' (BELL) for an error.
//
//   ""       empty line (python-can sends one after Sn)      -> \r
//   Sn       bitrate while closed: S4 125k, S5 250k, S6 500k, S8 1M; other n -> BELL
//   O        open: start CAN (default 500 kbit/s) and forward frames both ways
//   C        close: stop forwarding (the controller itself stays started)
//   tIIILDD  send a standard data frame while open: no reply, BELL if refused
//   V / N    version "V0100" / serial "NWB01"
//
// Sent frames are not acknowledged with 'z': the host does not need it, and every line that is
// not a frame ends a non-blocking read on the host early.
//
// Not supported, answered with BELL: L (listen-only: no such mode in the Arduino_CAN API),
// sxxyy (BTR registers), T/R (extended IDs), r (remote frames: Arduino_CAN sends data frames
// only), d/D/b/B (CAN FD), F (status flags), Z (timestamps), and anything else.
//
// Received standard frames are sent to the host as tIIILDD..\r while open. The bridge forwards
// whatever the host sends: it does not check MKS checksums or apply any motion limits.
//
// reportStatus() sends the host a frame from kCarrierStatusId that never goes on the bus:
//   byte 0     sequence, counting status frames
//   byte 1     flags: 0x01 e-stop sensed, 0x02 e-stop pressed, 0x04 supply sensed
//   bytes 2-3  supply voltage in mV, big-endian
//   bytes 4-5  CAN controller error events, big-endian, saturating
//   byte 6     frames the controller would not queue, saturating
//   byte 7     lines refused with BELL, saturating
class SlcanBridge
{
public:
  static constexpr size_t kMaxLine = 32;
  static constexpr uint32_t kDefaultBitrate = 500000;
  static constexpr uint16_t kCarrierStatusId = 0x7F0;

  SlcanBridge(ICanTransport & can, CanStarter & starter, SlcanOutput & out);

  void handleChar(char c);
  // Forward any received CAN frames to the host (drains and drops them while closed).
  void pollCan();
  // Send the host a status frame; nothing while closed.
  void reportStatus(const CarrierStatus & status);

  bool isOpen() const {return open_;}
  uint32_t bitrate() const {return bitrate_;}
  uint32_t refusedLines() const {return refused_;}

  static bool parseTransmit(const char * line, size_t length, Frame & out);
  // Writes "tIIILDD..\r" plus a terminating NUL; out needs at least 23 bytes.
  static void formatFrame(const Frame & frame, char * out, size_t out_size);
  static bool bitrateForCode(char code, uint32_t & out);

private:
  void handleLine(const char * line, size_t length);
  void ok() {out_.write("\r");}
  void error()
  {
    ++refused_;
    out_.write("\a");
  }

  ICanTransport & can_;
  CanStarter & starter_;
  SlcanOutput & out_;
  bool open_ = false;
  uint32_t bitrate_ = kDefaultBitrate;
  uint32_t refused_ = 0;
  uint8_t status_sequence_ = 0;
  char line_[kMaxLine + 1] = {};
  size_t length_ = 0;
  bool overflow_ = false;
};

}  // namespace waybionic
