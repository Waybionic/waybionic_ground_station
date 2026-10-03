#pragma once

#include <stddef.h>
#include <stdint.h>

#include "common/Frame.h"
#include "common/ICanTransport.h"
#include "common/MksFrame.h"

namespace waybionic
{

class ConsoleOutput
{
public:
  virtual ~ConsoleOutput() = default;
  virtual void line(const char * text) = 0;
};

// Bench safety policy for the first powered tests. These are team limits, not protocol: the
// mks_can.py limits (axis +/-0x7FFFFF, 0-3000 rpm, acc 0-255) are always enforced on top by the
// frame builders. Change them only with Yassin or Mujtaba, then rebuild.
struct MotionPolicy
{
  uint16_t max_rpm = 60;
  // acc 0 means "no ramp" and 255 the steepest one (sim_drives.py: 1 rpm per (256 - acc) * 50 us).
  uint8_t min_acc = 1;
  uint8_t max_acc = 128;
  int32_t max_step_counts = 0x4000;  // one motor turn from the last encoder reading
  uint32_t encoder_fresh_ms = 10000;
};

// Serial command handler for MKS drives. Every frame it sends is built by MksFrame (ported from
// waybionic_teleop/mks_can.py) and logged as "TX <candump>"; every frame handed to handleFrame
// is logged as "RX <candump>". It never sends anything on its own.
//
// A move to a drive is refused unless, in this session, that drive confirmed F3h enable with
// status 1 and returned a 31h encoder reading after the previous move and within
// encoder_fresh_ms; the target must also be within max_step_counts of that reading.
class BringupConsole
{
public:
  static constexpr uint8_t kMaxDrives = 8;

  BringupConsole(ICanTransport & can, ConsoleOutput & out, const MotionPolicy & policy);

  // Returns false when the first word is not a console command, so a sketch can add its own.
  // The line is tokenised in place.
  bool handleLine(char * line, uint32_t now_ms);
  void handleFrame(const Frame & frame, uint32_t now_ms);

  void printHelp();
  void printPolicy();
  void printDrives(uint32_t now_ms);

  bool enabledConfirmed(uint16_t can_id) const;
  bool encoderFresh(uint16_t can_id, uint32_t now_ms) const;
  const MotionPolicy & policy() const {return policy_;}

  // Shared helpers, also used by the sketches.
  static bool firstWordIs(const char * line, const char * word);
  static bool parseInteger(const char * text, int32_t & out);  // decimal or 0x hex, with sign
  static void formatInt64(int64_t value, char * out, size_t out_size);

private:
  struct Drive
  {
    uint16_t id = 0;
    bool enable_pending = false;
    bool enable_requested = false;
    bool enabled = false;
    bool encoder_valid = false;
    int64_t encoder = 0;
    uint32_t encoder_ms = 0;
  };

  Drive * find(uint16_t can_id);
  const Drive * find(uint16_t can_id) const;
  Drive * findOrAdd(uint16_t can_id);

  bool parseDriveId(const char * text, uint16_t & out);
  bool transmit(const Frame & frame, uint32_t now_ms, const char * note);
  void emit(const char * format, ...);

  void cmdRead(char ** args, uint8_t count, uint32_t now_ms);
  void cmdEnable(char ** args, uint8_t count, bool on, uint32_t now_ms);
  void cmdMove(char ** args, uint8_t count, uint32_t now_ms);
  void cmdStop(char ** args, uint8_t count, uint32_t now_ms);
  void cmdMode(char ** args, uint8_t count, uint32_t now_ms);
  void cmdResponse(char ** args, uint8_t count, uint32_t now_ms);
  void cmdHeartbeat(char ** args, uint8_t count, uint32_t now_ms);
  void cmdRaw(char ** args, uint8_t count, uint32_t now_ms);
  void cmdChecksum(char ** args, uint8_t count);

  ICanTransport & can_;
  ConsoleOutput & out_;
  MotionPolicy policy_;
  Drive drives_[kMaxDrives];
  char buffer_[192];
};

}  // namespace waybionic
