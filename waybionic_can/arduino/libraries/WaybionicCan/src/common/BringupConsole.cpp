#include "common/BringupConsole.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "common/Candump.h"

namespace waybionic
{
namespace
{

constexpr uint8_t kMaxArgs = 6;

const char * const kHelp[] = {
  "commands (each sends one frame; nothing is ever sent automatically):",
  "  read <id>                      31h READ_ENCODER (do this first)",
  "  enable <id> | disable <id>     F3h ENABLE 01 / 00",
  "  move <id> <axis> <rpm> <acc>   F5h ABSOLUTE_AXIS (all four required)",
  "  stop <id> [acc]                F5h with speed 0 = stop (default acc 0 = at once)",
  "  estop <id>                     TODO: F7h is not defined in the repo sources",
  "  mode <id>                      82h SET_MODE 05 (SR_vFOC); writes a drive setting",
  "  response <id> <0|1> <0|1>      8Ch SET_RESPONSE respond, active; writes a setting",
  "  heartbeat <id> <ms>            98h SET_HEARTBEAT (0 = off); writes a setting",
  "  raw <ID#HEX>                   send bytes exactly as typed (F3h/F5h refused)",
  "  ck <ID#HEX>                    print the frame with its MKS checksum; sends nothing",
  "  drives | policy | help",
  "ids are 1-2047 (0 = broadcast is refused); numbers may be decimal or 0x hex",
};

const char * const kRunStatus[] = {"run failed", "running", "run complete", "stopped at end limit"};

uint8_t tokenize(char * line, char ** words, uint8_t max_words)
{
  uint8_t count = 0;
  char * p = line;
  while (*p != '\0') {
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') {
      *p++ = '\0';
    }
    if (*p == '\0') {
      break;
    }
    if (count == max_words) {
      return static_cast<uint8_t>(max_words + 1);  // too many words
    }
    words[count++] = p;
    while (*p != '\0' && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') {
      ++p;
    }
  }
  return count;
}

}  // namespace

BringupConsole::BringupConsole(
  ICanTransport & can, ConsoleOutput & out, const MotionPolicy & policy)
: can_(can), out_(out), policy_(policy)
{
}

bool BringupConsole::firstWordIs(const char * line, const char * word)
{
  while (*line == ' ' || *line == '\t') {
    ++line;
  }
  const size_t length = strlen(word);
  if (strncmp(line, word, length) != 0) {
    return false;
  }
  const char next = line[length];
  return next == '\0' || next == ' ' || next == '\t' || next == '\r' || next == '\n';
}

bool BringupConsole::parseInteger(const char * text, int32_t & out)
{
  if (text == nullptr || *text == '\0') {
    return false;
  }
  bool negative = false;
  if (*text == '+' || *text == '-') {
    negative = *text == '-';
    ++text;
  }
  uint32_t base = 10;
  if (text[0] == '0' && (text[1] == 'x' || text[1] == 'X')) {
    base = 16;
    text += 2;
  }
  if (*text == '\0') {
    return false;
  }
  int64_t value = 0;
  for (; *text != '\0'; ++text) {
    int digit = -1;
    if (*text >= '0' && *text <= '9') {
      digit = *text - '0';
    } else if (base == 16 && *text >= 'a' && *text <= 'f') {
      digit = *text - 'a' + 10;
    } else if (base == 16 && *text >= 'A' && *text <= 'F') {
      digit = *text - 'A' + 10;
    }
    if (digit < 0) {
      return false;
    }
    value = value * base + digit;
    if (value > 0x80000000LL) {
      return false;
    }
  }
  value = negative ? -value : value;
  if (value > 0x7FFFFFFFLL) {
    return false;
  }
  out = static_cast<int32_t>(value);
  return true;
}

void BringupConsole::formatInt64(const int64_t value, char * out, const size_t out_size)
{
  // Embedded printf builds (newlib-nano) may lack %lld, so format by hand.
  char digits[21];
  uint8_t count = 0;
  uint64_t magnitude = value < 0 ? 0 - static_cast<uint64_t>(value) : static_cast<uint64_t>(value);
  do {
    digits[count++] = static_cast<char>('0' + magnitude % 10);
    magnitude /= 10;
  } while (magnitude != 0 && count < sizeof(digits));
  size_t position = 0;
  if (value < 0 && position + 1 < out_size) {
    out[position++] = '-';
  }
  while (count > 0 && position + 1 < out_size) {
    out[position++] = digits[--count];
  }
  if (out_size > 0) {
    out[position] = '\0';
  }
}

void BringupConsole::emit(const char * format, ...)
{
  va_list args;
  va_start(args, format);
  vsnprintf(buffer_, sizeof(buffer_), format, args);
  va_end(args);
  out_.line(buffer_);
}

BringupConsole::Drive * BringupConsole::find(const uint16_t can_id)
{
  for (Drive & drive : drives_) {
    if (drive.id == can_id && can_id != 0) {
      return &drive;
    }
  }
  return nullptr;
}

const BringupConsole::Drive * BringupConsole::find(const uint16_t can_id) const
{
  return const_cast<BringupConsole *>(this)->find(can_id);
}

BringupConsole::Drive * BringupConsole::findOrAdd(const uint16_t can_id)
{
  Drive * drive = find(can_id);
  if (drive != nullptr) {
    return drive;
  }
  for (Drive & slot : drives_) {
    if (slot.id == 0) {
      slot = Drive{};
      slot.id = can_id;
      return &slot;
    }
  }
  emit("ERR drive table full (%u drives); reset the board to start over",
    static_cast<unsigned>(kMaxDrives));
  return nullptr;
}

bool BringupConsole::enabledConfirmed(const uint16_t can_id) const
{
  const Drive * drive = find(can_id);
  return drive != nullptr && drive->enabled;
}

bool BringupConsole::encoderFresh(const uint16_t can_id, const uint32_t now_ms) const
{
  const Drive * drive = find(can_id);
  return drive != nullptr && drive->encoder_valid &&
         now_ms - drive->encoder_ms <= policy_.encoder_fresh_ms;
}

bool BringupConsole::parseDriveId(const char * text, uint16_t & out)
{
  int32_t id = 0;
  if (!parseInteger(text, id) || id < 1 || id > kMaxStandardCanId) {
    emit("ERR CAN ID must be 1-2047 (0 is broadcast and is refused)");
    return false;
  }
  out = static_cast<uint16_t>(id);
  return true;
}

bool BringupConsole::transmit(const Frame & frame, const uint32_t now_ms, const char * note)
{
  char text[24];
  mks::formatFrame(frame, text, sizeof(text));
  const bool sent = can_.send(frame);
  emit("TX %s t=%lu %s%s", text, static_cast<unsigned long>(now_ms), note,
    sent ? "" : " SEND FAILED (controller did not queue the frame)");
  return sent;
}

bool BringupConsole::handleLine(char * line, const uint32_t now_ms)
{
  char * words[kMaxArgs + 1];
  const uint8_t count = tokenize(line, words, kMaxArgs + 1);
  if (count == 0 || words[0][0] == '#') {
    return true;
  }
  if (count > kMaxArgs + 1) {
    emit("ERR too many arguments");
    return true;
  }
  const char * verb = words[0];
  char ** args = words + 1;
  const uint8_t argc = static_cast<uint8_t>(count - 1);

  if (strcmp(verb, "help") == 0) {
    printHelp();
  } else if (strcmp(verb, "policy") == 0) {
    printPolicy();
  } else if (strcmp(verb, "drives") == 0) {
    printDrives(now_ms);
  } else if (strcmp(verb, "read") == 0) {
    cmdRead(args, argc, now_ms);
  } else if (strcmp(verb, "enable") == 0) {
    cmdEnable(args, argc, true, now_ms);
  } else if (strcmp(verb, "disable") == 0) {
    cmdEnable(args, argc, false, now_ms);
  } else if (strcmp(verb, "move") == 0) {
    cmdMove(args, argc, now_ms);
  } else if (strcmp(verb, "stop") == 0) {
    cmdStop(args, argc, now_ms);
  } else if (strcmp(verb, "estop") == 0) {
    out_.line("ERR estop: F7h emergency stop is NOT implemented and nothing was sent.");
    out_.line("    Its layout is not in mks_can.py, its tests or any MKS manual in the repo.");
    out_.line("    Use the PHYSICAL E-stop. 'stop <id>' (F5h speed 0) and 'disable <id>'");
    out_.line("    are software stops only.");
  } else if (strcmp(verb, "mode") == 0) {
    cmdMode(args, argc, now_ms);
  } else if (strcmp(verb, "response") == 0) {
    cmdResponse(args, argc, now_ms);
  } else if (strcmp(verb, "heartbeat") == 0) {
    cmdHeartbeat(args, argc, now_ms);
  } else if (strcmp(verb, "raw") == 0) {
    cmdRaw(args, argc, now_ms);
  } else if (strcmp(verb, "ck") == 0) {
    cmdChecksum(args, argc);
  } else {
    return false;
  }
  return true;
}

void BringupConsole::cmdRead(char ** args, const uint8_t count, const uint32_t now_ms)
{
  uint16_t id = 0;
  Frame frame;
  if (count != 1) {
    emit("ERR usage: read <id>");
  } else if (parseDriveId(args[0], id) && findOrAdd(id) != nullptr &&
    mks::readEncoder(id, frame))
  {
    transmit(frame, now_ms, "READ_ENCODER request");
  }
}

void BringupConsole::cmdEnable(
  char ** args, const uint8_t count, const bool on, const uint32_t now_ms)
{
  uint16_t id = 0;
  if (count != 1) {
    emit(on ? "ERR usage: enable <id>" : "ERR usage: disable <id>");
    return;
  }
  Drive * drive = nullptr;
  Frame frame;
  if (!parseDriveId(args[0], id) || (drive = findOrAdd(id)) == nullptr ||
    !mks::enable(id, on, frame))
  {
    return;
  }
  if (on) {
    emit("MOTION F3h ENABLE id=%u: the drive will energise and hold the shaft",
      static_cast<unsigned>(id));
  } else {
    // Treat the drive as disabled as soon as we ask, whatever the reply says.
    drive->enabled = false;
  }
  drive->enable_pending = true;
  drive->enable_requested = on;
  transmit(frame, now_ms, on ? "ENABLE on" : "ENABLE off (release shaft)");
}

void BringupConsole::cmdMove(char ** args, const uint8_t count, const uint32_t now_ms)
{
  if (count != 4) {
    emit("ERR usage: move <id> <axis> <rpm> <acc> (all four are required)");
    return;
  }
  uint16_t id = 0;
  int32_t axis = 0, rpm = 0, acc = 0;
  if (!parseDriveId(args[0], id)) {
    return;
  }
  if (!parseInteger(args[1], axis) || !parseInteger(args[2], rpm) ||
    !parseInteger(args[3], acc))
  {
    emit("ERR axis, rpm and acc must be integers");
    return;
  }
  Frame frame;
  if (!mks::absoluteAxis(id, axis, rpm, acc, frame)) {
    emit("ERR outside mks_can.py limits: axis -8388607..8388607, rpm 0-3000, acc 0-255");
    return;
  }
  if (rpm == 0) {
    emit("ERR rpm 0 is the F5h stop command; use 'stop <id> [acc]'");
    return;
  }
  if (rpm > policy_.max_rpm || acc < policy_.min_acc || acc > policy_.max_acc) {
    emit("ERR bench policy: rpm 1-%u and acc %u-%u (see 'policy')",
      static_cast<unsigned>(policy_.max_rpm), static_cast<unsigned>(policy_.min_acc),
      static_cast<unsigned>(policy_.max_acc));
    return;
  }
  Drive * drive = find(id);
  if (drive == nullptr || !drive->enabled) {
    emit("ERR drive %u has not confirmed 'enable %u' (F3h reply status 1) this session",
      static_cast<unsigned>(id), static_cast<unsigned>(id));
    return;
  }
  if (!encoderFresh(id, now_ms)) {
    emit("ERR no fresh encoder reading from drive %u: run 'read %u' first (needed before "
      "every move, max age %lu ms)", static_cast<unsigned>(id), static_cast<unsigned>(id),
      static_cast<unsigned long>(policy_.encoder_fresh_ms));
    return;
  }
  const int64_t delta = static_cast<int64_t>(axis) - drive->encoder;
  const int64_t distance = delta < 0 ? -delta : delta;
  char from[24], step[24];
  formatInt64(drive->encoder, from, sizeof(from));
  formatInt64(delta, step, sizeof(step));
  if (distance > policy_.max_step_counts) {
    emit("ERR bench policy: target is %s counts from the last reading %s; the limit is %ld",
      step, from, static_cast<long>(policy_.max_step_counts));
    return;
  }
  emit("MOTION F5h ABSOLUTE_AXIS id=%u axis=%ld rpm=%ld acc=%ld (from %s, delta %s counts)",
    static_cast<unsigned>(id), static_cast<long>(axis), static_cast<long>(rpm),
    static_cast<long>(acc), from, step);
  // The reading is stale once the motor moves: the next move needs a new 31h reply.
  drive->encoder_valid = false;
  transmit(frame, now_ms, "ABSOLUTE_AXIS");
}

void BringupConsole::cmdStop(char ** args, const uint8_t count, const uint32_t now_ms)
{
  if (count < 1 || count > 2) {
    emit("ERR usage: stop <id> [acc]");
    return;
  }
  uint16_t id = 0;
  int32_t acc = 0;
  if (!parseDriveId(args[0], id)) {
    return;
  }
  if (count == 2 && (!parseInteger(args[1], acc) || acc < 0 || acc > 255)) {
    emit("ERR acc must be 0-255");
    return;
  }
  Frame frame;
  if (!mks::absoluteAxis(id, 0, 0, acc, frame)) {
    return;
  }
  // Allowed in any state: stopping must never be gated.
  emit("MOTION F5h speed-0 stop id=%u acc=%ld (software stop, not F7h, not the E-stop)",
    static_cast<unsigned>(id), static_cast<long>(acc));
  transmit(frame, now_ms, "ABSOLUTE_AXIS speed 0 (stop)");
}

void BringupConsole::cmdMode(char ** args, const uint8_t count, const uint32_t now_ms)
{
  uint16_t id = 0;
  Frame frame;
  if (count != 1) {
    emit("ERR usage: mode <id> (only 05h SR_vFOC is defined in mks_can.py)");
  } else if (parseDriveId(args[0], id) && mks::setMode(id, mks::kModeSrVfoc, frame)) {
    transmit(frame, now_ms, "SET_MODE SR_vFOC");
  }
}

void BringupConsole::cmdResponse(char ** args, const uint8_t count, const uint32_t now_ms)
{
  uint16_t id = 0;
  int32_t respond = 0, active = 0;
  Frame frame;
  if (count != 3 || !parseInteger(args[1], respond) || !parseInteger(args[2], active) ||
    respond < 0 || respond > 1 || active < 0 || active > 1)
  {
    emit("ERR usage: response <id> <0|1> <0|1>");
  } else if (parseDriveId(args[0], id) &&
    mks::setResponse(id, respond != 0, active != 0, frame))
  {
    if (respond == 0) {
      emit("WARN with responses off the drive cannot confirm enable, so 'move' stays locked");
    }
    transmit(frame, now_ms, "SET_RESPONSE");
  }
}

void BringupConsole::cmdHeartbeat(char ** args, const uint8_t count, const uint32_t now_ms)
{
  uint16_t id = 0;
  int32_t ms = 0;
  Frame frame;
  if (count != 2 || !parseInteger(args[1], ms) || ms < 0) {
    emit("ERR usage: heartbeat <id> <ms>");
  } else if (parseDriveId(args[0], id) &&
    mks::setHeartbeat(id, static_cast<uint32_t>(ms), frame))
  {
    transmit(frame, now_ms, "SET_HEARTBEAT");
  }
}

void BringupConsole::cmdRaw(char ** args, const uint8_t count, const uint32_t now_ms)
{
  Frame frame;
  if (count != 1 || !parseCandump(args[0], frame)) {
    emit("ERR usage: raw <ID#HEX>, e.g. raw 001#3132 (1-8 bytes, ID up to 7FF)");
    return;
  }
  if (frame.id == 0 || frame.dlc == 0) {
    emit("ERR raw needs ID 1-7FF (no broadcast) and 1-8 data bytes");
    return;
  }
  if (frame.data[0] == mks::kAbsoluteAxis || frame.data[0] == mks::kEnable) {
    emit("ERR raw F3h/F5h refused: use enable/move/stop so the safety checks apply");
    return;
  }
  emit("WARN raw frame: bytes are sent exactly as typed, with no MKS checksum added or checked");
  transmit(frame, now_ms, "RAW");
}

void BringupConsole::cmdChecksum(char ** args, const uint8_t count)
{
  Frame body;
  if (count != 1 || !parseCandump(args[0], body) || body.dlc == 0 || body.dlc > 7) {
    emit("ERR usage: ck <ID#HEX> with 1-7 bytes (code + arguments, no checksum)");
    return;
  }
  Frame frame;
  mks::buildFrame(
    body.id, body.data[0], body.data + 1, static_cast<uint8_t>(body.dlc - 1), frame);
  char text[24];
  mks::formatFrame(frame, text, sizeof(text));
  emit("CK %s (checksum %02X = (ID + bytes) & 0xFF; not sent)", text,
    static_cast<unsigned>(frame.data[frame.dlc - 1]));
}

void BringupConsole::handleFrame(const Frame & frame, const uint32_t now_ms)
{
  char text[24];
  mks::formatFrame(frame, text, sizeof(text));
  mks::Message message;
  const mks::ParseResult result = mks::parseFrame(frame, message);
  if (result == mks::ParseResult::kBadChecksum) {
    const uint8_t expected = mks::checksum(frame.id, frame.data, static_cast<uint8_t>(frame.dlc - 1));
    emit("RX %s t=%lu MKS checksum BAD (got %02X, expected %02X)", text,
      static_cast<unsigned long>(now_ms), static_cast<unsigned>(frame.data[frame.dlc - 1]),
      static_cast<unsigned>(expected));
    return;
  }
  if (result != mks::ParseResult::kOk) {
    emit("RX %s t=%lu not an MKS frame: %s", text, static_cast<unsigned long>(now_ms),
      mks::parseResultName(result));
    return;
  }

  Drive * drive = find(frame.id);
  int64_t value = 0;
  uint8_t status = 0;
  if (mks::decodeEncoderValue(message, value)) {
    char number[24];
    formatInt64(value, number, sizeof(number));
    emit("RX %s t=%lu READ_ENCODER value=%s counts (0x4000 per turn)", text,
      static_cast<unsigned long>(now_ms), number);
    if (drive != nullptr) {
      drive->encoder_valid = true;
      drive->encoder = value;
      drive->encoder_ms = now_ms;
    }
  } else if (mks::decodeStatus(message, status)) {
    const char * meaning = status == mks::kStatusOk ? "ok" : "failed";
    if (message.code == mks::kAbsoluteAxis && status <= mks::kRunEndLimit) {
      meaning = kRunStatus[status];
    }
    emit("RX %s t=%lu %s status=%u (%s)", text, static_cast<unsigned long>(now_ms),
      mks::codeName(message.code), static_cast<unsigned>(status), meaning);
    if (drive != nullptr && message.code == mks::kEnable && drive->enable_pending) {
      drive->enable_pending = false;
      drive->enabled = status == mks::kStatusOk && drive->enable_requested;
      if (drive->enable_requested && status != mks::kStatusOk) {
        emit("WARN drive %u refused enable", static_cast<unsigned>(frame.id));
      }
    }
  } else {
    emit("RX %s t=%lu %s with %u argument bytes", text, static_cast<unsigned long>(now_ms),
      mks::codeName(message.code), static_cast<unsigned>(message.argument_count));
  }
}

void BringupConsole::printHelp()
{
  for (const char * line : kHelp) {
    out_.line(line);
  }
}

void BringupConsole::printPolicy()
{
  emit("policy: rpm 1-%u, acc %u-%u, step <= %ld counts from a 31h reading at most %lu ms old",
    static_cast<unsigned>(policy_.max_rpm), static_cast<unsigned>(policy_.min_acc),
    static_cast<unsigned>(policy_.max_acc), static_cast<long>(policy_.max_step_counts),
    static_cast<unsigned long>(policy_.encoder_fresh_ms));
  emit("protocol limits (mks_can.py): axis -8388607..8388607, rpm 0-3000, acc 0-255");
}

void BringupConsole::printDrives(const uint32_t now_ms)
{
  bool any = false;
  for (const Drive & drive : drives_) {
    if (drive.id == 0) {
      continue;
    }
    any = true;
    char number[24] = "none";
    if (drive.encoder_valid) {
      formatInt64(drive.encoder, number, sizeof(number));
    }
    emit("drive %u: enabled=%s encoder=%s%s", static_cast<unsigned>(drive.id),
      drive.enabled ? "confirmed" : (drive.enable_pending ? "pending" : "no"), number,
      drive.encoder_valid && !encoderFresh(drive.id, now_ms) ? " (stale)" : "");
  }
  if (!any) {
    emit("no drives addressed yet");
  }
}

}  // namespace waybionic
