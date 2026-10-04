#include "r4/R4CanTransport.h"

#if defined(ARDUINO_MINIMA) || defined(ARDUINO_UNOWIFIR4)

#include <Arduino.h>
#include <Arduino_CAN.h>

namespace waybionic
{

bool R4CanTransport::isSupportedBitrate(const uint32_t bitrate)
{
  return bitrate == 125000 || bitrate == 250000 || bitrate == 500000 || bitrate == 1000000;
}

bool R4CanTransport::begin(const uint32_t bitrate)
{
  CanBitRate rate;
  switch (bitrate) {
    case 125000: rate = CanBitRate::BR_125k; break;
    case 250000: rate = CanBitRate::BR_250k; break;
    case 500000: rate = CanBitRate::BR_500k; break;
    case 1000000: rate = CanBitRate::BR_1000k; break;
    default: return false;
  }
  if (open_) {
    return bitrate == bitrate_;
  }
  open_ = CAN.begin(rate);
  bitrate_ = open_ ? bitrate : 0;
  bus_load_.reset(bitrate_, millis());
  return open_;
}

bool R4CanTransport::send(const Frame & frame)
{
  if (!open_ || !isValidFrame(frame)) {
    return false;
  }
  const CanMsg msg(arduino::CanStandardId(frame.id), frame.dlc, frame.data);
  // write() returns 1 once the frame is queued in a transmit mailbox, or a negative FSP code.
  const int rc = CAN.write(msg);
  if (rc != 1) {
    ++failed_writes_;
    last_write_error_ = rc;
    return false;
  }
  bus_load_.note(frame, millis());
  return true;
}

bool R4CanTransport::receive(Frame & frame, const uint32_t timeout_ms)
{
  if (!open_) {
    return false;
  }
  const uint32_t start = millis();
  do {
    while (CAN.available() > 0) {
      const CanMsg msg = CAN.read();
      if (msg.isExtendedId()) {
        ++dropped_extended_;
        continue;
      }
      frame = Frame{};
      frame.id = static_cast<uint16_t>(msg.getStandardId());
      frame.dlc = msg.data_length > kMaxCanDataBytes ? kMaxCanDataBytes : msg.data_length;
      for (uint8_t i = 0; i < frame.dlc; ++i) {
        frame.data[i] = msg.data[i];
      }
      bus_load_.note(frame, millis());
      return true;
    }
  } while (millis() - start < timeout_ms);
  return false;
}

bool R4CanTransport::takeError(int & code)
{
  if (!CAN.isError(code)) {
    return false;
  }
  CAN.clearError();
  ++error_events_;
  return true;
}

bool R4CanTransport::takeErrorReport(const uint32_t now_ms, int & code, uint32_t & suppressed)
{
  if (!takeError(code)) {
    return false;
  }
  if (reported_error_ && now_ms - last_report_ms_ < 1000) {
    ++suppressed_errors_;
    return false;
  }
  reported_error_ = true;
  last_report_ms_ = now_ms;
  suppressed = suppressed_errors_;
  suppressed_errors_ = 0;
  return true;
}

}  // namespace waybionic

#endif
