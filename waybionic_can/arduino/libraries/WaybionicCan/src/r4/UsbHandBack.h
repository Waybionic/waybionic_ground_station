#pragma once

namespace waybionic
{

// For sketches that take the UNO R4 WiFi's USB-C port off the ESP32-S3 bridge (see
// carrier_bridge.ino). Call at the top of loop(): when the host sets 1200 baud and keeps DTR
// high, it disconnects for long enough that the host notices, then restarts in the bootloader
// with the port back on the bridge, ready for an upload. Does nothing on other boards.
//
// The core's own 1200-baud reset (DTR low) also restarts in the bootloader, but it drops the
// connection and resets at once. The bridge, whose pull-up never went off, takes the line
// back too quickly for macOS to see a disconnect, so it keeps the dead device and never finds
// the bridge.
void handBackUsbWhenAsked();

}  // namespace waybionic
