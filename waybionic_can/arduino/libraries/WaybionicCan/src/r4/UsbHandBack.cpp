#include "r4/UsbHandBack.h"

#if defined(ARDUINO_UNOWIFIR4)

#include <Arduino.h>

void goBootloader();

namespace waybionic
{

static const uint32_t kHandBackBaud = 1200;
static const uint32_t kDisconnectMs = 250;

void handBackUsbWhenAsked()
{
  if (SerialUSB.baud() != kHandBackBaud) {
    return;
  }
  // The host's DTR drop would otherwise run the core's immediate reset from the USB interrupt.
  __disable_irq();
  R_USB_FS0->SYSCFG_b.DPRPU = 0;
  R_BSP_SoftwareDelay(kDisconnectMs, BSP_DELAY_UNITS_MILLISECONDS);
  // Already handed back: stop the bootloader pulsing P408 to do it again while the host is
  // enumerating the bridge.
  R_SYSTEM->PRCR = 0xA502;
  R_SYSTEM->VBTBKR[1] = 0;
  R_SYSTEM->PRCR = 0xA500;
  goBootloader();
}

}  // namespace waybionic

#else

namespace waybionic
{

void handBackUsbWhenAsked() {}

}  // namespace waybionic

#endif
