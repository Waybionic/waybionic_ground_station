#pragma once

// Host stand-in for AA_MCP2515: the same classes and calls, backed by one modeled chip instead of
// SPI. The model keeps the datasheet behavior the transport relies on: CNF registers change only
// in configuration mode, a mode request can take a while, only normal mode joins the bus, a frame
// rolls over into the second receive buffer only with BUKT set, and a frame with nowhere to go
// sets an overflow flag.

#include <stdint.h>
#include <string.h>

#include <vector>

#include "CANRegisters.h"

namespace CANBitrate
{

struct Config
{
  uint8_t cnf1;
  uint8_t cnf2;
  uint8_t cnf3;
};

// AA_MCP2515 1.0.7's values for an 8 MHz crystal.
constexpr Config Config_8MHz_125kbps {0x01, 0xac, 0x03};
constexpr Config Config_8MHz_250kbps {0x00, 0xac, 0x03};
constexpr Config Config_8MHz_500kbps {0x00, 0x91, 0x01};

}  // namespace CANBitrate

struct CANConfig
{
  CANConfig(CANBitrate::Config bitrate_config, uint8_t cs_pin, int8_t int_pin)
  : bitrate(bitrate_config), pin_cs(cs_pin), pin_int(int_pin) {}

  const CANBitrate::Config bitrate;
  const uint8_t pin_cs;
  const int8_t pin_int;
};

constexpr uint8_t kModeNormal = 0x00;
constexpr uint8_t kModeConfig = 0x80;

struct FakeMcp2515
{
  bool present = true;
  // Reads of CANSTAT before a requested mode takes effect.
  int mode_delay = 0;
  uint8_t registers[128] = {};
  // The first transmit buffer, from SIDH to the last data byte, each time it was sent.
  std::vector<std::vector<uint8_t>> transmitted;

  void reset()
  {
    memset(registers, 0, sizeof(registers));
    registers[CANSTAT] = kModeConfig;
    registers[CANCTRL] = kModeConfig;
    pending_ = -1;
  }

  uint8_t mode() const {return registers[CANSTAT] & CANSTAT_OPMOD_MASK;}

  void requestMode(uint8_t mode)
  {
    registers[CANCTRL] = static_cast<uint8_t>((registers[CANCTRL] & ~CANCTRL_REQOP_MASK) | mode);
    pending_ = mode;
    delay_ = mode_delay;
    settle();
  }

  // A standard (or, with sidl_flags, extended or remote) frame from another node. False if the
  // chip was not listening or had nowhere to put it.
  bool arrive(uint16_t id, const std::vector<uint8_t> & data, uint8_t sidl_flags = 0)
  {
    if (mode() != kModeNormal) {
      return false;
    }
    uint8_t buffer[13] = {};
    buffer[0] = static_cast<uint8_t>(id >> 3);
    buffer[1] = static_cast<uint8_t>(((id & 0x07) << 5) | sidl_flags);
    buffer[4] = static_cast<uint8_t>(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
      buffer[5 + i] = data[i];
    }
    if ((registers[CANINTF] & CANINTF_RX0IF) == 0) {
      memcpy(&registers[RXB0SIDH], buffer, sizeof(buffer));
      registers[CANINTF] |= CANINTF_RX0IF;
      return true;
    }
    if ((registers[RXB0CTRL] & RXB0CTRL_BUKT) == 0) {
      registers[EFLG] |= EFLG_RX0OVR;
      return false;
    }
    if ((registers[CANINTF] & CANINTF_RX1IF) == 0) {
      memcpy(&registers[RXB1SIDH], buffer, sizeof(buffer));
      registers[CANINTF] |= CANINTF_RX1IF;
      return true;
    }
    registers[EFLG] |= EFLG_RX1OVR;
    return false;
  }

  // The frame in the first transmit buffer has left the bus.
  void finishTransmit() {registers[TXB0CTRL] &= ~TXBnCTRL_TXREQ;}

  uint8_t readStatus() const
  {
    uint8_t status = 0;
    if (registers[CANINTF] & CANINTF_RX0IF) {
      status |= READSTATUS_RX0IF;
    }
    if (registers[CANINTF] & CANINTF_RX1IF) {
      status |= READSTATUS_RX1IF;
    }
    if (registers[TXB0CTRL] & TXBnCTRL_TXREQ) {
      status |= READSTATUS_TX0REQ;
    }
    return status;
  }

  void read(uint8_t address, uint8_t * data, uint8_t length)
  {
    if (address == CANSTAT) {
      settle();
    }
    memcpy(data, &registers[address], length);
  }

  void write(uint8_t address, const uint8_t * data, uint8_t length)
  {
    for (uint8_t i = 0; i < length; ++i) {
      const uint8_t target = static_cast<uint8_t>(address + i);
      if (target >= CNF3 && target <= CNF1 && mode() != kModeConfig) {
        continue;
      }
      registers[target] = data[i];
    }
  }

  void instruction(uint8_t value)
  {
    if (value != INSTRUCTION_RTS0 || mode() != kModeNormal) {
      return;
    }
    registers[TXB0CTRL] |= TXBnCTRL_TXREQ;
    const uint8_t * start = &registers[TXB0SIDH];
    transmitted.emplace_back(start, start + 5 + (start[4] & 0x0F));
  }

private:
  void settle()
  {
    if (pending_ < 0 || delay_-- > 0) {
      return;
    }
    registers[CANSTAT] = static_cast<uint8_t>(
      (registers[CANSTAT] & ~CANSTAT_OPMOD_MASK) | static_cast<uint8_t>(pending_));
    pending_ = -1;
  }

  int pending_ = -1;
  int delay_ = 0;
};

inline FakeMcp2515 & fakeChip()
{
  static FakeMcp2515 chip;
  return chip;
}

class CANController
{
public:
  enum Result : int8_t { OK = 0, FAIL = -1 };
  enum class Mode : uint8_t { Normal = 0x00, Sleep = 0x20, Loopback = 0x40, ListenOnly = 0x60,
    Config = 0x80 };

  explicit CANController(CANConfig & config)
  : config_(config) {}

  // Like AA_MCP2515: reset, check the chip answers, set the bitrate, request the mode.
  int8_t begin(Mode mode = Mode::Normal)
  {
    FakeMcp2515 & chip = fakeChip();
    if (!chip.present) {
      return FAIL;
    }
    chip.reset();
    const uint8_t cnf[] = {config_.bitrate.cnf3, config_.bitrate.cnf2, config_.bitrate.cnf1};
    chip.write(CNF3, cnf, sizeof(cnf));
    chip.requestMode(static_cast<uint8_t>(mode));
    return OK;
  }

  bool setMode(Mode mode)
  {
    fakeChip().requestMode(static_cast<uint8_t>(mode));
    return true;
  }

private:
  CANConfig config_;
};

class CANIO
{
public:
  explicit CANIO(CANConfig &) {}

  uint8_t read(uint8_t address)
  {
    uint8_t value = 0;
    read(address, &value, 1);
    return value;
  }
  void read(uint8_t address, uint8_t * data, uint8_t length) {fakeChip().read(address, data, length);}
  void write(uint8_t value) {fakeChip().instruction(value);}
  void write(uint8_t address, uint8_t value) {write(address, &value, 1);}
  void write(uint8_t address, uint8_t * data, uint8_t length)
  {
    fakeChip().write(address, data, length);
  }
  void writeBits(uint8_t address, uint8_t mask, uint8_t value)
  {
    uint8_t & reg = fakeChip().registers[address];
    reg = static_cast<uint8_t>((reg & ~mask) | (value & mask));
  }
  uint8_t readStatus() {return fakeChip().readStatus();}
};
