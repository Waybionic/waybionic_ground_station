#pragma once

#include <stdint.h>

#include "common/BusLoad.h"
#include "common/Frame.h"
#include "common/ICanTransport.h"

namespace waybionic
{

// SPI access to one MCP2515: chip select, byte transfer, its INT pin and a clock. The Arduino
// implementation is mcp2515/ArduinoMcp2515Spi.h; the unit tests use a simulated chip.
class Mcp2515Spi
{
public:
  virtual ~Mcp2515Spi() = default;
  virtual void select() = 0;
  virtual uint8_t transfer(uint8_t out) = 0;
  virtual void deselect() = 0;
  // True while INT is low. Only the receive-buffer interrupts are enabled, so this means a
  // received frame is waiting.
  virtual bool interruptAsserted() = 0;
  virtual uint32_t millis() = 0;
  virtual void delayMs(uint32_t ms) = 0;
};

// CAN through an MCP2515 controller on SPI, the common "MCP2515 + TJA1050" module.
//
// Register reads use the one method measured clean on the WayBionic bench link. On that link
// the chip sometimes shifts a read out one bit early (its SO output is one clock ahead), which
// depends on the previous SPI transaction, and multi-byte "burst" reads slip inside the burst.
// Writes, bit-modify and instructions were not observed to be affected in those tests.
// Every read here is a single byte, preceded by a two-byte read of TXB2's data registers,
// which are kept at zero and never
// transmitted. With that, 8000 loopback frames had no bad byte and 800 drive replies out of 800
// were valid; burst reads of the same frames were wrong 84 % of the time.
//
// Transmission uses TXB0 only, so frames leave in the order they are sent. A frame that has
// not left after kTransmitTimeoutMs (nobody ACKs it: drives unpowered, open bus) is aborted and
// counted rather than sent late. Reception uses both buffers with rollover; INT shows when a
// frame is waiting, so an idle poll costs no SPI traffic. The chip holds only two frames and a
// conditioned read of one takes about a quarter of a millisecond, so service() moves them into a
// queue here as they arrive and receive() hands them out in arrival order.
// RXM=00 with standard-frame filters and zero masks admits all standard IDs but rejects
// errored CAN messages, unlike debug RXM=11 which can expose partially assembled messages.
// RXnIF remains set until the full frame is copied, so new traffic cannot overwrite a
// buffer while it is being read.
//
// begin() resets the chip, checks that it answers and holds its configuration, then sends
// kSelfTestFrames through the chip's internal loopback (nothing reaches the bus) and only joins
// the bus if every frame read back exactly. A link too noisy for the read method is refused
// instead of handing out wrong data.
class Mcp2515CanTransport : public ICanTransport
{
public:
  static constexpr uint32_t kTransmitTimeoutMs = 20;
  static constexpr uint32_t kErrorSamplePeriodMs = 10;
  static constexpr uint32_t kLinkCheckPeriodMs = 100;
  static constexpr uint16_t kSelfTestFrames = 96;
  static constexpr uint8_t kReceiveQueueCapacity = 16;
  // At most this many frames are read from the chip per call, which bounds a call's time.
  static constexpr uint8_t kMaxReadsPerCall = 4;

  struct SelfTest
  {
    uint16_t frames = 0;
    uint16_t bad = 0;      // read back with a different ID, length or data byte
    uint16_t missing = 0;  // never arrived
    bool passed() const {return frames > 0 && bad == 0 && missing == 0;}
  };

  // crystal_hz is the oscillator on the module, usually 8 or 16 MHz.
  Mcp2515CanTransport(Mcp2515Spi & spi, uint32_t crystal_hz);

  // Reset, configure, self-test and join the bus. Calling it again at the running bitrate is a
  // no-op; another bitrate reconfigures the chip. False leaves the chip off the bus. If the
  // chip later loses its configuration, service() restarts it.
  bool begin(uint32_t bitrate);
  // Reset the chip into configuration mode: off the bus, nothing acknowledged.
  void end();
  bool isOpen() const {return open_;}
  uint32_t bitrate() const {return bitrate_;}

  bool send(const Frame & frame) override;
  // Extended and remote frames that reach software are dropped and counted: every protocol
  // here uses 11-bit data frames.
  bool receive(Frame & frame, uint32_t timeout_ms) override;

  // Call every loop pass, and between slow jobs such as writing to a serial port: moves received
  // frames off the chip, finishes or aborts the pending transmission, samples the controller's
  // error state and checks the SPI link. Does SPI work only when something is due.
  void service();

  const SelfTest & lastSelfTest() const {return self_test_;}
  // Controller error-state changes: error counters rising, warning/passive/bus-off entered.
  uint32_t errorEvents() const {return error_events_;}
  // Frames aborted because they did not leave in time.
  uint32_t failedWrites() const {return failed_writes_;}
  // Failed checks of the SPI link or the chip's configuration, and failed (re)starts.
  uint32_t linkFaults() const {return link_faults_;}
  // Times the chip lost a frame because both of its receive buffers were full.
  uint32_t receiveOverflows() const {return receive_overflows_;}
  uint32_t droppedFrames() const {return dropped_frames_;}
  uint32_t restarts() const {return restarts_;}
  uint8_t transmitErrorCount() const {return tec_;}
  uint8_t receiveErrorCount() const {return rec_;}
  uint8_t errorFlags() const {return eflg_;}
  // Frames sent and received since begin().
  BusLoad & busLoad() {return bus_load_;}

  // CNF1..CNF3 for 8 time quanta per bit with the sample point at 75 %. False if the crystal
  // cannot make the bitrate exactly with those quanta (8 MHz cannot make 1 Mbit/s).
  static bool timing(
    uint32_t crystal_hz, uint32_t bitrate, uint8_t & cnf1, uint8_t & cnf2, uint8_t & cnf3);

private:
  enum class ReadResult : uint8_t {kNone, kFrame, kDropped};

  uint8_t readRegister(uint8_t address);
  void writeRegisters(uint8_t address, const uint8_t * data, uint8_t length);
  void writeRegister(uint8_t address, uint8_t value) {writeRegisters(address, &value, 1);}
  void bitModify(uint8_t address, uint8_t mask, uint8_t value);
  void instruction(uint8_t code);
  bool setMode(uint8_t mode);
  bool configure(uint8_t mode);
  bool start();
  void loadTransmit(const Frame & frame);
  ReadResult readReceived(Frame & frame);
  void collectReceived();
  void clearReceived();
  bool transmitIdle(uint32_t now_ms);
  void sampleErrors();
  bool linkHealthy(uint8_t mode);
  void checkLink();
  SelfTest runSelfTest(uint16_t frames);

  Mcp2515Spi & spi_;
  uint32_t crystal_hz_;
  bool running_ = false;  // opened by begin(); service() restarts it if the chip drops out
  bool open_ = false;
  uint32_t bitrate_ = 0;
  uint8_t cnf_[3] = {};
  bool tx_pending_ = false;
  bool tx_aborting_ = false;
  uint32_t tx_started_ms_ = 0;
  uint32_t last_error_sample_ms_ = 0;
  uint32_t last_link_check_ms_ = 0;
  bool link_suspect_ = false;
  // Set when RXB1 holds a frame older than whatever RXB0 receives next.
  bool rxb1_older_ = false;
  Frame rx_queue_[kReceiveQueueCapacity];
  uint8_t rx_head_ = 0;
  uint8_t rx_count_ = 0;
  SelfTest self_test_;
  uint32_t error_events_ = 0;
  uint32_t failed_writes_ = 0;
  uint32_t link_faults_ = 0;
  uint32_t receive_overflows_ = 0;
  uint32_t dropped_frames_ = 0;
  uint32_t restarts_ = 0;
  uint8_t tec_ = 0;
  uint8_t rec_ = 0;
  uint8_t eflg_ = 0;
  BusLoad bus_load_;
};

}  // namespace waybionic
