// Simulated carrier: the SlcanBridge used by carrier_bridge.ino, on a Linux pseudo-terminal,
// in front of simulated MKS drives (NodeLogic + SoftwareActuator). Lets python-can's slcan
// interface be exercised without any hardware:
//
//   slcan_sim [--ids 1] [--move-ms 500]
//   -> prints the pty path, e.g. /dev/pts/3; then in Python:
//      can.Bus(interface="slcan", channel="/dev/pts/3", bitrate=500000)

#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

#include <deque>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "common/NodeLogic.h"
#include "common/SlcanBridge.h"
#include "common/SoftwareActuator.h"
#include "sim/Clock.h"

namespace
{

using waybionic::Frame;

volatile sig_atomic_t g_running = 1;

void onSignal(int)
{
  g_running = 0;
}

// The bridge's side of the bus: frames it sends go to the drives, drive replies come back.
class SimBus : public waybionic::ICanTransport, public waybionic::CanStarter
{
public:
  bool start(uint32_t bitrate) override
  {
    if (started_ && bitrate != bitrate_) {
      return false;
    }
    started_ = true;
    bitrate_ = bitrate;
    return true;
  }
  bool send(const Frame & frame) override
  {
    if (!started_) {
      return false;
    }
    sent.push_back(frame);
    return true;
  }
  bool receive(Frame & frame, uint32_t) override
  {
    if (!started_ || inbox.empty()) {
      return false;
    }
    frame = inbox.front();
    inbox.pop_front();
    return true;
  }

  std::deque<Frame> sent;
  std::deque<Frame> inbox;

private:
  bool started_ = false;
  uint32_t bitrate_ = 0;
};

class FdOutput : public waybionic::SlcanOutput
{
public:
  explicit FdOutput(int fd)
  : fd_(fd) {}
  void write(const char * text) override
  {
    size_t length = 0;
    while (text[length] != '\0') {
      ++length;
    }
    if (::write(fd_, text, length) < 0) {
      perror("write");
    }
  }

private:
  int fd_;
};

struct Drive
{
  explicit Drive(uint16_t id, uint32_t move_ms)
  : actuator(move_ms), node(id, actuator, waybionic::monotonicMs()) {}
  waybionic::SoftwareActuator actuator;
  waybionic::NodeLogic node;
};

}  // namespace

int main(int argc, char ** argv)
{
  std::string ids = "1";
  long move_ms = 500;
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string flag = argv[i];
    if (flag == "--ids") {
      ids = argv[i + 1];
    } else if (flag == "--move-ms") {
      move_ms = strtol(argv[i + 1], nullptr, 0);
    } else {
      fprintf(stderr, "usage: slcan_sim [--ids 1,2] [--move-ms 500]\n");
      return 2;
    }
  }

  std::vector<std::unique_ptr<Drive>> drives;
  std::stringstream list(ids);
  std::string item;
  while (std::getline(list, item, ',')) {
    const long id = strtol(item.c_str(), nullptr, 0);
    if (id < 1 || id > waybionic::kMaxStandardCanId) {
      fprintf(stderr, "invalid id %s\n", item.c_str());
      return 2;
    }
    drives.push_back(std::make_unique<Drive>(static_cast<uint16_t>(id), move_ms));
  }

  const int master = posix_openpt(O_RDWR | O_NOCTTY);
  if (master < 0 || grantpt(master) != 0 || unlockpt(master) != 0) {
    perror("pty");
    return 1;
  }
  termios settings;
  tcgetattr(master, &settings);
  cfmakeraw(&settings);
  tcsetattr(master, TCSANOW, &settings);

  signal(SIGINT, onSignal);
  signal(SIGTERM, onSignal);
  setvbuf(stdout, nullptr, _IOLBF, 0);
  printf("%s\n", ptsname(master));
  fprintf(stderr, "[slcan_sim] drives %s on %s; Ctrl-C to stop\n", ids.c_str(), ptsname(master));

  SimBus bus;
  FdOutput output(master);
  waybionic::SlcanBridge bridge(bus, bus, output);

  while (g_running) {
    pollfd fd{master, POLLIN, 0};
    if (::poll(&fd, 1, 2) > 0 && (fd.revents & POLLIN)) {
      char chunk[256];
      const ssize_t count = read(master, chunk, sizeof(chunk));
      for (ssize_t i = 0; i < count; ++i) {
        bridge.handleChar(chunk[i]);
      }
    }
    const uint32_t now = waybionic::monotonicMs();
    while (!bus.sent.empty()) {
      const Frame frame = bus.sent.front();
      bus.sent.pop_front();
      fprintf(stderr, "[slcan_sim] bus frame %03X dlc=%u\n", frame.id, frame.dlc);
      for (auto & drive : drives) {
        const waybionic::NodeEvent event = drive->node.handleFrame(frame, now);
        if (event.has_reply) {
          bus.inbox.push_back(event.reply);
        }
      }
    }
    for (auto & drive : drives) {
      const waybionic::PollEvent event = drive->node.poll(now);
      if (event.has_frame) {
        bus.inbox.push_back(event.frame);
      }
    }
    bridge.pollCan();
  }
  close(master);
  return 0;
}
