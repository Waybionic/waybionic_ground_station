// Simulated receiver node (Arduinos 2-4): NodeLogic + SoftwareActuator on SocketCAN.
//
//   receiver --id 2 [--iface vcan0] [--move-ms 500] [--reply-delay-ms 0]

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <deque>
#include <string>
#include <utility>

#include "common/NodeLogic.h"
#include "sim/Clock.h"
#include "sim/SocketCanTransport.h"
#include "common/SoftwareActuator.h"

namespace
{

volatile sig_atomic_t g_running = 1;

void onSignal(int)
{
  g_running = 0;
}

void usage()
{
  fprintf(
    stderr,
    "usage: receiver --id <1-2047> [--iface vcan0] [--move-ms 500] [--reply-delay-ms 0]\n");
}

bool sameState(const waybionic::ActuatorState & a, const waybionic::ActuatorState & b)
{
  return a.enabled == b.enabled && a.moving == b.moving && a.target == b.target &&
         a.position == b.position;
}

}  // namespace

int main(int argc, char ** argv)
{
  using waybionic::Frame;

  long id = -1;
  std::string interface_name = "vcan0";
  long move_ms = 500;
  long reply_delay_ms = 0;
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string flag = argv[i];
    if (flag == "--id") {
      id = strtol(argv[i + 1], nullptr, 0);
    } else if (flag == "--iface") {
      interface_name = argv[i + 1];
    } else if (flag == "--move-ms") {
      move_ms = strtol(argv[i + 1], nullptr, 0);
    } else if (flag == "--reply-delay-ms") {
      reply_delay_ms = strtol(argv[i + 1], nullptr, 0);
    } else {
      usage();
      return 2;
    }
  }
  if (argc % 2 == 0 || id < 1 || id > waybionic::kMaxStandardCanId || move_ms < 0 ||
    reply_delay_ms < 0)
  {
    usage();
    return 2;
  }

  setvbuf(stdout, nullptr, _IOLBF, 0);
  signal(SIGINT, onSignal);
  signal(SIGTERM, onSignal);

  waybionic::SocketCanTransport transport;
  std::string error;
  if (!transport.open(interface_name, error)) {
    fprintf(stderr, "[node %ld] %s\n", id, error.c_str());
    return 1;
  }

  waybionic::SoftwareActuator actuator(static_cast<uint32_t>(move_ms));
  waybionic::NodeLogic node(static_cast<uint16_t>(id), actuator, waybionic::monotonicMs());
  std::deque<std::pair<uint32_t, Frame>> outbox;  // replies held back by --reply-delay-ms
  waybionic::ActuatorState last_state = actuator.state();
  char text[32];

  printf("[node %ld] listening on %s\n", id, interface_name.c_str());
  auto transmit = [&](const Frame & frame, const char * why) {
      waybionic::mks::formatFrame(frame, text, sizeof(text));
      printf("[node %ld] TX %s %s%s\n", id, text, why, transport.send(frame) ? "" : " (send failed)");
    };

  while (g_running) {
    Frame frame;
    if (transport.receive(frame, 5)) {
      const uint32_t now = waybionic::monotonicMs();
      const waybionic::NodeEvent event = node.handleFrame(frame, now);
      waybionic::mks::formatFrame(frame, text, sizeof(text));
      if (event.action == waybionic::NodeAction::kIgnored) {
        printf("[node %ld] RX %s ignored (addressed to %u)\n", id, text, frame.id);
      } else if (event.action == waybionic::NodeAction::kRejected) {
        printf(
          "[node %ld] RX %s rejected: %s\n", id, text,
          waybionic::mks::parseResultName(event.parse_result));
      } else {
        printf(
          "[node %ld] RX %s %s %s\n", id, text, waybionic::mks::codeName(event.code),
          waybionic::nodeActionName(event.action));
      }
      if (event.has_reply) {
        outbox.emplace_back(now + static_cast<uint32_t>(reply_delay_ms), event.reply);
      }
    }

    const uint32_t now = waybionic::monotonicMs();
    const waybionic::PollEvent poll = node.poll(now);
    if (poll.heartbeat_stop) {
      printf("[node %ld] heartbeat timeout: stopped\n", id);
    }
    if (poll.has_frame) {
      outbox.emplace_back(now + static_cast<uint32_t>(reply_delay_ms), poll.frame);
    }
    while (!outbox.empty() && static_cast<int32_t>(now - outbox.front().first) >= 0) {
      transmit(outbox.front().second, "reply");
      outbox.pop_front();
    }

    const waybionic::ActuatorState state = actuator.state();
    if (!sameState(state, last_state)) {
      printf(
        "[node %ld] actuator enabled=%d moving=%d target=%ld position=%ld\n", id, state.enabled,
        state.moving, static_cast<long>(state.target), static_cast<long>(state.position));
      last_state = state;
    }
  }
  printf("[node %ld] shutting down\n", id);
  return 0;
}
