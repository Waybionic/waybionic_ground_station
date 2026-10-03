// Simulated gateway (Arduino 1): reads text commands on stdin, where the laptop's USB serial
// link will be, sends MKS frames and reports what the addressed receivers answered.
//
//   gateway [--iface vcan0] [--nodes 1,2,3] [--poll-ms 1000] [--timeout-ms 500]

#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sstream>
#include <string>
#include <vector>

#include "common/GatewayLogic.h"
#include "common/MksFrame.h"
#include "sim/Clock.h"
#include "sim/SocketCanTransport.h"

namespace
{

using waybionic::Frame;
namespace mks = waybionic::mks;

volatile sig_atomic_t g_running = 1;

void onSignal(int)
{
  g_running = 0;
}

const char * kHelp =
  "commands:\n"
  "  mode <id> [mode]              SET_MODE 82h (default 5 = SR_vFOC)\n"
  "  enable <id> [0|1]             ENABLE F3h\n"
  "  move <id> <axis> <rpm> <acc>  ABSOLUTE_AXIS F5h\n"
  "  read <id>                     READ_ENCODER 31h\n"
  "  response <id> <0|1> <0|1>     SET_RESPONSE 8Ch (respond, active)\n"
  "  heartbeat <id> <ms>           SET_HEARTBEAT 98h\n"
  "  raw <ID#HEX>                  send any frame, e.g. raw 002#3134 (bad checksum)\n"
  "  wait <ms> | status | help | quit\n"
  "  estop: TODO, F7h is not defined in waybionic_teleop/mks_can.py yet\n";

bool parseCandump(const std::string & text, Frame & out)
{
  const size_t hash = text.find('#');
  if (hash == std::string::npos || hash == 0 || hash > 3) {
    return false;
  }
  const std::string hex = text.substr(hash + 1);
  if (hex.size() % 2 != 0 || hex.size() > 16) {
    return false;
  }
  char * end = nullptr;
  const unsigned long id = strtoul(text.substr(0, hash).c_str(), &end, 16);
  if (*end != '\0' || id > waybionic::kMaxStandardCanId) {
    return false;
  }
  out = Frame{};
  out.id = static_cast<uint16_t>(id);
  out.dlc = static_cast<uint8_t>(hex.size() / 2);
  for (uint8_t i = 0; i < out.dlc; ++i) {
    const unsigned long byte = strtoul(hex.substr(2 * i, 2).c_str(), &end, 16);
    if (*end != '\0') {
      return false;
    }
    out.data[i] = static_cast<uint8_t>(byte);
  }
  return true;
}

// Returns false for unknown or malformed commands; builders reject out-of-range values.
bool buildCommand(const std::string & verb, std::istringstream & in, Frame & out)
{
  long id = -1;
  if (verb == "raw") {
    std::string text;
    return static_cast<bool>(in >> text) && parseCandump(text, out);
  }
  if (!(in >> id) || id < 0 || id > waybionic::kMaxStandardCanId) {
    return false;
  }
  const uint16_t can_id = static_cast<uint16_t>(id);
  long a = 0, b = 0, c = 0;
  if (verb == "mode") {
    a = mks::kModeSrVfoc;
    in >> a;
    return a >= 0 && a <= 255 && mks::setMode(can_id, static_cast<uint8_t>(a), out);
  }
  if (verb == "enable") {
    a = 1;
    in >> a;
    return mks::enable(can_id, a != 0, out);
  }
  if (verb == "move") {
    return static_cast<bool>(in >> a >> b >> c) &&
           mks::absoluteAxis(
      can_id, static_cast<int32_t>(a), static_cast<int32_t>(b), static_cast<int32_t>(c), out);
  }
  if (verb == "read") {
    return mks::readEncoder(can_id, out);
  }
  if (verb == "response") {
    return static_cast<bool>(in >> a >> b) && mks::setResponse(can_id, a != 0, b != 0, out);
  }
  if (verb == "heartbeat") {
    return static_cast<bool>(in >> a) && a >= 0 &&
           mks::setHeartbeat(can_id, static_cast<uint32_t>(a), out);
  }
  return false;
}

std::string describeStatus(const mks::Message & message)
{
  int64_t value = 0;
  if (mks::decodeEncoderValue(message, value)) {
    return "value=" + std::to_string(value);
  }
  uint8_t status = 0;
  if (!mks::decodeStatus(message, status)) {
    return "arguments=" + std::to_string(message.argument_count);
  }
  static const char * kRunStatus[] = {"run failed", "running", "run complete",
    "stopped at end limit"};
  if (message.code == mks::kAbsoluteAxis && status <= 3) {
    return "status=" + std::to_string(status) + " (" + kRunStatus[status] + ")";
  }
  return "status=" + std::to_string(status) + (status == mks::kStatusOk ? " (ok)" : " (failed)");
}

std::vector<uint16_t> parseNodes(const std::string & text)
{
  std::vector<uint16_t> nodes;
  std::stringstream in(text);
  std::string item;
  while (std::getline(in, item, ',')) {
    const long id = strtol(item.c_str(), nullptr, 0);
    if (id < 1 || id > waybionic::kMaxStandardCanId) {
      return {};
    }
    nodes.push_back(static_cast<uint16_t>(id));
  }
  return nodes;
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string interface_name = "vcan0";
  std::string node_list = "1,2,3";
  long poll_ms = 1000;
  long timeout_ms = 500;
  for (int i = 1; i < argc; i += 2) {
    const std::string flag = argv[i];
    const char * value = i + 1 < argc ? argv[i + 1] : nullptr;
    if (value == nullptr) {
      fprintf(stderr, "missing value for %s\n", flag.c_str());
      return 2;
    } else if (flag == "--iface") {
      interface_name = value;
    } else if (flag == "--nodes") {
      node_list = value;
    } else if (flag == "--poll-ms") {
      poll_ms = strtol(value, nullptr, 0);
    } else if (flag == "--timeout-ms") {
      timeout_ms = strtol(value, nullptr, 0);
    } else {
      fprintf(stderr, "usage: gateway [--iface vcan0] [--nodes 1,2,3] [--poll-ms 1000] "
        "[--timeout-ms 500]\n");
      return 2;
    }
  }
  const std::vector<uint16_t> nodes = parseNodes(node_list);
  if (nodes.empty() || nodes.size() > waybionic::GatewayLogic::kMaxNodes || poll_ms < 0 ||
    timeout_ms <= 0)
  {
    fprintf(stderr, "invalid --nodes, --poll-ms or --timeout-ms\n");
    return 2;
  }

  setvbuf(stdout, nullptr, _IOLBF, 0);
  signal(SIGINT, onSignal);
  signal(SIGTERM, onSignal);

  waybionic::SocketCanTransport transport;
  std::string error;
  if (!transport.open(interface_name, error)) {
    fprintf(stderr, "[gateway] %s\n", error.c_str());
    return 1;
  }
  waybionic::GatewayLogic gateway(
    nodes.data(), static_cast<uint8_t>(nodes.size()), static_cast<uint32_t>(timeout_ms));
  std::vector<waybionic::NodeHealth> last_health(nodes.size(), waybionic::NodeHealth::kUnknown);

  char text[32];
  auto transmit = [&](const Frame & frame) {
      mks::formatFrame(frame, text, sizeof(text));
      // send() success is only local; the reply decides whether the node acted.
      if (transport.send(frame)) {
        gateway.noteSent(frame, waybionic::monotonicMs());
        printf("[gateway] TX %s\n", text);
      } else {
        printf("[gateway] TX %s send failed\n", text);
      }
    };

  printf("[gateway] %s nodes=%s poll=%ldms timeout=%ldms (type 'help')\n",
    interface_name.c_str(), node_list.c_str(), poll_ms, timeout_ms);
  bool stdin_open = true;
  std::string input;  // raw read() buffer: stdio buffering would hide lines from poll()
  uint32_t resume_ms = waybionic::monotonicMs();
  uint32_t next_poll_ms = resume_ms;

  while (g_running) {
    uint32_t now = waybionic::monotonicMs();
    pollfd stdin_poll{STDIN_FILENO, POLLIN, 0};
    if (stdin_open && ::poll(&stdin_poll, 1, 0) > 0) {
      char chunk[256];
      const ssize_t count = read(STDIN_FILENO, chunk, sizeof(chunk));
      if (count > 0) {
        input.append(chunk, static_cast<size_t>(count));
      } else {
        stdin_open = false;
        input += '\n';
      }
    }
    const size_t newline = input.find('\n');
    if (newline != std::string::npos && static_cast<int32_t>(now - resume_ms) >= 0) {
      const std::string line = input.substr(0, newline);
      input.erase(0, newline + 1);
      std::istringstream in(line);
      std::string verb;
      in >> verb;
      Frame frame;
      long ms = 0;
      if (verb.empty() || verb[0] == '#') {
      } else if (verb == "quit") {
        break;
      } else if (verb == "help") {
        printf("%s", kHelp);
      } else if (verb == "wait" && in >> ms && ms >= 0) {
        resume_ms = now + static_cast<uint32_t>(ms);
      } else if (verb == "status") {
        for (uint8_t i = 0; i < gateway.nodeCount(); ++i) {
          const uint16_t id = gateway.nodeId(i);
          printf("[gateway] node %u %s unexpected=%u\n", id,
            waybionic::nodeHealthName(gateway.health(id, now)), gateway.unexpectedReplies(id));
        }
      } else if (buildCommand(verb, in, frame)) {
        transmit(frame);
      } else {
        printf("[gateway] rejected command: %s\n", line.c_str());
      }
    }

    if (poll_ms > 0 && static_cast<int32_t>(now - next_poll_ms) >= 0) {
      next_poll_ms = now + static_cast<uint32_t>(poll_ms);
      for (const uint16_t id : nodes) {
        Frame frame;
        mks::readEncoder(id, frame);
        transmit(frame);
      }
    }

    Frame frame;
    if (transport.receive(frame, 5)) {
      now = waybionic::monotonicMs();
      const waybionic::ReplyEvent event = gateway.handleFrame(frame, now);
      mks::formatFrame(frame, text, sizeof(text));
      if (event.kind == waybionic::ReplyKind::kNotANode) {
        printf("[gateway] RX %s from unknown ID\n", text);
      } else if (event.kind == waybionic::ReplyKind::kMalformed) {
        printf("[gateway] RX %s node %u malformed: %s\n", text, frame.id,
          mks::parseResultName(event.parse_result));
      } else {
        printf("[gateway] RX %s node %u %s %s %s", text, frame.id,
          mks::codeName(event.message.code), waybionic::replyKindName(event.kind),
          describeStatus(event.message).c_str());
        if (event.kind == waybionic::ReplyKind::kReply) {
          printf(" latency=%ums", event.latency_ms);
        }
        printf("\n");
      }
    }

    now = waybionic::monotonicMs();
    for (size_t i = 0; i < nodes.size(); ++i) {
      const waybionic::NodeHealth health = gateway.health(nodes[i], now);
      if (health != last_health[i]) {
        printf("[gateway] node %u %s\n", nodes[i], waybionic::nodeHealthName(health));
        last_health[i] = health;
      }
    }
  }
  printf("[gateway] shutting down\n");
  return 0;
}
