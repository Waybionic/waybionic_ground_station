// Gateway + three receiver processes on one SocketCAN interface (vcan0 by default, or
// $WAYBIONIC_CAN_IFACE). The test process listens on the same bus and judges every scenario
// by the addressed receiver's MKS reply, never by send() succeeding. Skips when the
// interface does not exist.

#include <fcntl.h>
#include <net/if.h>
#include <signal.h>
#include <stdlib.h>
#include <sys/wait.h>
#include <unistd.h>

#include <gtest/gtest.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "common/GatewayLogic.h"
#include "common/MksFrame.h"
#include "sim/Clock.h"
#include "sim/SocketCanTransport.h"

namespace waybionic
{
namespace
{

using namespace std::chrono_literals;

constexpr uint32_t kReplyTimeoutMs = 500;
constexpr int kMoveMs = 200;

std::string interfaceName()
{
  const char * name = getenv("WAYBIONIC_CAN_IFACE");
  return name != nullptr ? name : "vcan0";
}

std::string readFile(const std::string & path)
{
  std::ifstream in(path);
  std::stringstream text;
  text << in.rdbuf();
  return text.str();
}

pid_t spawn(
  const std::string & exe, const std::vector<std::string> & args, const std::string & log_path,
  const std::string & stdin_path = "/dev/null")
{
  const pid_t pid = fork();
  if (pid != 0) {
    return pid;
  }
  const int log = open(log_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
  const int input = open(stdin_path.c_str(), O_RDONLY);
  dup2(log, STDOUT_FILENO);
  dup2(log, STDERR_FILENO);
  dup2(input, STDIN_FILENO);
  std::vector<char *> argv{const_cast<char *>(exe.c_str())};
  for (const auto & arg : args) {
    argv.push_back(const_cast<char *>(arg.c_str()));
  }
  argv.push_back(nullptr);
  execv(exe.c_str(), argv.data());
  _exit(127);
}

bool waitExit(const pid_t pid, const std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    int status = 0;
    if (waitpid(pid, &status, WNOHANG) == pid) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }
  return false;
}

void terminate(const pid_t pid)
{
  kill(pid, SIGTERM);
  if (!waitExit(pid, 2s)) {
    kill(pid, SIGKILL);
    waitpid(pid, nullptr, 0);
  }
}

class VcanFourNode : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (if_nametoindex(interfaceName().c_str()) == 0) {
      GTEST_SKIP() << interfaceName() << " not found; run scripts/setup_vcan.sh";
    }
    std::string error;
    ASSERT_TRUE(bus_.open(interfaceName(), error)) << error;
    char directory[] = "/tmp/waybionic_can_XXXXXX";
    ASSERT_NE(mkdtemp(directory), nullptr);
    log_dir_ = directory;
    for (const uint16_t id : kIds) {
      startReceiver(id);
    }
    for (const uint16_t id : kIds) {
      ASSERT_TRUE(waitReady(id)) << "receiver " << id << " never answered";
    }
  }

  void TearDown() override
  {
    for (auto & [name, pid] : processes_) {
      terminate(pid);
    }
    if (HasFailure()) {
      for (const auto & [name, pid] : processes_) {
        std::cout << "----- " << name << "\n" << readFile(logPath(name));
      }
    }
  }

  std::string logPath(const std::string & name) const {return log_dir_ + "/" + name + ".log";}

  std::string startReceiver(uint16_t id, std::vector<std::string> extra = {}, std::string name = "")
  {
    name = name.empty() ? "receiver" + std::to_string(id) : name;
    std::vector<std::string> args{"--id", std::to_string(id), "--iface", interfaceName(),
      "--move-ms", std::to_string(kMoveMs)};
    args.insert(args.end(), extra.begin(), extra.end());
    processes_[name] = spawn(RECEIVER_EXE, args, logPath(name));
    // The socket is bound before this line is printed, so no frame can be missed after it.
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (readFile(logPath(name)).find("listening") == std::string::npos &&
      std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(10ms);
    }
    return name;
  }

  void stopProcess(const std::string & name)
  {
    terminate(processes_.at(name));
    processes_.erase(name);
  }

  void transmit(const Frame & frame)
  {
    ASSERT_TRUE(bus_.send(frame));
    gateway_.noteSent(frame, monotonicMs());
  }

  // Everything the gateway side hears for `ms`.
  std::vector<std::pair<Frame, ReplyEvent>> listen(uint32_t ms)
  {
    std::vector<std::pair<Frame, ReplyEvent>> heard;
    const uint32_t start = monotonicMs();
    Frame frame;
    while (monotonicMs() - start < ms) {
      if (bus_.receive(frame, 5)) {
        heard.emplace_back(frame, gateway_.handleFrame(frame, monotonicMs()));
      }
    }
    return heard;
  }

  // Send and wait for the addressed node's answer to this function code.
  bool request(const Frame & frame, ReplyEvent & reply, uint32_t timeout_ms = kReplyTimeoutMs)
  {
    transmit(frame);
    const uint32_t start = monotonicMs();
    Frame heard;
    while (monotonicMs() - start < timeout_ms) {
      if (!bus_.receive(heard, 5)) {
        continue;
      }
      reply = gateway_.handleFrame(heard, monotonicMs());
      if (reply.kind == ReplyKind::kReply && heard.id == frame.id &&
        reply.message.code == frame.data[0])
      {
        return true;
      }
    }
    return false;
  }

  uint8_t requestStatus(const Frame & frame)
  {
    ReplyEvent reply;
    uint8_t status = 0xFF;
    EXPECT_TRUE(request(frame, reply)) << "no reply from node " << frame.id;
    EXPECT_TRUE(mks::decodeStatus(reply.message, status));
    return status;
  }

  int64_t encoder(uint16_t id)
  {
    Frame frame;
    mks::readEncoder(id, frame);
    ReplyEvent reply;
    int64_t value = -1;
    EXPECT_TRUE(request(frame, reply)) << "no encoder reply from node " << id;
    EXPECT_TRUE(mks::decodeEncoderValue(reply.message, value));
    return value;
  }

  bool waitReady(uint16_t id)
  {
    Frame frame;
    mks::readEncoder(id, frame);
    ReplyEvent reply;
    for (int attempt = 0; attempt < 10; ++attempt) {
      if (request(frame, reply, 300)) {
        return true;
      }
    }
    return false;
  }

  void configure(uint16_t id)
  {
    Frame frame;
    mks::setMode(id, mks::kModeSrVfoc, frame);
    EXPECT_EQ(requestStatus(frame), mks::kStatusOk);
    mks::enable(id, true, frame);
    EXPECT_EQ(requestStatus(frame), mks::kStatusOk);
  }

  static constexpr uint16_t kIds[3] = {1, 2, 3};
  SocketCanTransport bus_;
  GatewayLogic gateway_{kIds, 3, kReplyTimeoutMs};
  std::string log_dir_;
  std::map<std::string, pid_t> processes_;
  Frame f;
};

TEST_F(VcanFourNode, OnlyTheAddressedReceiverActs)
{
  configure(2);
  mks::absoluteAxis(2, 0x4000, 600, 2, f);
  EXPECT_EQ(requestStatus(f), 1u);  // running

  bool completed = false;
  for (const auto & [frame, event] : listen(kMoveMs + 500)) {
    completed |= frame.id == 2 && event.kind == ReplyKind::kRunComplete;
  }
  EXPECT_TRUE(completed);
  EXPECT_EQ(encoder(2), 0x4000);
  EXPECT_EQ(encoder(1), 0);
  EXPECT_EQ(encoder(3), 0);

  const std::string node1 = readFile(logPath("receiver1"));
  EXPECT_NE(node1.find("ignored (addressed to 2)"), std::string::npos);
  EXPECT_EQ(node1.find("actuator enabled=1"), std::string::npos);
  EXPECT_NE(readFile(logPath("receiver2")).find("actuator enabled=1 moving=1 target=16384"),
    std::string::npos);
}

TEST_F(VcanFourNode, StatusReplyComesFromTheAddressedReceiver)
{
  mks::enable(3, true, f);
  ReplyEvent reply;
  ASSERT_TRUE(request(f, reply));
  Frame expected;
  mks::statusReply(3, mks::kEnable, mks::kStatusOk, expected);
  EXPECT_EQ(reply.message.can_id, 3u);
  EXPECT_EQ(reply.message.code, mks::kEnable);
  EXPECT_EQ(reply.message.arguments[0], mks::kStatusOk);
  EXPECT_EQ(gateway_.health(3, monotonicMs()), NodeHealth::kOnline);
}

TEST_F(VcanFourNode, WrongIdIsIgnoredByEveryReceiver)
{
  mks::enable(7, true, f);
  transmit(f);  // send() succeeds: the frame is on the bus even though nobody owns ID 7
  EXPECT_TRUE(listen(300).empty());
  for (const uint16_t id : kIds) {
    EXPECT_EQ(readFile(logPath("receiver" + std::to_string(id))).find("actuator enabled=1"),
      std::string::npos);
  }
}

TEST_F(VcanFourNode, ValidAndInvalidApplicationChecksum)
{
  mks::readEncoder(2, f);
  ReplyEvent reply;
  EXPECT_TRUE(request(f, reply));

  mks::enable(2, true, f);
  f.data[f.dlc - 1] ^= 0x01;
  EXPECT_FALSE(request(f, reply, 300));
  EXPECT_NE(readFile(logPath("receiver2")).find("rejected: bad checksum"), std::string::npos);
}

TEST_F(VcanFourNode, InvalidDlcAndUnsupportedCommandGetNoReply)
{
  Frame short_frame;
  short_frame.id = 2;
  short_frame.dlc = 1;
  short_frame.data[0] = mks::kReadEncoder;
  ReplyEvent reply;
  EXPECT_FALSE(request(short_frame, reply, 300));

  const uint8_t two[2] = {1, 1};
  mks::buildFrame(2, mks::kEnable, two, 2, f);
  EXPECT_FALSE(request(f, reply, 300));

  const std::string log = readFile(logPath("receiver2"));
  EXPECT_NE(log.find("rejected: bad DLC"), std::string::npos);
  EXPECT_NE(log.find("ENABLE unsupported"), std::string::npos);
  EXPECT_EQ(log.find("actuator enabled=1"), std::string::npos);
}

TEST_F(VcanFourNode, MissingReceiverIsDetectedOffline)
{
  stopProcess("receiver3");
  mks::readEncoder(3, f);
  ReplyEvent reply;
  EXPECT_FALSE(request(f, reply, kReplyTimeoutMs + 100));
  EXPECT_EQ(gateway_.health(3, monotonicMs()), NodeHealth::kOffline);
  EXPECT_EQ(gateway_.health(1, monotonicMs()), NodeHealth::kOnline);
}

TEST_F(VcanFourNode, RestartedReceiverRecoversAfterReconfiguration)
{
  configure(2);
  stopProcess("receiver2");
  mks::readEncoder(2, f);
  ReplyEvent reply;
  EXPECT_FALSE(request(f, reply, kReplyTimeoutMs + 100));
  EXPECT_EQ(gateway_.health(2, monotonicMs()), NodeHealth::kOffline);

  startReceiver(2);
  ASSERT_TRUE(waitReady(2));
  EXPECT_EQ(gateway_.health(2, monotonicMs()), NodeHealth::kOnline);
  mks::absoluteAxis(2, 0x4000, 600, 2, f);
  EXPECT_EQ(requestStatus(f), mks::kStatusFailed);  // state was lost with the process
  configure(2);
  EXPECT_EQ(requestStatus(f), mks::kStatusOk);
}

TEST_F(VcanFourNode, DelayedReplyIsOfflineUntilItArrives)
{
  stopProcess("receiver2");
  startReceiver(2, {"--reply-delay-ms", "800"});
  mks::readEncoder(2, f);
  transmit(f);
  const uint32_t sent = monotonicMs();
  std::this_thread::sleep_for(std::chrono::milliseconds(kReplyTimeoutMs + 100));
  EXPECT_EQ(gateway_.health(2, monotonicMs()), NodeHealth::kOffline);

  bool answered = false;
  for (const auto & [frame, event] : listen(1000)) {
    if (frame.id == 2 && event.kind == ReplyKind::kReply) {
      answered = true;
      EXPECT_GE(event.latency_ms, 800u);
    }
  }
  EXPECT_TRUE(answered);
  EXPECT_EQ(gateway_.health(2, monotonicMs()), NodeHealth::kOnline);
  EXPECT_GE(monotonicMs() - sent, 800u);
}

TEST_F(VcanFourNode, DuplicateCanIdIsVisibleAsAnUnexpectedReply)
{
  startReceiver(2, {}, "receiver2_twin");
  mks::readEncoder(2, f);
  ReplyEvent reply;
  ASSERT_TRUE(request(f, reply));
  listen(300);
  EXPECT_EQ(gateway_.unexpectedReplies(2), 1u);
}

TEST_F(VcanFourNode, GatewayProcessDrivesTheReceivers)
{
  const std::string script = log_dir_ + "/script.txt";
  std::ofstream(script) << "mode 2\nenable 2\nmove 2 16384 600 2\nwait 700\n"
    "read 1\nread 2\nraw 002#3134\nwait 300\nstatus\nquit\n";
  processes_["gateway"] = spawn(GATEWAY_EXE,
    {"--iface", interfaceName(), "--poll-ms", "0", "--timeout-ms", "500"},
    logPath("gateway"), script);
  ASSERT_TRUE(waitExit(processes_.at("gateway"), 10s));
  processes_.erase("gateway");

  const std::string log = readFile(logPath("gateway"));
  for (const char * line : {
      "node 2 SET_MODE reply status=1 (ok)",
      "node 2 ENABLE reply status=1 (ok)",
      "node 2 ABSOLUTE_AXIS reply status=1 (running)",
      "node 2 ABSOLUTE_AXIS run complete status=2 (run complete)",
      "node 1 READ_ENCODER reply value=0",
      "node 2 READ_ENCODER reply value=16384",
      "node 2 ONLINE",
      "node 3 UNKNOWN",
    })
  {
    EXPECT_NE(log.find(line), std::string::npos) << "missing: " << line << "\n" << log;
  }
  EXPECT_EQ(log.find("node 3 READ_ENCODER"), std::string::npos);
  EXPECT_NE(readFile(logPath("receiver2")).find("rejected: bad checksum"), std::string::npos);
}

TEST_F(VcanFourNode, EmergencyStopIsPending)
{
  GTEST_SKIP() << "TODO(MKS manual): F7h emergency stop is not defined in "
    "waybionic_teleop/mks_can.py; add it from the manual before testing it";
}

}  // namespace
}  // namespace waybionic
