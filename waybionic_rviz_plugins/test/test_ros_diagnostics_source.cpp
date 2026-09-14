// Regression tests for live/mock diagnostics source handoff.
//
// Before the handoff fix, replacing a live source destroyed RosDiagnosticsSource
// while an executor thread could still be inside its subscription callback. The
// churn tests below reproduce that pattern deterministically: they create, stop
// and destroy live sources in a tight loop while diagnostics traffic is flowing
// on a multi-threaded executor.

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>

#include "waybionic_rviz_plugins/diagnostics_contract.hpp"
#include "waybionic_rviz_plugins/ros_diagnostics_source.hpp"

namespace waybionic_rviz_plugins
{
namespace
{

using namespace std::chrono_literals;

constexpr const char * kTopic = "/diagnostics_source_test";

diagnostic_msgs::msg::DiagnosticArray makeArray(const unsigned char level, const std::string & value)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "board.temperature";
  status.level = level;
  status.message = "temperature reading";

  diagnostic_msgs::msg::KeyValue value_entry;
  value_entry.key = "value";
  value_entry.value = value;
  status.values.push_back(value_entry);

  diagnostic_msgs::msg::KeyValue unit_entry;
  unit_entry.key = "unit";
  unit_entry.value = "C";
  status.values.push_back(unit_entry);

  diagnostic_msgs::msg::DiagnosticArray array;
  array.status.push_back(status);
  return array;
}

template<typename Predicate>
bool waitFor(Predicate predicate, const std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(5ms);
  }
  return predicate();
}

/// Spins a subscriber node on a multi-threaded executor while a separate thread
/// floods the diagnostics topic, so callbacks genuinely overlap source teardown.
class DiagnosticsTrafficFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    subscriber_node_ = std::make_shared<rclcpp::Node>("diagnostics_source_test_subscriber");
    publisher_node_ = std::make_shared<rclcpp::Node>("diagnostics_source_test_publisher");
    publisher_ = publisher_node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      kTopic, rclcpp::QoS(10));

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 4);
    executor_->add_node(subscriber_node_);
    executor_thread_ = std::thread([this]() { executor_->spin(); });
  }

  void TearDown() override
  {
    stopTraffic();
    executor_->cancel();
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
    executor_->remove_node(subscriber_node_);
    publisher_.reset();
    publisher_node_.reset();
    subscriber_node_.reset();
  }

  void startTraffic(const std::chrono::microseconds period = 200us)
  {
    traffic_running_ = true;
    traffic_thread_ = std::thread([this, period]() {
      unsigned int counter = 0;
      while (traffic_running_) {
        const auto level = (counter % 5 == 0)
          ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
          : diagnostic_msgs::msg::DiagnosticStatus::OK;
        publisher_->publish(makeArray(level, std::to_string(counter)));
        ++counter;
        published_ = counter;
        std::this_thread::sleep_for(period);
      }
    });
  }

  void stopTraffic()
  {
    traffic_running_ = false;
    if (traffic_thread_.joinable()) {
      traffic_thread_.join();
    }
  }

  std::unique_ptr<RosDiagnosticsSource> makeSource()
  {
    return std::make_unique<RosDiagnosticsSource>(subscriber_node_, kTopic);
  }

  rclcpp::Time now() const
  {
    rclcpp::Clock clock(RCL_SYSTEM_TIME);
    return clock.now();
  }

  rclcpp::Node::SharedPtr subscriber_node_;
  rclcpp::Node::SharedPtr publisher_node_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
  std::thread executor_thread_;
  std::thread traffic_thread_;
  std::atomic<bool> traffic_running_{false};
  std::atomic<unsigned int> published_{0};
};

TEST_F(DiagnosticsTrafficFixture, ReportsWaitingBeforeAnyMessageArrives)
{
  const auto source = makeSource();

  const auto messages = source->messages(now());
  ASSERT_EQ(messages.size(), 1u);
  EXPECT_EQ(messages.front().signal_name, "diagnostics.topic");
  EXPECT_EQ(messages.front().status, DiagnosticStatus::Stale);
  EXPECT_NE(source->connectionStatus(now()).find("Waiting for"), std::string::npos);
}

TEST_F(DiagnosticsTrafficFixture, NormalizesReceivedDiagnostics)
{
  const auto source = makeSource();
  startTraffic(2ms);

  ASSERT_TRUE(waitFor([&]() {
    const auto messages = source->messages(now());
    return messages.size() == 1u && messages.front().signal_name == "board.temperature";
  }, 10s));

  const auto messages = source->messages(now());
  ASSERT_EQ(messages.size(), 1u);
  EXPECT_EQ(messages.front().signal_name, "board.temperature");
  ASSERT_TRUE(messages.front().unit.has_value());
  EXPECT_EQ(*messages.front().unit, "C");
  EXPECT_NE(source->connectionStatus(now()).find("Connected to"), std::string::npos);

  stopTraffic();
}

TEST_F(DiagnosticsTrafficFixture, OkDiagnosticsDoNotCarryAlertMessage)
{
  const auto source = makeSource();

  publisher_->publish(makeArray(diagnostic_msgs::msg::DiagnosticStatus::OK, "42"));
  ASSERT_TRUE(waitFor([&]() {
    const auto messages = source->messages(now());
    return messages.size() == 1u && messages.front().signal_name == "board.temperature";
  }, 5s));

  const auto messages = source->messages(now());
  ASSERT_EQ(messages.size(), 1u);
  EXPECT_EQ(messages.front().status, DiagnosticStatus::Ok);
  EXPECT_FALSE(messages.front().alert_message.has_value());
}

TEST_F(DiagnosticsTrafficFixture, StopFreezesStateAndIgnoresLaterMessages)
{
  auto source = makeSource();
  startTraffic(2ms);

  ASSERT_TRUE(waitFor([&]() {
    return source->messages(now()).front().signal_name == "board.temperature";
  }, 10s));

  source->stop();
  EXPECT_TRUE(source->isStopped());

  const auto frozen = source->messages(now());
  ASSERT_EQ(frozen.size(), 1u);
  const auto frozen_value = frozen.front().value;

  // Let a substantial amount of further traffic flow past the retired source.
  const auto published_at_stop = published_.load();
  ASSERT_TRUE(waitFor([&]() { return published_.load() > published_at_stop + 50; }, 10s));

  const auto after = source->messages(now());
  ASSERT_EQ(after.size(), 1u);
  EXPECT_EQ(after.front().value, frozen_value) << "retired source must not keep ingesting messages";

  stopTraffic();
}

TEST_F(DiagnosticsTrafficFixture, StopIsIdempotent)
{
  auto source = makeSource();
  startTraffic(2ms);

  ASSERT_TRUE(waitFor([&]() {
    return source->messages(now()).front().signal_name == "board.temperature";
  }, 10s));

  source->stop();
  source->stop();
  source->stop();
  EXPECT_TRUE(source->isStopped());
  EXPECT_NO_THROW(source->messages(now()));
  EXPECT_NO_THROW(source->connectionStatus(now()));

  stopTraffic();
}

TEST_F(DiagnosticsTrafficFixture, RepeatedLiveMockChurnUnderTrafficIsSafe)
{
  startTraffic(200us);

  // Mirrors a user toggling "Use Mock Diagnostics" repeatedly while the cycle
  // publisher is running. Each iteration replaces the live source exactly the
  // way DiagnosticsPanel::configureSource does.
  constexpr int kIterations = 300;
  for (int iteration = 0; iteration < kIterations; ++iteration) {
    auto live_source = makeSource();

    // Read like the refresh timer does while callbacks are arriving.
    (void)live_source->messages(now());
    (void)live_source->connectionStatus(now());

    live_source->stop();
    live_source.reset();
  }

  EXPECT_GT(published_.load(), 0u);
  stopTraffic();
}

TEST_F(DiagnosticsTrafficFixture, DestructionWithoutExplicitStopIsSafe)
{
  startTraffic(200us);

  constexpr int kIterations = 300;
  for (int iteration = 0; iteration < kIterations; ++iteration) {
    auto live_source = makeSource();
    (void)live_source->messages(now());
    // No stop() call: the destructor must retire the subscription on its own.
  }

  EXPECT_GT(published_.load(), 0u);
  stopTraffic();
}

TEST_F(DiagnosticsTrafficFixture, ConcurrentReadsDuringTeardownAreSafe)
{
  startTraffic(200us);

  constexpr int kIterations = 150;
  for (int iteration = 0; iteration < kIterations; ++iteration) {
    auto live_source = makeSource();

    std::atomic<bool> reading{true};
    std::thread reader([&]() {
      while (reading) {
        (void)live_source->messages(now());
        (void)live_source->connectionStatus(now());
      }
    });

    std::this_thread::sleep_for(1ms);
    live_source->stop();
    reading = false;
    reader.join();

    live_source.reset();
  }

  stopTraffic();
}

TEST_F(DiagnosticsTrafficFixture, ChurnLeavesNoLingeringSubscription)
{
  constexpr int kIterations = 25;
  for (int iteration = 0; iteration < kIterations; ++iteration) {
    auto live_source = makeSource();
    live_source->stop();
  }

  // A leaked subscription would keep the publisher's subscriber count above zero.
  EXPECT_TRUE(waitFor([&]() {
    return publisher_node_->count_subscribers(kTopic) == 0u;
  }, 15s)) << "expected every retired source to drop its subscription";
}

}  // namespace
}  // namespace waybionic_rviz_plugins

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
