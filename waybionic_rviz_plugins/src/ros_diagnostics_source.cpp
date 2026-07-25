#include "waybionic_rviz_plugins/ros_diagnostics_source.hpp"

#include <algorithm>
#include <cctype>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/key_value.hpp>

namespace waybionic_rviz_plugins
{
namespace
{

constexpr double kStaleAfterSeconds = 5.0;

DiagnosticStatus mapLevel(const unsigned char level)
{
  switch (level) {
    case diagnostic_msgs::msg::DiagnosticStatus::OK:
      return DiagnosticStatus::Ok;
    case diagnostic_msgs::msg::DiagnosticStatus::WARN:
      return DiagnosticStatus::Warn;
    case diagnostic_msgs::msg::DiagnosticStatus::ERROR:
      return DiagnosticStatus::Fault;
    case diagnostic_msgs::msg::DiagnosticStatus::STALE:
      return DiagnosticStatus::Stale;
    default:
      return DiagnosticStatus::Warn;
  }
}

std::string lowerCopy(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](const unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  return value;
}

bool hasContent(const std::string & value)
{
  return !value.empty();
}

}  // namespace

RosDiagnosticsSource::RosDiagnosticsSource(
  rclcpp::Node::SharedPtr node,
  std::string diagnostics_topic)
: node_(std::move(node)),
  diagnostics_topic_(std::move(diagnostics_topic))
{
  subscription_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    diagnostics_topic_,
    rclcpp::QoS(10),
    [this](diagnostic_msgs::msg::DiagnosticArray::SharedPtr message) {
      diagnosticsCallback(std::move(message));
    });
}

std::string RosDiagnosticsSource::sourceName() const
{
  return "ROS " + diagnostics_topic_;
}

std::string RosDiagnosticsSource::connectionStatus(const rclcpp::Time & now) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_received_) {
    return "Waiting for " + diagnostics_topic_;
  }

  const double age_seconds = std::max(0.0, (now - last_received_time_).seconds());
  if (age_seconds > kStaleAfterSeconds) {
    return "No recent messages on " + diagnostics_topic_;
  }

  return "Connected to " + diagnostics_topic_;
}

std::vector<DiagnosticMessage> RosDiagnosticsSource::messages(const rclcpp::Time & now) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_received_) {
    return {{
      "diagnostics.topic",
      DiagnosticStatus::Stale,
      now,
      std::nullopt,
      std::nullopt,
      "Live diagnostics mode active; waiting for " + diagnostics_topic_ + " messages",
    }};
  }

  auto messages = latest_messages_;
  const double age_seconds = std::max(0.0, (now - last_received_time_).seconds());
  if (age_seconds <= kStaleAfterSeconds) {
    return messages;
  }

  for (auto & message : messages) {
    if (message.status == DiagnosticStatus::Ok || message.status == DiagnosticStatus::Warn) {
      message.status = DiagnosticStatus::Stale;
      message.alert_message = "No recent update from " + diagnostics_topic_;
    }
  }
  return messages;
}

void RosDiagnosticsSource::diagnosticsCallback(
  diagnostic_msgs::msg::DiagnosticArray::SharedPtr message)
{
  rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
  const auto received_at = system_clock.now();
  rclcpp::Time timestamp(message->header.stamp, RCL_SYSTEM_TIME);
  if (timestamp.nanoseconds() == 0) {
    timestamp = received_at;
  }

  std::vector<DiagnosticMessage> normalized_messages;
  normalized_messages.reserve(message->status.size());
  for (const auto & status : message->status) {
    normalized_messages.push_back(toDiagnosticMessage(status, timestamp));
  }

  std::lock_guard<std::mutex> lock(mutex_);
  latest_messages_ = std::move(normalized_messages);
  last_received_time_ = received_at;
  has_received_ = true;
}

DiagnosticMessage RosDiagnosticsSource::toDiagnosticMessage(
  const diagnostic_msgs::msg::DiagnosticStatus & status,
  const rclcpp::Time & timestamp) const
{
  std::optional<std::string> value;
  std::optional<std::string> unit;

  for (const auto & key_value : status.values) {
    const auto key = lowerCopy(key_value.key);
    if (key == "value") {
      value = key_value.value;
      continue;
    }
    if (key == "unit") {
      unit = key_value.value;
      continue;
    }
    if (!value.has_value() && hasContent(key_value.value)) {
      value = key_value.value;
      unit = key_value.key;
    }
  }

  const auto normalized_status = mapLevel(status.level);
  std::optional<std::string> alert_message;
  if (hasContent(status.message) && normalized_status != DiagnosticStatus::Ok) {
    alert_message = status.message;
  }

  return {
    status.name,
    normalized_status,
    timestamp,
    value,
    unit,
    alert_message,
  };
}

}  // namespace waybionic_rviz_plugins
