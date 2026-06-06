#include "waybionic_rviz_plugins/mock_diagnostics_source.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

namespace waybionic_rviz_plugins
{
namespace
{

rclcpp::Time secondsAgo(const rclcpp::Time & now, const double seconds)
{
  const auto nanoseconds = static_cast<int64_t>(seconds * 1'000'000'000.0);
  return now - rclcpp::Duration::from_nanoseconds(nanoseconds);
}

std::string formatNumber(const double value, const int precision)
{
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.*f", precision, value);
  return std::string(buffer);
}

}  // namespace

void MockDiagnosticsSource::setMode(const DemoMode mode)
{
  mode_ = mode;
}

DemoMode MockDiagnosticsSource::mode() const
{
  return mode_;
}

std::string MockDiagnosticsSource::sourceName() const
{
  return "Mock";
}

std::vector<DiagnosticMessage> MockDiagnosticsSource::messages(const rclcpp::Time & now) const
{
  if (mode_ == DemoMode::Fault) {
    return faultMessages(now);
  }
  return normalMessages(now);
}

std::vector<DiagnosticMessage> MockDiagnosticsSource::normalMessages(const rclcpp::Time & now) const
{
  const double pulse = std::sin(now.seconds() / 4.0);
  return {
    {"board.temperature", DiagnosticStatus::Ok, secondsAgo(now, 0.4), formatNumber(42.0 + pulse, 1), "C", std::nullopt},
    {"motor.current", DiagnosticStatus::Ok, secondsAgo(now, 0.5), formatNumber(0.8 + pulse * 0.05, 2), "A", std::nullopt},
    {"imu.roll", DiagnosticStatus::Ok, secondsAgo(now, 0.2), formatNumber(1.2 + pulse * 0.1, 1), "deg", std::nullopt},
    {"imu.pitch", DiagnosticStatus::Ok, secondsAgo(now, 0.2), formatNumber(-0.4 + pulse * 0.1, 1), "deg", std::nullopt},
    {"imu.yaw", DiagnosticStatus::Ok, secondsAgo(now, 0.2), formatNumber(12.9 + pulse * 0.2, 1), "deg", std::nullopt},
  };
}

std::vector<DiagnosticMessage> MockDiagnosticsSource::faultMessages(const rclcpp::Time & now) const
{
  const double pulse = std::sin(now.seconds() / 3.0);
  return {
    {
      "board.temperature",
      DiagnosticStatus::Fault,
      secondsAgo(now, 0.2),
      formatNumber(82.0 + pulse * 0.5, 1),
      "C",
      "High temperature detected",
    },
    {"motor.current", DiagnosticStatus::Ok, secondsAgo(now, 0.5), "0.80", "A", std::nullopt},
    {"imu.roll", DiagnosticStatus::Ok, secondsAgo(now, 0.2), "1.2", "deg", std::nullopt},
    {"imu.pitch", DiagnosticStatus::Ok, secondsAgo(now, 0.2), "-0.4", "deg", std::nullopt},
    {"imu.yaw", DiagnosticStatus::Ok, secondsAgo(now, 0.2), "12.9", "deg", std::nullopt},
    {"imu.heartbeat", DiagnosticStatus::Stale, secondsAgo(now, 5.2), std::nullopt, std::nullopt, "Sensor timeout"},
  };
}

}  // namespace waybionic_rviz_plugins
