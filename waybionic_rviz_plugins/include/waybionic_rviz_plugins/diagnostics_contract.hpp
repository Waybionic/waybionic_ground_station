#ifndef WAYBIONIC_RVIZ_PLUGINS__DIAGNOSTICS_CONTRACT_HPP_
#define WAYBIONIC_RVIZ_PLUGINS__DIAGNOSTICS_CONTRACT_HPP_

#include <optional>
#include <string>

#include <rclcpp/time.hpp>

namespace waybionic_rviz_plugins
{

enum class DiagnosticStatus
{
  Ok,
  Warn,
  Fault,
  Stale
};

struct DiagnosticMessage
{
  std::string signal_name;
  DiagnosticStatus status;
  rclcpp::Time timestamp;
  std::optional<std::string> value;
  std::optional<std::string> unit;
  std::optional<std::string> alert_message;
};

inline const char * toString(const DiagnosticStatus status)
{
  switch (status) {
    case DiagnosticStatus::Ok:
      return "OK";
    case DiagnosticStatus::Warn:
      return "WARN";
    case DiagnosticStatus::Fault:
      return "FAULT";
    case DiagnosticStatus::Stale:
      return "STALE";
  }
  return "UNKNOWN";
}

inline bool isAlertStatus(const DiagnosticStatus status)
{
  return status != DiagnosticStatus::Ok;
}

}  // namespace waybionic_rviz_plugins

#endif  // WAYBIONIC_RVIZ_PLUGINS__DIAGNOSTICS_CONTRACT_HPP_
