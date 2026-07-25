#ifndef WAYBIONIC_RVIZ_PLUGINS__DIAGNOSTICS_SOURCE_HPP_
#define WAYBIONIC_RVIZ_PLUGINS__DIAGNOSTICS_SOURCE_HPP_

#include <string>
#include <vector>

#include <rclcpp/time.hpp>

#include "waybionic_rviz_plugins/diagnostics_contract.hpp"

namespace waybionic_rviz_plugins
{

class DiagnosticsSource
{
public:
  virtual ~DiagnosticsSource() = default;

  virtual std::string sourceName() const = 0;
  virtual std::string connectionStatus(const rclcpp::Time & now) const = 0;
  virtual std::vector<DiagnosticMessage> messages(const rclcpp::Time & now) const = 0;
};

}  // namespace waybionic_rviz_plugins

#endif  // WAYBIONIC_RVIZ_PLUGINS__DIAGNOSTICS_SOURCE_HPP_
