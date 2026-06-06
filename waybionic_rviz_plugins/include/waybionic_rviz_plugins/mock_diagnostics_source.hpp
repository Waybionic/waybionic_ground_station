#ifndef WAYBIONIC_RVIZ_PLUGINS__MOCK_DIAGNOSTICS_SOURCE_HPP_
#define WAYBIONIC_RVIZ_PLUGINS__MOCK_DIAGNOSTICS_SOURCE_HPP_

#include <string>
#include <vector>

#include <rclcpp/time.hpp>

#include "waybionic_rviz_plugins/diagnostics_contract.hpp"

namespace waybionic_rviz_plugins
{

enum class DemoMode
{
  Normal,
  Fault
};

class MockDiagnosticsSource
{
public:
  void setMode(DemoMode mode);
  DemoMode mode() const;
  std::string sourceName() const;

  std::vector<DiagnosticMessage> messages(const rclcpp::Time & now) const;

private:
  std::vector<DiagnosticMessage> normalMessages(const rclcpp::Time & now) const;
  std::vector<DiagnosticMessage> faultMessages(const rclcpp::Time & now) const;

  DemoMode mode_{DemoMode::Normal};
};

}  // namespace waybionic_rviz_plugins

#endif  // WAYBIONIC_RVIZ_PLUGINS__MOCK_DIAGNOSTICS_SOURCE_HPP_
