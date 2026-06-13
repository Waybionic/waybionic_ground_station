#ifndef WAYBIONIC_RVIZ_PLUGINS__MOCK_DIAGNOSTICS_SOURCE_HPP_
#define WAYBIONIC_RVIZ_PLUGINS__MOCK_DIAGNOSTICS_SOURCE_HPP_

#include <string>
#include <vector>

#include <rclcpp/time.hpp>

#include "waybionic_rviz_plugins/diagnostics_contract.hpp"
#include "waybionic_rviz_plugins/diagnostics_source.hpp"

namespace waybionic_rviz_plugins
{

enum class MockDiagnosticsState
{
  Normal,
  Fault
};

class MockDiagnosticsSource : public DiagnosticsSource
{
public:
  void setMode(MockDiagnosticsState mode);
  MockDiagnosticsState mode() const;
  std::string sourceName() const override;
  std::string connectionStatus(const rclcpp::Time & now) const override;

  std::vector<DiagnosticMessage> messages(const rclcpp::Time & now) const override;

private:
  std::vector<DiagnosticMessage> normalMessages(const rclcpp::Time & now) const;
  std::vector<DiagnosticMessage> faultMessages(const rclcpp::Time & now) const;

  MockDiagnosticsState mode_{MockDiagnosticsState::Normal};
};

}  // namespace waybionic_rviz_plugins

#endif  // WAYBIONIC_RVIZ_PLUGINS__MOCK_DIAGNOSTICS_SOURCE_HPP_
