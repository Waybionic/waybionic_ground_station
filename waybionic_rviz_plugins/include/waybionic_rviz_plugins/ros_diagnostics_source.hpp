#ifndef WAYBIONIC_RVIZ_PLUGINS__ROS_DIAGNOSTICS_SOURCE_HPP_
#define WAYBIONIC_RVIZ_PLUGINS__ROS_DIAGNOSTICS_SOURCE_HPP_

#include <mutex>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include "waybionic_rviz_plugins/diagnostics_source.hpp"

namespace waybionic_rviz_plugins
{

class RosDiagnosticsSource : public DiagnosticsSource
{
public:
  explicit RosDiagnosticsSource(
    rclcpp::Node::SharedPtr node,
    std::string diagnostics_topic = "/diagnostics");

  std::string sourceName() const override;
  std::string connectionStatus(const rclcpp::Time & now) const override;
  std::vector<DiagnosticMessage> messages(const rclcpp::Time & now) const override;

private:
  void diagnosticsCallback(diagnostic_msgs::msg::DiagnosticArray::SharedPtr message);
  DiagnosticMessage toDiagnosticMessage(
    const diagnostic_msgs::msg::DiagnosticStatus & status,
    const rclcpp::Time & timestamp) const;

  rclcpp::Node::SharedPtr node_;
  std::string diagnostics_topic_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr subscription_;

  mutable std::mutex mutex_;
  std::vector<DiagnosticMessage> latest_messages_;
  rclcpp::Time last_received_time_{0, 0, RCL_SYSTEM_TIME};
  bool has_received_{false};
};

}  // namespace waybionic_rviz_plugins

#endif  // WAYBIONIC_RVIZ_PLUGINS__ROS_DIAGNOSTICS_SOURCE_HPP_
