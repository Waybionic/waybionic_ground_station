#ifndef WAYBIONIC_RVIZ_PLUGINS__ROS_DIAGNOSTICS_SOURCE_HPP_
#define WAYBIONIC_RVIZ_PLUGINS__ROS_DIAGNOSTICS_SOURCE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include "waybionic_rviz_plugins/diagnostics_source.hpp"

namespace waybionic_rviz_plugins
{

/// Live diagnostics source backed by a ROS 2 subscription.
///
/// The subscription callback runs on an executor thread while the RViz panel
/// reads and replaces sources from the Qt thread. Received data therefore lives
/// in a separately owned state block that the callback keeps alive, so tearing
/// this object down can never leave an in-flight callback writing into freed
/// memory.
class RosDiagnosticsSource : public DiagnosticsSource
{
public:
  explicit RosDiagnosticsSource(
    rclcpp::Node::SharedPtr node,
    std::string diagnostics_topic = "/diagnostics");

  ~RosDiagnosticsSource() override;

  RosDiagnosticsSource(const RosDiagnosticsSource &) = delete;
  RosDiagnosticsSource & operator=(const RosDiagnosticsSource &) = delete;
  RosDiagnosticsSource(RosDiagnosticsSource &&) = delete;
  RosDiagnosticsSource & operator=(RosDiagnosticsSource &&) = delete;

  /// Drops the subscription and retires the shared state. Any callback that is
  /// already executing finishes against the retired state and is discarded.
  void stop() override;

  bool isStopped() const;

  std::string sourceName() const override;
  std::string connectionStatus(const rclcpp::Time & now) const override;
  std::vector<DiagnosticMessage> messages(const rclcpp::Time & now) const override;

private:
  /// Data shared between the executor thread and the Qt thread. Held by
  /// shared_ptr so the subscription callback always operates on live memory.
  struct SharedState
  {
    mutable std::mutex mutex;
    std::vector<DiagnosticMessage> latest_messages;
    rclcpp::Time last_received_time{0, 0, RCL_SYSTEM_TIME};
    bool has_received{false};
    bool active{true};
  };

  static void handleMessage(
    const std::shared_ptr<SharedState> & state,
    const diagnostic_msgs::msg::DiagnosticArray & message);

  rclcpp::Node::SharedPtr node_;
  std::string diagnostics_topic_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr subscription_;
  std::shared_ptr<SharedState> state_;
};

}  // namespace waybionic_rviz_plugins

#endif  // WAYBIONIC_RVIZ_PLUGINS__ROS_DIAGNOSTICS_SOURCE_HPP_
