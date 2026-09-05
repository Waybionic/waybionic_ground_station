#ifndef WAYBIONIC_RVIZ_PLUGINS__IK_DEMO_PANEL_HPP_
#define WAYBIONIC_RVIZ_PLUGINS__IK_DEMO_PANEL_HPP_

#include <chrono>
#include <memory>
#include <optional>

#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rviz_common/panel.hpp>
#include <std_srvs/srv/trigger.hpp>

class QLabel;
class QPushButton;
class QTimer;

namespace waybionic_rviz_plugins
{

/// Small RViz panel that requests a replay from the standalone IK demo node.
///
/// Service futures are polled by a Qt timer. This keeps every widget update on
/// the GUI thread and avoids ROS callbacks retaining a pointer to this panel.
class IkDemoPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit IkDemoPanel(QWidget * parent = nullptr);
  ~IkDemoPanel() override;

  void onInitialize() override;

private:
  using ReplayService = std_srvs::srv::Trigger;
  using ReplayClient = rclcpp::Client<ReplayService>;

  void cancelPendingRequest();
  void requestReplay();
  void pollReplayService();

  rclcpp::Node::SharedPtr rviz_node_;
  ReplayClient::SharedPtr replay_client_;
  std::optional<ReplayClient::FutureAndRequestId> pending_request_;
  std::chrono::steady_clock::time_point request_deadline_;
  bool service_was_ready_{false};

  QTimer * poll_timer_{nullptr};
  QPushButton * replay_button_{nullptr};
  QLabel * status_label_{nullptr};
};

}  // namespace waybionic_rviz_plugins

#endif  // WAYBIONIC_RVIZ_PLUGINS__IK_DEMO_PANEL_HPP_
