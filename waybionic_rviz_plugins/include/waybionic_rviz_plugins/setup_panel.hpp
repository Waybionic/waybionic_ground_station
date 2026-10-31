#ifndef WAYBIONIC_RVIZ_PLUGINS__SETUP_PANEL_HPP_
#define WAYBIONIC_RVIZ_PLUGINS__SETUP_PANEL_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <QString>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rviz_common/panel.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "waybionic_rviz_plugins/setup_model.hpp"

class QLabel;
class QGridLayout;
class QPushButton;
class QTimer;
class QWidget;

namespace waybionic_rviz_plugins
{

struct SetupMailbox;

/// Walks the operator through starting the arm: carrier, drives, controller, zero, ready.
///
/// The setup_guide node turns the other nodes' diagnostics into those steps and latches them
/// on /arm/setup. The panel shows them, and offers one button, Zero, when the guide asks for
/// it: that calls the drive node's own zero service after the operator confirms the arm is in
/// its zero pose. It never publishes anything, and never moves the arm.
///
/// Threading: ROS callbacks only write into a mutex-guarded mailbox, and a Qt timer drains it,
/// so all widget and bookkeeping state is touched by the Qt thread alone and no service call
/// ever blocks it.
class SetupPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit SetupPanel(QWidget * parent = nullptr);
  ~SetupPanel() override;

  void onInitialize() override;

  /// Subscribes to the setup steps and prepares the zero client on `node`. onInitialize()
  /// calls this with RViz's node. Calling it again replaces the connection.
  void connectToNode(rclcpp::Node::SharedPtr node);

private:
  using Trigger = std_srvs::srv::Trigger;
  using TriggerClient = rclcpp::Client<Trigger>;

  struct RowWidgets
  {
    QLabel * icon{nullptr};
    QLabel * title{nullptr};
    QLabel * message{nullptr};
  };

  struct PendingZero
  {
    std::uint64_t token{0};
    std::int64_t request_id{0};
    std::chrono::steady_clock::time_point sent;
  };

  void buildUi();
  void rebuildRows(const std::vector<std::string> & step_names);
  void clearRows();
  void disconnectFromNode();

  void poll();
  void applyMailbox();
  void applySnapshot(SetupSnapshot snapshot);
  void expirePendingZero(std::chrono::steady_clock::time_point now);
  void requestZero();
  void showStatus(const QString & text, bool success);
  void render();

  bool zeroOffered() const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr subscription_;
  TriggerClient::SharedPtr zero_client_;
  std::shared_ptr<SetupMailbox> mailbox_;

  // The last valid message, kept only while the guide keeps talking.
  SetupSnapshot snapshot_;
  bool has_snapshot_{false};
  std::optional<std::chrono::steady_clock::time_point> last_message_;

  std::optional<PendingZero> pending_zero_;
  std::uint64_t next_token_{0};

  QString status_text_;
  bool status_success_{false};
  std::chrono::steady_clock::time_point status_expires_;

  std::vector<std::string> row_names_;
  std::vector<RowWidgets> row_widgets_;

  QTimer * poll_timer_{nullptr};
  QLabel * summary_label_{nullptr};
  QLabel * status_label_{nullptr};
  QPushButton * zero_button_{nullptr};
  QWidget * rows_widget_{nullptr};
  QGridLayout * rows_layout_{nullptr};
};

}  // namespace waybionic_rviz_plugins

#endif  // WAYBIONIC_RVIZ_PLUGINS__SETUP_PANEL_HPP_
