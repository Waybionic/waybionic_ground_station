#ifndef WAYBIONIC_RVIZ_PLUGINS__JOINT_CONTROL_PANEL_HPP_
#define WAYBIONIC_RVIZ_PLUGINS__JOINT_CONTROL_PANEL_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <map>
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
#include <std_srvs/srv/set_bool.hpp>

#include "waybionic_rviz_plugins/joint_control_model.hpp"

class QCheckBox;
class QGridLayout;
class QLabel;
class QScrollArea;
class QTimer;
class QWidget;

namespace waybionic_rviz_plugins
{

struct JointControlMailbox;

/// Lets the operator hold or release arm joints.
///
/// The drive node latches the state of every joint on /arm/joint_control and
/// offers a SetBool service per joint. The panel only ever calls those services;
/// it never publishes motion commands or any other topic. The published state is
/// the single source of truth: a checkbox changes only when the topic reports a
/// change, never because it was clicked.
///
/// Threading: RViz spins its node from the Qt thread, but nothing here relies on
/// that. ROS callbacks only write into a mutex-guarded mailbox, and a Qt timer
/// drains it, so all widget and bookkeeping state is touched by the Qt thread
/// alone and no service call ever blocks it.
class JointControlPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit JointControlPanel(QWidget * parent = nullptr);
  ~JointControlPanel() override;

  void onInitialize() override;

  /// Subscribes to the joint states and prepares the service clients on `node`.
  /// onInitialize() calls this with RViz's node; it is public so the panel can
  /// also be driven by another node. Calling it again replaces the connection.
  void connectToNode(rclcpp::Node::SharedPtr node);

private:
  using SetBool = std_srvs::srv::SetBool;
  using SetBoolClient = rclcpp::Client<SetBool>;

  struct RowWidgets
  {
    QCheckBox * checkbox{nullptr};
    QLabel * state{nullptr};
    QLabel * detail{nullptr};
    QString state_style;
  };

  struct PendingCall
  {
    std::uint64_t token{0};
    std::int64_t request_id{0};
    bool enable{false};
    std::chrono::steady_clock::time_point sent;
  };

  void buildUi();
  void rebuildRows(const std::vector<std::string> & joint_names);
  void clearRows();
  void disconnectFromNode();

  void poll();
  void applyMailbox();
  void applySnapshot(JointControlSnapshot snapshot);
  void expirePendingCalls(std::chrono::steady_clock::time_point now);
  void requestToggle(const std::string & joint_name);
  SetBoolClient::SharedPtr clientFor(const std::string & joint_name);
  void forgetCall(const std::string & joint_name, const PendingCall & call);
  void showStatus(const QString & text);
  void render();

  const JointRow * findRow(const std::string & joint_name) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr subscription_;
  std::map<std::string, SetBoolClient::SharedPtr> clients_;
  std::shared_ptr<JointControlMailbox> mailbox_;

  // The last valid message, kept only while the drive node keeps talking.
  JointControlSnapshot snapshot_;
  bool has_snapshot_{false};
  std::optional<std::chrono::steady_clock::time_point> last_message_;

  std::map<std::string, PendingCall> pending_;
  std::uint64_t next_token_{0};

  QString status_text_;
  std::chrono::steady_clock::time_point status_expires_;

  std::vector<std::string> row_names_;
  std::vector<RowWidgets> row_widgets_;
  QString connection_style_;

  QTimer * poll_timer_{nullptr};
  QLabel * connection_label_{nullptr};
  QLabel * status_label_{nullptr};
  QScrollArea * rows_scroll_{nullptr};
  QWidget * rows_widget_{nullptr};
  QGridLayout * rows_layout_{nullptr};
};

}  // namespace waybionic_rviz_plugins

#endif  // WAYBIONIC_RVIZ_PLUGINS__JOINT_CONTROL_PANEL_HPP_
