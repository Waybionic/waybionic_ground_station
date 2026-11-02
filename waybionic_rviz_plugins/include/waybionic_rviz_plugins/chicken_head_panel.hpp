#ifndef WAYBIONIC_RVIZ_PLUGINS__CHICKEN_HEAD_PANEL_HPP_
#define WAYBIONIC_RVIZ_PLUGINS__CHICKEN_HEAD_PANEL_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>

#include <QString>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rviz_common/panel.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "waybionic_rviz_plugins/chicken_head_model.hpp"

class QCheckBox;
class QLabel;
class QResizeEvent;
class QTimer;
class QWidget;

namespace waybionic_rviz_plugins
{

struct ChickenHeadMailbox;

/// Switches chicken head mode on and off.
///
/// In chicken head mode the teleop node moves the tool tip to a preset head spot and holds it
/// there while the sticks tilt and roll the tool. The node latches the mode's state on
/// /arm/chicken_head and serves a SetBool service to switch it. The panel only calls that
/// service; it never publishes motion commands or any other topic. The published state is the
/// single source of truth: the checkbox changes only when the topic reports a change, never
/// because it was clicked, so a refused switch (Start not pressed, a joint switched off) leaves it
/// unchecked and the node's reason is shown.
///
/// Threading: ROS callbacks only write into a mutex-guarded mailbox, and a Qt timer drains it, so
/// all widget and bookkeeping state is touched by the Qt thread alone and no service call ever
/// blocks it.
class ChickenHeadPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ChickenHeadPanel(QWidget * parent = nullptr);
  ~ChickenHeadPanel() override;

  void onInitialize() override;

  /// Subscribes to the mode's state and prepares the service client on `node`. onInitialize()
  /// calls this with RViz's node. Calling it again replaces the connection.
  void connectToNode(rclcpp::Node::SharedPtr node);

protected:
  void resizeEvent(QResizeEvent * event) override;

private:
  using SetBool = std_srvs::srv::SetBool;
  using SetBoolClient = rclcpp::Client<SetBool>;

  struct PendingCall
  {
    std::uint64_t token{0};
    std::int64_t request_id{0};
    std::chrono::steady_clock::time_point sent;
  };

  void buildUi();
  void disconnectFromNode();

  void poll();
  void applyMailbox();
  void applySnapshot(HeadSnapshot snapshot);
  void expirePendingCall(std::chrono::steady_clock::time_point now);
  void requestToggle();
  void showStatus(const QString & text);
  void render();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr subscription_;
  SetBoolClient::SharedPtr client_;
  std::shared_ptr<ChickenHeadMailbox> mailbox_;

  // The last valid message, kept only while the teleop node keeps talking.
  HeadSnapshot snapshot_;
  bool has_snapshot_{false};
  std::optional<std::chrono::steady_clock::time_point> last_message_;

  std::optional<PendingCall> pending_;
  std::uint64_t next_token_{0};

  QString status_text_;
  std::chrono::steady_clock::time_point status_expires_;
  QString state_style_;

  QTimer * poll_timer_{nullptr};
  QCheckBox * checkbox_{nullptr};
  QLabel * state_label_{nullptr};
  QLabel * detail_label_{nullptr};
  QLabel * status_label_{nullptr};
};

}  // namespace waybionic_rviz_plugins

#endif  // WAYBIONIC_RVIZ_PLUGINS__CHICKEN_HEAD_PANEL_HPP_
