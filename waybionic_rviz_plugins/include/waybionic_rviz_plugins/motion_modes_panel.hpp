#ifndef WAYBIONIC_RVIZ_PLUGINS__MOTION_MODES_PANEL_HPP_
#define WAYBIONIC_RVIZ_PLUGINS__MOTION_MODES_PANEL_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include <QString>

#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rviz_common/panel.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include "waybionic_rviz_plugins/mode_model.hpp"

class QCheckBox;
class QLabel;
class QResizeEvent;
class QTimer;
class QWidget;

namespace waybionic_rviz_plugins
{

struct ModeMailbox;

/// Switches the teleop node's motion modes on and off: one checkbox per mode.
///
/// In a motion mode the teleop node moves the arm by itself, for example by holding the tool tip
/// at a spot while the sticks tilt the tool, or by moving the arm around slowly. The node latches
/// each mode's state on a topic of its own and serves a SetBool service to switch it. The panel
/// only calls those services; it never publishes motion commands or any other topic. The
/// published state is the single source of truth: a checkbox changes only when the topic reports
/// a change, never because it was clicked, so a refused switch (Start not pressed, a joint
/// switched off) leaves it as it was and the node's reason is shown.
///
/// Threading: ROS callbacks only write into a mutex-guarded mailbox, and a Qt timer drains it, so
/// all widget and bookkeeping state is touched by the Qt thread alone and no service call ever
/// blocks it.
class MotionModesPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit MotionModesPanel(QWidget * parent = nullptr);
  ~MotionModesPanel() override;

  void onInitialize() override;

  /// Subscribes to every mode's state and prepares the service clients on `node`.
  /// onInitialize() calls this with RViz's node. Calling it again replaces the connection.
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

  /// A mode: what it reports and says, the connection to its node, and its widgets.
  struct Row
  {
    const ModeSpec * spec{nullptr};
    std::shared_ptr<ModeMailbox> mailbox;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr subscription;
    SetBoolClient::SharedPtr client;
    // The last valid message, kept only while the teleop node keeps talking.
    ModeSnapshot snapshot;
    bool has_snapshot{false};
    std::optional<std::chrono::steady_clock::time_point> last_message;
    std::optional<PendingCall> pending;
    QCheckBox * checkbox{nullptr};
    QLabel * state{nullptr};
    QLabel * detail{nullptr};
    QString state_style;
    QString detail_style;
  };

  void buildUi();
  void disconnectFromNode();

  void poll();
  void applyMailbox(Row & row);
  void applySnapshot(Row & row, ModeSnapshot snapshot);
  void expirePendingCall(Row & row, std::chrono::steady_clock::time_point now);
  void requestToggle(Row & row);
  void showStatus(const QString & text);
  void render();

  rclcpp::Node::SharedPtr node_;
  std::vector<Row> rows_;
  std::uint64_t next_token_{0};

  QString status_text_;
  std::chrono::steady_clock::time_point status_expires_;

  QTimer * poll_timer_{nullptr};
  QLabel * connection_label_{nullptr};
  QLabel * status_label_{nullptr};
};

}  // namespace waybionic_rviz_plugins

#endif  // WAYBIONIC_RVIZ_PLUGINS__MOTION_MODES_PANEL_HPP_
