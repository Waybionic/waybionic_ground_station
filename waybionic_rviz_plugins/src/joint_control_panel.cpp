#include "waybionic_rviz_plugins/joint_control_panel.hpp"

#include <algorithm>
#include <deque>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <QCheckBox>
#include <QGridLayout>
#include <QLabel>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QTimer>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace waybionic_rviz_plugins
{
namespace
{

constexpr int kPollIntervalMs = 100;
constexpr std::chrono::seconds kServiceTimeout{5};
constexpr std::chrono::seconds kStatusLifetime{8};

constexpr const char * kPanelStyle = R"(
QWidget {
  background-color: #071019;
  color: #e8f1f8;
  font-family: "Segoe UI", "Ubuntu", sans-serif;
  font-size: 12px;
}
QLabel#PanelTitle {
  font-size: 15px;
  font-weight: 700;
}
QLabel#Muted {
  color: #8ea3b1;
}
QLabel#Warning {
  color: #ffb84d;
  font-weight: 700;
}
QCheckBox {
  font-weight: 700;
  spacing: 8px;
}
QCheckBox:disabled {
  color: #5d7282;
}
QScrollArea {
  border: none;
}
)";

constexpr const char * kEnabledStyle = "color: #3ddc84; font-weight: 700;";
constexpr const char * kDisabledStyle = "color: #ffb84d; font-weight: 700;";
constexpr const char * kNoDriveStyle = "color: #8ea3b1;";
constexpr const char * kWaitingStyle = "color: #5d7282;";

const char * styleFor(const JointState state)
{
  switch (state) {
    case JointState::Enabled:
      return kEnabledStyle;
    case JointState::Disabled:
      return kDisabledStyle;
    case JointState::NoDrive:
      return kNoDriveStyle;
  }
  return kWaitingStyle;
}

}  // namespace

/// What the ROS threads hand to the Qt thread. Callbacks only ever write here.
struct JointControlMailbox
{
  struct Reply
  {
    std::string joint_name;
    std::uint64_t token{0};
    bool success{false};
    std::string message;
  };

  std::mutex mutex;
  std::optional<JointControlSnapshot> snapshot;
  std::optional<std::chrono::steady_clock::time_point> received_at;
  std::deque<Reply> replies;
};

JointControlPanel::JointControlPanel(QWidget * parent)
: rviz_common::Panel(parent), mailbox_(std::make_shared<JointControlMailbox>())
{
  buildUi();
  poll_timer_ = new QTimer(this);
  connect(poll_timer_, &QTimer::timeout, this, [this]() {poll();});
  poll_timer_->start(kPollIntervalMs);
  render();
}

JointControlPanel::~JointControlPanel()
{
  if (poll_timer_ != nullptr) {
    poll_timer_->stop();
  }
  disconnectFromNode();
}

void JointControlPanel::onInitialize()
{
  if (auto ros_node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock()) {
    connectToNode(ros_node_abstraction->get_raw_node());
  }
}

void JointControlPanel::connectToNode(rclcpp::Node::SharedPtr node)
{
  disconnectFromNode();
  node_ = std::move(node);
  if (!node_) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mailbox_->mutex);
    mailbox_->snapshot.reset();
    mailbox_->received_at.reset();
    mailbox_->replies.clear();
  }
  subscription_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    kJointControlTopic, rclcpp::QoS(1).reliable().transient_local(),
    [mailbox = mailbox_](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message) {
      auto snapshot = parseJointControl(*message);
      std::lock_guard<std::mutex> lock(mailbox->mutex);
      mailbox->snapshot = std::move(snapshot);
      mailbox->received_at = std::chrono::steady_clock::now();
    });
}

void JointControlPanel::disconnectFromNode()
{
  subscription_.reset();
  for (const auto & [joint_name, call] : pending_) {
    const auto client = clients_.find(joint_name);
    if (client != clients_.end()) {
      client->second->remove_pending_request(call.request_id);
    }
  }
  pending_.clear();
  clients_.clear();
  node_.reset();
  has_snapshot_ = false;
  snapshot_ = JointControlSnapshot{};
  last_message_.reset();
}

void JointControlPanel::buildUi()
{
  setStyleSheet(kPanelStyle);
  setMinimumWidth(320);

  auto * root_layout = new QVBoxLayout(this);
  root_layout->setContentsMargins(10, 10, 10, 10);
  root_layout->setSpacing(6);

  auto * title = new QLabel("Joint control");
  title->setObjectName("PanelTitle");
  root_layout->addWidget(title);

  auto * hint = new QLabel(
    "A checked joint moves with the sticks. An unchecked joint is held where it is. "
    "Switch joints with teleop stopped.");
  hint->setObjectName("Muted");
  hint->setWordWrap(true);
  root_layout->addWidget(hint);

  connection_label_ = new QLabel();
  connection_label_->setObjectName("Muted");
  connection_label_->setWordWrap(true);
  root_layout->addWidget(connection_label_);

  rows_scroll_ = new QScrollArea();
  rows_scroll_->setWidgetResizable(true);
  rows_scroll_->setMinimumHeight(200);
  rows_widget_ = new QWidget();
  rows_layout_ = new QGridLayout(rows_widget_);
  rows_layout_->setContentsMargins(0, 0, 0, 0);
  rows_layout_->setHorizontalSpacing(12);
  rows_layout_->setVerticalSpacing(6);
  rows_layout_->setColumnStretch(2, 1);
  rows_scroll_->setWidget(rows_widget_);
  root_layout->addWidget(rows_scroll_, 1);

  status_label_ = new QLabel();
  status_label_->setObjectName("Warning");
  status_label_->setWordWrap(true);
  root_layout->addWidget(status_label_);
}

void JointControlPanel::clearRows()
{
  // A checkbox may be running its own click handler, so the widgets go when Qt is done.
  for (auto & row : row_widgets_) {
    for (QWidget * widget : {static_cast<QWidget *>(row.checkbox),
        static_cast<QWidget *>(row.state), static_cast<QWidget *>(row.detail)})
    {
      rows_layout_->removeWidget(widget);
      widget->hide();
      widget->deleteLater();
    }
  }
  row_widgets_.clear();
  row_names_.clear();
}

void JointControlPanel::rebuildRows(const std::vector<std::string> & joint_names)
{
  clearRows();
  row_names_ = joint_names;
  int line = 0;
  for (const auto & joint_name : joint_names) {
    RowWidgets row;
    row.checkbox = new QCheckBox(QString::fromStdString(jointLabel(joint_name)));
    row.state = new QLabel();
    row.detail = new QLabel();
    row.detail->setObjectName("Muted");
    row.detail->setWordWrap(true);
    connect(row.checkbox, &QCheckBox::clicked, this, [this, joint_name](bool) {
        // The checkbox follows the published state, not the click: the click only asks.
        requestToggle(joint_name);
        render();
      });
    rows_layout_->addWidget(row.checkbox, line, 0);
    rows_layout_->addWidget(row.state, line, 1);
    rows_layout_->addWidget(row.detail, line, 2);
    row_widgets_.push_back(row);
    ++line;
  }
  rows_layout_->setRowStretch(line, 1);
}

const JointRow * JointControlPanel::findRow(const std::string & joint_name) const
{
  const auto found = std::find_if(
    snapshot_.rows.begin(), snapshot_.rows.end(),
    [&joint_name](const JointRow & row) {return row.joint_name == joint_name;});
  return found == snapshot_.rows.end() ? nullptr : &*found;
}

void JointControlPanel::poll()
{
  applyMailbox();
  expirePendingCalls(std::chrono::steady_clock::now());
  render();
}

void JointControlPanel::applyMailbox()
{
  std::optional<JointControlSnapshot> snapshot;
  std::optional<std::chrono::steady_clock::time_point> received_at;
  std::deque<JointControlMailbox::Reply> replies;
  {
    std::lock_guard<std::mutex> lock(mailbox_->mutex);
    snapshot.swap(mailbox_->snapshot);
    received_at = mailbox_->received_at;
    replies.swap(mailbox_->replies);
  }
  if (received_at.has_value()) {
    last_message_ = received_at;
  }
  if (snapshot.has_value()) {
    applySnapshot(std::move(*snapshot));
  }
  for (const auto & reply : replies) {
    const auto call = pending_.find(reply.joint_name);
    if (call == pending_.end() || call->second.token != reply.token) {
      continue;  // Answers a call that already timed out.
    }
    pending_.erase(call);
    if (!reply.success) {
      const auto * row = findRow(reply.joint_name);
      const auto label = row != nullptr ? row->label : reply.joint_name;
      showStatus(QString::fromStdString(label + ": " + reply.message));
    }
  }
}

void JointControlPanel::applySnapshot(JointControlSnapshot snapshot)
{
  if (!snapshot.valid) {
    // Keep showing the last good report, and say that this one was ignored.
    showStatus(QString::fromStdString("Ignoring a malformed joint report: " + snapshot.error));
    return;
  }
  snapshot_ = std::move(snapshot);
  has_snapshot_ = true;
}

void JointControlPanel::expirePendingCalls(const std::chrono::steady_clock::time_point now)
{
  std::vector<std::string> expired;
  for (const auto & [joint_name, call] : pending_) {
    if (now - call.sent > kServiceTimeout) {
      expired.push_back(joint_name);
    }
  }
  for (const auto & joint_name : expired) {
    const auto call = pending_.at(joint_name);
    forgetCall(joint_name, call);
    const auto * row = findRow(joint_name);
    const auto label = row != nullptr ? row->label : joint_name;
    showStatus(QString::fromStdString("No answer from the drive node for " + label));
  }
}

void JointControlPanel::forgetCall(const std::string & joint_name, const PendingCall & call)
{
  const auto client = clients_.find(joint_name);
  if (client != clients_.end()) {
    client->second->remove_pending_request(call.request_id);
  }
  pending_.erase(joint_name);
}

JointControlPanel::SetBoolClient::SharedPtr JointControlPanel::clientFor(
  const std::string & joint_name)
{
  const auto found = clients_.find(joint_name);
  if (found != clients_.end()) {
    return found->second;
  }
  auto client = node_->create_client<SetBool>(setEnabledServiceName(joint_name));
  clients_[joint_name] = client;
  return client;
}

void JointControlPanel::requestToggle(const std::string & joint_name)
{
  if (!node_ || !has_snapshot_ || pending_.count(joint_name) > 0 ||
    driveNodeQuiet(last_message_, std::chrono::steady_clock::now()))
  {
    return;
  }
  const auto * row = findRow(joint_name);
  if (row == nullptr || !row->serviceable) {
    return;
  }
  const auto client = clientFor(joint_name);
  if (!client->service_is_ready()) {
    showStatus(QString::fromStdString(row->label + ": drive node not available"));
    return;
  }
  auto request = std::make_shared<SetBool::Request>();
  request->data = row->state != JointState::Enabled;
  PendingCall call;
  call.token = ++next_token_;
  call.enable = request->data;
  call.sent = std::chrono::steady_clock::now();
  const auto sent = client->async_send_request(
    request,
    [mailbox = mailbox_, joint_name, token = call.token](SetBoolClient::SharedFuture future) {
      const auto response = future.get();
      std::lock_guard<std::mutex> lock(mailbox->mutex);
      mailbox->replies.push_back({joint_name, token, response->success, response->message});
    });
  call.request_id = sent.request_id;
  pending_[joint_name] = call;
}

void JointControlPanel::showStatus(const QString & text)
{
  status_text_ = text;
  status_expires_ = std::chrono::steady_clock::now() + kStatusLifetime;
}

void JointControlPanel::render()
{
  const auto now = std::chrono::steady_clock::now();
  const bool quiet = driveNodeQuiet(last_message_, now);
  const bool shown = has_snapshot_ && !quiet;

  if (quiet) {
    connection_label_->setText("Waiting for the drive node (sim_arm_drives)...");
  } else if (!has_snapshot_) {
    connection_label_->setText("Waiting for a valid joint report...");
  } else {
    connection_label_->setText("Drive node connected");
  }

  std::vector<std::string> names = knownJointNames();
  if (has_snapshot_) {
    names.clear();
    for (const auto & row : snapshot_.rows) {
      names.push_back(row.joint_name);
    }
  }
  if (names != row_names_) {
    rebuildRows(names);
  }

  for (std::size_t index = 0; index < row_names_.size(); ++index) {
    auto & widgets = row_widgets_[index];
    const auto * row = shown ? findRow(row_names_[index]) : nullptr;
    const QSignalBlocker blocker(widgets.checkbox);
    if (row == nullptr) {
      widgets.checkbox->setChecked(false);
      widgets.checkbox->setEnabled(false);
      widgets.state->setText(QString::fromUtf8("\xE2\x80\x94"));
      widgets.state->setStyleSheet(kWaitingStyle);
      widgets.detail->setText(QString());
      continue;
    }
    const auto call = pending_.find(row->joint_name);
    const bool waiting = call != pending_.end();
    widgets.checkbox->setChecked(row->state == JointState::Enabled);
    widgets.checkbox->setEnabled(row->serviceable && !waiting);
    QString word = QString::fromUtf8(stateWord(row->state));
    if (waiting) {
      word = call->second.enable ? "enabling..." : "disabling...";
    }
    widgets.state->setText(word);
    const QString style = QString::fromUtf8(styleFor(row->state));
    if (widgets.state_style != style) {
      widgets.state->setStyleSheet(style);
      widgets.state_style = style;
    }
    const auto detail = QString::fromStdString(row->message);
    widgets.detail->setText(detail);
    widgets.checkbox->setToolTip(detail);
  }

  status_label_->setText(now < status_expires_ ? status_text_ : QString());
}

}  // namespace waybionic_rviz_plugins

PLUGINLIB_EXPORT_CLASS(waybionic_rviz_plugins::JointControlPanel, rviz_common::Panel)
