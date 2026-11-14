#include "waybionic_rviz_plugins/motion_modes_panel.hpp"

#include <deque>
#include <mutex>
#include <string>
#include <utility>

#include <QCheckBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QResizeEvent>
#include <QSignalBlocker>
#include <QTimer>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include "waybionic_rviz_plugins/panel_fit.hpp"

namespace waybionic_rviz_plugins
{
namespace
{

constexpr int kPollIntervalMs = 100;
// The most height the panel asks of its dock for its text; a dock that is shorter scrolls it.
constexpr int kMaxFittedHeight = 360;
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
)";

constexpr const char * kActiveStyle = "color: #3ddc84; font-weight: 700;";
constexpr const char * kWarningStyle = "color: #ffb84d; font-weight: 700;";
constexpr const char * kQuietStyle = "color: #8ea3b1;";
constexpr const char * kWaitingStyle = "color: #5d7282;";

const char * styleFor(const Tone tone)
{
  switch (tone) {
    case Tone::Active:
      return kActiveStyle;
    case Tone::Warning:
      return kWarningStyle;
    case Tone::Quiet:
      return kQuietStyle;
  }
  return kWaitingStyle;
}

}  // namespace

/// What the ROS threads hand to the Qt thread, for one mode. Callbacks only ever write here.
struct ModeMailbox
{
  struct Reply
  {
    std::uint64_t token{0};
    bool success{false};
    std::string message;
  };

  std::mutex mutex;
  std::optional<ModeSnapshot> snapshot;
  std::optional<std::chrono::steady_clock::time_point> received_at;
  std::deque<Reply> replies;
};

MotionModesPanel::MotionModesPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  buildUi();
  poll_timer_ = new QTimer(this);
  connect(poll_timer_, &QTimer::timeout, this, [this]() {poll();});
  poll_timer_->start(kPollIntervalMs);
  render();
}

MotionModesPanel::~MotionModesPanel()
{
  if (poll_timer_ != nullptr) {
    poll_timer_->stop();
  }
  disconnectFromNode();
}

void MotionModesPanel::onInitialize()
{
  if (auto ros_node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock()) {
    connectToNode(ros_node_abstraction->get_raw_node());
  }
}

void MotionModesPanel::connectToNode(rclcpp::Node::SharedPtr node)
{
  disconnectFromNode();
  node_ = std::move(node);
  if (!node_) {
    return;
  }
  for (auto & row : rows_) {
    {
      std::lock_guard<std::mutex> lock(row.mailbox->mutex);
      row.mailbox->snapshot.reset();
      row.mailbox->received_at.reset();
      row.mailbox->replies.clear();
    }
    row.subscription = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      row.spec->topic, rclcpp::QoS(1).reliable().transient_local(),
      [mailbox = row.mailbox, spec = row.spec](
        diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message) {
        auto snapshot = parseMode(*message, *spec);
        std::lock_guard<std::mutex> lock(mailbox->mutex);
        mailbox->snapshot = std::move(snapshot);
        mailbox->received_at = std::chrono::steady_clock::now();
      });
    row.client = node_->create_client<SetBool>(row.spec->service);
  }
}

void MotionModesPanel::disconnectFromNode()
{
  for (auto & row : rows_) {
    row.subscription.reset();
    if (row.pending.has_value() && row.client) {
      row.client->remove_pending_request(row.pending->request_id);
    }
    row.pending.reset();
    row.client.reset();
    row.has_snapshot = false;
    row.snapshot = ModeSnapshot{};
    row.last_message.reset();
  }
  node_.reset();
}

void MotionModesPanel::buildUi()
{
  setStyleSheet(kPanelStyle);
  setMinimumWidth(320);

  auto * root_layout = new QVBoxLayout(this);
  root_layout->setContentsMargins(10, 10, 10, 10);
  root_layout->setSpacing(6);

  auto * title = new QLabel("Motion modes");
  title->setObjectName("PanelTitle");
  root_layout->addWidget(title);

  connection_label_ = new QLabel();
  connection_label_->setObjectName("Muted");
  connection_label_->setWordWrap(true);
  root_layout->addWidget(connection_label_);

  for (const ModeSpec * spec : modeSpecs()) {
    Row row;
    row.spec = spec;
    row.mailbox = std::make_shared<ModeMailbox>();
    row.checkbox = new QCheckBox(QString::fromStdString(spec->label));
    row.state = new QLabel();
    row.detail = new QLabel();
    row.detail->setObjectName("Muted");
    row.detail->setWordWrap(true);

    auto * switch_row = new QHBoxLayout();
    switch_row->addWidget(row.checkbox);
    switch_row->addStretch(1);
    switch_row->addWidget(row.state);
    root_layout->addLayout(switch_row);
    root_layout->addWidget(row.detail);

    // The checkbox follows the published state, not the click: the click only asks.
    const auto index = rows_.size();
    connect(row.checkbox, &QCheckBox::clicked, this, [this, index](bool) {
        requestToggle(rows_[index]);
        render();
      });
    rows_.push_back(std::move(row));
  }

  status_label_ = new QLabel();
  status_label_->setObjectName("Warning");
  status_label_->setWordWrap(true);
  root_layout->addWidget(status_label_);
  root_layout->addStretch(1);
}

void MotionModesPanel::resizeEvent(QResizeEvent * event)
{
  rviz_common::Panel::resizeEvent(event);
  fitPanelHeight(this, kMaxFittedHeight, true);
}

void MotionModesPanel::poll()
{
  const auto now = std::chrono::steady_clock::now();
  for (auto & row : rows_) {
    applyMailbox(row);
    expirePendingCall(row, now);
  }
  render();
}

void MotionModesPanel::applyMailbox(Row & row)
{
  std::optional<ModeSnapshot> snapshot;
  std::optional<std::chrono::steady_clock::time_point> received_at;
  std::deque<ModeMailbox::Reply> replies;
  {
    std::lock_guard<std::mutex> lock(row.mailbox->mutex);
    snapshot.swap(row.mailbox->snapshot);
    received_at = row.mailbox->received_at;
    replies.swap(row.mailbox->replies);
  }
  if (received_at.has_value()) {
    row.last_message = received_at;
  }
  if (snapshot.has_value()) {
    applySnapshot(row, std::move(*snapshot));
  }
  for (const auto & reply : replies) {
    if (!row.pending.has_value() || row.pending->token != reply.token) {
      continue;  // Answers a call that already timed out.
    }
    row.pending.reset();
    if (!reply.success) {
      showStatus(QString::fromStdString(reply.message));
    }
  }
}

void MotionModesPanel::applySnapshot(Row & row, ModeSnapshot snapshot)
{
  if (!snapshot.valid) {
    // Keep showing the last good report, and say that this one was ignored.
    showStatus(
      QString::fromStdString(
        "Ignoring a malformed " + row.spec->row + " report: " + snapshot.error));
    return;
  }
  row.snapshot = std::move(snapshot);
  row.has_snapshot = true;
}

void MotionModesPanel::expirePendingCall(Row & row, const std::chrono::steady_clock::time_point now)
{
  if (!row.pending.has_value() || now - row.pending->sent <= kServiceTimeout) {
    return;
  }
  row.client->remove_pending_request(row.pending->request_id);
  row.pending.reset();
  showStatus("No answer from the teleop node");
}

void MotionModesPanel::requestToggle(Row & row)
{
  if (!node_ || !row.client || !row.has_snapshot || row.pending.has_value() ||
    teleopQuiet(row.last_message, std::chrono::steady_clock::now()))
  {
    return;
  }
  if (!row.client->service_is_ready()) {
    showStatus("The teleop node is not available");
    return;
  }
  auto request = std::make_shared<SetBool::Request>();
  request->data = !row.snapshot.state.on;
  PendingCall call;
  call.token = ++next_token_;
  call.sent = std::chrono::steady_clock::now();
  const auto sent = row.client->async_send_request(
    request,
    [mailbox = row.mailbox, token = call.token](SetBoolClient::SharedFuture future) {
      const auto response = future.get();
      std::lock_guard<std::mutex> lock(mailbox->mutex);
      mailbox->replies.push_back({token, response->success, response->message});
    });
  call.request_id = sent.request_id;
  row.pending = call;
}

void MotionModesPanel::showStatus(const QString & text)
{
  status_text_ = text;
  status_expires_ = std::chrono::steady_clock::now() + kStatusLifetime;
}

void MotionModesPanel::render()
{
  const auto now = std::chrono::steady_clock::now();
  QString waiting_for;

  for (auto & row : rows_) {
    const bool quiet = teleopQuiet(row.last_message, now);
    const bool shown = row.has_snapshot && !quiet;
    const bool pending = row.pending.has_value();
    if (!shown && waiting_for.isEmpty()) {
      waiting_for = QString::fromStdString(modeWaitingMessage(row.has_snapshot, quiet));
    }

    QString word = QString::fromUtf8("\xE2\x80\x94");
    QString style = kWaitingStyle;
    QString detail = QString::fromStdString(row.spec->hint);
    QString detail_style;
    bool on = false;
    if (shown) {
      on = row.snapshot.state.on;
      word = QString::fromStdString(row.snapshot.state.word);
      style = QString::fromUtf8(styleFor(row.snapshot.state.tone));
      if (!row.snapshot.message.empty()) {
        detail = QString::fromStdString(row.snapshot.message);
        if (row.snapshot.warning) {
          detail_style = kWarningStyle;
        }
      }
      if (pending) {
        word = on ? "switching off..." : "switching on...";
      }
    }

    {
      const QSignalBlocker blocker(row.checkbox);
      row.checkbox->setChecked(on);
      row.checkbox->setEnabled(shown && !pending);
    }
    row.state->setText(word);
    if (row.state_style != style) {
      row.state->setStyleSheet(style);
      row.state_style = style;
    }
    row.detail->setText(detail);
    if (row.detail_style != detail_style) {
      row.detail->setStyleSheet(detail_style);
      row.detail_style = detail_style;
    }
    row.checkbox->setToolTip(detail);
  }

  connection_label_->setText(waiting_for);
  connection_label_->setVisible(!waiting_for.isEmpty());
  status_label_->setText(now < status_expires_ ? status_text_ : QString());
  fitPanelHeight(this, kMaxFittedHeight, true);
}

}  // namespace waybionic_rviz_plugins

PLUGINLIB_EXPORT_CLASS(waybionic_rviz_plugins::MotionModesPanel, rviz_common::Panel)
