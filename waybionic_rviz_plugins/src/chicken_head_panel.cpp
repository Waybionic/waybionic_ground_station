#include "waybionic_rviz_plugins/chicken_head_panel.hpp"

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
constexpr int kMaxFittedHeight = 240;
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

constexpr const char * kRunningStyle = "color: #3ddc84; font-weight: 700;";
constexpr const char * kBlockedStyle = "color: #ffb84d; font-weight: 700;";
constexpr const char * kOffStyle = "color: #8ea3b1;";
constexpr const char * kWaitingStyle = "color: #5d7282;";

constexpr const char * kHint =
  "Moves the tool tip to a preset spot and holds it there while the sticks tilt and roll the "
  "tool.";

const char * styleFor(const HeadState state)
{
  switch (state) {
    case HeadState::Approach:
    case HeadState::Hold:
      return kRunningStyle;
    case HeadState::Blocked:
      return kBlockedStyle;
    case HeadState::Off:
    case HeadState::Unavailable:
      return kOffStyle;
  }
  return kWaitingStyle;
}

}  // namespace

/// What the ROS threads hand to the Qt thread. Callbacks only ever write here.
struct ChickenHeadMailbox
{
  struct Reply
  {
    std::uint64_t token{0};
    bool success{false};
    std::string message;
  };

  std::mutex mutex;
  std::optional<HeadSnapshot> snapshot;
  std::optional<std::chrono::steady_clock::time_point> received_at;
  std::deque<Reply> replies;
};

ChickenHeadPanel::ChickenHeadPanel(QWidget * parent)
: rviz_common::Panel(parent), mailbox_(std::make_shared<ChickenHeadMailbox>())
{
  buildUi();
  poll_timer_ = new QTimer(this);
  connect(poll_timer_, &QTimer::timeout, this, [this]() {poll();});
  poll_timer_->start(kPollIntervalMs);
  render();
}

ChickenHeadPanel::~ChickenHeadPanel()
{
  if (poll_timer_ != nullptr) {
    poll_timer_->stop();
  }
  disconnectFromNode();
}

void ChickenHeadPanel::onInitialize()
{
  if (auto ros_node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock()) {
    connectToNode(ros_node_abstraction->get_raw_node());
  }
}

void ChickenHeadPanel::connectToNode(rclcpp::Node::SharedPtr node)
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
    kChickenHeadTopic, rclcpp::QoS(1).reliable().transient_local(),
    [mailbox = mailbox_](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message) {
      auto snapshot = parseChickenHead(*message);
      std::lock_guard<std::mutex> lock(mailbox->mutex);
      mailbox->snapshot = std::move(snapshot);
      mailbox->received_at = std::chrono::steady_clock::now();
    });
  client_ = node_->create_client<SetBool>(kChickenHeadService);
}

void ChickenHeadPanel::disconnectFromNode()
{
  subscription_.reset();
  if (pending_.has_value() && client_) {
    client_->remove_pending_request(pending_->request_id);
  }
  pending_.reset();
  client_.reset();
  node_.reset();
  has_snapshot_ = false;
  snapshot_ = HeadSnapshot{};
  last_message_.reset();
}

void ChickenHeadPanel::buildUi()
{
  setStyleSheet(kPanelStyle);
  setMinimumWidth(320);

  auto * root_layout = new QVBoxLayout(this);
  root_layout->setContentsMargins(10, 10, 10, 10);
  root_layout->setSpacing(6);

  auto * title = new QLabel("Chicken head");
  title->setObjectName("PanelTitle");
  root_layout->addWidget(title);

  auto * switch_row = new QHBoxLayout();
  checkbox_ = new QCheckBox("Chicken head mode");
  state_label_ = new QLabel();
  switch_row->addWidget(checkbox_);
  switch_row->addStretch(1);
  switch_row->addWidget(state_label_);
  root_layout->addLayout(switch_row);
  // The checkbox follows the published state, not the click: the click only asks.
  connect(checkbox_, &QCheckBox::clicked, this, [this](bool) {
      requestToggle();
      render();
    });

  detail_label_ = new QLabel();
  detail_label_->setObjectName("Muted");
  detail_label_->setWordWrap(true);
  root_layout->addWidget(detail_label_);

  status_label_ = new QLabel();
  status_label_->setObjectName("Warning");
  status_label_->setWordWrap(true);
  root_layout->addWidget(status_label_);
  root_layout->addStretch(1);
}

void ChickenHeadPanel::resizeEvent(QResizeEvent * event)
{
  rviz_common::Panel::resizeEvent(event);
  fitPanelHeight(this, kMaxFittedHeight, true);
}

void ChickenHeadPanel::poll()
{
  applyMailbox();
  expirePendingCall(std::chrono::steady_clock::now());
  render();
}

void ChickenHeadPanel::applyMailbox()
{
  std::optional<HeadSnapshot> snapshot;
  std::optional<std::chrono::steady_clock::time_point> received_at;
  std::deque<ChickenHeadMailbox::Reply> replies;
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
    if (!pending_.has_value() || pending_->token != reply.token) {
      continue;  // Answers a call that already timed out.
    }
    pending_.reset();
    if (!reply.success) {
      showStatus(QString::fromStdString(reply.message));
    }
  }
}

void ChickenHeadPanel::applySnapshot(HeadSnapshot snapshot)
{
  if (!snapshot.valid) {
    // Keep showing the last good report, and say that this one was ignored.
    showStatus(
      QString::fromStdString("Ignoring a malformed chicken head report: " + snapshot.error));
    return;
  }
  snapshot_ = std::move(snapshot);
  has_snapshot_ = true;
}

void ChickenHeadPanel::expirePendingCall(const std::chrono::steady_clock::time_point now)
{
  if (!pending_.has_value() || now - pending_->sent <= kServiceTimeout) {
    return;
  }
  client_->remove_pending_request(pending_->request_id);
  pending_.reset();
  showStatus("No answer from the teleop node");
}

void ChickenHeadPanel::requestToggle()
{
  if (!node_ || !client_ || !has_snapshot_ || pending_.has_value() ||
    teleopQuiet(last_message_, std::chrono::steady_clock::now()))
  {
    return;
  }
  if (!client_->service_is_ready()) {
    showStatus("The teleop node is not available");
    return;
  }
  auto request = std::make_shared<SetBool::Request>();
  request->data = !headIsOn(snapshot_.state);
  PendingCall call;
  call.token = ++next_token_;
  call.sent = std::chrono::steady_clock::now();
  const auto sent = client_->async_send_request(
    request,
    [mailbox = mailbox_, token = call.token](SetBoolClient::SharedFuture future) {
      const auto response = future.get();
      std::lock_guard<std::mutex> lock(mailbox->mutex);
      mailbox->replies.push_back({token, response->success, response->message});
    });
  call.request_id = sent.request_id;
  pending_ = call;
}

void ChickenHeadPanel::showStatus(const QString & text)
{
  status_text_ = text;
  status_expires_ = std::chrono::steady_clock::now() + kStatusLifetime;
}

void ChickenHeadPanel::render()
{
  const auto now = std::chrono::steady_clock::now();
  const bool quiet = teleopQuiet(last_message_, now);
  const bool shown = has_snapshot_ && !quiet;
  const bool waiting = pending_.has_value();

  QString word = QString::fromUtf8("\xE2\x80\x94");
  QString style = kWaitingStyle;
  QString detail = QString::fromStdString(headWaitingMessage(has_snapshot_, quiet));
  bool on = false;
  if (shown) {
    on = headIsOn(snapshot_.state);
    word = QString::fromUtf8(headStateWord(snapshot_.state));
    style = QString::fromUtf8(styleFor(snapshot_.state));
    detail = snapshot_.message.empty() ? QString::fromUtf8(kHint) :
      QString::fromStdString(snapshot_.message);
    if (waiting) {
      word = on ? "switching off..." : "switching on...";
    }
  }

  {
    const QSignalBlocker blocker(checkbox_);
    checkbox_->setChecked(on);
    checkbox_->setEnabled(shown && !waiting);
  }
  state_label_->setText(word);
  if (state_style_ != style) {
    state_label_->setStyleSheet(style);
    state_style_ = style;
  }
  detail_label_->setText(detail);
  checkbox_->setToolTip(detail);
  status_label_->setText(now < status_expires_ ? status_text_ : QString());
  fitPanelHeight(this, kMaxFittedHeight, true);
}

}  // namespace waybionic_rviz_plugins

PLUGINLIB_EXPORT_CLASS(waybionic_rviz_plugins::ChickenHeadPanel, rviz_common::Panel)
