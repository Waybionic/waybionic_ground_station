#include "waybionic_rviz_plugins/setup_panel.hpp"

#include <algorithm>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <QDockWidget>
#include <QGridLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QResizeEvent>
#include <QStyle>
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
// The most height the panel asks of its dock for its text; a dock that is shorter scrolls it.
constexpr int kMaxFittedHeight = 420;
// The drive node waits up to half a second for the drives' replies before it answers.
constexpr std::chrono::seconds kZeroTimeout{10};
constexpr std::chrono::seconds kStatusLifetime{10};

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
QLabel#Ready {
  color: #3ddc84;
  font-weight: 700;
}
QPushButton {
  background-color: #101d2a;
  border: 1px solid #43a6ff;
  border-radius: 6px;
  color: #e8f1f8;
  font-weight: 700;
  padding: 6px;
}
QPushButton:disabled {
  border-color: #284052;
  color: #5d7282;
}
)";

struct StepLook
{
  const char * icon;
  const char * color;
};

StepLook lookFor(const StepState state)
{
  switch (state) {
    case StepState::Ok:
      return {"\xE2\x9C\x93", "#3ddc84"};  // check mark
    case StepState::Working:
      return {"\xE2\x80\xA6", "#43a6ff"};  // ellipsis
    case StepState::Todo:
      return {"\xE2\x97\x8F", "#ffb84d"};  // black circle
    case StepState::Blocked:
      return {"\xE2\x9C\x97", "#ff4d5e"};  // ballot x
  }
  return {"?", "#8ea3b1"};
}

}  // namespace

/// What the ROS threads hand to the Qt thread. Callbacks only ever write here.
struct SetupMailbox
{
  struct ZeroReply
  {
    std::uint64_t token{0};
    bool success{false};
    std::string message;
  };

  std::mutex mutex;
  std::optional<SetupSnapshot> snapshot;
  std::optional<std::chrono::steady_clock::time_point> received_at;
  std::optional<ZeroReply> zero_reply;
};

SetupPanel::SetupPanel(QWidget * parent)
: rviz_common::Panel(parent), mailbox_(std::make_shared<SetupMailbox>())
{
  buildUi();
  poll_timer_ = new QTimer(this);
  connect(poll_timer_, &QTimer::timeout, this, [this]() {poll();});
  poll_timer_->start(kPollIntervalMs);
  render();
}

SetupPanel::~SetupPanel()
{
  if (poll_timer_ != nullptr) {
    poll_timer_->stop();
  }
  disconnectFromNode();
}

void SetupPanel::onInitialize()
{
  if (auto ros_node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock()) {
    connectToNode(ros_node_abstraction->get_raw_node());
  }
}

void SetupPanel::connectToNode(rclcpp::Node::SharedPtr node)
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
    mailbox_->zero_reply.reset();
  }
  subscription_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    kSetupTopic, rclcpp::QoS(1).reliable().transient_local(),
    [mailbox = mailbox_](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message) {
      auto snapshot = parseSetup(*message);
      std::lock_guard<std::mutex> lock(mailbox->mutex);
      mailbox->snapshot = std::move(snapshot);
      mailbox->received_at = std::chrono::steady_clock::now();
    });
  zero_client_ = node_->create_client<Trigger>(kZeroServiceName);
}

void SetupPanel::disconnectFromNode()
{
  subscription_.reset();
  if (pending_zero_.has_value() && zero_client_) {
    zero_client_->remove_pending_request(pending_zero_->request_id);
  }
  pending_zero_.reset();
  zero_client_.reset();
  node_.reset();
  has_snapshot_ = false;
  seen_guide_ = false;
  snapshot_ = SetupSnapshot{};
  last_message_.reset();
  dock_policy_ = SetupDockPolicy();
}

void SetupPanel::buildUi()
{
  setStyleSheet(kPanelStyle);
  setMinimumWidth(320);

  auto * root_layout = new QVBoxLayout(this);
  root_layout->setContentsMargins(10, 10, 10, 10);
  root_layout->setSpacing(6);

  auto * title = new QLabel("Arm setup");
  title->setObjectName("PanelTitle");
  root_layout->addWidget(title);

  summary_label_ = new QLabel();
  summary_label_->setObjectName("Muted");
  summary_label_->setWordWrap(true);
  root_layout->addWidget(summary_label_);

  rows_widget_ = new QWidget();
  rows_layout_ = new QGridLayout(rows_widget_);
  rows_layout_->setContentsMargins(0, 0, 0, 0);
  rows_layout_->setHorizontalSpacing(8);
  rows_layout_->setVerticalSpacing(6);
  rows_layout_->setColumnStretch(2, 1);
  root_layout->addWidget(rows_widget_);

  zero_button_ = new QPushButton("Zero the arm");
  zero_button_->setToolTip(
    "Makes the arm's present position the zero of every drive. Only with the arm in its zero "
    "pose and teleop stopped.");
  connect(zero_button_, &QPushButton::clicked, this, [this]() {requestZero();});
  root_layout->addWidget(zero_button_);

  status_label_ = new QLabel();
  status_label_->setWordWrap(true);
  root_layout->addWidget(status_label_);
  root_layout->addStretch(1);
}

void SetupPanel::clearRows()
{
  // The widgets go when Qt is done with any handler that is running.
  for (auto & row : row_widgets_) {
    for (QWidget * widget : {static_cast<QWidget *>(row.icon),
        static_cast<QWidget *>(row.title), static_cast<QWidget *>(row.message)})
    {
      rows_layout_->removeWidget(widget);
      widget->hide();
      widget->deleteLater();
    }
  }
  row_widgets_.clear();
  row_names_.clear();
}

void SetupPanel::rebuildRows(const std::vector<std::string> & step_names)
{
  clearRows();
  row_names_ = step_names;
  int line = 0;
  for (std::size_t index = 0; index < step_names.size(); ++index) {
    RowWidgets row;
    row.icon = new QLabel();
    row.title = new QLabel();
    row.message = new QLabel();
    row.message->setWordWrap(true);
    rows_layout_->addWidget(row.icon, line, 0, Qt::AlignTop);
    rows_layout_->addWidget(row.title, line, 1, Qt::AlignTop);
    rows_layout_->addWidget(row.message, line, 2);
    row_widgets_.push_back(row);
    ++line;
  }
}

void SetupPanel::poll()
{
  applyMailbox();
  expirePendingZero(std::chrono::steady_clock::now());
  render();
}

void SetupPanel::applyMailbox()
{
  std::optional<SetupSnapshot> snapshot;
  std::optional<std::chrono::steady_clock::time_point> received_at;
  std::optional<SetupMailbox::ZeroReply> reply;
  {
    std::lock_guard<std::mutex> lock(mailbox_->mutex);
    snapshot.swap(mailbox_->snapshot);
    received_at = mailbox_->received_at;
    reply.swap(mailbox_->zero_reply);
  }
  if (received_at.has_value()) {
    last_message_ = received_at;
  }
  if (snapshot.has_value()) {
    applySnapshot(std::move(*snapshot));
  }
  if (reply.has_value() && pending_zero_.has_value() && pending_zero_->token == reply->token) {
    pending_zero_.reset();
    showStatus(QString::fromStdString(reply->message), reply->success);
  }
}

void SetupPanel::applySnapshot(SetupSnapshot snapshot)
{
  if (!snapshot.valid) {
    // Keep showing the last good steps, and say that this message was ignored.
    showStatus(QString::fromStdString("Ignoring a malformed setup report: " + snapshot.error), false);
    return;
  }
  snapshot_ = std::move(snapshot);
  has_snapshot_ = true;
  seen_guide_ = true;
}

void SetupPanel::expirePendingZero(const std::chrono::steady_clock::time_point now)
{
  if (pending_zero_.has_value() && now - pending_zero_->sent > kZeroTimeout) {
    if (zero_client_) {
      zero_client_->remove_pending_request(pending_zero_->request_id);
    }
    pending_zero_.reset();
    showStatus("No answer from the drive node to the zero request", false);
  }
}

bool SetupPanel::zeroOffered() const
{
  return has_snapshot_ && std::any_of(
    snapshot_.steps.begin(), snapshot_.steps.end(), [](const SetupStep & step) {
      return step.action == "zero" && step.state != StepState::Ok;
    });
}

void SetupPanel::requestZero()
{
  if (!node_ || !zero_client_ || pending_zero_.has_value() || !zeroOffered() ||
    setupGuideQuiet(last_message_, std::chrono::steady_clock::now()))
  {
    return;
  }
  const auto answer = QMessageBox::question(
    this, "Zero the arm",
    "Is the arm in its zero pose, upright as RViz shows it, with teleop stopped?\n\n"
    "Zeroing makes the arm's present position the zero of every drive.",
    QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
  if (answer != QMessageBox::Yes) {
    return;
  }
  if (!zero_client_->service_is_ready()) {
    showStatus("Drive node not available", false);
    return;
  }
  PendingZero call;
  call.token = ++next_token_;
  call.sent = std::chrono::steady_clock::now();
  const auto sent = zero_client_->async_send_request(
    std::make_shared<Trigger::Request>(),
    [mailbox = mailbox_, token = call.token](TriggerClient::SharedFuture future) {
      const auto response = future.get();
      std::lock_guard<std::mutex> lock(mailbox->mutex);
      mailbox->zero_reply = SetupMailbox::ZeroReply{token, response->success, response->message};
    });
  call.request_id = sent.request_id;
  pending_zero_ = call;
  render();
}

void SetupPanel::showStatus(const QString & text, const bool success)
{
  status_text_ = text;
  status_success_ = success;
  status_expires_ = std::chrono::steady_clock::now() + kStatusLifetime;
}

SetupDockInput SetupPanel::dockInput(const bool quiet) const
{
  if (quiet) {
    // A guide that was there and has gone is a fault; one that has not started yet is not.
    return seen_guide_ ? SetupDockInput::NeedsAttention : SetupDockInput::Waiting;
  }
  if (!has_snapshot_) {
    return SetupDockInput::Waiting;
  }
  if (snapshot_.needsAttention()) {
    return SetupDockInput::NeedsAttention;
  }
  return snapshot_.ready() ? SetupDockInput::Ready : SetupDockInput::Waiting;
}

QDockWidget * SetupPanel::dock() const
{
  // The dock may hold the panel through a scroll area.
  for (QWidget * widget = parentWidget(); widget != nullptr; widget = widget->parentWidget()) {
    if (auto * found = qobject_cast<QDockWidget *>(widget)) {
      return found;
    }
  }
  return nullptr;
}

void SetupPanel::applyDockAction(const DockAction action)
{
  // RViz keeps every panel in a dock of its own. Hiding it is what the dock's close button does,
  // and the Panels menu shows it again.
  auto * panel_dock = dock();
  if (panel_dock == nullptr || action == DockAction::None) {
    return;
  }
  if (action == DockAction::Hide) {
    panel_dock->hide();
  } else {
    panel_dock->show();
    panel_dock->raise();
  }
}

void SetupPanel::resizeEvent(QResizeEvent * event)
{
  rviz_common::Panel::resizeEvent(event);
  fitHeight();
}

// A word-wrapped label reports a one-line minimum height, and a dock gives its panel no height
// for its width, so a long message was squeezed over the row below it. Asking for the height the
// layout needs at this width keeps the text readable: a dock or scroll area that is shorter
// scrolls the panel instead.
void SetupPanel::fitHeight()
{
  // Before the dock has laid the panel out it is far narrower than it will be, so the height it
  // would need there is meaningless, and as a minimum it would stop the window from maximizing.
  if (layout() == nullptr || width() < minimumWidth()) {
    return;
  }
  const int needed = std::min(layout()->totalHeightForWidth(width()), kMaxFittedHeight);
  if (needed > 0 && needed != minimumHeight()) {
    setMinimumHeight(needed);
  }
}

void SetupPanel::render()
{
  const auto now = std::chrono::steady_clock::now();
  const bool quiet = setupGuideQuiet(last_message_, now);
  const bool shown = has_snapshot_ && !quiet;
  const bool ready = shown && snapshot_.ready();
  applyDockAction(dock_policy_.update(dockInput(quiet), now));

  if (!shown) {
    summary_label_->setObjectName("Muted");
    summary_label_->setText(QString::fromStdString(setupWaitingMessage(seen_guide_, quiet)));
  } else if (ready) {
    summary_label_->setObjectName("Ready");
    summary_label_->setText(QString::fromUtf8("\xE2\x9C\x93 Ready: the arm can be operated"));
  } else {
    summary_label_->setObjectName("Muted");
    const auto * current = snapshot_.currentStep();
    summary_label_->setText(
      current != nullptr ? QString::fromStdString("Next: " + current->title) : QString());
  }
  // The object name picks the style, so it has to be applied again.
  summary_label_->style()->unpolish(summary_label_);
  summary_label_->style()->polish(summary_label_);

  std::vector<std::string> names;
  if (shown && !ready) {
    for (const auto & step : snapshot_.steps) {
      names.push_back(step.name);
    }
  }
  if (names != row_names_) {
    rebuildRows(names);
  }
  rows_widget_->setVisible(!names.empty());

  for (std::size_t index = 0; index < row_names_.size(); ++index) {
    const auto & step = snapshot_.steps[index];
    auto & widgets = row_widgets_[index];
    const auto look = lookFor(step.state);
    widgets.icon->setText(QString::fromUtf8(look.icon));
    widgets.icon->setStyleSheet(QString("color: %1; font-weight: 700;").arg(look.color));
    widgets.title->setText(QString::fromStdString(step.title));
    widgets.title->setStyleSheet(step.current ? "font-weight: 700;" : "color: #8ea3b1;");
    widgets.message->setText(QString::fromStdString(step.message));
    widgets.message->setStyleSheet(step.current ? "" : "color: #8ea3b1;");
  }

  const bool offered = shown && zeroOffered();
  zero_button_->setVisible(offered);
  zero_button_->setEnabled(offered && !pending_zero_.has_value());
  zero_button_->setText(pending_zero_.has_value() ? "Zeroing..." : "Zero the arm");

  fitHeight();

  if (now < status_expires_) {
    status_label_->setStyleSheet(
      status_success_ ? "color: #3ddc84; font-weight: 700;" : "color: #ffb84d; font-weight: 700;");
    status_label_->setText(status_text_);
  } else {
    status_label_->setText(QString());
  }
}

}  // namespace waybionic_rviz_plugins

PLUGINLIB_EXPORT_CLASS(waybionic_rviz_plugins::SetupPanel, rviz_common::Panel)
