#include "waybionic_rviz_plugins/ik_demo_panel.hpp"

#include <chrono>
#include <exception>
#include <memory>
#include <string>

#include <QLabel>
#include <QPushButton>
#include <QString>
#include <QTimer>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace waybionic_rviz_plugins
{
namespace
{

constexpr auto kPollInterval = std::chrono::milliseconds(200);
constexpr auto kRequestTimeout = std::chrono::seconds(3);
constexpr char kReplayService[] = "/ik_demo/replay";

}  // namespace

IkDemoPanel::IkDemoPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  setMinimumWidth(260);

  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(10, 10, 10, 10);
  layout->setSpacing(8);

  auto * title = new QLabel("XYZ Inverse Kinematics", this);
  auto title_font = title->font();
  title_font.setBold(true);
  title->setFont(title_font);

  auto * instructions = new QLabel(
    "Move the mock arm along X, Y, and Z, then return it to center.", this);
  instructions->setWordWrap(true);

  replay_button_ = new QPushButton("Replay XYZ Demo", this);
  replay_button_->setEnabled(false);
  connect(replay_button_, &QPushButton::clicked, this, [this]() {requestReplay();});

  status_label_ = new QLabel("Waiting for the IK demo service...", this);
  status_label_->setWordWrap(true);

  layout->addWidget(title);
  layout->addWidget(instructions);
  layout->addWidget(replay_button_);
  layout->addWidget(status_label_);
  layout->addStretch(1);
}

IkDemoPanel::~IkDemoPanel()
{
  if (poll_timer_ != nullptr) {
    poll_timer_->stop();
  }
  cancelPendingRequest();
  replay_client_.reset();
}

void IkDemoPanel::onInitialize()
{
  if (auto ros_node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock()) {
    rviz_node_ = ros_node_abstraction->get_raw_node();
  }

  if (rviz_node_) {
    replay_client_ = rviz_node_->create_client<ReplayService>(kReplayService);
  } else {
    status_label_->setText("RViz ROS node is unavailable.");
  }

  poll_timer_ = new QTimer(this);
  connect(poll_timer_, &QTimer::timeout, this, [this]() {pollReplayService();});
  poll_timer_->start(static_cast<int>(kPollInterval.count()));
  pollReplayService();
}

void IkDemoPanel::cancelPendingRequest()
{
  if (pending_request_ && replay_client_) {
    replay_client_->remove_pending_request(*pending_request_);
  }
  pending_request_.reset();
}

void IkDemoPanel::requestReplay()
{
  if (pending_request_) {
    return;
  }
  if (!replay_client_ || !replay_client_->service_is_ready()) {
    replay_button_->setEnabled(false);
    status_label_->setText("IK demo service is unavailable.");
    service_was_ready_ = false;
    return;
  }

  try {
    auto request = std::make_shared<ReplayService::Request>();
    pending_request_.emplace(replay_client_->async_send_request(request));
    request_deadline_ = std::chrono::steady_clock::now() + kRequestTimeout;
    replay_button_->setEnabled(false);
    status_label_->setText("Requesting XYZ demo replay...");
  } catch (const std::exception & error) {
    pending_request_.reset();
    replay_button_->setEnabled(true);
    status_label_->setText(QString("Could not request replay: %1").arg(error.what()));
  }
}

void IkDemoPanel::pollReplayService()
{
  if (pending_request_) {
    if (pending_request_->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      try {
        const auto response = pending_request_->get();
        status_label_->setText(QString::fromStdString(response->message));
      } catch (const std::exception & error) {
        status_label_->setText(QString("Replay request failed: %1").arg(error.what()));
      }
      pending_request_.reset();
    } else if (std::chrono::steady_clock::now() >= request_deadline_) {
      cancelPendingRequest();
      status_label_->setText("Replay request timed out.");
    }
  }

  const bool service_ready = replay_client_ && replay_client_->service_is_ready();
  replay_button_->setEnabled(service_ready && !pending_request_);

  if (service_ready != service_was_ready_) {
    if (service_ready) {
      status_label_->setText("Ready to replay the XYZ demo.");
    } else if (!pending_request_) {
      status_label_->setText("Waiting for /ik_demo/replay...");
    }
    service_was_ready_ = service_ready;
  }
}

}  // namespace waybionic_rviz_plugins

PLUGINLIB_EXPORT_CLASS(waybionic_rviz_plugins::IkDemoPanel, rviz_common::Panel)
