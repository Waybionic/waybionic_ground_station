#include "waybionic_rviz_plugins/motion_test_panel.hpp"

#include <QHBoxLayout>
#include <QMetaObject>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>

namespace waybionic_rviz_plugins
{

  MotionTestPanel::MotionTestPanel(QWidget *parent)
      : rviz_common::Panel(parent)
  {
    buildUi();
  }

  MotionTestPanel::~MotionTestPanel()
  {
    if (executor_)
    {
      executor_->cancel();
    }
    if (executor_thread_.joinable())
    {
      executor_thread_.join();
    }
  }

  void MotionTestPanel::onInitialize()
  {
    ros_node_ = std::make_shared<rclcpp::Node>("waybionic_motion_test_panel");

    command_publisher_ = ros_node_->create_publisher<std_msgs::msg::String>(
        "/old_arm_motion_test/command", rclcpp::QoS(10));
    status_subscription_ = ros_node_->create_subscription<std_msgs::msg::String>(
        "/old_arm_motion_test/status", rclcpp::QoS(10),
        [this](const std_msgs::msg::String::SharedPtr message)
        { handleStatus(message); });
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(ros_node_);
    executor_thread_ = std::thread([this]()
                                   { executor_->spin(); });
  }
  void MotionTestPanel::buildUi()
  {
    setMinimumWidth(360);

    auto *layout = new QVBoxLayout(this);

    auto *title = new QLabel("Old Arm Prototype Motion Test");
    title->setStyleSheet(
        "font-size: 16px;"
        "font-weight: 700;"
        "padding: 6px;");

    status_label_ = new QLabel("Status: READY");
    status_label_->setStyleSheet(
        "color: #3ddc84;"
        "font-weight: 800;"
        "padding: 6px;");

    position_label_ = new QLabel("Four-joint trajectory via /joint_states");

    result_label_ = new QLabel("No test run yet.");
    result_label_->setWordWrap(true);

    run_button_ = new QPushButton("▶  RUN MOTION TEST");
    home_button_ = new QPushButton("↩  HOME");
    stop_button_ = new QPushButton("■  STOP");

    stop_button_->setEnabled(false);

    connect(run_button_, &QPushButton::clicked, this, [this]()
            { runTest(); });

    connect(home_button_, &QPushButton::clicked, this, [this]()
            { moveHome(); });

    connect(stop_button_, &QPushButton::clicked, this, [this]()
            { stopTest(); });

    auto *button_row = new QHBoxLayout();
    button_row->addWidget(home_button_);
    button_row->addWidget(stop_button_);

    layout->addWidget(title);
    layout->addWidget(status_label_);
    layout->addWidget(position_label_);
    layout->addWidget(run_button_);
    layout->addLayout(button_row);
    layout->addWidget(result_label_);
    layout->addStretch();
  }

  void MotionTestPanel::runTest()
  {
    if (test_running_)
    {
      return;
    }

    test_running_ = true;

    result_label_->setText("Running motion sequence...");
    status_label_->setText("RUNNING: synchronized 4-DOF sequence");

    run_button_->setEnabled(false);
    home_button_->setEnabled(false);
    stop_button_->setEnabled(true);

    publishCommand("RUN");
  }

  void MotionTestPanel::moveHome()
  {
    test_running_ = false;

    status_label_->setText("Moving to HOME");
    result_label_->setText("Returning to home position...");

    run_button_->setEnabled(false);
    home_button_->setEnabled(false);
    stop_button_->setEnabled(true);

    publishCommand("HOME");
  }

  void MotionTestPanel::stopTest()
  {
    test_running_ = false;
    publishCommand("STOP");

    status_label_->setText("STOP REQUESTED");
    result_label_->setText("Waiting for the Arduino to confirm HOLD.");

    run_button_->setEnabled(false);
    home_button_->setEnabled(false);
    stop_button_->setEnabled(false);
  }

  void MotionTestPanel::publishCommand(const std::string &command)
  {
    if (!command_publisher_)
    {
      return;
    }

    std_msgs::msg::String message;
    message.data = command;
    command_publisher_->publish(message);
  }

  void MotionTestPanel::handleStatus(const std_msgs::msg::String::SharedPtr message)
  {
    const QString status = QString::fromStdString(message->data);
    QMetaObject::invokeMethod(this, [this, status]()
                              {
    if (status == "COMPLETE" || status == "READY") {
      test_running_ = false;
      status_label_->setText("READY");
      result_label_->setText(status == "COMPLETE" ? "Motion sequence complete." : "Ready.");
      run_button_->setEnabled(true);
      home_button_->setEnabled(true);
      stop_button_->setEnabled(false);
    } else if (status == "HOME_REQUIRED") {
      test_running_ = false;
      status_label_->setText("HOME REQUIRED");
      result_label_->setText("Move to HOME before running the sequence.");
      run_button_->setEnabled(false);
      home_button_->setEnabled(true);
      stop_button_->setEnabled(false);
    } else if (status == "CONNECTING") {
      test_running_ = false;
      status_label_->setText("CONNECTING");
      result_label_->setText("Waiting for the Arduino handshake.");
      run_button_->setEnabled(false);
      home_button_->setEnabled(false);
      stop_button_->setEnabled(false);
    } else if (status == "STOP_REQUESTED") {
      test_running_ = false;
      status_label_->setText("STOP REQUESTED");
      result_label_->setText("Waiting for the Arduino to confirm HOLD.");
      run_button_->setEnabled(false);
      home_button_->setEnabled(false);
      stop_button_->setEnabled(false);
    } else if (status == "STOPPED" || status == "FAULT" || status == "ERROR") {
      test_running_ = false;
      status_label_->setText(status);
      result_label_->setText("Motion unavailable or stopped.");
      run_button_->setEnabled(status == "STOPPED");
      home_button_->setEnabled(status == "STOPPED");
      stop_button_->setEnabled(false);
    } }, Qt::QueuedConnection);
  }

} // namespace waybionic_rviz_plugins

PLUGINLIB_EXPORT_CLASS(
    waybionic_rviz_plugins::MotionTestPanel,
    rviz_common::Panel)
