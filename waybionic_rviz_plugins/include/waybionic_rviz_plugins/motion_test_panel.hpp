#ifndef WAYBIONIC_RVIZ_PLUGINS__MOTION_TEST_PANEL_HPP_
#define WAYBIONIC_RVIZ_PLUGINS__MOTION_TEST_PANEL_HPP_

#include <memory>
#include <string>
#include <thread>

#include <QLabel>
#include <QPushButton>

#include <rclcpp/node.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/string.hpp>

#include <rviz_common/panel.hpp>

namespace waybionic_rviz_plugins
{

  class MotionTestPanel : public rviz_common::Panel
  {
    Q_OBJECT

  public:
    explicit MotionTestPanel(QWidget *parent = nullptr);
    ~MotionTestPanel() override;

    void onInitialize() override;

  private:
    void buildUi();

    void runTest();
    void moveHome();
    void stopTest();

    void publishCommand(const std::string &command);
    void handleStatus(const std_msgs::msg::String::SharedPtr message);

    QPushButton *run_button_{nullptr};
    QPushButton *home_button_{nullptr};
    QPushButton *stop_button_{nullptr};

    QLabel *status_label_{nullptr};
    QLabel *position_label_{nullptr};
    QLabel *result_label_{nullptr};

    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    std::thread executor_thread_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscription_;
    bool test_running_{false};
  };

} // namespace waybionic_rviz_plugins

#endif // WAYBIONIC_RVIZ_PLUGINS__MOTION_TEST_PANEL_HPP_
