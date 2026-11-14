#include <algorithm>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include <QApplication>
#include <QDebug>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/ros_integration/ros_client_abstraction.hpp"
#include "rviz_common/visualization_frame.hpp"
#include "rviz_common/visualizer_app.hpp"

int main(int argc, char ** argv)
{
  auto arguments = rclcpp::remove_ros_arguments(argc, argv);
  // RViz's Ogre renderer needs X11 on Wayland unless the operator overrides Qt.
  if (qEnvironmentVariable("XDG_SESSION_TYPE") == "wayland" &&
    !qEnvironmentVariableIsSet("QT_QPA_PLATFORM") &&
    std::find(arguments.begin(), arguments.end(), "-platform") == arguments.end())
  {
    qputenv("QT_QPA_PLATFORM", "xcb");
  }
  std::vector<char *> qt_arguments;
  for (auto & argument : arguments) {
    qt_arguments.push_back(argument.data());
  }
  int qt_argc = static_cast<int>(qt_arguments.size());
  qt_arguments.push_back(nullptr);
  QApplication application(qt_argc, qt_arguments.data());
  const auto logger = rclcpp::get_logger("rviz2");
  rviz_common::set_logging_handlers(
    [logger](const std::string & message, const std::string &, size_t) {
      RCLCPP_DEBUG(logger, "%s", message.c_str());
    },
    [logger](const std::string & message, const std::string &, size_t) {
      RCLCPP_INFO(logger, "%s", message.c_str());
    },
    [logger](const std::string & message, const std::string &, size_t) {
      RCLCPP_WARN(logger, "%s", message.c_str());
    },
    [logger](const std::string & message, const std::string &, size_t) {
      RCLCPP_ERROR(logger, "%s", message.c_str());
    });
  rviz_common::VisualizerApp visualizer(
    std::make_unique<rviz_common::ros_integration::RosClientAbstraction>());
  visualizer.setApp(&application);
  if (!visualizer.init(argc, argv)) {
    return EXIT_FAILURE;
  }
  for (auto * widget : QApplication::topLevelWidgets()) {
    if (auto * frame = qobject_cast<rviz_common::VisualizationFrame *>(widget)) {
      frame->showMaximized();
      return application.exec();
    }
  }
  qCritical("RViz initialized without a visualization window");
  return EXIT_FAILURE;
}
