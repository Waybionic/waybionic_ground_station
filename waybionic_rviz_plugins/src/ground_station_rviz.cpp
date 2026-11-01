#include <algorithm>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include <QApplication>
#include <QDebug>
#include <QDockWidget>
#include <QFrame>
#include <QList>
#include <QMainWindow>
#include <QScrollArea>
#include <QString>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/ros_integration/ros_client_abstraction.hpp"
#include "rviz_common/visualization_frame.hpp"
#include "rviz_common/visualizer_app.hpp"

namespace
{

// A panel asks for the height of everything in it. The three stacked below ask for more than a
// laptop screen has between them, which made the window taller than the screen and cut off its
// bottom. In a scroll area a dock can be shorter than its panel, and the panel scrolls.
void scrollInDock(QDockWidget * dock)
{
  QWidget * panel = dock->widget();
  if (panel == nullptr || qobject_cast<QScrollArea *>(panel) != nullptr) {
    return;
  }
  auto * scroll = new QScrollArea();
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setWidgetResizable(true);
  // Giving the dock another widget hides the panel, so put it in the scroll area and show it.
  dock->setWidget(scroll);
  scroll->setWidget(panel);
  panel->show();
}

// The station's own panels, from the top of the right-hand dock down, with the share of its
// height each gets. RViz puts a panel its saved window state does not know in the left dock, so
// they are stacked here whatever the layout file says. Panels that are not loaded are skipped.
// The setup panel never scrolls: it asks for exactly the height its text needs, and it is the
// one the operator has to read while it is open.
void stackRightDock(QMainWindow * frame)
{
  struct Entry
  {
    const char * name;
    int weight;
    bool scrolls;
  };
  const Entry order[] = {
    {"SetupPanel", 2, false}, {"JointControlPanel", 2, true}, {"DiagnosticsPanel", 5, true}};
  QList<QDockWidget *> docks;
  QList<int> weights;
  for (const auto & entry : order) {
    if (auto * dock = frame->findChild<QDockWidget *>(QString::fromLatin1(entry.name))) {
      if (entry.scrolls) {
        scrollInDock(dock);
      }
      docks.append(dock);
      weights.append(entry.weight);
    }
  }
  if (docks.isEmpty()) {
    return;
  }
  frame->addDockWidget(Qt::RightDockWidgetArea, docks.first());
  for (int index = 1; index < docks.size(); ++index) {
    frame->splitDockWidget(docks[index - 1], docks[index], Qt::Vertical);
  }
  frame->resizeDocks(docks, weights, Qt::Vertical);
}

}  // namespace

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
      stackRightDock(frame);
      frame->showMaximized();
      return application.exec();
    }
  }
  qCritical("RViz initialized without a visualization window");
  return EXIT_FAILURE;
}
