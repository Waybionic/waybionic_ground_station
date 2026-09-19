/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include <moveit/moveit_cpp/moveit_cpp.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <tf2_ros/transform_listener.h>
#include <moveit/move_group/move_group_capability.hpp>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.hpp>
#include <boost/tokenizer.hpp>
#include <moveit/macros/console_colors.hpp>
#include <moveit/move_group/move_group_context.hpp>
#include <memory>
#include <set>
#include <moveit/utils/logger.hpp>

static const std::string ROBOT_DESCRIPTION =
    "robot_description";  // name of the robot description (a param name, so it can be changed externally)

namespace move_group
{
namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.ros.move_group.executable");
}
}  // namespace

// These capabilities are loaded unless listed in disable_capabilities
// clang-format off
static const char* const DEFAULT_CAPABILITIES[] = {
   "move_group/LoadGeometryFromFileService",
   "move_group/SaveGeometryToFileService",
   "move_group/GetUrdfService",
   "move_group/MoveGroupCartesianPathService",
   "move_group/MoveGroupKinematicsService",
   "move_group/MoveGroupExecuteTrajectoryAction",
   "move_group/MoveGroupMoveAction",
   "move_group/MoveGroupPlanService",
   "move_group/MoveGroupQueryPlannersService",
   "move_group/MoveGroupStateValidationService",
   "move_group/MoveGroupGetPlanningSceneService",
   "move_group/ApplyPlanningSceneService",
   "move_group/ClearOctomapService",
};
// clang-format on

class MoveGroupExe
{
public:
  MoveGroupExe(const moveit_cpp::MoveItCppPtr& moveit_cpp, const std::string& default_planning_pipeline, bool debug,
               const std::shared_ptr<pluginlib::ClassLoader<MoveGroupCapability>>& capability_plugin_loader)
    : capability_plugin_loader_(capability_plugin_loader)
  {
    // if the user wants to be able to disable execution of paths, they can just set this ROS param to false
    bool allow_trajectory_execution;
    moveit_cpp->getNode()->get_parameter_or("allow_trajectory_execution", allow_trajectory_execution, true);

    context_ =
        std::make_shared<MoveGroupContext>(moveit_cpp, default_planning_pipeline, allow_trajectory_execution, debug);

    // start the capabilities
    configureCapabilities();
  }

  ~MoveGroupExe()
  {
    // WayBionic: the plugin loader is deliberately NOT reset here. The caller
    // keeps it alive until after the node is destroyed; see run() below.
    capabilities_.clear();
    context_.reset();
  }

  void status()
  {
    if (context_)
    {
      if (context_->status())
      {
        if (capabilities_.empty())
        {
          printf("\n" MOVEIT_CONSOLE_COLOR_BLUE
                 "move_group is running but no capabilities are loaded." MOVEIT_CONSOLE_COLOR_RESET "\n\n");
        }
        else
        {
          printf("\n" MOVEIT_CONSOLE_COLOR_GREEN "You can start planning now!" MOVEIT_CONSOLE_COLOR_RESET "\n\n");
        }
        fflush(stdout);
      }
    }
    else
      RCLCPP_ERROR(getLogger(), "No MoveGroup context created. Nothing will work.");
  }

  MoveGroupContextPtr getContext()
  {
    return context_;
  }

private:
  void configureCapabilities()
  {
    if (!capability_plugin_loader_)
    {
      RCLCPP_FATAL(getLogger(), "No plugin loader for move_group capabilities");
      return;
    }

    std::set<std::string> capabilities;

    // add default capabilities
    for (const char* capability : DEFAULT_CAPABILITIES)
      capabilities.insert(capability);

    // add capabilities listed in ROS parameter
    std::string capability_plugins;
    if (context_->moveit_cpp_->getNode()->get_parameter("capabilities", capability_plugins))
    {
      boost::char_separator<char> sep(" ");
      boost::tokenizer<boost::char_separator<char>> tok(capability_plugins, sep);
      capabilities.insert(tok.begin(), tok.end());
    }

    // add capabilities configured for planning pipelines
    for (const auto& pipeline_entry : context_->moveit_cpp_->getPlanningPipelines())
    {
      const auto& pipeline_name = pipeline_entry.first;
      std::string pipeline_capabilities;
      if (context_->moveit_cpp_->getNode()->get_parameter(pipeline_name + ".capabilities", pipeline_capabilities))
      {
        boost::char_separator<char> sep(" ");
        boost::tokenizer<boost::char_separator<char>> tok(pipeline_capabilities, sep);
        capabilities.insert(tok.begin(), tok.end());
      }
    }

    // drop capabilities that have been explicitly disabled
    if (context_->moveit_cpp_->getNode()->get_parameter("disable_capabilities", capability_plugins))
    {
      boost::char_separator<char> sep(" ");
      boost::tokenizer<boost::char_separator<char>> tok(capability_plugins, sep);
      for (boost::tokenizer<boost::char_separator<char>>::iterator cap_name = tok.begin(); cap_name != tok.end();
           ++cap_name)
        capabilities.erase(*cap_name);
    }

    for (const std::string& capability : capabilities)
    {
      try
      {
        printf(MOVEIT_CONSOLE_COLOR_CYAN "Loading '%s'..." MOVEIT_CONSOLE_COLOR_RESET "\n", capability.c_str());
        MoveGroupCapabilityPtr cap = capability_plugin_loader_->createUniqueInstance(capability);
        cap->setContext(context_);
        cap->initialize();
        capabilities_.push_back(cap);
      }
      catch (pluginlib::PluginlibException& ex)
      {
        RCLCPP_ERROR_STREAM(getLogger(),
                            "Exception while loading move_group capability '" << capability << "': " << ex.what());
      }
    }

    std::stringstream ss;
    ss << '\n';
    ss << '\n';
    ss << "********************************************************" << '\n';
    ss << "* MoveGroup using: " << '\n';
    for (const MoveGroupCapabilityPtr& cap : capabilities_)
      ss << "*     - " << cap->getName() << '\n';
    ss << "********************************************************" << '\n';
    RCLCPP_INFO(getLogger(), "%s", ss.str().c_str());
  }

  MoveGroupContextPtr context_;
  std::shared_ptr<pluginlib::ClassLoader<MoveGroupCapability>> capability_plugin_loader_;
  std::vector<MoveGroupCapabilityPtr> capabilities_;
};
}  // namespace move_group

/* WayBionic: main() rewritten so move_group shuts down without a segfault.
 *
 * Root cause of the upstream crash (moveit/moveit2#3680, ROS 2 Jazzy):
 * MoveIt loads its capabilities and its controller manager through pluginlib.
 * On shutdown the pluginlib ClassLoaders are destroyed -- which dlclose()s the
 * plugin shared libraries -- BEFORE the rclcpp nodes those plugins created
 * services/action clients on. The nodes' callback groups still hold expired
 * weak_ptrs whose control-block vtables live in the unmapped library, so
 * ~CallbackGroup faults with "Address not mapped to object".
 *   * capabilities: ~MoveGroupExe resets the loader before main destroys `nh`
 *   * controller manager: in TrajectoryExecutionManager the loader member is
 *     declared after `controller_mgr_node_`, so it is destroyed first
 *
 * This main() keeps both plugin libraries mapped until every node is gone:
 *   1. The capability ClassLoader is created here, before the node, and handed
 *      to MoveGroupExe, so it is destroyed after the node.
 *   2. A second ClassLoader pins the controller-manager plugin library.
 *      class_loader reference-counts libraries, so TEM's own loader can no
 *      longer unmap it while ours is alive.
 * It also owns the shutdown order explicitly: no rclcpp signal handler, the
 * executor is stopped first, and rclcpp::shutdown() runs last.
 */

#include <atomic>
#include <chrono>
#include <csignal>
#include <thread>
#include <moveit/controller_manager/controller_manager.hpp>

namespace
{
std::atomic<int> g_signal_received{ 0 };

void onSignal(int signum)
{
  g_signal_received.store(signum);
}

std::string resolveDefaultPipeline(const rclcpp::Node::SharedPtr& nh, moveit_cpp::MoveItCpp::Options& options)
{
  options.planning_pipeline_options.parent_namespace = nh->get_effective_namespace() + ".planning_pipelines";
  std::vector<std::string> planning_pipeline_configs;
  if (nh->get_parameter("planning_pipelines", planning_pipeline_configs))
  {
    if (planning_pipeline_configs.empty())
    {
      RCLCPP_ERROR(nh->get_logger(), "Failed to read parameter 'move_group.planning_pipelines'");
    }
    else
    {
      for (const auto& config : planning_pipeline_configs)
        options.planning_pipeline_options.pipeline_names.push_back(config);
    }
  }

  auto& pipeline_names = options.planning_pipeline_options.pipeline_names;
  std::string default_planning_pipeline;
  if (nh->get_parameter("default_planning_pipeline", default_planning_pipeline))
  {
    if (std::find(pipeline_names.begin(), pipeline_names.end(), default_planning_pipeline) == pipeline_names.end())
    {
      RCLCPP_WARN(nh->get_logger(),
                  "MoveGroup launched with ~default_planning_pipeline '%s' not configured in ~planning_pipelines",
                  default_planning_pipeline.c_str());
      default_planning_pipeline = "";
    }
  }
  else if (pipeline_names.size() > 1)
  {
    RCLCPP_WARN(nh->get_logger(),
                "MoveGroup launched without ~default_planning_pipeline specifying the namespace for the default "
                "planning pipeline configuration");
  }

  if (default_planning_pipeline.empty())
  {
    if (!pipeline_names.empty())
    {
      RCLCPP_WARN(nh->get_logger(), "Using default pipeline '%s'", pipeline_names[0].c_str());
      default_planning_pipeline = pipeline_names[0];
    }
    else
    {
      RCLCPP_WARN(nh->get_logger(), "Falling back to using the the move_group node namespace (deprecated behavior).");
      default_planning_pipeline = "move_group";
      options.planning_pipeline_options.pipeline_names = { default_planning_pipeline };
      options.planning_pipeline_options.parent_namespace = nh->get_effective_namespace();
    }
    nh->set_parameter(rclcpp::Parameter("default_planning_pipeline", default_planning_pipeline));
  }
  return default_planning_pipeline;
}

/// Keep the controller-manager plugin library mapped for as long as this object lives.
void pinControllerManagerLibrary(
    pluginlib::ClassLoader<moveit_controller_manager::MoveItControllerManager>& pin, const rclcpp::Node::SharedPtr& nh)
{
  std::string plugin;
  if (!nh->get_parameter("moveit_controller_manager", plugin))
  {
    const auto& classes = pin.getDeclaredClasses();
    if (classes.size() != 1)
      return;  // TEM will not load a controller manager either
    plugin = classes[0];
  }
  try
  {
    pin.loadLibraryForClass(plugin);
  }
  catch (const pluginlib::PluginlibException& ex)
  {
    RCLCPP_WARN(nh->get_logger(), "Could not pin controller manager plugin '%s': %s", plugin.c_str(), ex.what());
  }
}

int run(int argc, char** argv)
{
  // Declared first => destroyed last, after every node below.
  auto capability_loader = std::make_shared<pluginlib::ClassLoader<move_group::MoveGroupCapability>>(
      "moveit_ros_move_group", "move_group::MoveGroupCapability");
  pluginlib::ClassLoader<moveit_controller_manager::MoveItControllerManager> controller_manager_pin(
      "moveit_core", "moveit_controller_manager::MoveItControllerManager");

  rclcpp::NodeOptions opt;
  opt.allow_undeclared_parameters(true);
  opt.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("move_group", opt);
  moveit::setNodeLoggerName(nh->get_name());
  pinControllerManagerLibrary(controller_manager_pin, nh);

  moveit_cpp::MoveItCpp::Options moveit_cpp_options(nh);
  const std::string default_planning_pipeline = resolveDefaultPipeline(nh, moveit_cpp_options);

  auto moveit_cpp = std::make_shared<moveit_cpp::MoveItCpp>(nh, moveit_cpp_options);
  auto planning_scene_monitor = moveit_cpp->getPlanningSceneMonitorNonConst();
  if (!planning_scene_monitor->getPlanningScene())
  {
    RCLCPP_ERROR(nh->get_logger(), "Planning scene not configured");
    return 1;
  }

  bool debug = false;
  for (int i = 1; i < argc; ++i)
  {
    if (strncmp(argv[i], "--debug", 7) == 0)
    {
      debug = true;
      break;
    }
  }
  RCLCPP_INFO(nh->get_logger(), "MoveGroup debug mode is %s", debug ? "ON" : "OFF");

  auto executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
  auto mge = std::make_unique<move_group::MoveGroupExe>(moveit_cpp, default_planning_pipeline, debug,
                                                        capability_loader);

  bool monitor_dynamics;
  if (nh->get_parameter("monitor_dynamics", monitor_dynamics) && monitor_dynamics)
  {
    RCLCPP_INFO(nh->get_logger(), "MoveGroup monitors robot dynamics (higher load)");
    planning_scene_monitor->getStateMonitor()->enableCopyDynamics(true);
  }
  planning_scene_monitor->publishDebugInformation(debug);

  mge->status();
  executor->add_node(nh);
  std::thread spin_thread([&executor]() { executor->spin(); });

  while (g_signal_received.load() == 0 && rclcpp::ok())
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

  RCLCPP_INFO(nh->get_logger(), "Shutting down move_group (signal %d)", g_signal_received.load());
  executor->cancel();
  spin_thread.join();
  executor->remove_node(nh);

  // Explicit teardown order, all while the ROS context is still valid.
  mge.reset();
  planning_scene_monitor.reset();
  executor.reset();
  moveit_cpp.reset();
  nh.reset();
  return 0;  // plugin loaders are destroyed here, after the nodes
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::InitOptions init_options;
  init_options.shutdown_on_signal = false;
  rclcpp::init(argc, argv, init_options, rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  int exit_code = 0;
  try
  {
    exit_code = run(argc, argv);
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("move_group"), "move_group failed: %s", e.what());
    exit_code = 1;
  }

  rclcpp::shutdown();
  return exit_code;
}
