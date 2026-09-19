cmake_minimum_required(VERSION 3.22)
project(waybionic_move_group)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost REQUIRED)

add_executable(move_group src/move_group.cpp)
ament_target_dependencies(move_group
  rclcpp
  pluginlib
  moveit_core
  moveit_ros_planning
  moveit_ros_move_group
  tf2_ros
)
target_link_libraries(move_group Boost::boost)

install(TARGETS move_group DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
