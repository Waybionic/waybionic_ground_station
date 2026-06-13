cmake_minimum_required(VERSION 3.8)
project(waybionic_rviz_plugins)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(sensor_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  diagnostic_msgs
  pluginlib
  rclcpp
  rviz_common
  rviz_default_plugins
  sensor_msgs
)

add_library(${PROJECT_NAME} SHARED
  include/waybionic_rviz_plugins/diagnostics_source.hpp
  include/waybionic_rviz_plugins/diagnostics_panel.hpp
  include/waybionic_rviz_plugins/ros_diagnostics_source.hpp
  include/waybionic_rviz_plugins/surgeon_camera_panel.hpp
  src/diagnostics_panel.cpp
  src/mock_diagnostics_source.cpp
  src/ros_diagnostics_source.cpp
  src/surgeon_camera_panel.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY config launch docs
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
