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
find_package(std_srvs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  diagnostic_msgs
  pluginlib
  rclcpp
  rviz_common
  std_srvs
)

add_library(${PROJECT_NAME} SHARED
  include/waybionic_rviz_plugins/diagnostics_source.hpp
  include/waybionic_rviz_plugins/diagnostics_panel.hpp
  include/waybionic_rviz_plugins/ik_demo_panel.hpp
  include/waybionic_rviz_plugins/ros_diagnostics_source.hpp
  src/diagnostics_panel.cpp
  src/ik_demo_panel.cpp
  src/mock_diagnostics_source.cpp
  src/ros_diagnostics_source.cpp
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

install(
  PROGRAMS scripts/temporary_diagnostics_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_lint_cmake()
  ament_xmllint()

  ament_add_pytest_test(
    test_package_metadata
    test/test_package_metadata.py
    TIMEOUT 60
  )

  # Built from sources directly so the stress test links neither Qt nor RViz and
  # can run headless.
  ament_add_gtest(test_ros_diagnostics_source
    test/test_ros_diagnostics_source.cpp
    src/ros_diagnostics_source.cpp
    TIMEOUT 300
  )
  if(TARGET test_ros_diagnostics_source)
    target_include_directories(test_ros_diagnostics_source PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    ament_target_dependencies(test_ros_diagnostics_source
      diagnostic_msgs
      rclcpp
    )
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
