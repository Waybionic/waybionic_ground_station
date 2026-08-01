cmake_minimum_required(VERSION 3.8)
project(waybionic_camera_tools)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(image_latency_metrics src/metrics.cpp)
target_include_directories(image_latency_metrics
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

add_executable(image_latency_monitor src/image_latency_monitor.cpp)
target_link_libraries(image_latency_monitor image_latency_metrics)
target_include_directories(image_latency_monitor PRIVATE include)
ament_target_dependencies(image_latency_monitor
  diagnostic_msgs
  rclcpp
  sensor_msgs)

install(
  TARGETS image_latency_metrics image_latency_monitor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(image_latency_metrics)
ament_package()
