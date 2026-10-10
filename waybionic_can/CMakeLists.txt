cmake_minimum_required(VERSION 3.8)
project(waybionic_can)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Protocol, node, console and SLCAN logic live in the Arduino library so the sketches and the
# host build share one copy. No Linux, ROS or Arduino headers.
set(ARDUINO_LIB_SRC ${CMAKE_CURRENT_SOURCE_DIR}/arduino/libraries/WaybionicCan/src)
set(COMMON_SOURCES
  ${ARDUINO_LIB_SRC}/common/BringupConsole.cpp
  ${ARDUINO_LIB_SRC}/common/BusLoad.cpp
  ${ARDUINO_LIB_SRC}/common/Candump.cpp
  ${ARDUINO_LIB_SRC}/common/GatewayLogic.cpp
  ${ARDUINO_LIB_SRC}/common/MksFrame.cpp
  ${ARDUINO_LIB_SRC}/common/NodeLogic.cpp
  ${ARDUINO_LIB_SRC}/common/SlcanBridge.cpp
  ${ARDUINO_LIB_SRC}/common/SoftwareActuator.cpp
)
add_library(waybionic_can_common STATIC ${COMMON_SOURCES})
target_compile_features(waybionic_can_common PUBLIC cxx_std_17)
target_include_directories(waybionic_can_common PUBLIC ${ARDUINO_LIB_SRC} ${CMAKE_CURRENT_SOURCE_DIR})

# Compile the same files the way the Arduino build constrains them: no exceptions, no RTTI and
# no C++ standard library headers. Catches STL or exception use creeping into common/.
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_library(waybionic_can_embedded_check OBJECT ${COMMON_SOURCES})
  target_compile_features(waybionic_can_embedded_check PRIVATE cxx_std_17)
  target_include_directories(waybionic_can_embedded_check PRIVATE ${ARDUINO_LIB_SRC})
  target_compile_options(waybionic_can_embedded_check PRIVATE
    -fno-exceptions -fno-rtti -nostdinc++ -Werror)
endif()

# Host-only simulation pieces. SocketCAN is Linux-only, so a macOS ground station builds the
# common code, slcan_sim and unit tests without it.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  add_library(waybionic_can_sim STATIC
    sim/SocketCanTransport.cpp
  )
  target_link_libraries(waybionic_can_sim PUBLIC waybionic_can_common)

  add_executable(gateway sim/gateway_main.cpp)
  target_link_libraries(gateway waybionic_can_sim)

  add_executable(receiver sim/receiver_main.cpp)
  target_link_libraries(receiver waybionic_can_sim)

  install(TARGETS gateway receiver DESTINATION lib/${PROJECT_NAME})
  install(PROGRAMS scripts/run_four_node_demo.sh DESTINATION lib/${PROJECT_NAME})
endif()

if(UNIX)
  add_executable(slcan_sim sim/slcan_sim_main.cpp)
  target_link_libraries(slcan_sim waybionic_can_common)
  install(TARGETS slcan_sim DESTINATION lib/${PROJECT_NAME})
endif()

add_executable(mks_frame_tool sim/mks_frame_tool.cpp)
target_link_libraries(mks_frame_tool waybionic_can_common)

install(TARGETS mks_frame_tool DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  set(UNIT_TESTS
    test_mks_frame test_node_logic test_gateway_logic test_bringup_console test_slcan_bridge
    test_bus_load)
  foreach(test_name ${UNIT_TESTS})
    ament_add_gtest(${test_name} tests/unit/${test_name}.cpp)
    if(TARGET ${test_name})
      target_link_libraries(${test_name} waybionic_can_common)
    endif()
  endforeach()

  # Skips itself when the vcan interface is missing (for example inside Docker CI).
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    ament_add_gtest(test_vcan_four_node tests/integration/test_vcan_four_node.cpp TIMEOUT 240)
    if(TARGET test_vcan_four_node)
      target_link_libraries(test_vcan_four_node waybionic_can_sim)
      target_compile_definitions(test_vcan_four_node PRIVATE
        GATEWAY_EXE="$<TARGET_FILE:gateway>"
        RECEIVER_EXE="$<TARGET_FILE:receiver>"
      )
      add_dependencies(test_vcan_four_node gateway receiver)
    endif()
  endif()
endif()

ament_package()
