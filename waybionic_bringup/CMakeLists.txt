cmake_minimum_required(VERSION 3.8)
project(waybionic_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  # colcon tests packages in parallel; a private ROS domain keeps this /tf out of other tests.
  set(isolated RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
  add_launch_test(test/test_ground_station_launch.py ${isolated})
  add_launch_test(test/test_arm_model_launch.py ${isolated})
  add_launch_test(test/test_joint_demo_launch.py ${isolated})
endif()

install(PROGRAMS scripts/joint_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
