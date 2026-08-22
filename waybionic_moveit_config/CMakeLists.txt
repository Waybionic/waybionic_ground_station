cmake_minimum_required(VERSION 3.8)
project(waybionic_moveit_config)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  add_launch_test(
    test/test_ik_demo_launch.py
    TIMEOUT 120
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
  )
endif()

install(DIRECTORY config launch rviz srdf urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/ik_xyz_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
