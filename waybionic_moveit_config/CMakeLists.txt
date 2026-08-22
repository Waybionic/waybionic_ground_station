cmake_minimum_required(VERSION 3.8)
project(waybionic_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz srdf urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/ik_xyz_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
